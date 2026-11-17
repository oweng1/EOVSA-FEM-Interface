# EOVSA 2-Meter Front End Controller (`fe2.solar.pvt`)

MicroPython firmware for the **WIZnet EVB-Pico2** (`RP2350` + WIZnet Ethernet) implementing the **EOVSA 2-Meter Front End Assembly Programming Model** over Modbus TCP (`Port 502`, Unit ID `0xFF`), synchronized to a `1 PPS` hardware trigger.

---

## 1. Network & Firmware Configuration

* **Hardware:** WIZnet EVB-Pico2 (`W5500-EVB-Pico2` / `W5100S-EVB-Pico2` / `W6100-EVB-Pico2`)
* **MicroPython Driver:** `network.WIZNET6K()` (WIZnet-ioNIC UF2 build; auto-configures onboard SPI pins)
* **Assigned Hostname & IP:** `fe2.solar.pvt` (`192.168.24.62`, DHCP reservation; front-end fleet range `.61`–`.76`)
* **Modbus Transport:** Modbus TCP on **Port `502`**, Unit ID **`0xFF` (`255`)**
* **Watchdog:** RP2350 Hardware Watchdog (`machine.WDT`) enabled with an **8,000 ms** timeout

---

## 2. GPIO & ADC Pinout

| Pico 2 Pin | Direction | Function | Notes |
| :--- | :--- | :--- | :--- |
| **`GPIO 0–4`** | Output | Attenuator Data Bits (`D0–D4`) | Inverted 5-bit attenuation value (`~val & 0x1F`, `0–31` dB) |
| **`GPIO 5`** | Output | H POL Attenuator #1 Latch Enable | Active-high pulse (`1` -> `0`) |
| **`GPIO 6`** | Output | H POL Attenuator #2 Latch Enable | Active-high pulse (`1` -> `0`) |
| **`GPIO 7`** | Output | V POL Attenuator #1 Latch Enable | Active-high pulse (`1` -> `0`) |
| **`GPIO 8`** | Output | V POL Attenuator #2 Latch Enable | Active-high pulse (`1` -> `0`) |
| **`GPIO 9`** | Output | Noise Source DC Power Control | `1` = ON, `0` = OFF |
| **`GPIO 10`** | Input (IRQ) | `1 PPS` Rising-Edge Trigger | Latches queued outputs and samples ADCs |
| **`ADC 26`** | Analog In | H POL Detector Output | 16-sample average, scaled to 11-bit (`0–2047`) |
| **`ADC 27`** | Analog In | V POL Detector Output | 16-sample average, scaled to 11-bit (`0–2047`) |
| **`ADC 28`** | Analog In | Temperature Sensor | 16-sample average, scaled to 11-bit (`T = 30 * (2.0 * N / 2048)`) |

---

## 3. Active Modbus TCP Register Map

*(Obsolete optical transmitter commands `0x03` at `0x0000`/`0x0042` and `0x05` at `0x0000`/`0x0001` are excluded.)*

| Function Code | Address | Direction | Name & Format |
| :--- | :--- | :--- | :--- |
| **`0x06`** (Write Reg) | `0x0000` | Master → Pico | **V POL Attenuators:** High byte = Att #1 (`0–31` dB), Low byte = Att #2 (`0–31` dB). Queued for next `1 PPS`. |
| **`0x06`** (Write Reg) | `0x0001` | Master → Pico | **H POL Attenuators:** High byte = Att #1 (`0–31` dB), Low byte = Att #2 (`0–31` dB). Queued for next `1 PPS`. |
| **`0x05`** (Write Coil) | `0x0002` | Master → Pico | **Noise Source DC Power:** High byte `0xFF` = ON, `0x00` = OFF. Queued for next `1 PPS`. |
| **`0x04`** (Read IReg) | `0x0000` | Pico → Master | **V POL Detector Monitor:** 11-bit unsigned integer (`0–2047`). |
| **`0x04`** (Read IReg) | `0x0001` | Pico → Master | **H POL Detector Monitor:** 11-bit unsigned integer (`0–2047`). |
| **`0x04`** (Read IReg) | `0x0002` | Pico → Master | **Temperature Sensor Monitor:** 11-bit unsigned integer (`0–2047`). |
| **`0x04`** (Read IReg) | `0xFFFF` | Pico → Master | **Firmware Revision:** 16-bit unsigned integer (`2`). |
| **`0x01`** (Read Coil) | `0x0002` | Pico → Master | **Noise Source Status Readback:** `0x01` = ON, `0x00` = OFF. |
| **`0x03`** (Read HRegs) | `0x01D4` (`2` regs) | Pico → Master | **Attenuator Readback:** 4 bytes `[V_Att1, V_Att2, H_Att1, H_Att2]`. |

---

## 4. Production MicroPython Code (`main.py`)

```python
from machine import Pin, ADC, WDT
import network
import socket
import struct
import time

# ==============================================================================
# 1. HARDWARE PIN & ADC SETUP
# ==============================================================================

attnpins = [Pin(i, Pin.OUT, value=1) for i in range(0, 5)]   # GPIO 0-4: Attenuator bits
lepins = [Pin(i, Pin.OUT, value=0) for i in range(5, 9)]     # GPIO 5-8: Latch enables
ndpin = Pin(9, Pin.OUT, value=0)                             # GPIO 9: Noise diode
detadc = [ADC(i) for i in range(26, 28)]                     # ADC 26 (H Pol), ADC 27 (V Pol)
tempadc = ADC(28)                                            # ADC 28: Temperature sensor
ppspin = Pin(10, Pin.IN)                                     # GPIO 10: 1 PPS trigger

sethpolattn = False
setvpolattn = False
hpola1 = 0
hpola2 = 0
vpola1 = 0
vpola2 = 0

setnd = False
ndval = 0

latest_hpol_det = 0
latest_vpol_det = 0
latest_temp = 0

ADC_SAMPLES = 16
FW_REV = 2

# ==============================================================================
# 2. HARDWARE CONTROL, ADC AVERAGING & 1-PPS INTERRUPT
# ==============================================================================

def get_attn_bits(num):
    bits = ~num & 31
    return [(bits >> i) & 1 for i in range(5)]

def set_attenuators(pol, a1, a2):
    # pol = 0 for H Pol (latches 0, 1); pol = 1 for V Pol (latches 2, 3)
    idx = pol * 2
    for i, b in enumerate(get_attn_bits(a1)):
        attnpins[i].value(b)
    lepins[idx].value(1)
    lepins[idx].value(0)

    idx = pol * 2 + 1
    for i, b in enumerate(get_attn_bits(a2)):
        attnpins[i].value(b)
    lepins[idx].value(1)
    lepins[idx].value(0)

def set_noisediode(nd):
    ndpin.value(nd)

def read_voltage(pol):
    adc = detadc[pol]
    total = 0
    for _ in range(ADC_SAMPLES):
        total += adc.read_u16()
    return (total // ADC_SAMPLES) >> 5

def read_temperature():
    total = 0
    for _ in range(ADC_SAMPLES):
        total += tempadc.read_u16()
    return (total // ADC_SAMPLES) >> 5

def on_rising_edge(pin):
    global sethpolattn, setvpolattn
    global hpola1, hpola2, vpola1, vpola2
    global setnd, ndval
    global latest_hpol_det, latest_vpol_det, latest_temp

    if setnd:
        set_noisediode(ndval)
        setnd = False

    if sethpolattn:
        set_attenuators(0, hpola1, hpola2)
        sethpolattn = False

    if setvpolattn:
        set_attenuators(1, vpola1, vpola2)
        setvpolattn = False

    latest_hpol_det = read_voltage(0)
    latest_vpol_det = read_voltage(1)
    latest_temp = read_temperature()

latest_hpol_det = read_voltage(0)
latest_vpol_det = read_voltage(1)
latest_temp = read_temperature()

ppspin.irq(trigger=Pin.IRQ_RISING, handler=on_rising_edge)

# ==============================================================================
# 3. ETHERNET, HARDWARE WATCHDOG & MODBUS TCP SERVER
# ==============================================================================

nic = network.WIZNET6K()
nic.active(True)
nic.ifconfig('dhcp')

while not nic.isconnected():
    time.sleep(0.5)

local_ip, subnet, gateway, dns = nic.ifconfig()
while local_ip == '0.0.0.0':
    time.sleep(0.5)
    local_ip, subnet, gateway, dns = nic.ifconfig()

print("fe2.solar.pvt listening on:", local_ip, "Port 502")

wdt = WDT(timeout=8000)

def build_response(trans_id, proto_id, unit_id, pdu):
    length = 1 + len(pdu)
    mbap = struct.pack('>HHHB', trans_id, proto_id, length, unit_id)
    return mbap + pdu

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((local_ip, 502))
server.listen(2)
server.settimeout(1.0)

while True:
    wdt.feed()
    try:
        conn, addr = server.accept()
    except OSError:
        continue

    print("\nMaster connected from:", addr)
    conn.settimeout(2.0)

    try:
        while True:
            wdt.feed()
            try:
                rx = conn.recv(260)
            except OSError:
                if not nic.isconnected():
                    break
                continue

            if not rx:
                break

            if len(rx) < 12:
                continue

            trans_id, proto_id, mbap_len, unit_id, func_code, reg_addr, val_hi, val_lo = struct.unpack('>HHHBBHBB', rx[:12])
            reg_val = (val_hi << 8) | val_lo
            pdu_resp = None

            if func_code == 0x01:
                if reg_addr == 0x0002:
                    coil_byte = 0x01 if ndval else 0x00
                    pdu_resp = struct.pack('>BBB', 0x01, 0x01, coil_byte)
                else:
                    pdu_resp = struct.pack('>BBB', 0x01, 0x01, 0x00)

            elif func_code == 0x03:
                qty = reg_val
                if reg_addr == 0x01D4 and qty == 2:
                    pdu_resp = struct.pack('>BBBBBB', 0x03, 0x04, vpola1, vpola2, hpola1, hpola2)
                else:
                    byte_count = qty * 2
                    pdu_resp = struct.pack('>BB', 0x03, byte_count) + (b'\x00' * byte_count)

            elif func_code == 0x06:
                if reg_addr == 0x0000:
                    vpola1 = val_hi & 0x1F
                    vpola2 = val_lo & 0x1F
                    setvpolattn = True
                elif reg_addr == 0x0001:
                    hpola1 = val_hi & 0x1F
                    hpola2 = val_lo & 0x1F
                    sethpolattn = True
                pdu_resp = struct.pack('>BHBB', func_code, reg_addr, val_hi, val_lo)

            elif func_code == 0x05:
                if reg_addr == 0x0002:
                    ndval = 1 if (val_hi == 0xFF) else 0
                    setnd = True
                pdu_resp = struct.pack('>BHBB', func_code, reg_addr, val_hi, val_lo)

            elif func_code == 0x04:
                val = 0
                if reg_addr == 0x0000:
                    val = latest_vpol_det & 0x07FF
                elif reg_addr == 0x0001:
                    val = latest_hpol_det & 0x07FF
                elif reg_addr == 0x0002:
                    val = latest_temp & 0x07FF
                elif reg_addr == 0xFFFF:
                    val = FW_REV & 0xFFFF
                pdu_resp = struct.pack('>BBH', 0x04, 0x02, val)

            if pdu_resp:
                tx = build_response(trans_id, proto_id, unit_id, pdu_resp)
                conn.send(tx)

    except Exception as e:
        print("Connection closed:", e)
    finally:
        conn.close()
```

---

## 5. Deployment Notes

* **Flashing:** Use the WIZnet-ioNIC MicroPython `.uf2` for EVB-Pico2 (`network.WIZNET6K`).
* **Auto-Start:** Save the Python code above as `main.py` on the Pico 2 root filesystem.
* **Fleet Rollout (`fe1`–`fe16`):** Because `nic.ifconfig('dhcp')` relies on your network's MAC-to-IP reservations (`192.168.24.61`–`.76`), the identical `main.py` file can be deployed across all front-end controllers without modification.
