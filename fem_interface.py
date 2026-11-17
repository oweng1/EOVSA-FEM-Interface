import secrets
from machine import Pin, ADC, WDT
import network
import socket
import struct
import time

# ==============================================================================
# 1. HARDWARE PIN & ADC SETUP (from fem_interface)
# ==============================================================================

# Attenuator data bits: GPIO 0 to 4
attnpins = [Pin(i, Pin.OUT, value=1) for i in range(0, 5)]

# Latch enable pins: GPIO 5 to 8 (0,1 = H Pol; 2,3 = V Pol)
lepins = [Pin(i, Pin.OUT, value=0) for i in range(5, 9)]

# Noise diode control pin: GPIO 9
ndpin = Pin(9, Pin.OUT, value=0)

# Voltage detector ADC pins: ADC 26 (H Pol) and ADC 27 (V Pol)
detadc = [ADC(i) for i in range(26, 28)]

# Temperature sensor ADC pin: ADC 28
tempadc = ADC(28)

# 1 Pulse Per Second (1 PPS) trigger pin: GPIO 10
ppspin = Pin(10, Pin.IN)

# Hardware state & 1-PPS command flags
sethpolattn = False     # Flag to update H Pol attenuators on next 1 PPS
setvpolattn = False     # Flag to update V Pol attenuators on next 1 PPS
hpola1 = 0              # Pending/current H Pol attenuator 1 (0-31 dB)
hpola2 = 0              # Pending/current H Pol attenuator 2 (0-31 dB)
vpola1 = 0              # Pending/current V Pol attenuator 1 (0-31 dB)
vpola2 = 0              # Pending/current V Pol attenuator 2 (0-31 dB)

setnd = False           # Flag to update noise diode on next 1 PPS
ndval = 0               # Pending/current noise diode state (0=OFF, 1=ON)

# Latest 1-PPS synchronized 11-bit ADC readings (0-2047)
latest_hpol_det = 0
latest_vpol_det = 0
latest_temp = 0

ADC_SAMPLES = 16        # Number of samples averaged per 1 PPS tick
FW_REV = 2              # Firmware revision returned on FC 0x04 Reg 0xFFFF


# ==============================================================================
# 2. HARDWARE CONTROL, ADC AVERAGING & 1-PPS INTERRUPT
# ==============================================================================

def get_attn_bits(num):
    # Converts attenuation value (0-31) to list of 5 inverted bits
    bits = ~num & 31
    return [(bits >> i) & 1 for i in range(5)]

def set_attenuators(pol, a1, a2):
    # pol = 0 for H Pol (latches 0, 1); pol = 1 for V Pol (latches 2, 3)
    idx = pol * 2
    bits1 = get_attn_bits(a1)
    for i, b in enumerate(bits1):
        attnpins[i].value(b)
    lepins[idx].value(1)
    lepins[idx].value(0)

    idx = pol * 2 + 1
    bits2 = get_attn_bits(a2)
    for i, b in enumerate(bits2):
        attnpins[i].value(b)
    lepins[idx].value(1)
    lepins[idx].value(0)

def set_noisediode(nd):
    # nd = 0 for OFF, 1 for ON
    ndpin.value(nd)

def read_voltage(pol):
    # Average ADC_SAMPLES reads on pol (0=H Pol ADC26, 1=V Pol ADC27) -> 11-bit (0-2047)
    adc = detadc[pol]
    total = 0
    for _ in range(ADC_SAMPLES):
        total += adc.read_u16()
    return (total // ADC_SAMPLES) >> 5

def read_temperature():
    # Average ADC_SAMPLES reads on ADC28 -> 11-bit (0-2047)
    total = 0
    for _ in range(ADC_SAMPLES):
        total += tempadc.read_u16()
    return (total // ADC_SAMPLES) >> 5

def on_rising_edge(pin):
    # Executes synchronously on every GPIO 10 (1 PPS) rising edge
    global sethpolattn, setvpolattn
    global hpola1, hpola2, vpola1, vpola2
    global setnd, ndval
    global latest_hpol_det, latest_vpol_det, latest_temp

    # 1. Apply any pending hardware outputs
    if setnd:
        set_noisediode(ndval)
        setnd = False

    if sethpolattn:
        set_attenuators(0, hpola1, hpola2)
        sethpolattn = False

    if setvpolattn:
        set_attenuators(1, vpola1, vpola2)
        setvpolattn = False

    # 2. Sample and average all 3 ADCs on the 1 PPS edge
    latest_hpol_det = read_voltage(0)
    latest_vpol_det = read_voltage(1)
    latest_temp = read_temperature()

# Take initial averaged ADC readings at boot before first 1 PPS arrives
latest_hpol_det = read_voltage(0)
latest_vpol_det = read_voltage(1)
latest_temp = read_temperature()

# Attach rising-edge interrupt to GPIO 10 (1 PPS)
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

#print("fe2.solar.pvt listening on:", local_ip, secrets.port)

# Enable RP2350 Hardware Watchdog (8-second timeout; auto-reboots on hang)
wdt = WDT(timeout=8000)

def build_response(trans_id, proto_id, unit_id, pdu):
    length = 1 + len(pdu)
    mbap = struct.pack('>HHHB', trans_id, proto_id, length, unit_id)
    return mbap + pdu

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((local_ip, 502))
server.listen(2)
server.settimeout(1.0)  # 1s timeout so main loop feeds WDT while waiting for master

while True:
    wdt.feed()
    try:
        conn, addr = server.accept()
    except OSError:
        # Accept timed out after 1s (no master connected yet); loop and feed WDT
        continue

    #print("\nMaster connected from:", addr)
    conn.settimeout(2.0)  # 2s timeout on recv() so dropped links don't stall WDT

    try:
        while True:
            wdt.feed()
            try:
                rx = conn.recv(260)
            except OSError:
                # No packet in last 2s; verify link is still up and keep feeding WDT
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

            # --- FC 0x01: Read Coil Status (Noise Source Status Check) ---
            if func_code == 0x01:
                if reg_addr == 0x0002:
                    coil_byte = 0x01 if ndval else 0x00
                    pdu_resp = struct.pack('>BBB', 0x01, 0x01, coil_byte)
                else:
                    pdu_resp = struct.pack('>BBB', 0x01, 0x01, 0x00)

            # --- FC 0x03: Read Holding Registers (0x01D4 -> Attenuator Readback) ---
            elif func_code == 0x03:
                qty = reg_val
                if reg_addr == 0x01D4 and qty == 2:
                    pdu_resp = struct.pack('>BBBBBB', 0x03, 0x04, vpola1, vpola2, hpola1, hpola2)
                else:
                    byte_count = qty * 2
                    pdu_resp = struct.pack('>BB', 0x03, byte_count) + (b'\x00' * byte_count)

            # --- FC 0x06: Digital Attenuator Settings (Queue for next 1 PPS) ---
            elif func_code == 0x06:
                if reg_addr == 0x0000:
                    vpola1 = val_hi & 0x1F
                    vpola2 = val_lo & 0x1F
                    setvpolattn = True
                    #print(f"[CMD -> 1PPS] V POL Attenuators queued: Att#1={vpola1} dB, Att#2={vpola2} dB")
                elif reg_addr == 0x0001:
                    hpola1 = val_hi & 0x1F
                    hpola2 = val_lo & 0x1F
                    sethpolattn = True
                    #print(f"[CMD -> 1PPS] H POL Attenuators queued: Att#1={hpola1} dB, Att#2={hpola2} dB")
                pdu_resp = struct.pack('>BHBB', func_code, reg_addr, val_hi, val_lo)

            # --- FC 0x05: Noise Source DC Power Write (Queue for next 1 PPS) ---
            elif func_code == 0x05:
                if reg_addr == 0x0002:
                    ndval = 1 if (val_hi == 0xFF) else 0
                    setnd = True
                    #print(f"[CMD -> 1PPS] Noise Diode queued: {'ON (1)' if ndval else 'OFF (0)'}")
                pdu_resp = struct.pack('>BHBB', func_code, reg_addr, val_hi, val_lo)

            # --- FC 0x04: Monitor Readouts (Latest 1-PPS Averaged ADCs & FW Rev) ---
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
        #print("Connection closed:", e)
    finally:
        conn.close()
