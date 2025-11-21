#!/usr/bin/env python3
"""
TF-Luna LiDAR I2C reader (register-polling, robust)
- Works with TF-Luna variants that expose distance/strength/temp as registers
- Also auto-detects if the device returns the UART-like 0x59 0x59 header
"""

import time
from smbus2 import SMBus

BUS    = 7         # you found the device here
ADDR   = 0x10      # common default; change if i2cdetect shows another
NBYTES = 9         # 9-byte frame/register block

def parse_as_register_block(buf):
    """Parse 9 bytes as DS (dist, strength, temp) block with checksum."""
    if len(buf) != 9:
        return None
    checksum_ok = ((sum(buf[:8]) & 0xFF) == buf[8])
    if not checksum_ok:
        return None
    dist_cm   = buf[0] | (buf[1] << 8)
    strength  = buf[2] | (buf[3] << 8)
    temp_raw  = buf[4] | (buf[5] << 8)
    # Per Benewake format: T(°C) = (temp_raw / 8) - 256
    temp_c    = (temp_raw / 8.0) - 256.0
    return dist_cm, strength, temp_c

def parse_as_stream_frame(buf):
    """Parse 9 bytes expecting header 0x59 0x59 like UART frame."""
    if len(buf) != 9:
        return None
    if buf[0] != 0x59 or buf[1] != 0x59:
        return None
    checksum_ok = ((sum(buf[:8]) & 0xFF) == buf[8])
    if not checksum_ok:
        return None
    dist_cm   = buf[2] | (buf[3] << 8)
    strength  = buf[4] | (buf[5] << 8)
    temp_raw  = buf[6] | (buf[7] << 8)
    temp_c    = (temp_raw / 8.0) - 256.0
    return dist_cm, strength, temp_c

def main():
    print(f"[I2C] Opening /dev/i2c-{BUS}, addr 0x{ADDR:02X} (register-polling)")
    with SMBus(BUS) as bus:
        # Many modules require setting the internal register pointer to 0x00
        # SMBus helper read_i2c_block_data() does a write of the command (0x00) then a read.
        mode = None  # "reg" or "stream", detected on first good read
        count = 0
        while True:
            try:
                block = bus.read_i2c_block_data(ADDR, 0x00, NBYTES)
                # Try register mode first
                parsed = parse_as_register_block(block)
                if parsed:
                    if mode is None:
                        mode = "reg"
                        print("[I2C] Detected register-polling mode.")
                    d, s, t = parsed
                    count += 1
                    print(f"[{count:05d}] Distance={d:4d} cm | Strength={s:5d} | Temp={t:5.1f} °C")
                    time.sleep(0.03)
                    continue
                # If that failed, maybe it’s returning UART-like frames even on I2C
                parsed = parse_as_stream_frame(block)
                if parsed:
                    if mode is None:
                        mode = "stream"
                        print("[I2C] Detected UART-like frame mode over I2C.")
                    d, s, t = parsed
                    count += 1
                    print(f"[{count:05d}] Distance={d:4d} cm | Strength={s:5d} | Temp={t:5.1f} °C")
                    time.sleep(0.03)
                    continue

                # If neither parser worked, just show raw once then keep trying
                if mode is None:
                    print("[I2C] Received unexpected data; retrying… Raw:", block)
                time.sleep(0.02)

            except KeyboardInterrupt:
                print("\nStopping.")
                break
            except Exception as e:
                # Typical transient error: Remote I/O error when device NACKs
                print("[I2C] Read error:", e)
                

if __name__ == "__main__":
    main()
