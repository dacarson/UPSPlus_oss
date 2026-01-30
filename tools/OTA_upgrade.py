#!/usr/bin/env python3

import os
import time
import smbus2  # pip3 install smbus2

# I2C
DEVICE_BUS = 1
DEVICE_ADDR = 0x18

# Firmware file to upload
FIRMWARE_PATH = "upsplus_oss.bin"   # or "/path/to/upsplus_oss.bin"

def read_uid(bus, addr):
    # Read registers 0xF0..0xFB (240..251)
    regs = [bus.read_byte_data(addr, r) for r in range(240, 252)]

    uid0 = (regs[3]  << 24) | (regs[2]  << 16) | (regs[1]  << 8) | regs[0]
    uid1 = (regs[7]  << 24) | (regs[6]  << 16) | (regs[5]  << 8) | regs[4]
    uid2 = (regs[11] << 24) | (regs[10] << 16) | (regs[9]  << 8) | regs[8]

    return f"{uid0:08X}", f"{uid1:08X}", f"{uid2:08X}"

def upload_firmware(bus, addr, path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"Firmware file not found: {path}")

    print(f"Uploading firmware: {path}")

    with open(path, "rb") as f:
        while True:
            data = f.read(16)

            # EOF: finish
            if not data:
                bus.write_byte_data(addr, 50, 0)
                print("\nThe firmware upgrade is complete.")
                print("Please disconnect all power/batteries and reinstall to use the new firmware.")
                return

            # Write 16-byte chunk into registers 1..16
            for i, b in enumerate(data):
                bus.write_byte_data(addr, i + 1, b)

            # Trigger write/commit for this chunk
            bus.write_byte_data(addr, 50, 250)

            time.sleep(0.1)
            print(".", end="", flush=True)

def main():
    bus = smbus2.SMBus(DEVICE_BUS)
    try:
        uid0, uid1, uid2 = read_uid(bus, DEVICE_ADDR)
        print(f"UID0: {uid0}")
        print(f"UID1: {uid1}")
        print(f"UID2: {uid2}")

        upload_firmware(bus, DEVICE_ADDR, FIRMWARE_PATH)

        # If you really want to halt automatically after success:
        # os.system("sudo halt")
        # while True:
        #     time.sleep(10)

    finally:
        try:
            bus.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()