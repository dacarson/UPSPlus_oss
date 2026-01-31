#!/usr/bin/env python3
"""
Read UPSPlus I2C registers and print human-readable values.

Excludes factory test registers (0xFC-0xFF).
"""

import sys
from typing import List

try:
    from smbus2 import SMBus  # type: ignore
except ImportError:
    try:
        from smbus import SMBus  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "Missing I2C library. Install smbus2 (pip install smbus2) or smbus."
        ) from exc


class I2CDevice:
    def __init__(self, bus_id: int, addr: int) -> None:
        self.bus_id = bus_id
        self.addr = addr
        self.bus = SMBus(bus_id)

    def close(self) -> None:
        self.bus.close()

    def read_u8(self, reg: int) -> int:
        return self.bus.read_byte_data(self.addr, reg) & 0xFF

    def read_block(self, reg: int, length: int) -> List[int]:
        return [b & 0xFF for b in self.bus.read_i2c_block_data(self.addr, reg, length)]

    def read_u16(self, reg_l: int) -> int:
        data = self.read_block(reg_l, 2)
        return (data[1] << 8) | data[0]

    def read_u32(self, reg_l: int) -> int:
        data = self.read_block(reg_l, 4)
        return (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]


def format_bool(value: int) -> str:
    return "yes" if value else "no"


def format_power_status(value: int) -> str:
    power_on = bool(value & 0x01)
    calibration_window = bool(value & 0x02)
    return (
        f"0x{value:02X} (power_on={format_bool(power_on)}, "
        f"calibration_window={format_bool(calibration_window)})"
    )


def print_kv(label: str, value: str) -> None:
    print(f"{label:<28} {value}")


def main() -> int:
    dev = I2CDevice(1, 0x17)
    try:
        print("Measurements")
        print_kv("MCU voltage (mV)", str(dev.read_u16(0x01)))
        print_kv("Pogopin voltage (mV)", str(dev.read_u16(0x03)))
        print_kv("Battery voltage (mV)", str(dev.read_u16(0x05)))
        print_kv("USB-C voltage (mV)", str(dev.read_u16(0x07)))
        print_kv("Micro-USB voltage (mV)", str(dev.read_u16(0x09)))
        print_kv("Temperature (C)", str(dev.read_u16(0x0B)))

        print("\nConfiguration")
        print_kv("Full voltage (mV)", str(dev.read_u16(0x0D)))
        print_kv("Empty voltage (mV)", str(dev.read_u16(0x0F)))
        print_kv("Protect voltage (mV)", str(dev.read_u16(0x11)))
        battery_percent_raw = dev.read_u16(0x13)
        print_kv("Battery percent", f"{battery_percent_raw & 0xFF} (raw 0x{battery_percent_raw:04X})")
        print_kv("Sample period (min)", str(dev.read_u16(0x15)))
        print_kv("Power status", format_power_status(dev.read_u8(0x17)))
        print_kv("Shutdown countdown (s)", str(dev.read_u8(0x18)))
        print_kv("Auto power on", format_bool(dev.read_u8(0x19)))
        print_kv("Restart countdown (s)", str(dev.read_u8(0x1A)))
        print_kv("Factory reset flag", f"0x{dev.read_u8(0x1B):02X}")

        print("\nRuntime counters")
        print_kv("Runtime all (s)", str(dev.read_u32(0x1C)))
        print_kv("Runtime charging (s)", str(dev.read_u32(0x20)))
        print_kv("Runtime current (s)", str(dev.read_u32(0x24)))

        print("\nVersioning")
        print_kv("Firmware version", str(dev.read_u16(0x28)))
        print_kv("Battery self programmed", format_bool(dev.read_u8(0x2A)))
        print_kv("Low battery percent", str(dev.read_u8(0x2B)))
        print_kv("Load on delay (s)", str(dev.read_u16(0x2C)))

        serial = dev.read_block(0xF0, 12)
        serial_hex = " ".join(f"{b:02X}" for b in serial)
        serial_ascii = "".join(chr(b) if 32 <= b <= 126 else "." for b in serial)
        print("\nSerial")
        print_kv("Serial (hex)", serial_hex)
        print_kv("Serial (ascii)", serial_ascii)

        return 0
    finally:
        dev.close()


if __name__ == "__main__":
    sys.exit(main())
