#!/usr/bin/env python3
"""
Diagnostic dump for the USB-C/temperature ADC bug: reads the raw pre-scaling
ADC codes (factory-test selectors 0x0A-0x0C) and the factory temperature
sensor calibration constants (selectors 0x0D-0x0E), then recomputes what the
firmware's own formulas *should* produce, so the raw numbers can be compared
against what register reads (0x01, 0x07, 0x0B) actually report.

Requires firmware built with UPS_ADC_FACTORY_DIAG_ENABLED=1 (see main.c) --
these registers are compiled out by default to save flash on this part.

Usage: python3 tools/upsplus_adc_diag.py
"""

import sys
import time
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

ADDR = 0x17
FACTORY_TEST_START = 0xFC

# ADC_RESOLUTION_12B full-scale code count.
ADC_FULL_SCALE = 4095
# Factory temp sensor calibration reference voltage (mV); fixed by silicon, not by board.
TEMPSENSOR_CAL_VREFANALOG = 3300
TEMPSENSOR_CAL1_TEMP = 30
TEMPSENSOR_CAL2_TEMP = 110


class I2CDevice:
    def __init__(self, bus_id: int, addr: int) -> None:
        self.bus = SMBus(bus_id)
        self.addr = addr

    def close(self) -> None:
        self.bus.close()

    def read_u8(self, reg: int) -> int:
        return self.bus.read_byte_data(self.addr, reg) & 0xFF

    def write_u8(self, reg: int, value: int) -> None:
        self.bus.write_byte_data(self.addr, reg, value & 0xFF)

    def read_block(self, reg: int, length: int) -> List[int]:
        return [b & 0xFF for b in self.bus.read_i2c_block_data(self.addr, reg, length)]

    def read_u16(self, reg_l: int) -> int:
        data = self.read_block(reg_l, 2)
        return (data[1] << 8) | data[0]


def read_factory_page(dev: I2CDevice, selector: int, timeout_s: float = 0.5) -> List[int]:
    """Select a factory-test page and read it back, polling for the echo
    since the firmware only updates the page on its own main-loop cadence."""
    dev.write_u8(FACTORY_TEST_START, selector)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        page = dev.read_block(FACTORY_TEST_START, 4)
        if page[0] == selector:
            return page
        time.sleep(0.01)
    raise TimeoutError(f"Selector 0x{selector:02X} did not echo back within {timeout_s}s")


def u16_from_page(page: List[int]) -> int:
    return page[1] | (page[2] << 8)


def calc_temperature(vdda_mv: int, raw_adc: int, cal1: int, cal2: int) -> float:
    normalized = raw_adc * vdda_mv / TEMPSENSOR_CAL_VREFANALOG
    return ((normalized - cal1) * (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) / (cal2 - cal1)) + TEMPSENSOR_CAL1_TEMP


def calc_voltage(vref_mv: int, raw_adc: int) -> float:
    return raw_adc * vref_mv / ADC_FULL_SCALE


def main() -> int:
    dev = I2CDevice(1, ADDR)
    try:
        mcu_voltage_mv = dev.read_u16(0x01)
        usbc_voltage_mv = dev.read_u16(0x07)
        temperature_c = dev.read_u16(0x0B)

        raw_battery = u16_from_page(read_factory_page(dev, 0x0A))
        raw_usbc = u16_from_page(read_factory_page(dev, 0x0B))
        raw_temp = u16_from_page(read_factory_page(dev, 0x0C))
        ts_cal1 = u16_from_page(read_factory_page(dev, 0x0D))
        ts_cal2 = u16_from_page(read_factory_page(dev, 0x0E))

        print("Register reads")
        print(f"  MCU voltage (0x01)        {mcu_voltage_mv} mV")
        print(f"  USB-C voltage (0x07)      {usbc_voltage_mv} mV")
        print(f"  Temperature (0x0B)        {temperature_c} C")
        print()
        print("Raw ADC codes (0-4095, pre-scaling/pre-formula)")
        print(f"  Battery channel (idx 1)   {raw_battery}")
        print(f"  USB-C channel   (idx 2)   {raw_usbc}")
        print(f"  Temp channel    (idx 4)   {raw_temp}")
        print()
        print("Factory temp-sensor calibration (fixed in silicon at 0x1FFFF7B8/C2)")
        print(f"  TS_CAL1 (raw code @ 30C)  {ts_cal1}" + ("  <-- looks unprogrammed!" if ts_cal1 in (0x0000, 0xFFFF) else ""))
        print(f"  TS_CAL2 (raw code @ 110C) {ts_cal2}" + ("  <-- looks unprogrammed!" if ts_cal2 in (0x0000, 0xFFFF) else ""))
        if ts_cal2 <= ts_cal1:
            print("  <-- CAL2 <= CAL1: calibration data is invalid, formula will misbehave")
        print()
        print("Recomputed from raw codes (should match the register reads above)")
        print(f"  USB-C from raw idx2:      {calc_voltage(mcu_voltage_mv * 4, raw_usbc):.0f} mV")
        if ts_cal2 > ts_cal1:
            print(f"  Temp from raw idx4:       {calc_temperature(mcu_voltage_mv, raw_temp, ts_cal1, ts_cal2):.1f} C")
        return 0
    finally:
        dev.close()


if __name__ == "__main__":
    sys.exit(main())
