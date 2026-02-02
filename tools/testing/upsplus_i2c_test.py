#!/usr/bin/env python3
"""
UPSPlus I2C test script.

Runs non-interactive I2C register tests against the UPSPlus firmware at 0x17.
Defaults are conservative; use flags to enable higher-risk tests.
"""

import argparse
import sys
import time

try:
    from smbus2 import SMBus  # type: ignore
except ImportError:
    try:
        from smbus import SMBus  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "Missing I2C library. Install smbus2 (pip install smbus2) or smbus."
        ) from exc


DEFAULT_BUS = 1
DEFAULT_ADDR = 0x17

REG = {
    "MCU_VOLTAGE_L": 0x01,
    "MCU_VOLTAGE_H": 0x02,
    "POGOPIN_VOLTAGE_L": 0x03,
    "POGOPIN_VOLTAGE_H": 0x04,
    "BATTERY_VOLTAGE_L": 0x05,
    "BATTERY_VOLTAGE_H": 0x06,
    "USBC_VOLTAGE_L": 0x07,
    "USBC_VOLTAGE_H": 0x08,
    "MICROUSB_VOLTAGE_L": 0x09,
    "MICROUSB_VOLTAGE_H": 0x0A,
    "TEMPERATURE_L": 0x0B,
    "TEMPERATURE_H": 0x0C,
    "FULL_VOLTAGE_L": 0x0D,
    "FULL_VOLTAGE_H": 0x0E,
    "EMPTY_VOLTAGE_L": 0x0F,
    "EMPTY_VOLTAGE_H": 0x10,
    "PROTECT_VOLTAGE_L": 0x11,
    "PROTECT_VOLTAGE_H": 0x12,
    "BATTERY_PERCENT_L": 0x13,
    "BATTERY_PERCENT_H": 0x14,
    "SAMPLE_PERIOD_L": 0x15,
    "SAMPLE_PERIOD_H": 0x16,
    "POWER_STATUS": 0x17,
    "SHUTDOWN_COUNTDOWN": 0x18,
    "AUTO_POWER_ON": 0x19,
    "RESTART_COUNTDOWN": 0x1A,
    "FACTORY_RESET": 0x1B,
    "RUNTIME_ALL_L": 0x1C,
    "RUNTIME_CHARGING_L": 0x20,
    "RUNTIME_CURRENT_L": 0x24,
    "VERSION_L": 0x28,
    "VERSION_H": 0x29,
    "BATTERY_SELF_PROG": 0x2A,
    "LOW_BATTERY_PERCENT": 0x2B,
    "LOAD_ON_DELAY_L": 0x2C,
    "LOAD_ON_DELAY_H": 0x2D,
    "SERIAL_START": 0xF0,
    "SERIAL_END": 0xFB,
    "FACTORY_TEST_START": 0xFC,
    "FACTORY_TEST_END": 0xFF,
}

RESERVED_SAMPLES = [0x2E, 0x50, 0xEF]

VBAT_FULL_MAX_MV = 4500
VBAT_EMPTY_MAX_MV = 4500
VBAT_PROTECT_MIN_MV = 2800
VBAT_PROTECT_MAX_MV = 4500
VBAT_MIN_DELTA_MV = 50


def u16_from_le(lsb: int, msb: int) -> int:
    return (msb << 8) | lsb


class I2CDevice:
    def __init__(self, bus_id: int, addr: int) -> None:
        self.bus_id = bus_id
        self.addr = addr
        self.bus = SMBus(bus_id)

    def close(self) -> None:
        self.bus.close()

    def read_u8(self, reg: int) -> int:
        return self.bus.read_byte_data(self.addr, reg) & 0xFF

    def write_u8(self, reg: int, value: int) -> None:
        self.bus.write_byte_data(self.addr, reg, value & 0xFF)

    def read_block(self, reg: int, length: int) -> list[int]:
        return [b & 0xFF for b in self.bus.read_i2c_block_data(self.addr, reg, length)]

    def write_block(self, reg: int, data: list[int]) -> None:
        if len(data) == 1:
            self.write_u8(reg, data[0])
        else:
            self.bus.write_i2c_block_data(self.addr, reg, [b & 0xFF for b in data])

    def read_u16(self, reg_l: int) -> int:
        data = self.read_block(reg_l, 2)
        return u16_from_le(data[0], data[1])

    def write_u16(self, reg_l: int, value: int) -> None:
        self.write_block(reg_l, [value & 0xFF, (value >> 8) & 0xFF])

    def read_u32(self, reg_l: int) -> int:
        data = self.read_block(reg_l, 4)
        return (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]


class TestRunner:
    def __init__(self) -> None:
        self.results: list[tuple[str, bool, str]] = []

    def record(self, name: str, ok: bool, detail: str = "") -> None:
        self.results.append((name, ok, detail))

    def summary(self) -> int:
        failed = [r for r in self.results if not r[1]]
        for name, ok, detail in self.results:
            status = "PASS" if ok else "FAIL"
            if detail:
                print(f"{status} - {name}: {detail}")
            else:
                print(f"{status} - {name}")
        print(f"\nTotal: {len(self.results)}  Passed: {len(self.results) - len(failed)}  Failed: {len(failed)}")
        return 1 if failed else 0


def choose_alternate_u16(value: int, min_v: int, max_v: int) -> int:
    if value < max_v:
        return value + 1
    if value > min_v:
        return value - 1
    return value


def choose_alternate_u8(value: int, min_v: int, max_v: int) -> int:
    if value < max_v:
        return value + 1
    if value > min_v:
        return value - 1
    return value


def wait_for_reg_value(dev: I2CDevice, reg: int, value: int, timeout_s: float = 1.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if dev.read_u8(reg) == value:
            return True
        time.sleep(0.02)
    return False


def wait_for_u16_value(dev: I2CDevice, reg_l: int, value: int, timeout_s: float = 1.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if dev.read_u16(reg_l) == value:
            return True
        time.sleep(0.02)
    return False


def test_reserved_reads_zero(t: TestRunner, dev: I2CDevice) -> None:
    for reg in RESERVED_SAMPLES:
        val = dev.read_u8(reg)
        t.record(f"Reserved read 0x{reg:02X} == 0x00", val == 0, f"read 0x{val:02X}")


def test_factory_test_pages(t: TestRunner, dev: I2CDevice) -> None:
    dev.write_u8(REG["FACTORY_TEST_START"], 0)
    if not wait_for_reg_value(dev, REG["FACTORY_TEST_START"], 0):
        t.record("Factory test selector timeout", False, "waited for 0xFC=0x00")
        return
    data = dev.read_block(REG["FACTORY_TEST_START"], 4)
    t.record("Factory test disabled reads 0x00", data == [0, 0, 0, 0], f"read {data}")
    for selector in (1, 2, 3, 4):
        dev.write_u8(REG["FACTORY_TEST_START"], selector)
        if not wait_for_reg_value(dev, REG["FACTORY_TEST_START"], selector):
            t.record(
                f"Factory test selector {selector} timeout",
                False,
                f"waited for 0xFC={selector}",
            )
            return
        data = dev.read_block(REG["FACTORY_TEST_START"], 4)
        t.record(
            f"Factory test selector {selector} echoed",
            data[0] == selector,
            f"read {data}",
        )
        if selector == 1:
            power_state_ok = 0 <= data[1] <= 3
            charger_state_ok = 0 <= data[2] <= 2
            learning_mode_ok = 0 <= data[3] <= 1
            t.record(
                "Factory test page 0x01 ranges",
                power_state_ok and charger_state_ok and learning_mode_ok,
                f"read {data}",
            )
        elif selector == 2:
            button_state_ok = 0 <= data[1] <= 4
            click_ok = 0 <= data[2] <= 2
            t.record(
                "Factory test page 0x02 ranges",
                button_state_ok and click_ok,
                f"read {data}",
            )
    dev.write_u8(REG["FACTORY_TEST_START"], 5)
    if not wait_for_reg_value(dev, REG["FACTORY_TEST_START"], 5):
        t.record("Factory test selector 5 timeout", False, "waited for 0xFC=0x05")
        return
    data = dev.read_block(REG["FACTORY_TEST_START"], 4)
    t.record("Factory test selector 5 echoed", data[0] == 5, f"read {data}")
    flash_status = data[1]
    auto_info = data[2]
    seq = data[3]
    reserved_ok = (flash_status & 0xF8) == 0 and (auto_info & 0xFC) == 0
    detail = (
        f"flash_status=0x{flash_status:02X} auto_info=0x{auto_info:02X} seq={seq} "
        f"(record_valid={flash_status & 0x01} save_attempted={(flash_status >> 1) & 0x01} "
        f"save_success={(flash_status >> 2) & 0x01} "
        f"auto_loaded={auto_info & 0x01} auto_effective={(auto_info >> 1) & 0x01})"
    )
    t.record("Factory test page 0x05 bitfields", reserved_ok, detail)
    dev.write_u8(REG["FACTORY_TEST_START"], 0)


def test_factory_unknown_selector(t: TestRunner, dev: I2CDevice) -> None:
    dev.write_u8(REG["FACTORY_TEST_START"], 0x99)
    if not wait_for_reg_value(dev, REG["FACTORY_TEST_START"], 0x99):
        t.record("Factory test unknown selector timeout", False, "waited for 0xFC=0x99")
        return
    data = dev.read_block(REG["FACTORY_TEST_START"], 4)
    ok = data[0] == 0x99 and data[1:4] == [0, 0, 0]
    t.record("Factory test unknown selector returns zeros", ok, f"read {data}")
    dev.write_u8(REG["FACTORY_TEST_START"], 0)


def test_factory_writes_ignored(t: TestRunner, dev: I2CDevice) -> None:
    dev.write_u8(REG["FACTORY_TEST_START"], 0x99)
    if not wait_for_reg_value(dev, REG["FACTORY_TEST_START"], 0x99):
        t.record("Factory test ignore-writes timeout", False, "waited for 0xFC=0x99")
        return
    baseline = dev.read_block(REG["FACTORY_TEST_START"], 4)
    dev.write_u8(0xFD, baseline[1] ^ 0xFF)
    dev.write_u8(0xFE, baseline[2] ^ 0xFF)
    dev.write_u8(0xFF, baseline[3] ^ 0xFF)
    time.sleep(0.1)
    after = dev.read_block(REG["FACTORY_TEST_START"], 4)
    ok = after[0] == 0x99 and after[1:4] == [0, 0, 0]
    t.record("Factory test writes to 0xFD-0xFF ignored", ok, f"read {after}")
    dev.write_u8(REG["FACTORY_TEST_START"], 0)


def test_basic_reads(t: TestRunner, dev: I2CDevice) -> None:
    block = dev.read_block(REG["MCU_VOLTAGE_L"], 12)
    t.record("Basic voltage/temperature block read", len(block) == 12, f"len {len(block)}")
    bp = dev.read_block(REG["BATTERY_PERCENT_L"], 2)
    t.record("Battery percent MSB is 0x00", bp[1] == 0, f"read {bp}")
    ver = dev.read_block(REG["VERSION_L"], 2)
    t.record("Firmware version readable", len(ver) == 2, f"read {ver}")
    serial = dev.read_block(REG["SERIAL_START"], 12)
    t.record("Serial number block readable", len(serial) == 12, f"read {serial}")


def test_ro_write_ignored(t: TestRunner, dev: I2CDevice) -> None:
    before = dev.read_u8(REG["VERSION_L"])
    dev.write_u8(REG["VERSION_L"], before ^ 0xFF)
    time.sleep(0.2)
    after = dev.read_u8(REG["VERSION_L"])
    t.record("RO register ignores write (0x28)", before == after, f"before {before} after {after}")


def test_rw_sample_period(t: TestRunner, dev: I2CDevice) -> None:
    orig = dev.read_u16(REG["SAMPLE_PERIOD_L"])
    alt = choose_alternate_u16(orig, 1, 1440)
    if alt == orig:
        t.record("Sample period write skipped (no alternate)", True)
        return
    dev.write_u16(REG["SAMPLE_PERIOD_L"], alt)
    if not wait_for_u16_value(dev, REG["SAMPLE_PERIOD_L"], alt):
        t.record("Sample period write timeout", False, f"waited for {alt}")
        return
    read_back = dev.read_u16(REG["SAMPLE_PERIOD_L"])
    ok = read_back == alt
    dev.write_u16(REG["SAMPLE_PERIOD_L"], orig)
    if not wait_for_u16_value(dev, REG["SAMPLE_PERIOD_L"], orig):
        t.record("Sample period restore timeout", False, f"waited for {orig}")
        return
    restored = dev.read_u16(REG["SAMPLE_PERIOD_L"]) == orig
    t.record("Sample period write/read", ok, f"wrote {alt} read {read_back}")
    t.record("Sample period restore", restored, f"restored {orig}")


def test_rw_low_battery_percent(t: TestRunner, dev: I2CDevice) -> None:
    orig = dev.read_u8(REG["LOW_BATTERY_PERCENT"])
    alt = choose_alternate_u8(orig, 0, 100)
    if alt == orig:
        t.record("Low battery percent write skipped (no alternate)", True)
        return
    dev.write_u8(REG["LOW_BATTERY_PERCENT"], alt)
    if not wait_for_reg_value(dev, REG["LOW_BATTERY_PERCENT"], alt):
        t.record("Low battery percent write timeout", False, f"waited for {alt}")
        return
    read_back = dev.read_u8(REG["LOW_BATTERY_PERCENT"])
    ok = read_back == alt
    dev.write_u8(REG["LOW_BATTERY_PERCENT"], orig)
    if not wait_for_reg_value(dev, REG["LOW_BATTERY_PERCENT"], orig):
        t.record("Low battery percent restore timeout", False, f"waited for {orig}")
        return
    restored = dev.read_u8(REG["LOW_BATTERY_PERCENT"]) == orig
    t.record("Low battery percent write/read", ok, f"wrote {alt} read {read_back}")
    t.record("Low battery percent restore", restored, f"restored {orig}")


def test_rw_battery_voltages(t: TestRunner, dev: I2CDevice) -> None:
    orig_full = dev.read_u16(REG["FULL_VOLTAGE_L"])
    orig_empty = dev.read_u16(REG["EMPTY_VOLTAGE_L"])
    orig_protect = dev.read_u16(REG["PROTECT_VOLTAGE_L"])

    full_new = None
    empty_new = None
    protect_new = None

    if orig_full > VBAT_FULL_MAX_MV:
        t.record("Full voltage write skipped (orig out of range)", True, f"orig {orig_full}")
    else:
        if orig_full + 10 <= VBAT_FULL_MAX_MV and (orig_full + 10) - orig_empty >= VBAT_MIN_DELTA_MV:
            full_new = orig_full + 10
        elif orig_full - 10 >= orig_empty + VBAT_MIN_DELTA_MV:
            full_new = orig_full - 10
        else:
            t.record("Full voltage write skipped (no safe alternate)", True)

    full_for_empty = full_new if full_new is not None else orig_full
    if orig_empty > VBAT_EMPTY_MAX_MV:
        t.record("Empty voltage write skipped (orig out of range)", True, f"orig {orig_empty}")
    else:
        if (
            orig_empty + 10 <= VBAT_EMPTY_MAX_MV
            and orig_empty + 10 <= full_for_empty - VBAT_MIN_DELTA_MV
            and orig_empty + 10 >= orig_protect
        ):
            empty_new = orig_empty + 10
        elif (
            orig_empty - 10 >= orig_protect
            and full_for_empty - (orig_empty - 10) >= VBAT_MIN_DELTA_MV
        ):
            empty_new = orig_empty - 10
        else:
            t.record("Empty voltage write skipped (no safe alternate)", True)

    empty_for_protect = empty_new if empty_new is not None else orig_empty
    full_for_protect = full_new if full_new is not None else orig_full
    if orig_protect < VBAT_PROTECT_MIN_MV or orig_protect > VBAT_PROTECT_MAX_MV:
        t.record("Protection voltage write skipped (orig out of range)", True, f"orig {orig_protect}")
    else:
        if (
            orig_protect + 10 <= VBAT_PROTECT_MAX_MV
            and orig_protect + 10 <= empty_for_protect
            and orig_protect + 10 <= full_for_protect
        ):
            protect_new = orig_protect + 10
        elif orig_protect - 10 >= VBAT_PROTECT_MIN_MV:
            protect_new = orig_protect - 10
        else:
            t.record("Protection voltage write skipped (no safe alternate)", True)

    try:
        if full_new is not None and full_new != orig_full:
            dev.write_u16(REG["FULL_VOLTAGE_L"], full_new)
            if not wait_for_u16_value(dev, REG["FULL_VOLTAGE_L"], full_new):
                t.record("Full voltage write timeout", False, f"waited for {full_new}")
                return
            t.record("Full voltage write/read", True, f"wrote {full_new}")

        if empty_new is not None and empty_new != orig_empty:
            dev.write_u16(REG["EMPTY_VOLTAGE_L"], empty_new)
            if not wait_for_u16_value(dev, REG["EMPTY_VOLTAGE_L"], empty_new):
                t.record("Empty voltage write timeout", False, f"waited for {empty_new}")
                return
            t.record("Empty voltage write/read", True, f"wrote {empty_new}")

        if protect_new is not None and protect_new != orig_protect:
            dev.write_u16(REG["PROTECT_VOLTAGE_L"], protect_new)
            if not wait_for_u16_value(dev, REG["PROTECT_VOLTAGE_L"], protect_new):
                t.record("Protection voltage write timeout", False, f"waited for {protect_new}")
                return
            t.record("Protection voltage write/read", True, f"wrote {protect_new}")
    finally:
        if protect_new is not None and protect_new != orig_protect:
            dev.write_u16(REG["PROTECT_VOLTAGE_L"], orig_protect)
            if not wait_for_u16_value(dev, REG["PROTECT_VOLTAGE_L"], orig_protect):
                t.record("Protection voltage restore timeout", False, f"waited for {orig_protect}")
            else:
                t.record("Protection voltage restore", True, f"restored {orig_protect}")
        if empty_new is not None and empty_new != orig_empty:
            dev.write_u16(REG["EMPTY_VOLTAGE_L"], orig_empty)
            if not wait_for_u16_value(dev, REG["EMPTY_VOLTAGE_L"], orig_empty):
                t.record("Empty voltage restore timeout", False, f"waited for {orig_empty}")
            else:
                t.record("Empty voltage restore", True, f"restored {orig_empty}")
        if full_new is not None and full_new != orig_full:
            dev.write_u16(REG["FULL_VOLTAGE_L"], orig_full)
            if not wait_for_u16_value(dev, REG["FULL_VOLTAGE_L"], orig_full):
                t.record("Full voltage restore timeout", False, f"waited for {orig_full}")
            else:
                t.record("Full voltage restore", True, f"restored {orig_full}")


def test_optional_power_controls(t: TestRunner, dev: I2CDevice) -> None:
    orig = dev.read_u8(REG["AUTO_POWER_ON"])
    alt = 0 if orig != 0 else 1
    dev.write_u8(REG["AUTO_POWER_ON"], alt)
    if not wait_for_reg_value(dev, REG["AUTO_POWER_ON"], alt):
        t.record("Auto power on write timeout", False, f"waited for {alt}")
        return
    read_back = dev.read_u8(REG["AUTO_POWER_ON"])
    ok = read_back == alt
    dev.write_u8(REG["AUTO_POWER_ON"], orig)
    if not wait_for_reg_value(dev, REG["AUTO_POWER_ON"], orig):
        t.record("Auto power on restore timeout", False, f"waited for {orig}")
        return
    restored = dev.read_u8(REG["AUTO_POWER_ON"]) == orig
    t.record("Auto power on write/read", ok, f"wrote {alt} read {read_back}")
    t.record("Auto power on restore", restored, f"restored {orig}")


def test_optional_load_on_delay(t: TestRunner, dev: I2CDevice) -> None:
    # Use factory test page 0x01 to skip if currently in LOAD_ON_DELAY (state value 3).
    dev.write_u8(REG["FACTORY_TEST_START"], 1)
    if not wait_for_reg_value(dev, REG["FACTORY_TEST_START"], 1):
        t.record("Load on delay selector timeout", False, "waited for 0xFC=0x01")
        return
    page = dev.read_block(REG["FACTORY_TEST_START"], 4)
    dev.write_u8(REG["FACTORY_TEST_START"], 0)
    power_state = page[1]
    if power_state == 3:
        t.record("Load on delay write skipped (state active)", True)
        return
    orig = dev.read_u16(REG["LOAD_ON_DELAY_L"])
    alt = choose_alternate_u16(orig, 0, 0xFFFF)
    if alt == orig:
        t.record("Load on delay write skipped (no alternate)", True)
        return
    dev.write_u16(REG["LOAD_ON_DELAY_L"], alt)
    if not wait_for_u16_value(dev, REG["LOAD_ON_DELAY_L"], alt):
        t.record("Load on delay write timeout", False, f"waited for {alt}")
        return
    read_back = dev.read_u16(REG["LOAD_ON_DELAY_L"])
    ok = read_back == alt
    dev.write_u16(REG["LOAD_ON_DELAY_L"], orig)
    if not wait_for_u16_value(dev, REG["LOAD_ON_DELAY_L"], orig):
        t.record("Load on delay restore timeout", False, f"waited for {orig}")
        return
    restored = dev.read_u16(REG["LOAD_ON_DELAY_L"]) == orig
    t.record("Load on delay write/read", ok, f"wrote {alt} read {read_back}")
    t.record("Load on delay restore", restored, f"restored {orig}")


def record_timer_accuracy(
    t: TestRunner, label: str, actual: int, expected: int, tolerance: float
) -> None:
    ok = abs(actual - expected) <= tolerance
    detail = f"actual={actual}s expected~{expected}s tol={tolerance}s"
    t.record(label, ok, detail)


def test_timer_accuracy(t: TestRunner, dev: I2CDevice, duration_s: float, tolerance_s: float) -> None:
    power_status = dev.read_u8(REG["POWER_STATUS"])
    power_on = bool(power_status & 0x01)

    start_all = dev.read_u32(REG["RUNTIME_ALL_L"])
    start_charge = dev.read_u32(REG["RUNTIME_CHARGING_L"])
    start_current = dev.read_u32(REG["RUNTIME_CURRENT_L"])

    start = time.monotonic()
    time.sleep(duration_s)
    elapsed = time.monotonic() - start
    expected = int(round(elapsed))

    end_all = dev.read_u32(REG["RUNTIME_ALL_L"])
    end_charge = dev.read_u32(REG["RUNTIME_CHARGING_L"])
    end_current = dev.read_u32(REG["RUNTIME_CURRENT_L"])

    delta_all = end_all - start_all
    delta_charge = end_charge - start_charge
    delta_current = end_current - start_current

    record_timer_accuracy(t, "Runtime all accuracy", delta_all, expected, tolerance_s)

    page3 = read_factory_page(t, dev, 3)
    if page3 is None:
        t.record("Charging runtime accuracy skipped", True, "factory page read failed")
        charger_present = False
    else:
        charger_present = page3[1] == 1
    dev.write_u8(REG["FACTORY_TEST_START"], 0)
    wait_for_reg_value(dev, REG["FACTORY_TEST_START"], 0)

    if charger_present:
        record_timer_accuracy(t, "Charging runtime accuracy", delta_charge, expected, tolerance_s)
    else:
        t.record("Charging runtime accuracy skipped", True, "charger not present")

    if power_on:
        record_timer_accuracy(t, "Current runtime accuracy", delta_current, expected, tolerance_s)
    else:
        t.record("Current runtime accuracy skipped", True, "power_status indicates power off")


def test_countdown_accuracy(
    t: TestRunner, dev: I2CDevice, countdown_duration_s: float, tolerance_s: float
) -> None:
    # Countdown-based tests are safe if canceled before completion.
    start_value = int(round(countdown_duration_s)) + 10
    start_value = max(15, min(start_value, 120))

    dev.write_u8(REG["SHUTDOWN_COUNTDOWN"], start_value)
    t.record("Shutdown countdown started", True, f"wrote {start_value}")
    start = time.monotonic()
    time.sleep(countdown_duration_s)
    elapsed = time.monotonic() - start
    expected = max(start_value - int(round(elapsed)), 0)
    after = dev.read_u8(REG["SHUTDOWN_COUNTDOWN"])
    record_timer_accuracy(t, "Shutdown countdown accuracy", after, expected, tolerance_s)
    dev.write_u8(REG["SHUTDOWN_COUNTDOWN"], 0)
    time.sleep(0.2)
    t.record("Shutdown countdown canceled", dev.read_u8(REG["SHUTDOWN_COUNTDOWN"]) == 0)

    dev.write_u8(REG["RESTART_COUNTDOWN"], start_value)
    t.record("Restart countdown started", True, f"wrote {start_value}")
    start = time.monotonic()
    time.sleep(countdown_duration_s)
    elapsed = time.monotonic() - start
    expected = max(start_value - int(round(elapsed)), 0)
    after = dev.read_u8(REG["RESTART_COUNTDOWN"])
    record_timer_accuracy(t, "Restart countdown accuracy", after, expected, tolerance_s)
    dev.write_u8(REG["RESTART_COUNTDOWN"], 0)
    time.sleep(0.2)
    t.record("Restart countdown canceled", dev.read_u8(REG["RESTART_COUNTDOWN"]) == 0)


def test_destructive(t: TestRunner, dev: I2CDevice) -> None:
    # Factory reset triggers immediate reset of parameters.
    dev.write_u8(REG["FACTORY_RESET"], 1)
    t.record("Factory reset triggered (destructive)", True)


def read_factory_page(t: TestRunner, dev: I2CDevice, selector: int) -> list[int] | None:
    dev.write_u8(REG["FACTORY_TEST_START"], selector)
    if not wait_for_reg_value(dev, REG["FACTORY_TEST_START"], selector):
        t.record("Factory page selector timeout", False, f"waited for 0xFC={selector}")
        return None
    return dev.read_block(REG["FACTORY_TEST_START"], 4)


def test_state_machine_monitor(t: TestRunner, dev: I2CDevice, duration_s: float, interval_s: float) -> None:
    start = time.monotonic()
    samples = 0
    try:
        while time.monotonic() - start < duration_s:
            page1 = read_factory_page(t, dev, 1)
            page3 = read_factory_page(t, dev, 3)
            page4 = read_factory_page(t, dev, 4)
            page2 = read_factory_page(t, dev, 2)
            if page1 is None or page3 is None or page4 is None or page2 is None:
                return

            power_state = page1[1]
            charger_state = page1[2]
            learning_mode = page1[3]
            t.record("State page 0x01 ranges", 0 <= power_state <= 3 and 0 <= charger_state <= 2 and 0 <= learning_mode <= 1)

            button_state = page2[1]
            click_state = page2[2]
            t.record("Button page ranges", 0 <= button_state <= 4 and 0 <= click_state <= 2)

            charger_present = page3[1]
            window_active = page3[2]
            window_due = page3[3]
            t.record("Window page ranges", 0 <= charger_present <= 1 and 0 <= window_active <= 1 and 0 <= window_due <= 1)

            protection_active = page4[1]
            below_count = page4[2]
            pending_power_cut = page4[3]
            t.record("Protection page ranges", 0 <= protection_active <= 1 and 0 <= below_count <= 255 and 0 <= pending_power_cut <= 1)

            # Invariant: window_active == 1 iff charger_state == FORCED_OFF_WINDOW (2)
            t.record(
                "Window/charger consistency",
                (window_active == 1 and charger_state == 2) or (window_active == 0 and charger_state != 2),
                f"charger_state={charger_state} window_active={window_active}",
            )
            # window_due should be cleared on entry to active window
            if window_active == 1:
                t.record("Window due cleared when active", window_due == 0, f"window_due={window_due}")

            samples += 1
            time.sleep(interval_s)
    finally:
        dev.write_u8(REG["FACTORY_TEST_START"], 0)
        wait_for_reg_value(dev, REG["FACTORY_TEST_START"], 0)

    t.record("State monitor samples", samples > 0, f"samples={samples}")


def main() -> int:
    parser = argparse.ArgumentParser(description="UPSPlus I2C integration test.")
    parser.add_argument("--bus", type=int, default=DEFAULT_BUS, help="I2C bus number (default: 1)")
    parser.add_argument("--addr", type=lambda x: int(x, 0), default=DEFAULT_ADDR, help="I2C address (default: 0x17)")
    parser.add_argument(
        "--allow-power-actions",
        action="store_true",
        help="Allow tests that may affect power state (auto power on, load on delay)",
    )
    parser.add_argument(
        "--allow-destructive",
        action="store_true",
        help="Allow destructive tests (factory reset)",
    )
    parser.add_argument(
        "--state-monitor-seconds",
        type=float,
        default=5.0,
        help="Seconds to poll state machine pages (default: 5.0)",
    )
    parser.add_argument(
        "--state-monitor-interval",
        type=float,
        default=0.2,
        help="Polling interval for state pages in seconds (default: 0.2)",
    )
    parser.add_argument(
        "--timer-accuracy-seconds",
        type=float,
        default=6.0,
        help="Seconds to measure runtime counters (default: 6.0)",
    )
    parser.add_argument(
        "--timer-accuracy-tolerance",
        type=float,
        default=1.0,
        help="Allowed timer error in seconds (default: 1.0)",
    )
    parser.add_argument(
        "--countdown-accuracy-seconds",
        type=float,
        default=3.0,
        help="Seconds to measure countdown accuracy (default: 3.0)",
    )
    args = parser.parse_args()

    t = TestRunner()
    dev = I2CDevice(args.bus, args.addr)
    try:
        test_basic_reads(t, dev)
        test_reserved_reads_zero(t, dev)
        test_factory_test_pages(t, dev)
        test_factory_unknown_selector(t, dev)
        test_factory_writes_ignored(t, dev)
        test_ro_write_ignored(t, dev)
        test_rw_sample_period(t, dev)
        test_rw_low_battery_percent(t, dev)
        test_rw_battery_voltages(t, dev)
        test_state_machine_monitor(t, dev, args.state_monitor_seconds, args.state_monitor_interval)
        test_timer_accuracy(t, dev, args.timer_accuracy_seconds, args.timer_accuracy_tolerance)
        test_countdown_accuracy(t, dev, args.countdown_accuracy_seconds, args.timer_accuracy_tolerance)
        if args.allow_power_actions:
            test_optional_power_controls(t, dev)
            test_optional_load_on_delay(t, dev)
        else:
            t.record("Power-state tests skipped", True, "use --allow-power-actions")
        if args.allow_destructive:
            test_destructive(t, dev)
        else:
            t.record("Destructive tests skipped", True, "use --allow-destructive for factory reset")
    finally:
        dev.close()
    return t.summary()


if __name__ == "__main__":
    sys.exit(main())
