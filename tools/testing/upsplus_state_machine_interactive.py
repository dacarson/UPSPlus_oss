#!/usr/bin/env python3
"""
Interactive UPSPlus state machine verification.

Guides the user through charger attach/detach steps and validates factory
test pages 0x01 and 0x03 for expected transitions and invariants.
"""

from __future__ import annotations

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

REG_FACTORY_TEST = 0xFC
REG_SAMPLE_PERIOD_L = 0x15
REG_RUNTIME_ALL_L = 0x1C
REG_RUNTIME_CHARGING_L = 0x20
REG_RUNTIME_CURRENT_L = 0x24

CHARGER_STATE_ABSENT = 0
CHARGER_STATE_PRESENT = 1
CHARGER_STATE_FORCED_OFF_WINDOW = 2


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

    def read_u16(self, reg_l: int) -> int:
        data = self.read_block(reg_l, 2)
        return u16_from_le(data[0], data[1])

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


def wait_for_reg_value(dev: I2CDevice, reg: int, value: int, timeout_s: float = 1.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if dev.read_u8(reg) == value:
            return True
        time.sleep(0.02)
    return False


def read_factory_page(dev: I2CDevice, selector: int, timeout_s: float = 1.0) -> list[int]:
    dev.write_u8(REG_FACTORY_TEST, selector)
    if not wait_for_reg_value(dev, REG_FACTORY_TEST, selector, timeout_s=timeout_s):
        raise RuntimeError(f"Timeout waiting for selector 0x{selector:02X}")
    return dev.read_block(REG_FACTORY_TEST, 4)


def read_state(dev: I2CDevice) -> dict[str, int]:
    try:
        page1 = read_factory_page(dev, 1)
        page3 = read_factory_page(dev, 3)
    finally:
        dev.write_u8(REG_FACTORY_TEST, 0)
        wait_for_reg_value(dev, REG_FACTORY_TEST, 0)

    return {
        "power_state": page1[1],
        "charger_state": page1[2],
        "learning_mode": page1[3],
        "charger_present": page3[1],
        "window_active": page3[2],
        "window_due": page3[3],
    }


def format_state(state: dict[str, int]) -> str:
    return (
        "power_state={power_state} charger_state={charger_state} learning_mode={learning_mode} "
        "charger_present={charger_present} window_active={window_active} window_due={window_due}"
    ).format(**state)


def check_window_consistency(state: dict[str, int]) -> tuple[bool, str]:
    window_active = state["window_active"]
    charger_state = state["charger_state"]
    learning_mode = state["learning_mode"]
    window_matches = (window_active == 1 and charger_state == CHARGER_STATE_FORCED_OFF_WINDOW) or (
        window_active == 0 and charger_state != CHARGER_STATE_FORCED_OFF_WINDOW
    )
    learning_matches = learning_mode == window_active
    ok = window_matches and learning_matches
    detail = f"window_active={window_active} charger_state={charger_state} learning_mode={learning_mode}"
    return ok, detail


def record_timer_accuracy(
    t: TestRunner, label: str, actual: int, expected: int, tolerance: float
) -> None:
    ok = abs(actual - expected) <= tolerance
    detail = f"actual={actual}s expected~{expected}s tol={tolerance}s"
    t.record(label, ok, detail)


def measure_timer_delta(dev: I2CDevice, reg_l: int, duration_s: float) -> tuple[int, int]:
    start = dev.read_u32(reg_l)
    start_time = time.monotonic()
    time.sleep(duration_s)
    elapsed = time.monotonic() - start_time
    expected = int(round(elapsed))
    end = dev.read_u32(reg_l)
    return end - start, expected


def prompt(message: str) -> None:
    print(f"\n{message}")
    input("Press Enter to continue...")


def main() -> int:
    parser = argparse.ArgumentParser(description="Interactive UPSPlus state machine check.")
    parser.add_argument("--bus", type=int, default=DEFAULT_BUS, help="I2C bus number (default: 1)")
    parser.add_argument("--addr", type=lambda x: int(x, 0), default=DEFAULT_ADDR, help="I2C address (default: 0x17)")
    parser.add_argument(
        "--window-timeout-seconds",
        type=float,
        default=120.0,
        help="Timeout waiting for a calibration window (default: 120s)",
    )
    parser.add_argument(
        "--window-poll-interval",
        type=float,
        default=1.0,
        help="Polling interval while waiting for a window (default: 1.0s)",
    )
    parser.add_argument(
        "--skip-window-check",
        action="store_true",
        help="Skip the calibration window transition check",
    )
    parser.add_argument(
        "--timer-check-seconds",
        type=float,
        default=6.0,
        help="Seconds to measure timer increments (default: 6.0)",
    )
    parser.add_argument(
        "--timer-check-tolerance",
        type=float,
        default=1.0,
        help="Allowed timer error in seconds (default: 1.0)",
    )
    args = parser.parse_args()

    print("Initial conditions before running this script:")
    print("- UPSPlus is connected to the host on the I2C bus.")
    print("- Battery is installed and the UPSPlus is powered.")
    print("- Start with the charger attached if possible (for attach/detach transitions).")
    print("- Ensure you can physically attach/detach the charger during prompts.")
    print("- For current runtime checks, the load should be powered on.")

    t = TestRunner()
    dev = I2CDevice(args.bus, args.addr)
    try:
        sample_period_min = dev.read_u16(REG_SAMPLE_PERIOD_L)
        print(f"Sample period (minutes): {sample_period_min}")

        print("\nBaseline state")
        baseline = read_state(dev)
        print(format_state(baseline))
        ok, detail = check_window_consistency(baseline)
        t.record("Window/learning consistency (baseline)", ok, detail)

        prompt("Disconnect the charger and wait a few seconds.")
        state_disconnected = read_state(dev)
        print(format_state(state_disconnected))
        t.record(
            "Charger absent after disconnect",
            state_disconnected["charger_present"] == 0 and state_disconnected["charger_state"] == CHARGER_STATE_ABSENT,
            format_state(state_disconnected),
        )
        delta_all, expected = measure_timer_delta(dev, REG_RUNTIME_ALL_L, args.timer_check_seconds)
        record_timer_accuracy(t, "Runtime all accuracy (charger detached)", delta_all, expected, args.timer_check_tolerance)
        delta_charge, expected = measure_timer_delta(dev, REG_RUNTIME_CHARGING_L, args.timer_check_seconds)
        record_timer_accuracy(
            t,
            "Charging runtime stopped when detached",
            delta_charge,
            0,
            args.timer_check_tolerance,
        )
        if state_disconnected["power_state"] == 1:
            delta_current, expected = measure_timer_delta(
                dev, REG_RUNTIME_CURRENT_L, args.timer_check_seconds
            )
            record_timer_accuracy(
                t,
                "Current runtime accuracy (power on)",
                delta_current,
                expected,
                args.timer_check_tolerance,
            )
        else:
            t.record("Current runtime accuracy skipped", True, "power_state indicates power off")

        prompt("Attach the charger and wait a few seconds for detection.")
        state_connected = read_state(dev)
        print(format_state(state_connected))
        connected_ok = state_connected["charger_present"] == 1 and state_connected["charger_state"] in (
            CHARGER_STATE_PRESENT,
            CHARGER_STATE_FORCED_OFF_WINDOW,
        )
        t.record("Charger present after attach", connected_ok, format_state(state_connected))
        t.record(
            "Charger present transition (0 -> 1)",
            state_disconnected["charger_present"] == 0 and state_connected["charger_present"] == 1,
            f"before={state_disconnected['charger_present']} after={state_connected['charger_present']}",
        )
        ok, detail = check_window_consistency(state_connected)
        t.record("Window/learning consistency (attached)", ok, detail)
        delta_all, expected = measure_timer_delta(dev, REG_RUNTIME_ALL_L, args.timer_check_seconds)
        record_timer_accuracy(t, "Runtime all accuracy (charger attached)", delta_all, expected, args.timer_check_tolerance)
        delta_charge, expected = measure_timer_delta(dev, REG_RUNTIME_CHARGING_L, args.timer_check_seconds)
        record_timer_accuracy(
            t,
            "Charging runtime accuracy (charger attached)",
            delta_charge,
            expected,
            args.timer_check_tolerance,
        )

        prompt("Detach the charger and wait a few seconds for absence.")
        state_detached = read_state(dev)
        print(format_state(state_detached))
        t.record(
            "Charger absent after detach",
            state_detached["charger_present"] == 0 and state_detached["charger_state"] == CHARGER_STATE_ABSENT,
            format_state(state_detached),
        )
        t.record(
            "Charger present transition (1 -> 0)",
            state_connected["charger_present"] == 1 and state_detached["charger_present"] == 0,
            f"before={state_connected['charger_present']} after={state_detached['charger_present']}",
        )
        delta_charge, expected = measure_timer_delta(dev, REG_RUNTIME_CHARGING_L, args.timer_check_seconds)
        record_timer_accuracy(
            t,
            "Charging runtime stopped after detach",
            delta_charge,
            0,
            args.timer_check_tolerance,
        )

        if args.skip_window_check:
            t.record("Calibration window check skipped", True, "use --skip-window-check")
            return t.summary()

        prompt("Attach the charger and keep it connected for the window check.")
        start = time.monotonic()
        window_seen = False
        while time.monotonic() - start < args.window_timeout_seconds:
            state = read_state(dev)
            ok, detail = check_window_consistency(state)
            t.record("Window/learning consistency (poll)", ok, detail)
            if state["window_active"] == 1:
                window_seen = True
                print("Window active detected.")
                break
            time.sleep(args.window_poll_interval)
        t.record(
            "Calibration window became active",
            window_seen,
            f"timeout={args.window_timeout_seconds}s",
        )

        return t.summary()
    finally:
        dev.close()


if __name__ == "__main__":
    sys.exit(main())
