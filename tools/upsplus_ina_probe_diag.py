#!/usr/bin/env python3
"""
Continuously sample the STM32's INA219 current-read age counters (Factory Test
selector 0x07) to measure how often the guard-window probe actually succeeds
under live conditions (e.g. with NUT running).

A single point-in-time register dump only shows whether a read happened to be
fresh (age <= 2s) at that instant; it can't tell you whether the probe is
succeeding regularly with brief gaps, or barely succeeding at all. Sampling
the raw age value (0-255, in 10ms units, saturating at 255 = 2.55s+) at a
fast, steady interval shows the actual pattern over time.

IMPORTANT CAVEAT: every sample this tool takes is itself an I2C transaction addressed to the
STM32 -- this tool is another polling consumer competing for the same bus access it's trying to
measure, on top of NUT. `--mode age`'s continuous polling every --interval is itself
STM32-addressed traffic that refreshes the guard window's "recently active" tracking and
physically occupies bus time, so numbers from that mode reflect "NUT + this tool running", not
"NUT alone".

`--mode counts` avoids this: it takes a small, fixed number of I2C transactions (before and
after a plain, otherwise-silent time.sleep()), so the deltas it reports reflect genuine "NUT
alone" behavior. Prefer it for actual rate measurement; use `--mode age` only for
qualitative/exploratory looks, understanding it perturbs what it measures.

Two modes:
  --mode counts (recommended for rate measurement) brackets a silent sleep with reads of the
                guard-open/read-success/retry counters -- no self-generated traffic during the
                observation window.
  --mode age    sample output/battery current age + counter deltas continuously (see caveat
                above -- this mode's own polling contaminates the very thing it measures).

Usage:
    python3 tools/upsplus_ina_probe_diag.py [--mode counts|age] [--duration SEC] [--interval SEC]
"""

import argparse
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

REG_FACTORY_TEST_SELECTOR = 0xFC
REG_FACTORY_TEST_DATA0 = 0xFD  # selector 0x07: output_current_age_10ms (capped at 255)
REG_FACTORY_TEST_DATA1 = 0xFE  # selector 0x07: battery_current_age_10ms (capped at 255)
SELECTOR_INA_CURRENT_AGE = 0x07

# selector 0x08: ina_guard_open_count (0xFD), ina_read_success_count (0xFE) -- both
# saturating, cumulative, never auto-reset. A gap between the two counts means the guard
# window is permitting attempts that then fail at the actual I2C transaction level -- measured
# as consistently bus_error (BERR/ARLO/OVR), i.e. genuine collision with another master.
# 0xFF: session-long max ticks (10ms units, saturating at 255=2550ms) a FRESH (non-retry)
# request sat pending before the guard first granted it.
SELECTOR_INA_PROBE_COUNTS = 0x08
REG_GUARD_OPEN_COUNT = 0xFD
REG_READ_SUCCESS_COUNT = 0xFE
REG_MAX_PENDING_WAIT = 0xFF

# selector 0x10: retry effectiveness (saturating, never auto-reset -- diff two samples).
# attempted: a retry was scheduled after a failure. recovered: a probe succeeded after >=1
# retry (retry actually saved the read). (A third "exhausted" byte -- all retries used up with
# no success -- was removed from the firmware to reclaim flash; it read 0 in every test.)
SELECTOR_RETRY_STATS = 0x10
REG_RETRY_ATTEMPTED_COUNT = 0xFD
REG_RETRY_RECOVERED_COUNT = 0xFE

# selector 0x11: guard-window inputs tracked INTERNALLY by the firmware every main loop
# iteration (GuardDiagnostics_Update()) -- immune to the observer-effect problem external I2C
# polling has (an external read of these values would itself be STM32-addressed traffic).
# Session-long maximums (10ms units, saturating at 255 = 2.55s), never auto-reset.
SELECTOR_GUARD_DIAG_MAX = 0x11
REG_MAX_TICKS_SINCE_ACTIVITY = 0xFD
REG_MAX_TXN_ACTIVE_STUCK = 0xFE
REG_TXN_ACTIVE_NOW = 0xFF

# Matches CURRENT_VALID_AGE_TICKS in Inc/ups_state.h (200 * 10ms = 2s)
VALID_AGE_10MS_MAX = 200
AGE_SATURATED = 255


class I2CDevice:
    def __init__(self, bus_id: int, addr: int) -> None:
        self.bus = SMBus(bus_id)
        self.addr = addr

    def close(self) -> None:
        self.bus.close()

    def write_u8(self, reg: int, value: int) -> None:
        self.bus.write_byte_data(self.addr, reg, value & 0xFF)

    def read_u8(self, reg: int) -> int:
        return self.bus.read_byte_data(self.addr, reg) & 0xFF


class ChannelStats:
    def __init__(self, name: str) -> None:
        self.name = name
        self.samples: List[int] = []

    def add(self, age: int) -> None:
        self.samples.append(age)

    def summary(self) -> str:
        if not self.samples:
            return f"{self.name}: no samples"
        n = len(self.samples)
        fresh = sum(1 for a in self.samples if a <= VALID_AGE_10MS_MAX)
        saturated = sum(1 for a in self.samples if a >= AGE_SATURATED)
        avg = sum(self.samples) / n
        return (
            f"{self.name}: {n} samples, "
            f"fresh(age<=2s) {fresh}/{n} ({100.0 * fresh / n:.1f}%), "
            f"saturated(>=2.55s) {saturated}/{n} ({100.0 * saturated / n:.1f}%), "
            f"avg age {avg * 10:.0f}ms, max age {max(self.samples) * 10}ms"
        )


def read_probe_counts(dev: "I2CDevice") -> "tuple[int, int, int]":
    dev.write_u8(REG_FACTORY_TEST_SELECTOR, SELECTOR_INA_PROBE_COUNTS)
    guard_open = dev.read_u8(REG_GUARD_OPEN_COUNT)
    read_success = dev.read_u8(REG_READ_SUCCESS_COUNT)
    max_pending_wait = dev.read_u8(REG_MAX_PENDING_WAIT)
    return guard_open, read_success, max_pending_wait


def read_retry_stats(dev: "I2CDevice") -> "tuple[int, int]":
    dev.write_u8(REG_FACTORY_TEST_SELECTOR, SELECTOR_RETRY_STATS)
    attempted = dev.read_u8(REG_RETRY_ATTEMPTED_COUNT)
    recovered = dev.read_u8(REG_RETRY_RECOVERED_COUNT)
    return attempted, recovered


def read_guard_diag_max(dev: "I2CDevice") -> "tuple[int, int, int]":
    """Session-long maximums tracked internally by the firmware, immune to the observer-effect
    problem external polling has for these specific values -- safe to read at any time."""
    dev.write_u8(REG_FACTORY_TEST_SELECTOR, SELECTOR_GUARD_DIAG_MAX)
    max_ticks_since_activity = dev.read_u8(REG_MAX_TICKS_SINCE_ACTIVITY)
    max_txn_active_stuck = dev.read_u8(REG_MAX_TXN_ACTIVE_STUCK)
    txn_active_now = dev.read_u8(REG_TXN_ACTIVE_NOW)
    return max_ticks_since_activity, max_txn_active_stuck, txn_active_now


def print_retry_and_diag(dev: "I2CDevice", guard_open_delta: int, read_success_delta: int,
                          start_retry: "tuple[int, int]", end_retry: "tuple[int, int]",
                          start_diag: "tuple[int, int, int]") -> None:
    start_retry_attempted, start_retry_recovered = start_retry
    end_retry_attempted, end_retry_recovered = end_retry

    retry_attempted_delta = end_retry_attempted - start_retry_attempted
    retry_recovered_delta = end_retry_recovered - start_retry_recovered
    print("\n=== Retry effectiveness (delta over this run) ===")
    print(f"retries scheduled={retry_attempted_delta} recovered={retry_recovered_delta}")
    if retry_attempted_delta > 0:
        recovery_rate = 100.0 * retry_recovered_delta / retry_attempted_delta
        print(f"-> {recovery_rate:.1f}% of scheduled retries resulted in a recovered read")
    elif guard_open_delta - read_success_delta > 0:
        print("-> Failures occurred but no retries were scheduled -- check the retry logic/build.")

    start_max_ticks_since_activity, start_max_txn_active_stuck, _ = start_diag
    max_ticks_since_activity, max_txn_active_stuck, txn_active_now = read_guard_diag_max(dev)
    ticks_since_activity_new_high = max_ticks_since_activity > start_max_ticks_since_activity
    txn_active_stuck_new_high = max_txn_active_stuck > start_max_txn_active_stuck

    print(
        "\n=== Guard-window internal diagnostics (session-long max, immune to observer effect) ===\n"
        "start/end below bracket THIS run's observation window -- if end > start, that specific max\n"
        "was newly set during this run, not inherited from something earlier in the session."
    )
    print(
        f"max ticks-since-last-STM32-activity: start={start_max_ticks_since_activity * 10}ms "
        f"end={max_ticks_since_activity * 10}ms (capped at 2550ms)"
        + ("  <-- NEW HIGH THIS RUN" if ticks_since_activity_new_high else "")
    )
    print(
        f"max consecutive time txn_active stuck at 1: start={start_max_txn_active_stuck * 10}ms "
        f"end={max_txn_active_stuck * 10}ms (capped at 2550ms)"
        + ("  <-- NEW HIGH THIS RUN" if txn_active_stuck_new_high else "")
    )
    print(f"txn_active right now: {txn_active_now}")

    if ticks_since_activity_new_high:
        print(
            f"-> The STM32 itself went {max_ticks_since_activity * 10}ms without seeing ANY "
            "STM32-addressed I2C activity DURING THIS RUN -- this is not stale/historical, it "
            "happened just now. If Pi-side failures were observed in this same window, this is "
            "very likely the direct cause: the STM32 itself was silent, not competing/colliding."
        )
    elif txn_active_stuck_new_high:
        print(
            f"-> txn_active was newly observed stuck at 1 for up to {max_txn_active_stuck * 10}ms "
            "DURING THIS RUN -- that alone would explain chronic guard denial in this window "
            "(checked first, unconditional deny). Worth investigating the I2C1 ISR for a "
            "missed/mishandled STOP."
        )
    else:
        print(
            "-> No new STM32-side high-water mark was set during this run's observation window. "
            "If Pi-side symptoms (I2C read failures, strace-observed stalls) occurred during this "
            "same window, the STM32 was NOT internally deaf for an extended stretch at that time "
            "-- look at Pi-side causes instead (scheduling latency, a competing process such as "
            "nut_exporter, or several genuinely brief per-attempt collisions that individually "
            "don't explain a multi-hundred-ms streak but add up across retries)."
        )


def run_counts_mode(dev: "I2CDevice", duration: float) -> int:
    """Bracket a silent sleep with counter reads -- no self-generated I2C traffic during the
    observation window, so the result reflects NUT alone (or whatever else is on the bus),
    not this tool's own polling."""
    print(f"Reading start counts, then sleeping {duration:.1f}s with no further I2C traffic from this tool...")
    start_guard_open, start_read_success, start_max_wait = read_probe_counts(dev)
    start_retry = read_retry_stats(dev)
    start_diag = read_guard_diag_max(dev)
    time.sleep(duration)
    end_guard_open, end_read_success, end_max_wait = read_probe_counts(dev)
    end_retry = read_retry_stats(dev)

    guard_open_delta = end_guard_open - start_guard_open
    read_success_delta = end_read_success - start_read_success
    saturated_note = ""
    if end_guard_open == 0xFF or end_read_success == 0xFF:
        saturated_note = "  (a counter hit 255 and saturated -- delta is a lower bound)"

    print("\n=== Guard-open vs read-success (clean delta, no tool-generated traffic in between) ===")
    print(f"guard_open:   start={start_guard_open} end={end_guard_open} delta={guard_open_delta}")
    print(f"read_success: start={start_read_success} end={end_read_success} delta={read_success_delta}{saturated_note}")
    if guard_open_delta > 0:
        fail_rate = 100.0 * (guard_open_delta - read_success_delta) / guard_open_delta
        print(
            f"Of {guard_open_delta} attempts the guard allowed, {read_success_delta} actually "
            f"succeeded ({fail_rate:.1f}% failed at the transaction level after the guard said go)"
        )
    else:
        print("Guard never opened during this run -- the bottleneck is the guard/live-bus check itself.")

    print(
        f"\nMax time a FRESH request sat pending before the guard first granted it: "
        f"start={start_max_wait * 10}ms end={end_max_wait * 10}ms (session-long max, capped at 2550ms)"
    )
    if end_max_wait > start_max_wait:
        print(f"-> A new high-water mark ({end_max_wait * 10}ms) was set during this run.")

    print_retry_and_diag(dev, guard_open_delta, read_success_delta, start_retry, end_retry, start_diag)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode", choices=["counts", "age"], default="counts",
        help="Sampling mode (default counts -- see module docstring for the tradeoffs)",
    )
    parser.add_argument("--duration", type=float, default=30.0, help="Seconds to sample (default 30)")
    parser.add_argument("--interval", type=float, default=0.2, help="Seconds between samples (age mode only)")
    parser.add_argument("--quiet", action="store_true", help="Suppress per-sample lines (age mode only)")
    args = parser.parse_args()

    dev = I2CDevice(1, 0x17)

    if args.mode == "counts":
        try:
            return run_counts_mode(dev, args.duration)
        finally:
            try:
                dev.write_u8(REG_FACTORY_TEST_SELECTOR, 0)
            except OSError:
                pass
            dev.close()

    print(
        "NOTE: --mode age polls the STM32 every --interval seconds, which is itself "
        "STM32-addressed I2C traffic competing with NUT for the same bus. Prefer "
        "--mode counts for a rate measurement uncontaminated by this tool's own polling."
    )
    output_stats = ChannelStats("output_current_age")
    battery_stats = ChannelStats("battery_current_age")

    try:
        start_guard_open, start_read_success, start_max_wait = read_probe_counts(dev)
        start_retry = read_retry_stats(dev)
        start_diag = read_guard_diag_max(dev)

        dev.write_u8(REG_FACTORY_TEST_SELECTOR, SELECTOR_INA_CURRENT_AGE)
        start = time.monotonic()
        errors = 0
        while (time.monotonic() - start) < args.duration:
            t = time.monotonic() - start
            try:
                output_age = dev.read_u8(REG_FACTORY_TEST_DATA0)
                battery_age = dev.read_u8(REG_FACTORY_TEST_DATA1)
            except OSError as exc:
                errors += 1
                if not args.quiet:
                    print(f"[{t:6.2f}s] read error: {exc}")
                time.sleep(args.interval)
                continue
            output_stats.add(output_age)
            battery_stats.add(battery_age)
            if not args.quiet:
                print(
                    f"[{t:6.2f}s] output_age={output_age * 10:4d}ms  "
                    f"battery_age={battery_age * 10:4d}ms"
                )
            time.sleep(args.interval)

        end_guard_open, end_read_success, end_max_wait = read_probe_counts(dev)
        end_retry = read_retry_stats(dev)

        print("\n=== Summary ===")
        print(output_stats.summary())
        print(battery_stats.summary())
        if errors:
            print(f"I2C read errors: {errors}")

        print("\n=== Guard-open vs read-success (delta over this run -- CONTAMINATED by this")
        print("    mode's own polling traffic; use --mode counts for a clean measurement) ===")
        guard_open_delta = end_guard_open - start_guard_open
        read_success_delta = end_read_success - start_read_success
        saturated_note = ""
        if end_guard_open == 0xFF or end_read_success == 0xFF:
            saturated_note = "  (a counter hit 255 and saturated -- delta is a lower bound)"
        print(f"guard_open:   start={start_guard_open} end={end_guard_open} delta={guard_open_delta}")
        print(f"read_success: start={start_read_success} end={end_read_success} delta={read_success_delta}{saturated_note}")
        if guard_open_delta > 0:
            fail_rate = 100.0 * (guard_open_delta - read_success_delta) / guard_open_delta
            print(
                f"Of {guard_open_delta} attempts the guard allowed, {read_success_delta} actually "
                f"succeeded ({fail_rate:.1f}% failed at the transaction level after the guard said go)"
            )
        else:
            print("Guard never opened during this run -- the bottleneck is the guard/live-bus check itself.")

        print(
            f"\nMax time a FRESH request sat pending before the guard first granted it: "
            f"start={start_max_wait * 10}ms end={end_max_wait * 10}ms (session-long max, capped at 2550ms)"
        )
        if end_max_wait > start_max_wait:
            print(f"-> A new high-water mark ({end_max_wait * 10}ms) was set during this run.")

        print_retry_and_diag(dev, guard_open_delta, read_success_delta, start_retry, end_retry, start_diag)
    finally:
        # Disable factory test mode (selector 0) so normal register reads resume unaffected.
        try:
            dev.write_u8(REG_FACTORY_TEST_SELECTOR, 0)
        except OSError:
            pass
        dev.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
