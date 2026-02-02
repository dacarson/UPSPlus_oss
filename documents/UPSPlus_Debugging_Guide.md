## UPSPlus Debugging Guide

This document explains how to use the current debugging script and how to
perform practical hardware checks. It references the behavior spec as the
authoritative contract and avoids duplicating it.

Authoritative behavior reference: `documents/UPSPlus_Behavior_Spec.md`

---

## 1. Debugging Script Overview

Scripts:
- `tools/testing/upsplus_i2c_test.py` (non-interactive integration tests)
- `tools/testing/upsplus_state_machine_interactive.py` (guided, interactive checks)

Purpose:
- Run a non-interactive I2C integration test suite against the firmware.
- Validate read-only vs read-write register behavior.
- Exercise factory test pages and basic state-machine invariants.
- Optionally perform power-state and destructive actions when explicitly enabled.

What the non-interactive script does (high level):
- Reads voltage/temperature block, firmware version, and serial number.
- Verifies reserved registers read 0 and ignore writes.
- Exercises factory test selector pages and ABI rules.
- Validates that read-only registers ignore writes.
- Writes and restores safe alternate values for configurable registers.
- Samples factory test state pages over time to check invariants.
- Optionally runs power-state related tests (auto power on, load-on delay).
- Runs countdown accuracy checks (shutdown/restart) and optionally runs destructive tests (factory reset).

What the interactive script does (high level):
- Guides the user through charger attach/detach steps.
- Waits for user confirmation before validating transitions.
- Reads factory pages 0x01 and 0x03 to confirm state-machine transitions.
- Optionally waits for a calibration window to become active.

Important safety note:
- The scripts are conservative by default. Power-affecting tests are skipped
  unless explicitly enabled by flags. Destructive actions are also opt-in.

---

## 2. Script Options and What They Do

Command:
```
python3 tools/testing/upsplus_i2c_test.py [options]
```

Options:
- `--bus` (default: 1)
  - I2C bus number to use.
- `--addr` (default: 0x17)
  - I2C address of the UPSPlus.
- `--allow-power-actions`
  - Enables tests that may affect power state:
    - auto power on register
    - load on delay register
- `--allow-destructive`
  - Enables factory reset test.
- `--state-monitor-seconds` (default: 5.0)
  - Duration of the state page polling loop.
- `--state-monitor-interval` (default: 0.2)
  - Polling interval between state page samples.
- `--timer-accuracy-seconds` (default: 6.0)
  - Duration to measure runtime counter accuracy.
- `--timer-accuracy-tolerance` (default: 1.0)
  - Allowed timer error in seconds.
- `--countdown-accuracy-seconds` (default: 3.0)
  - Duration to measure countdown timer accuracy (tests are canceled before completion).

Examples:
```
python3 tools/testing/upsplus_i2c_test.py
python3 tools/testing/upsplus_i2c_test.py --allow-power-actions
python3 tools/testing/upsplus_i2c_test.py --allow-destructive
python3 tools/testing/upsplus_i2c_test.py --state-monitor-seconds 20 --state-monitor-interval 0.5
```

Interactive command:
```
python3 tools/testing/upsplus_state_machine_interactive.py [options]
```

Interactive options:
- `--bus` (default: 1)
  - I2C bus number to use.
- `--addr` (default: 0x17)
  - I2C address of the UPSPlus.
- `--window-timeout-seconds` (default: 120.0)
  - Timeout waiting for a calibration window to become active.
- `--window-poll-interval` (default: 1.0)
  - Polling interval while waiting for a window.
- `--skip-window-check`
  - Skip the calibration window transition check.
- `--timer-check-seconds` (default: 6.0)
  - Duration to measure timer increments during interactive checks.
- `--timer-check-tolerance` (default: 1.0)
  - Allowed timer error in seconds.

Interactive examples:
```
python3 tools/testing/upsplus_state_machine_interactive.py
python3 tools/testing/upsplus_state_machine_interactive.py --skip-window-check
python3 tools/testing/upsplus_state_machine_interactive.py --window-timeout-seconds 300
```

---

## 3. What Each Test Covers

The script reports PASS/FAIL for each test and summarizes at the end. The
following groupings describe the intent of each test set. If a test fails,
check the behavior spec to confirm expected semantics.

Basic reads:
- Reads a block of voltage/temperature registers.
- Confirms battery percent MSB is 0x00.
- Reads firmware version and serial number blocks.

Reserved and read-only behavior:
- Reserved addresses sample read as 0x00.
- Writes to reserved space are ignored.
- Read-only register ignores writes.

Factory test pages:
- Selector 0 returns zeros.
- Selectors 1-4 return expected range values.
- Selector 5 validates flash status and auto power on bitfields.
- Unknown selector returns zeros for page data.
- Writes to 0xFD-0xFF are ignored.

Read/write configuration tests (safe alternates):
- Sample period minutes: write/read/restore.
- Low battery percent: write/read/restore.
- Full/empty/protection voltages: write/read/restore with safe deltas.

State machine monitor:
- Periodically reads factory pages 1-4.
- Checks that enum values are within valid ranges.
- Checks basic invariants, such as window_active consistency.

Optional power actions:
- Auto power on toggle + restore.
- Load on delay write + restore (skipped if already in delay).

Timer accuracy checks:
- Runtime all/current/charging counters compared to host time.
- Shutdown/restart countdown accuracy (start + verify + cancel).

Destructive actions (opt-in only):
- Factory reset trigger.

---

## 4. Suggested Manual Debugging Scenarios

Use these to validate real-world behavior that the scripts cannot fully
exercise. The interactive script can guide some of these steps; use it when
noted. Always refer to the behavior spec for expected outcomes.

Power and charging:
- Use `tools/testing/upsplus_state_machine_interactive.py` to step through
  charger attach/detach and validate charger_present and charger_state.
- The interactive script also checks that charge time increments only when the
  charger is attached (and stops when detached).
- If running manually, poll factory page 0x03 to confirm charger_present
  changes 0 → 1 on attach and 1 → 0 on detach.
- Poll factory page 0x01 to verify charger_state changes (ABSENT ↔ PRESENT),
  and watch for the forced-off window transitions if you keep the charger attached.
- Disconnect charger and confirm battery percent trends downward over time.
- Let the battery run down to zero and observe the protection shutdown.

Load power behavior:
- Power the Raspberry Pi from the UPSPlus, then remove the charger.
- Confirm that power remains stable until the protection threshold.
- Observe auto power on behavior after reconnecting a charger.

Button behavior:
- Short press while off to power on (if battery above threshold).
- Long press while on to power off.
- Long press while off to trigger factory reset.

Calibration window and sampling:
- Use `tools/testing/upsplus_state_machine_interactive.py` without
  `--skip-window-check` to wait for a calibration window and validate
  learning_mode and window_active consistency.
- If running manually, keep the charger connected and confirm that the
  calibration window (factory page 0x03) becomes active periodically, and
  verify factory page 0x01 learning_mode reflects the window state.

Persistence checks:
- Trigger a configuration change, power cycle the system, and verify the
  new values persist.
- If factory reset is used, confirm values return to defaults afterward.

---

## 5. Troubleshooting Tips

- If the script reports "Missing I2C library", install `smbus2` on the host:
  `pip install smbus2`
- Use `i2cdetect -y 1` to confirm the device is visible on the bus.
- If tests time out, confirm the I2C bus number and address.
- If the factory reset test is not running, ensure `--allow-destructive` is set.

