## UPSPlus Debugging Guide

This document explains how to use the current debugging script and how to
perform practical hardware checks. It references the behavior spec as the
authoritative contract and avoids duplicating it.

Authoritative behavior reference: `documents/UPSPlus_Behavior_Spec.md`

---

## I2C Reading Examples

I2C address: `0x17` on bus `1` (Raspberry Pi default).

```bash
# Read a single byte register
i2cget -y 1 0x17 0x01

# Read a 16-bit little-endian register (e.g. full voltage)
i2cget -y 1 0x17 0x0D w

# Dump all registers
i2cdump -y 1 0x17
```

Use `i2cdetect -y 1` to confirm the device is visible on the bus. If it
does not appear at `0x17`, check power, wiring, and I2C bus number.

---

## Entering OTA Mode via I2C Command

**Caution:** This method reboots the UPSPlus immediately. Power to the
Raspberry Pi will be interrupted and the Pi will restart.

With the firmware running and the UPSPlus on the I2C bus, write **0x7F**
to register **0xFC**. The firmware persists the bootloader OTA flag, saves
flash, and reboots immediately into the bootloader. The device will stay in
OTA mode until a firmware update is performed.

```bash
i2cset -y 1 0x17 0xFC 0x7F b
```

Use the same I2C bus as your setup (e.g. `0` or `1` on Raspberry Pi).
After a few seconds the device resets into bootloader OTA mode. Verify with
`i2cdetect -y 1`; the bootloader appears at address `0x18`.

The button sequence (remove power, hold Func Key, reinsert batteries) is
the recommended alternative when avoiding a Pi restart is important.

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
- `--current-validation-seconds` (default: 8.0)
  - Duration to validate current cadence and freshness.
- `--current-validation-interval` (default: 0.1)
  - Polling interval during current validation.
- `--expected-output-current-ma`
  - Expected output current in mA for scaling/sign validation.
- `--expected-battery-current-ma`
  - Expected battery current in mA for scaling/sign validation.
- `--current-tolerance-ma` (default: 100)
  - Allowed current error in mA for scaling/sign validation.

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
- Selector 6 reports INA boot presence bits.
- Selector 7 reports output/battery age in 10 ms units (saturated to 255).
- Selector 0x0A reports the raw battery-channel ADC code (channel 1),
  pre-scaling/pre-clamp — see "Diagnosing Bad/Noisy Battery Voltage
  Readings" below. Compiled out by default (see note there); requires a
  firmware build with `UPS_ADC_FACTORY_DIAG_ENABLED=1`.
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

---

## 6. Diagnosing Bad/Noisy Battery Voltage Readings

Symptoms: register `0x05` (battery voltage) reads a fixed, implausibly low
value that never moves (e.g. pinned at or near the protection voltage,
register `0x11`), or it fluctuates wildly from read to read instead of
tracking the battery smoothly.

`state.battery_voltage_mv` is computed from ADC channel 1 and then
floor-clamped so it can never read below `max(VBAT_MIN_VALID_MV,
protection_voltage_mv)` (`Src/main.c:1072`). Since `protection_voltage_mv`
has a hard minimum of `VBAT_PROTECT_MIN_MV` (2800 mV,
`Inc/ups_state.h:71`), a frozen reading of ~2800 mV usually means the true
computed voltage is being clamped away, not that the ADC/DMA pipeline has
stalled. Use factory test selector `0x0A` to see the real value underneath
the clamp:

```bash
i2cset -y 1 0x17 0xFC 0x0A     # select page 0x0A
i2cget -y 1 0x17 0xFC i 4      # [0]=0x0A selector echo, [1..2]=raw ADC code (LE), [3]=reserved
```

Note: this selector (and 0x0B-0x0E, covering the other ADC channels and
the temperature-sensor calibration constants) is compiled out by default
to save flash on this part — see `UPS_ADC_FACTORY_DIAG_ENABLED` near the
top of `Src/main.c`. Rebuild with it set to `1` before using this section.

Decode: `raw = byte[1] | (byte[2] << 8)` (0-4095, 12-bit). Estimate the
voltage the MCU is actually seeing at the pin with `pin_mV = raw / 4095 *
mcu_voltage_mv` (register `0x01`), and compare against the divider
assumption in the code (`state.mcu_voltage_mv * 2` at `Src/main.c:1067`,
i.e. a 2:1 divider expected on VBAT_SENSE).

What the raw code tells you:
- **Stable but far below what a 2:1 divider predicts**: likely a real
  hardware fault or divider mismatch on the VBAT_SENSE sense path (wrong
  resistor value, bad joint, lifted pad) — check continuity/values against
  the schematic.
- **Wildly different value on every consecutive read** (large peak-to-peak
  swings, worse when a charger is actively connected/switching): the ADC
  input is floating/high-impedance rather than solidly referenced, so it's
  picking up EMI from nearby switching activity instead of measuring the
  battery.

Known cause (2026-07-20): on a HAT installation where the Raspberry Pi is
powered from a separate supply for testing, leaving 40-pin header pins 2,
4, and 6 (5V/5V/GND) disconnected left VBAT_SENSE without its expected
reference — frozen-low when idle, noisy when a charger was attached.
Reconnecting those header pins restored a stable, correctly-scaled
reading. If you see this symptom while bench-testing with the Pi powered
separately, reconnect (or jumper) pins 2/4/6 before suspecting the ADC
firmware or sense-divider hardware.

