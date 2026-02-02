# UPSPlus Behavior Specification

This document describes the expected behavior of the UPSPlus firmware based on the refactoring plan.
It is intended as the source of truth for feature development and future changes.

---

## 1. Purpose and Scope

- Provide a clear, implementation-agnostic description of system behavior.
- Define the external contract (I2C registers, timing, and observable behavior).
- Document state machines, invariants, and safety rules.
- Serve as reference for new feature development and regression testing.

---

## 2. Architectural Principles

1. **Single Source of Truth**  
   All authoritative state is owned by the main loop. ISRs only set flags.

2. **No Side Effects on Read**  
   I2C reads must never modify state, counters, timers, or flash.

3. **Atomic Snapshots**  
   I2C reads are served from a double-buffered snapshot. Multi-byte registers must be coherent.

4. **No Flash in ISR**  
   Flash erase/program is main-loop only.

5. **ADC Gating Rule**  
   Any logic that depends on ADC samples must be gated by `adc_sample_seq`.

---

## 3. Timing Model

- Canonical scheduler tick: **10ms** (TIM1 ISR sets flags only).
- Derived tick rates:  
  - 100ms heartbeat  
  - 500ms ADC trigger  
  - 1s counters
- **Measurement window:** 1.5 seconds (charger-off) for true VBAT sampling.
- Countdowns and debounce are driven by 10ms ticks, but only advanced in main loop.
- Snapshot updates are guaranteed at least once every 100ms while the main loop is running.

---

## 4. External Interfaces (I2C)

### 4.1 Register Map (high-level)
- **0x01–0x0C**: Voltages + temperature (RO).
- **0x0D–0x12**: Battery full/empty/protection thresholds (RW, validated).
- **0x13–0x14**: Battery percent (LSB=percent, MSB always 0x00).
- **0x15–0x16**: Sample period minutes (RW).
- **0x17**: Power status (derived).
- **0x18**: Shutdown countdown (RW).
- **0x19**: Auto power on (RW).
- **0x1A**: Restart countdown (RW).
- **0x1B**: Factory reset (RW, write 1 to trigger).
- **0x1C–0x27**: Runtime counters (RO).
- **0x28–0x29**: Firmware version (RO).
- **0x2A**: Battery parameters self-programmed (RW).
- **0x2B**: Low battery percent threshold (RW).
- **0x2C–0x2D**: Load on delay (RW, 16-bit).
- **0x2E–0xEF**: Reserved (RO zero, writes ignored).
- **0xF0–0xFB**: MCU serial number (RO).
- **0xFC–0xFF**: Factory Testing (runtime selector + pages).

### 4.2 Register Semantics
- All multi-byte values are **little-endian**.
- Reserved regions return **0x00**, writes are ACK’d and ignored.
- Read-only registers ignore writes.
- Configuration writes are validated and applied in the main loop.

### 4.2.1 Invalid Write Handling Rules
- Single-byte invalid values are **ignored** (ACK, no state change).
- Multi-byte partial writes are **ignored entirely** (no partial application).
- No register ever enters a half-updated state.
- Repeated-start reads after a register-pointer write observe the same snapshot latched at transaction start.

### 4.3 Special RW Registers
- **0x18 Shutdown Countdown**
  - Write 0: cancel.
  - Write 10–255: start/update countdown.
  - Write 1–9: ignored.
  - At 1 → power off; register returns 0.
- **0x1A Restart Countdown**
  - Write 0: cancel.
  - Write 10–255: start/update countdown.
  - At 1 → power cycle RPi (MT_EN LOW 5s, then HIGH); register returns 0.
- **0x1B Factory Reset**
  - Write 1: immediate reset to defaults, flash updated.
- **0x2A Battery Parameters Self-Programmed**
  - Write 0: enable self-programming of full/empty parameters; reset learned values.
  - Write 1: disable self-programming; current learned values become user values.
  - Note: This register controls self-programming only. The periodic calibration
    measurement window (true VBAT sampling) is independent of 0x2A.
- **0x2C–0x2D Load On Delay**
  - Read: remaining countdown if active, else configured delay.
  - Write: sets configured delay; if countdown active, resets remaining time.

### 4.4 Factory Testing (0xFC–0xFF)
- **Selector register:** write `0xFC` to select a page; write 0 to disable.
- **When disabled:** 0xFC–0xFF read as 0x00.
- **When enabled:** 0xFC returns selector, 0xFD–0xFF return page values.
- **Unknown selector:** accepted and read back; 0xFD–0xFF return 0x00.
- **Writes to 0xFD–0xFF:** always ignored.

**ABI evolution rules:**
- New selectors must not change existing selector meanings.
- Existing selector pages must never reinterpret byte positions.
- Enum values are append-only and version-gated.

**Page definitions (current):**
- Selector 0x01: State page  
  - 0xFD: `power_state_t`
  - 0xFE: `charger_state_t`
  - 0xFF: `learning_mode_t`
- Selector 0x02: Button page  
  - 0xFD: `button_state_t`
  - 0xFE: `button_click_t`
  - 0xFF: hold ticks (LSB)
- Selector 0x03: Charger/Window page  
  - 0xFD: charger physically present (0/1)
  - 0xFE: window active (0/1)
  - 0xFF: window due (0/1)
- Selector 0x04: Protection page  
  - 0xFD: protection active (0/1)
  - 0xFE: below-threshold count
  - 0xFF: pending power cut (0/1)
- Selector 0x05: Flash/Persistence page  
  - 0xFD: `flash_status` (bitfield)  
    - bit0: record_valid (1 = valid record accepted at boot)  
    - bit1: save_attempted (1 = at least one save attempt since boot)  
    - bit2: save_success (1 = last save succeeded)  
    - bits3–7: reserved (0)  
  - 0xFE: `auto_power_on_info` (bitfield)  
    - bit0: auto_power_on_loaded (value loaded from flash)  
    - bit1: auto_power_on_effective (current active value in RAM)  
    - bits2–7: reserved (0)  
  - 0xFF: flash_sequence_lsb (0–255, increments on each successful commit, wraps)

---

## 5. State Machines

### 5.1 Power State Machine (`power_state_t`)
States:
- `RPI_OFF`
- `RPI_ON`
- `PROTECTION_LATCHED`
- `LOAD_ON_DELAY`

Key transitions:
- `RPI_OFF → LOAD_ON_DELAY`: auto-power-on enabled AND charger present AND battery percent > low threshold AND battery voltage > protection threshold (+ hysteresis).
- `LOAD_ON_DELAY → RPI_ON`: delay elapsed AND conditions still valid.
- `LOAD_ON_DELAY → RPI_OFF`: conditions invalid.
- `RPI_ON → PROTECTION_LATCHED`: battery below protection voltage for required samples.
- `RPI_ON → RPI_OFF`: manual off (long press).
- `PROTECTION_LATCHED → LOAD_ON_DELAY`: charger present AND battery percent > low threshold AND battery voltage > protection threshold (+ hysteresis).
- `PROTECTION_LATCHED → RPI_OFF`: charger disconnected or conditions invalid.
- Battery percent is derived from true-VBAT when the charger is influencing VBAT; if true-VBAT is stale, percent is held.

### 5.2 Charger State Machine (`charger_state_t`)
States:
- `ABSENT`
- `PRESENT`
- `FORCED_OFF_WINDOW`

Key behaviors:
- `ABSENT → PRESENT`: post-scaled connector-referenced charger voltage stable above threshold.
- `PRESENT → ABSENT`: post-scaled connector-referenced charger voltage stable below threshold.
- `PRESENT → FORCED_OFF_WINDOW`: sample period elapsed (window due).
- `FORCED_OFF_WINDOW → PRESENT`: window elapsed (1.5s).

### 5.3 Calibration Window Flag (`learning_mode_t`)
- **Legacy name:** `learning_mode_t` is retained for ABI compatibility.
- Indicates **calibration window active** (charger forced off for true VBAT sampling).
- `ACTIVE` while the charger state is `FORCED_OFF_WINDOW`, otherwise `INACTIVE`.
- Calibration windows occur every `sample_period_minutes` and are independent of 0x2A.

### 5.4 Button FSM
- Debounced at 50ms.
- Short press: power on if off and safe.
- Long press (>= 10s):
  - If ON → power off.
  - If OFF → factory reset.

---

## 6. Measurement Window

- Window duration is **1.5 seconds** and is atomic.
- Window is never interrupted or restarted early.
- If charger unplugged mid-window, window completes and samples are true VBAT.
- Window states are represented in Factory Testing selector 0x03.
- `learning_mode_t` reports **ACTIVE** during the window.
- Purpose: eliminate charger influence on VBAT measurement and provide a stable
  condition for ADC calibration.

---

## 7. Battery Management

- Battery percent is based on full/empty calibration.
- Battery percent update direction uses charger state (charger path enabled), not VBUS voltage.
- Protection voltage is enforced with hysteresis (50mV).
- Protection latch requires multiple ADC samples below threshold (3 samples).
- **Charging plateau full detection (self-programming enabled):**
  - All VBAT values in this algorithm refer to **filtered** VBAT.
  - **Plateau definition (two knobs):** while charger state is `PRESENT`, a battery is
    considered **full** when filtered VBAT remains within `PLATEAU_DELTA_MV` over a continuous
    `PLATEAU_WINDOW_SEC` (i.e., `max(VBAT) - min(VBAT) <= PLATEAU_DELTA_MV` within the window).
    Default `PLATEAU_WINDOW_SEC` is **1800 seconds (30 minutes)** and
    default `PLATEAU_DELTA_MV` is **40 mV**.
  - **Window evaluation:** use a sliding window evaluated every `PLATEAU_EVAL_PERIOD_SEC`.
    Default `PLATEAU_EVAL_PERIOD_SEC` is **5 seconds**.
  - **Gating conditions:** plateau detection is attempted only when all are true:
    - Charger state is `PRESENT` **and** the battery is not in a discharge event.
      A discharge event is detected when `VBAT_end < VBAT_start - DISCHARGE_DETECT_MV`
      over the plateau window, where `VBAT_start` and `VBAT_end` are filtered VBAT
      sampled at the window boundaries. Default `DISCHARGE_DETECT_MV` is **15 mV**.
    - VBAT is above `PLATEAU_MIN_VBAT_MV` (near-full floor). Default
      `PLATEAU_MIN_VBAT_MV` is **4050 mV**.
    - Load state is stable: no `power_state_t` transitions during the window and
      `power_state_t == RPI_ON` has been stable for at least `PLATEAU_POWER_STABLE_SEC`.
      Default `PLATEAU_POWER_STABLE_SEC` is **60 seconds**.
    - Optional: if charger/charge-current telemetry exists, require taper/CV region; otherwise
      this condition is skipped.
  - **Learning update rule:** on a plateau event, compute the plateau level as the mean of
    filtered VBAT samples within the window.
    If `abs(plateau - learned_full) >= PLATEAU_MIN_CHANGE_MV`, update learned full as:
    `learned_full = (1 - PLATEAU_ALPHA) * learned_full + PLATEAU_ALPHA * plateau`.
    Default `PLATEAU_MIN_CHANGE_MV` is **10 mV** and default `PLATEAU_ALPHA` is **0.2**.
  - **Initialization / invalid learned_full:** if `learned_full` is unset, initialize it to
    `PLATEAU_MIN_VBAT_MV` (or 4200 mV if a fixed default is preferred) and allow learning normally.
  - **Clamping:** clamp `learned_full` to `[LEARNED_FULL_MIN_MV, LEARNED_FULL_MAX_MV]`.
    Default bounds are **3900–4500 mV**.
  - **Full state behavior:** on plateau detection, assert `battery_full=1`.
    Clear `battery_full` when charger is not `PRESENT` or when
    `VBAT < learned_full - FULL_HYST_MV` for `FULL_CLEAR_SEC`.
    Default `FULL_HYST_MV` is **50 mV** and default `FULL_CLEAR_SEC` is **300 seconds**.
  - **Re-trigger suppression:** after a plateau event, suppress further plateau detection
    until the charger leaves `PRESENT`.
  - **Persistence / flash wear:** persist the learned full voltage only if the
    accepted change is >= `PLATEAU_PERSIST_MIN_CHANGE_MV` and at most once per
    charger-present session. A charger-present session begins when the charger
    state transitions to `PRESENT` and ends when it leaves `PRESENT`. Default
    `PLATEAU_PERSIST_MIN_CHANGE_MV` is **20 mV**.
  - If the battery is replaced and a different plateau level persists for the same window,
    the learned full voltage automatically converges to the new plateau value.
- When protection triggers: pending power cut is set, flash save is attempted, then MT_EN is cut.
- If flash save fails, power is still cut after the attempt. On next boot, defaults may apply,
  but the protection latch behavior remains effective.
- If flash save continues to fail, MT_EN is force-cut after 30 seconds to avoid prolonged
  operation below the protection threshold.
- **Boot brownout backoff (load-on delay learning):**
  - **Boot attempt start:** when MT_EN is asserted (power_state enters `RPI_ON`).
  - **Boot failure event:** protection triggers (battery voltage <= configured protection threshold
    for required samples; see Section 7) within **5 minutes** of load enable (**inclusive**, ≤ 5:00).
    For this rule, “protection triggers” means the same event that initiates the shutdown sequence
    (pending power cut / transition into protection handling).
  - **Boot success:** the load remains enabled for **5 minutes** without a protection threshold
    violation.
  - **Learning rule:** after each boot failure event (no consecutive-failure requirement), the
    firmware increments the internal `load.on.delay` by **1 minute** before the next auto power-on
    attempt, and learning stops after a successful boot.
  - **Clamp:** learned delay is clamped to **60 minutes (3600 seconds)**.
  - **Persistence:** learned `load.on.delay` persists across power cycles and reboots and is cleared
    only by factory reset.
  - **Manual override:** a user write to `load.on.delay` replaces the learned value and becomes the
    new baseline for any future learning (learning adds minutes on top of the user-provided value).

---

## 8. Flash Persistence

- Single-slot persistence (1KB page).
- Dirty state is periodically flushed (60s) and on critical events.
- Flash write attempts are rate limited unless bypassed.
- CRC and sequence validation performed on load; invalid → defaults + dirty.

---

## 9. GPIO Behavior

- **IP_EN (PA5)**: controls charger path.
- **MT_EN (PA6)**: controls RPi power.
- **PWR_EN (PA7)**: always HIGH.
- **Button (PB1)**: EXTI edge → main-loop FSM.

---

## 10. Testing and Validation

- Automated I2C test script: `tools/testing/upsplus_i2c_test.py`.
- Manual hardware tests still required for charger transitions, measurement windows, and protection latching.
- Architecture validation tests:
  - I2C coherence under snapshot swaps.
  - ADC gating correctness.
  - Window atomicity edge cases.

---

## 11. Enumerations (ABI-Stable Values)

- `power_state_t`:  
  - `RPI_OFF=0`, `RPI_ON=1`, `PROTECTION_LATCHED=2`, `LOAD_ON_DELAY=3`
- `charger_state_t`:  
  - `ABSENT=0`, `PRESENT=1`, `FORCED_OFF_WINDOW=2`
- `learning_mode_t`:  
  - `INACTIVE=0`, `ACTIVE=1`
- `button_state_t`:  
  - `IDLE=0`, `PRESSED=1`, `HELD=2`, `RELEASED_SHORT=3`, `RELEASED_LONG=4`
- `button_click_t`:  
  - `NONE=0`, `SHORT=1`, `LONG=2`

These numeric values are part of the Factory Testing ABI and must remain stable.

---

## 12. Non-Goals

- No SOC estimation beyond voltage-based heuristic.
- No fast charger negotiation.
- No dynamic ADC reconfiguration at runtime.
- No interrupt-driven state transitions.

---

## 13. Glossary

- **true VBAT**: Battery voltage sampled while charger path is disabled (IP_EN LOW).
- **charger present**: Physical charger voltage above presence threshold with stability.
- **charger influencing VBAT**: Charger state is PRESENT at ADC sample time.
- **calibration window active (legacy field `learning_mode_t`)**: Charger forced off for true VBAT sampling and ADC calibration.
- **snapshot**: Coherent, double-buffered register image used for I2C reads.

---

## 14. Change Impact Map

- If ADC cadence changes, revisit: charger stability counters, protection sample count, window duration.
- If register map changes, revisit: Factory Testing ABI, test scripts, and external tools.
- If snapshot frequency changes, revisit: I2C coherence assumptions and staleness guarantees.
- If protection logic changes, revisit: power-cut ordering and flash save semantics.

---

## 15. Spec Authority

In the event of conflict, this Behavior Specification is authoritative over code comments,
test scripts, and historical behavior.
