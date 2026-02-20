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
- **0x2E–0x2F**: Output current (RO, signed int16, 1 mA/LSB).
- **0x30–0x31**: Battery current (RO, signed int16, 1 mA/LSB).
- **0x32**: Current valid flags (RO, bit0=output, bit1=battery).
- **0x33–0xEF**: Reserved (RO zero, writes ignored).
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
- **Selector register:** write a value to `0xFC` to select a page; write 0 to disable.
- **OTA command (write-only, consumed):** Writing **0x7F** to **0xFC** is a one-shot command. It does **not** update the factory test selector. The firmware persists the bootloader OTA flag (0x7F at 0x08003C64), saves flash, and reboots into the bootloader. After the command, readback of 0xFC returns **0** (selector remains 0). Do not treat 0x7F as a selector value; tools/scripts should use 0x7F only to trigger OTA.
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
- Selector 0x06: INA219 boot presence page  
  - 0xFD: bitfield (bit0=output INA at 0x40 present, bit1=battery INA at 0x45 present)  
  - 0xFE: 0  
  - 0xFF: 0  
- Selector 0x07: INA219 current age page  
  - 0xFD: output_current_age_10ms (uint8, min(age_10ms, 255))  
  - 0xFE: battery_current_age_10ms (uint8, min(age_10ms, 255))  
  - 0xFF: 0  
- Selector 0x08: Reset cause (this boot)  
  - 0xFD: raw reset-flag byte (RCC CSR bits 31:25): bit6=LPWRRSTF, bit5=WWDGRSTF, bit4=IWDGRSTF, bit3=SFTRSTF, bit2=PORRSTF, bit1=PINRSTF, bit0=BORRSTF  
  - 0xFE–0xFF: CSR bits 23:16 and 31:24 (high 16 bits of RCC_CSR). Client interprets; no firmware normalization.  
- Selector 0x09: Last persisted reset cause (from flash)  
  - 0xFD: last_reset_cause (raw reset-flag byte as stored at last flash save)  
  - 0xFE: last_reset_seq (sequence byte from flash)  
  - 0xFF: reserved (0)  

### 4.5 I2C Bus Robustness
- I2C input filters (analog and digital, 1 I2C clock digital filter) are enabled at init to improve robustness in noisy environments.
- Stuck-bus recovery is performed in software (e.g. SCL toggling); no hardware I2C timeout is used. Recovery behavior is internal and does not change the external I2C register contract.

### 4.6 Current Measurement Behavior
- Output and battery current values come from the INA219 **shunt voltage** register (0x01).
- The raw signed 16-bit shunt value is used directly as milliamps (1 LSB = 1 mA).
- Cached values update **only** on a successful INA read; failed/skip attempts do not overwrite.
- `current_valid` flags indicate freshness within the last **2 seconds** (age <= 200 * 10 ms).
- Ages increment on each 10 ms tick, **saturate** (never wrap), and start at **0xFFFF** on boot.
- Valid flags start at 0 until the first successful read.
- Snapshot values reflect the cached values and validity only; I2C reads never mutate state.
- Sampling is time-sliced and non-intrusive: INA reads are opportunistic and do not preempt
  STM32 slave responsiveness; if bus activity prevents sampling, updates pause until safe.
- Runtime INA reads occur in short master windows: the STM32 temporarily switches
  **slave → master → slave** and restores slave mode immediately after the read.
- Sampling cadence alternates channels at 500 ms intervals (each channel updates ~1 Hz) when
  the master window can be safely entered.

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
- **Full battery detection (hybrid: plateau + optional current taper)** (self-programming enabled):
  - **Definitions:**
    - **Filtered VBAT:** the filtered battery voltage used everywhere in this algorithm.
    - **Plateau window:** over `PLATEAU_WINDOW_SEC`, with tolerance `PLATEAU_DELTA_MV`:
      within the window, `max(VBAT) - min(VBAT) <= PLATEAU_DELTA_MV`. Default
      `PLATEAU_WINDOW_SEC` is **1800 seconds (30 minutes)** and default `PLATEAU_DELTA_MV` is **40 mV**.
    - **Current freshness:** a current measurement is usable only when the corresponding
      `*_current_valid` flag is set (fresh within the validity window).
  - **Primary full criterion (plateau)** — baseline behavior, MUST be supported. While charger
    state is `PRESENT`, the battery is considered **FULL** when:
    1. The plateau condition holds continuously for `PLATEAU_WINDOW_SEC`, and
    2. Filtered VBAT is at or above a near-top minimum, `VBAT_FULL_MIN_MV` (prevents plateauing
       at mid-voltage from being misclassified as full). Recommended **4150–4180 mV**, default **4180 mV**.
  - **Optional secondary criterion (taper gate):** If reliable charge-current telemetry exists
    (e.g. INA219 battery-path current) and the measurement is fresh, plateau qualification MAY
    be tightened by requiring a taper condition during the final portion of the plateau window.
    The taper hold interval is evaluated **concurrently** with the final portion of the plateau
    window. When enabled, the plateau is only accepted as FULL if, in addition to the primary criterion:
    3. Battery charge current is low (taper): battery current is <= `I_TAPER_MA` for at least
       `TAPER_HOLD_SEC` continuously, and the battery-current measurement is fresh
       (`battery_current_valid` set) for all samples contributing to the hold time. Battery charge
       current refers to **positive** (charging) battery current; negative current (discharge) does
       not satisfy the taper condition.
    If current telemetry is not fresh for a sustained period, the taper gate is skipped and the
    decision falls back to voltage plateau only.
  - **Defaults / recommended values:**
    - `VBAT_FULL_MIN_MV`: 4150–4180 mV (default 4180 mV).
    - `I_TAPER_MA`: conservative, e.g. C/20 equivalent (default **150 mA** for ~3000 mAh cells).
    - `TAPER_HOLD_SEC`: 300–600 s (default **300 s**).
  - **Latching and reset semantics:** When FULL is asserted, it is latched until either charger
    state becomes not `PRESENT`, or filtered VBAT drops below `VBAT_FULL_RESET_MV` for
    `FULL_RESET_HOLD_SEC`. Recommended: `VBAT_FULL_RESET_MV = VBAT_FULL_MIN_MV - 100 mV`,
    `FULL_RESET_HOLD_SEC` = **30–60 s**.
  - **Learning update rule (unchanged):** on a plateau event, compute the plateau level as the mean
    of filtered VBAT samples within the window; if `abs(plateau - learned_full) >= PLATEAU_MIN_CHANGE_MV`,
    update learned full (e.g. EMA). Clamp `learned_full` to `[LEARNED_FULL_MIN_MV, LEARNED_FULL_MAX_MV]`.
    Persist only if change >= `PLATEAU_PERSIST_MIN_CHANGE_MV` and at most once per charger-present session.
  - **Safety / robustness:** The taper gate MUST NOT use stale current readings: `*_current_valid` must
    be set. Current taper is a secondary confirmation, not the sole definition of full. The voltage
    plateau algorithm remains the canonical mechanism for “full” if current telemetry is unavailable
    or unreliable.
- **Empty voltage learning (self-programming enabled):** While the charger is not influencing VBAT,
  the firmware tracks the **minimum** battery voltage seen while the load is on (output current above
  `EMPTY_LEARN_OUTPUT_CURRENT_THRESHOLD_MA`). Output/battery current values are derived directly from
  the INA219 shunt-voltage register; on this board the shunt resistor is 10 mΩ so the register count
  equals 1 mA per LSB. The learned empty voltage is committed when the Pi is
  effectively off or about to turn off, detected by any of: **(1)** output current dropping to near zero
  (graceful shutdown / load removed); **(2)** a low-VBAT protection trigger that initiates a pending
  power cut; or **(3)** loss of output-current validity while the system is in `POWER_STATE_RPI_ON`
  (covers abrupt brownouts where telemetry stops before current reaches ~0). The committed value is
  that minimum—i.e. the lowest under-load voltage **before** the Pi turned off, not the rebound voltage
  after load removal. The tracked discharge-session minimum is an internal candidate value; the
  externally visible “empty voltage” (registers 0x0F–0x10) reflects only the last committed learned
  value. If the charger is influencing VBAT, the discharge-session minimum is discarded
  (no learning while charging).
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
- Integrity: record structure version **2**; validation uses **hardware CRC**. Records with older structure version or invalid CRC are rejected at load → defaults applied and state marked dirty.
- CRC and sequence validation performed on load; invalid or wrong-version → defaults + dirty.

---

## 9. GPIO Behavior

- **IP_EN (PA5)**: controls charger path.
- **MT_EN (PA6)**: controls RPi power.
- **PWR_EN (PA7)**: always HIGH.
- **Button (PB1)**: EXTI edge → main-loop FSM.

---

## 10. Reliability and Fail-Safe Behavior

- **Independent Watchdog (IWDG):** Timeout ~8 s (LSI-based). Refreshed **once per main-loop iteration**, after critical work (scheduler, I2C processing, INA probe, flash save, protection/GPIO). **Never** refreshed in ISRs (e.g. I2C ISR); a main-loop hang or I2C deadlock cannot keep the watchdog alive. If the main loop does not complete within the timeout, the device resets.
- **HardFault safe state (prioritize Pi uptime):** On HardFault the handler drives **IP_EN LOW** (charger path off) and keeps **PWR_EN HIGH** (MCU hold-up), but it **does not force MT_EN LOW**. MT_EN is left unchanged to avoid unnecessarily power-cycling the Raspberry Pi if it is otherwise running normally. The handler then triggers an immediate system reset. Note: if the application’s protection logic later determines the battery is below the protection threshold, it will still perform the normal shutdown sequence (attempt flash save, then cut MT_EN).
- **Reset cause:** Captured from RCC at boot (before clear). Persisted in flash and exported via I2C: **factory test selector 0x08** = this boot’s raw reset flags and CSR high bits; **selector 0x09** = last persisted reset cause and sequence from flash. Encoding is raw RCC_CSR bits; client interprets (e.g. IWDGRSTF, PINRSTF, PORRSTF).

---

## 11. Testing and Validation

- Automated I2C test script: `tools/testing/upsplus_i2c_test.py`.
- Manual hardware tests still required for charger transitions, measurement windows, and protection latching.
- Architecture validation tests:
  - I2C coherence under snapshot swaps.
  - ADC gating correctness.
  - Window atomicity edge cases.

---

## 12. Enumerations (ABI-Stable Values)

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

## 13. Non-Goals

- No SOC estimation beyond voltage-based heuristic.
- No fast charger negotiation.
- No dynamic ADC reconfiguration at runtime.
- No interrupt-driven state transitions.

---

## 14. Bootloader Behaviour

### 14.1 Purpose

The device includes a resident I²C bootloader responsible for:

- Conditional entry into OTA / programming mode  
- Erasing and programming the application flash region  
- Booting the application image when permitted  

The bootloader does **not** provide a runtime command to exit OTA mode or to jump directly to the application once OTA mode has been entered.

---

### 14.2 Flash Memory Layout

| Region | Address Range | Notes |
|------|---------------|------|
| Bootloader | `0x08000000 – 0x080007FF` | Resident; never erased |
| Application | `0x08000800 – 0x08003BFF` | Erased and reprogrammed during OTA |
| Device Settings / Persistence | `0x08003C00 – 0x08003FFF` | Erased during OTA |

- Flash page size is **1 KB (0x400 bytes)**.
- OTA erase operations cover the entire range `0x08000800` through `0x08003FFF`.

---

### 14.3 Boot-Time Mode Selection

On reset, the bootloader determines whether to enter OTA mode or boot the application using the following conditions.

#### 14.3.1 Force-Bootloader Button

- A dedicated GPIO input (GPIOB IDR bit 1) is sampled during boot.
- If asserted, the device **enters OTA mode**.
- If not asserted, normal application boot is allowed.

#### 14.3.2 OTA Request Flag (Flash)

- A predefined value stored in the device settings flash page requests OTA mode.
- If present at boot, the bootloader **enters OTA mode**.
- This flag is **cleared implicitly** when the settings page is erased during OTA.

#### 14.3.3 Application Vector Table Validation

Before booting the application, the bootloader validates the application image:

- Reads initial MSP from `0x08000800`
- MSP must be within the valid SRAM range (`0x2000xxxx`)
- Reads reset handler from `0x08000804`
- If validation fails, the device remains in OTA mode

---

### 14.4 Application Boot Sequence

If no OTA condition is present and the application vector table is valid, the bootloader performs the following steps:

1. Load MSP from `0x08000800`
2. Set MSP
3. Load reset handler from `0x08000804`
4. Branch to application reset handler

Application execution begins immediately after this branch.

---

### 14.5 OTA Mode Behaviour

#### 14.5.1 Entry Conditions

The device enters OTA mode at boot if **any** of the following are true:

- Force-boot GPIO is asserted  
- OTA request flag is present in flash  
- Application vector table is invalid  

#### 14.5.2 Flash Erase Behaviour

Upon entering OTA mode, the bootloader:

- Unlocks the FLASH peripheral
- Sequentially erases all 1 KB flash pages from:
  - `0x08000800` through `0x08003FFF`
- This erase includes the **device settings / persistence page**

Erased flash locations read back as `0xFF`.

As a result, the OTA request flag is cleared automatically during the erase process.

---

### 14.6 I²C OTA Programming Interface

- The device operates as an I²C slave at address `0x18`.
- Programming data is delivered using a framed protocol.

High-level behaviour:

- Each programming block begins with a marker byte (`0xFA`)
- Each block programs **16 bytes (8 halfwords)** of flash
- Flash programming is performed using **16-bit halfword writes**
- The flash write pointer auto-increments after each halfword
- Programming continues while valid frames are received

The bootloader does **not** expose:

- A total image length
- A checksum or CRC
- An explicit “end of programming” command

---

### 14.7 OTA Exit and Application Boot After OTA

#### 14.7.1 No In-Band Exit Command

Once OTA mode has been entered:

- There is **no I²C command** to:
  - Exit OTA mode
  - Jump directly to the application
  - Trigger a software reset

OTA mode persists until the device is reset or power-cycled.

#### 14.7.2 Required Exit Mechanism

To boot the application after OTA programming:

1. OTA request flag must be cleared  
   - This occurs automatically during the OTA erase sequence.
2. The device must be **reset or power-cycled**.
3. On reboot, if:
   - No force-boot condition is present, and
   - The application vector table is valid  
   → the bootloader jumps to the application.

---

### 14.8 I²C Register Observations

- The I²C register space is primarily read-only status.
- Register `0x00` reflects current mode/state.
- Internal OTA receive counters are exposed for diagnostics.
- Registers `0xF0–0xFF` expose identity and version information.
- No register functions as a command mailbox for reset or application boot.

---

### 14.9 Design Implications

- OTA workflows **must include a reset or power-cycle step** after programming.
- Application images **must be linked for base address `0x08000800`**.
- The application vector table must remain valid post-OTA.
- Device settings stored in the last flash page are erased during OTA and must be reinitialized by the application.
---

## 15. Glossary

- **true VBAT**: Battery voltage sampled while charger path is disabled (IP_EN LOW).
- **charger present**: Physical charger voltage above presence threshold with stability.
- **charger influencing VBAT**: Charger state is PRESENT at ADC sample time.
- **calibration window active (legacy field `learning_mode_t`)**: Charger forced off for true VBAT sampling and ADC calibration.
- **snapshot**: Coherent, double-buffered register image used for I2C reads.

---

## 16. Change Impact Map

- If ADC cadence changes, revisit: charger stability counters, protection sample count, window duration.
- If register map changes, revisit: Factory Testing ABI, test scripts, and external tools.
- If snapshot frequency changes, revisit: I2C coherence assumptions and staleness guarantees.
- If protection logic changes, revisit: power-cut ordering and flash save semantics.
- If reliability features change (IWDG timeout, HardFault safe outputs, reset-cause encoding), revisit: Section 10, factory test selectors 0x08/0x09, and any tools that interpret reset cause.

---

## 17. Spec Authority

In the event of conflict, this Behavior Specification is authoritative over code comments,
test scripts, and historical behavior.
