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

- Canonical scheduler tick: **10ms** (derived from SysTick; ISR sets flags only).
- Derived tick rates:  
  - 100ms heartbeat  
  - 500ms ADC trigger  
  - 1s counters
- **No measurement window / no true-VBAT-via-disconnect.** An earlier revision periodically
  forced `IP_EN` LOW for 1.5 seconds (`charger_state = FORCED_OFF_WINDOW`) to try to disconnect
  the charger path and sample "true VBAT" free of charger influence. IP_EN was later confirmed
  (datasheet review, then a live current measurement) to be the IP5328's KEY pin, which has no
  charge-gating function at all -- see Section 9 for the full history and Section 7 for how
  battery percent/protection behave without it. This mechanism has been **removed**; there is no
  scheduled measurement window. `sample_period_minutes` (registers `0x15-0x16`) is no longer tied
  to that removed window, but is **not** dead: it now sets the interval for the Section 9.2
  periodic battery-level LED check (see Section 9.2), an unrelated, later-added mechanism.
- **IP5328 periodic reset timer: 20 hours**, running only while `charger_state == PRESENT`. This
  is a distinct, newer mechanism from the removed measurement window above -- see Section 9.1 for
  full rationale and behavior. It exists to avoid the IP5328's own charge safety-timer (`TEND`,
  datasheet min 20h) rather than to sample VBAT.
- **Battery-level LED check (event-driven, not periodic):** a short `IP_EN` pulse triggered by a
  case-button press while running on battery (Section 9.2). Unlike the periodic reset above, this
  has no timer of its own -- it fires immediately on the button event, subject to Section 9.3
  arbitration.
- Countdowns and debounce are driven by 10ms ticks, but only advanced in main loop.
- Snapshot updates are guaranteed at least once every 100ms while the main loop is running.

---

## 4. External Interfaces (I2C)

### 4.1 Register Map (high-level)
- **0x01–0x0C**: Voltages + temperature (RO).
- **0x0D–0x12**: Battery full/empty/protection thresholds (RW, validated).
- **0x13–0x14**: Battery percent (LSB=percent, MSB always 0x00).
- **0x15–0x16**: Sample period minutes (RW). Interval for the Section 9.2 periodic battery-level
  LED check (not related to the removed measurement window -- see Section 3).
- **0x17**: Power status (derived). bit0 = RPi power (`power_state == RPI_ON`, MT_EN effectively
  asserted); bit1 = Section 9.1 periodic IP5328 reset in progress (repurposed from a legacy
  "calibration window active" flag that has been permanently unreachable since Section 6/9's
  removal of true-VBAT-via-disconnect -- see Section 5.3 and Section 9.1 for the current meaning
  and rationale).
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
- Selector 0x03: Charger page
  - 0xFD: charger physically present (0/1)
  - 0xFE: reserved (0x00) -- formerly "window active"; the measurement window feature was
    removed, see Section 9
  - 0xFF: reserved (0x00) -- formerly "window due"
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
- Selector 0x08: IP5328 periodic reset diagnostics page (see Section 9.1)
  - 0xFD: minutes remaining until next scheduled reset, LSB (uint16 LE across 0xFD-0xFE; 0 when
    `charger_state != PRESENT`, i.e. the timer is not running)
  - 0xFE: minutes remaining until next scheduled reset, MSB
  - 0xFF: reset pulse count since boot (uint8, saturates at 255)
  - This selector was previously used for a now-removed IP_EN charge-disconnect current
    diagnostic (see history note below) and is repurposed here for the new mechanism; it was
    free at the time of this change.
- Selector 0x09 (currently free): previously held INA219 guard-open/read-success counts and
  reset-cause diagnostics, used to trace a chronic STM32 I2C reliability problem to an external
  root cause (a NUT driver bug aliasing a reserved core variable name, busy-looping the driver
  ~17x faster than its configured poll interval) rather than anything in this firmware. Freed
  once that question was answered.
- Selectors 0x10 and 0x11 (retry effectiveness, and internal guard-window quiet-time tracking)
  were removed for the same original I2C-reliability reason and remain free.

**Reset cause not reported:** The application does not have access to the bootloader source. The bootloader clears the RCC reset flags (e.g. writes `RMVF=1` to `RCC_CSR`) before jumping to the application, so by the time the app runs the reset cause is already lost. Reset cause is not recorded or reported via I2C or factory test.
### 4.5 I2C Bus Robustness
- I2C input filters (analog and digital, 1 I2C clock digital filter) are enabled at init to improve robustness in noisy environments.
- Stuck-bus recovery is performed in software (e.g. SCL toggling); no hardware I2C timeout is used. Recovery behavior is internal and does not change the external I2C register contract.
- **Stuck-ADDR recovery timeout: ≤ 1 second.** If the I2C ADDR flag remains asserted continuously (master holding the bus waiting for a slow MCU response), the firmware reinitialises the I2C slave peripheral to recover. This timeout must be **≤ 1 second** so that the master (RPi) receives a bus error promptly rather than waiting through a kernel-level I2C timeout (~15–20 s on bcm2835). Shorter recovery reduces the window during which the RPi I2C driver stalls and fails to receive data updates.
- **Flash save must not disable I2C interrupts.** The I2C slave ISR must remain active during flash erase and program operations. A global `__disable_irq()` across the full flash write sequence causes the STM32 to be unresponsive to I2C for tens of milliseconds per write, and repeated writes (e.g. during calibration state changes) can produce sustained I2C timeout storms visible at the master. Flash write operations must be structured so only the minimum required flash-controller critical section disables interrupts, and the I2C IRQ is never masked during the erase/program wait loops.

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
- **Failure retry:** a failed master-mode read (measured in practice as consistently
  BERR/ARLO/OVR) is normal, recoverable multi-master I2C behavior: NUT's poll timer runs independently and doesn't
  coordinate with the STM32 in real time, so the guard's pre-check can reduce collision odds but
  can't eliminate them. Rather than waiting for the next full 500 ms alternation (and only after
  the other channel's turn), the same channel is retried after `INA_PROBE_RETRY_DELAY_TICKS`
  (~50 ms) -- long enough for the colliding transaction to finish. Bounded to
  `INA_PROBE_MAX_RETRIES` (3) attempts, after which the channel falls back to normal alternation
  rather than retrying forever (e.g. if the device is genuinely unresponsive).
- **Guard window (`I2C1_GuardWindowReady()`):** the STM32 tracks the last STOP/ADDR event
  addressed to its own slave address at **two** resolutions in parallel, captured at the same
  ISR moments (`I2C_Slave.c`'s ADDR/STOP handlers):
  - Microsecond (16-bit, free-running TIM3, `I2C1_GetLastStopUs()`/`GetLastAddrUs()`) — wraps
    every ~65.5 ms, so only trustworthy for very short "how long ago" comparisons.
  - Tick (32-bit, 10 ms resolution, `I2C1_GetLastStopTick()`/`GetLastAddrTick()`) — pushed in
    from main.c's `Scheduler_ISR_Tick10ms()` via `I2C1_SetTickCounter()` (since `sched_flags` is
    private to main.c); no realistic wraparound.
  Entry logic, always followed by a live bus-idle-stable-500us GPIO check before actually
  switching to master mode:
  1. If no STM32-addressed activity has ever been observed, wait `I2C_MASTER_IDLE_ALLOW_READ_TICKS`
     (5 s) since boot, then proceed to the live check — a defense-in-depth fallback for true
     standalone operation (no RPi/other I2C master ever present).
  2. Otherwise, once at least `I2C_STOP_SETTLE_TICKS` (1 tick) has passed since the last STOP,
     proceed straight to the live check, with **no** upper bound — tick resolution has no wrap
     ceiling, so trust doesn't need to expire the way it did under a microsecond-only scheme.
  3. Only when still within that first tick (true elapsed time necessarily under TIM3's wrap
     range) does the fine microsecond check apply: require ≥500 us since the STOP event
     (`I2C1_GetLastStopUs()`), per the original 500 us settle-time requirement.
  A master that polls steadily (e.g. NUT's default ~2 s `critical_update_interval`) leaves the
  STM32-addressed traffic clustered at one point in each cycle (per `upsdrv_updateinfo()`'s call
  order, the STM32 register-map read happens last), followed by a genuinely quiet bus for most
  of the remaining cycle — case (2) lets the STM32 use that entire quiet stretch, not just a
  sliver right after its own last transaction, closing what used to be a starvation gap between
  a short post-transaction window and a multi-second full-idle threshold.
  `output_current_valid`/`battery_current_valid` remain 0 whenever none of the above has applied
  recently enough (age > `CURRENT_VALID_AGE_SEC`, 2 s) — mainly in true standalone operation
  before the 5 s fallback is first reached. This is a deliberate constraint: the taper
  full-detection path will not trigger without a master present, and empty-learning falls back
  to assuming load-on whenever `power_state == RPI_ON` and current validity is absent.

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
- `LOAD_ON_DELAY → RPI_ON`: delay elapsed AND battery percent > low threshold, **or** manual short
  press (Section 5.4) subject to the same safety check as `RPI_OFF`/`PROTECTION_LATCHED`, which
  skips the remaining delay immediately.
- `LOAD_ON_DELAY → PROTECTION_LATCHED`: battery voltage drops below protection threshold while delay is in progress.
- `LOAD_ON_DELAY → RPI_OFF`: battery percent falls below low threshold (but battery still above protection threshold).
- `RPI_ON → PROTECTION_LATCHED`: battery below protection voltage for required samples.
- `RPI_ON → RPI_OFF`: manual off (long press).
- `PROTECTION_LATCHED → LOAD_ON_DELAY`: **auto-power-on enabled** AND charger present AND battery percent > low threshold AND battery voltage > protection threshold (+ hysteresis).
- `PROTECTION_LATCHED → RPI_OFF`: charger disconnected or conditions insufficient for power-on. **This transition is not gated on auto-power-on** — the latch must always clear when conditions do not support power-on, regardless of the auto-power-on setting, so the state machine cannot become permanently stuck.
- Battery percent is derived from the raw/filtered ADC VBAT reading directly, whether or not the
  charger is present (see Section 7 -- there is no true-VBAT-via-disconnect mechanism; see
  Section 9 for why).
- The periodic IP5328 reset (Section 9.1) never drives `MT_EN` and is not itself a `power_state_t`
  transition trigger; it is orthogonal to this state machine.

### 5.2 Charger State Machine (`charger_state_t`)
States:
- `ABSENT`
- `PRESENT`

Key behaviors:
- `ABSENT → PRESENT`: post-scaled connector-referenced charger voltage stable above threshold.
  This transition also (re)starts the Section 9.1 periodic-reset countdown at 20 hours.
- `PRESENT → ABSENT`: post-scaled connector-referenced charger voltage stable below threshold.
  This transition cancels the Section 9.1 periodic-reset countdown (no partial credit carried
  into the next charging session).
- The periodic IP5328 reset pulse (Section 9.1) does not itself force a `charger_state`
  transition. `charger_state` continues to be derived solely from the connector-referenced
  voltage measurement, independent of IP5328's internal (KEY-triggered) reset.

`CHARGER_STATE_FORCED_OFF_WINDOW` is a third enum value kept only for Factory Testing ABI
stability (Section 12); the state machine never transitions into it. See Section 9 for why the
measurement window it represented was removed. **Note:** the Section 9.1 periodic reset is a
distinct, unrelated mechanism and does not use or revive this state.

### 5.3 Calibration Window Flag (`learning_mode_t`)
- **Legacy name:** `learning_mode_t` is retained for ABI compatibility.
- Derivation is unchanged: `ACTIVE` iff charger state is `FORCED_OFF_WINDOW`, otherwise
  `INACTIVE`. Since that charger state is never entered (Section 5.2), this always reads
  `INACTIVE` in practice.
- **Register 0x17 bit1 no longer reports this field.** It previously did (as "calibration window
  active"), but since `learning_mode` is permanently `INACTIVE`, that bit was always 0 in
  practice. Bit1 has been repurposed to report the unrelated Section 9.1 periodic IP5328 reset
  instead (see Section 9.1 and Section 4.1) — this is a deliberate ABI change, not a bug: the old
  meaning was already permanently dead, and the new meaning gives external tools a real,
  observable signal (correlating a transient `battery_current_mA` swing to an expected cause)
  where the old one gave none. `learning_mode` itself is still computed as described above and is
  still readable via Factory Testing selector `0x01` byte 3 (Section 4.4) for anyone who still
  needs to confirm it reads `INACTIVE`.

### 5.4 Button FSM
- Debounced at 50ms.
- Short press: power on if in `RPI_OFF`, `PROTECTION_LATCHED`, **or** `LOAD_ON_DELAY` (in the
  latter case, immediately skips the remaining delay rather than waiting for it to elapse),
  subject to a safety check (battery above protection threshold).
- Short press while `power_state == RPI_ON`: triggers the Section 9.2 battery-level LED check if
  `charger_state == ABSENT`; no-op if `charger_state == PRESENT` (LEDs already visible via
  IP5328's own charging-mode display).
- Long press (>= 10s):
  - If ON → power off.
  - If OFF → factory reset.

---

## 6. Measurement Window (removed)

This section previously specified a 1.5-second periodic window (`charger_state =
FORCED_OFF_WINDOW`) that forced `IP_EN` LOW to disconnect the charger path and sample "true
VBAT" free of charger influence. It has been **removed**: `IP_EN` was confirmed (datasheet
review, then a live current measurement) to be the IP5328's KEY pin, which has no charge-gating
function, so the window never achieved its stated purpose and instead risked spurious IP5328
button presses (forcing the boost output on) every `sample_period_minutes`. See Section 9 for
the full investigation and Section 7 for how battery percent/protection behave without it.

There is currently no mechanism to isolate VBAT from charger influence; percent and protection
decisions use the raw/filtered ADC VBAT reading at all times, accepting reduced accuracy while a
charger is connected (Section 7).

**Not to be confused with Section 9.1.** The periodic IP5328 reset introduced in Section 9.1 also
drives `IP_EN`, but for an entirely different reason (avoiding the IP5328's charge safety-timer
fault) and with an entirely different hold time (≥10 s, the datasheet's full-reset action) rather
than the 1.5 s short-press pulse used by this removed feature. It does not sample VBAT and does
not reintroduce `FORCED_OFF_WINDOW`.

---

## 7. Battery Management

- Battery percent is based on full/empty calibration, computed from the raw/filtered ADC VBAT
  reading directly (`battery_voltage_mv`) whether the charger is present or not -- there is no
  true-VBAT-via-disconnect mechanism (Section 6/9), so no separate stale/fresh gating applies;
  reduced accuracy while charging is accepted.
- Battery percent update *direction* still uses charger state (charging vs. not), not VBUS
  voltage: percent only moves up while charging and down while not, preventing noise-driven
  reversals.
- Protection voltage is enforced with hysteresis (50mV). Protection is only evaluated while the
  charger is absent (`Protection_Step` skips entirely while `Charger_IsInfluencingVBAT()` is
  true) -- this was already independent of the true-VBAT mechanism and is unchanged by its
  removal.
- Protection latch requires multiple ADC samples below threshold (3 samples).
- **Full battery detection (plateau OR taper, independent paths)** (self-programming enabled):
  - **Overview:** FULL is declared when **either** the plateau path **or** the taper path
    independently satisfies its conditions (OR semantics). Both paths run concurrently; whichever
    triggers first declares FULL. The learned full-voltage calibration value is updated only from
    the plateau path (taper path has no plateau mean to learn from).
  - **Definitions:**
    - **Filtered VBAT:** the IIR-smoothed battery voltage used in both paths.
    - **Plateau window:** over `PLATEAU_WINDOW_SEC`, with tolerance `PLATEAU_DELTA_MV`:
      within the window, `max(VBAT) - min(VBAT) <= PLATEAU_DELTA_MV`. Default
      `PLATEAU_WINDOW_SEC` is **1800 seconds (30 minutes)** and default `PLATEAU_DELTA_MV` is **40 mV**.
    - **Current freshness:** a current measurement is usable only when the corresponding
      `*_current_valid` flag is set (fresh within the validity window).
  - **Plateau path:** While the charger state is not `ABSENT`, the battery is considered **FULL** when:
    1. The plateau condition holds continuously for `PLATEAU_WINDOW_SEC`,
    2. The plateau mean is at or above `PLATEAU_MEAN_MIN_MV` (prevents mid-voltage plateau
       misclassification; default **4150 mV**), and
    3. Filtered VBAT is at or above `VBAT_FULL_MIN_MV` (near-top guard; default **4180 mV**).
  - **Taper path:** Independent of plateau window stability. The battery is considered **FULL** when:
    1. `|battery_current_mA| <= I_TAPER_MA` continuously for `TAPER_HOLD_SEC`, with
       `battery_current_valid` set for every accumulating tick (stale/invalid readings reset
       the hold timer to zero), and
    2. Filtered VBAT is at or above `VBAT_FULL_MIN_MV` (same near-top guard as the plateau path).
    The taper window accepts both positive (charge) and negative (discharge) current within
    `+/-I_TAPER_MA`: slight power draws from the battery occur during normal operation and are
    consistent with a full-charge state. If `battery_current_valid` is never set (no INA219
    present, or no I2C master driving reads -- see Section 4.6), the taper path never triggers.
  - **Defaults / recommended values:**
    - `VBAT_FULL_MIN_MV`: 4150-4180 mV (default **4180 mV**).
    - `PLATEAU_MEAN_MIN_MV`: **4150 mV** (plateau-path prerequisite only).
    - `I_TAPER_MA`: conservative, e.g. C/20 equivalent (default **150 mA** for ~3000 mAh cells).
    - `TAPER_HOLD_SEC`: 300-600 s (default **300 s**).
  - **Latching and reset semantics:** When FULL is asserted (by either path), it is latched until
    the charger state becomes `ABSENT` (physical charger removal), or filtered VBAT drops below
    `VBAT_FULL_RESET_MV` for `FULL_RESET_HOLD_SEC`.
    Recommended: `VBAT_FULL_RESET_MV = VBAT_FULL_MIN_MV - 100 mV`, `FULL_RESET_HOLD_SEC` = **30-60 s**.
  - **Learning update rule:** On a plateau path event, compute the plateau level as the mean of
    filtered VBAT samples within the window and update learned full (EMA) toward that mean. Clamp
    `learned_full` to `[LEARNED_FULL_MIN_MV, LEARNED_FULL_MAX_MV]`. Persist only if change >=
    `PLATEAU_PERSIST_MIN_CHANGE_MV` and at most once per charger-present session. The taper path
    does **not** trigger a learning update.
  - **Interaction with the Section 9.1 periodic reset:** the reset pulse does not clear the FULL
    latch, does not reset the plateau/taper timers, and is not itself treated as a charger
    disconnect (`charger_state` is unaffected -- Section 5.2). No special-casing is required here.
- **Empty voltage learning (self-programming enabled):** While the charger is not influencing VBAT,
  the firmware tracks the **minimum** battery voltage seen while the load is on (output current above
  `EMPTY_LEARN_OUTPUT_CURRENT_THRESHOLD_MA`). Output/battery current values are derived directly from
  the INA219 shunt-voltage register; on this board the shunt resistor is 10 mΩ so the register count
  equals 1 mA per LSB. The learned empty voltage is committed when the Pi is
  effectively off or about to turn off, detected by either: **(1)** output current dropping to near zero
  (graceful shutdown / load removed); or **(2)** a low-VBAT protection trigger that initiates a pending
  power cut. (A third condition — inferring an abrupt brownout from output-current-validity loss
  and/or I2C master silence — was considered and removed: neither signal reliably indicates a dead
  Pi. Validity loss usually means the I2C bus is *busy*, i.e. a master is actively transacting.
  I2C silence alone is equally unsafe: it's indistinguishable from "NUT isn't running" while the
  Pi is otherwise healthy. Condition (1) already uses the one signal that's actually meaningful —
  measured current — and with the guard window's `I2C_STOP_SETTLE_TICKS` fix letting the INA219
  probe get prompt readings regardless of whether NUT is polling, it's sufficient on its own.)
  The committed value is
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

- Single-slot persistence (1KB page at `0x08003C00`).
- Dirty state is periodically flushed (60s) and on critical events.
- Flash write attempts are rate limited unless bypassed.
- **I2C interrupt must remain enabled during flash save.** The flash erase and program sequence must not use `__disable_irq()` across the full operation. Only the minimum flash-controller critical section (FLASH_CR_PG set / BSY poll / FLASH_CR_PG clear per halfword, and the page erase trigger) requires atomicity; the I2C ISR must be able to fire between halfword writes. Disabling all interrupts for the entire erase+program duration (tens of milliseconds) prevents I2C slave responses and causes timeout storms at the master during periods of frequent flash activity.
- Integrity: record structure version **3**; validation uses **hardware CRC** (STM32 polynomial `0x04C11DB7`, init `0xFFFFFFFF`, no reflection). Records with an older structure version or invalid CRC are rejected at load → defaults applied and state marked dirty.
- CRC and sequence validation performed on load; invalid or wrong-version → defaults + dirty.

### 8.1 Flash Storage Format (Stride-2)

The application persists settings using a **stride-2** format: each byte of the `flash_persistent_data_t` struct occupies one 16-bit halfword in flash, with the data byte in the LSByte and `0xFF` (erased state) in the MSByte.

```
Flash address:  base+0   base+1   base+2   base+3   ...  base+N*2
Contents:       data[0]  0xFF     data[1]  0xFF     ...  data[N]
```

**Why stride-2?** The resident bootloader reads and writes the settings page in exactly this format (one byte per halfword), using its internal 255-byte buffer indexed by halfword position. If the application were to pack two struct bytes per halfword (the default STM32 halfword-write pattern), the bootloader's save-restore cycle during OTA would destroy all odd-indexed struct bytes, corrupting the magic number and all multi-byte fields. Using the same stride-2 format ensures the bootloader can round-trip the page without corruption.

**OTA flag placement:** The `bootloader_ota_flag` field sits at struct byte offset `0x32`. With stride-2 storage this maps to flash address `0x08003C00 + 0x32 × 2 = 0x08003C64`, which is the address the bootloader checks at boot time. The mapping is enforced by a compile-time `STATIC_ASSERT`.

**CRC scope:** The CRC covers the configuration and runtime-state fields only (`full_voltage_mv` through `charging_time_sec`). The header (magic, version, sequence), the CRC field itself, and the OTA padding + flag are all excluded from the CRC calculation, making the OTA flag self-clearing without invalidating the record.

**Storage size:** Each struct byte uses two flash bytes, so the total flash footprint is `sizeof(flash_persistent_data_t) × 2`. A compile-time assert verifies this fits within the 1 KB settings page.

**Reserved fields:** two `uint16_t` fields formerly held the true-VBAT-at-last-sample value and
its age (for the since-removed true-VBAT-via-disconnect mechanism, see Section 9). They remain in
the struct, unused and always written 0, rather than being deleted -- removing them would shift
every field after them, including `bootloader_ota_flag`, requiring the `FLASH_OTA_FLAG_OFFSET`
assert and the bootloader's hardcoded `0x08003C64` check to be re-derived by hand with no way to
verify the result without hardware. Existing records with real values in those bytes still load
fine (layout and CRC region are unchanged); the values are just no longer read into anything.

---

## 9. GPIO Behavior

- **IP_EN (PA5)**: **normally HIGH.** Physically drives the IP5328's **KEY** pin (button-detect
  input, IP5328 pin 26, multiplexed with the WLED flashlight drive) — confirmed against the IP5328
  datasheet, timing table below, and against a live current measurement. **This is not a
  charger-enable gate of any kind.** Per the datasheet's "Charge/discharge path management"
  section, KEY only controls the output boost stage (VOUT1/VOUT2/USB-C), the battery-level LED
  display, and the WLED flashlight — it has **no effect on the VBUS→battery charge input path**.
  The firmware never drives this pin LOW to gate charging; there is no mechanism to disconnect the
  charger path (see history below and Section 6). The only firmware-initiated LOW-going activity
  on this pin is the deliberate, datasheet-confirmed full-reset action described in Section 9.1.
- **MT_EN (PA6)**: controls RPi power.
- **PWR_EN (PA7)**: always HIGH.
- **Button (PB1)**: EXTI edge → main-loop FSM.

**IP5328 KEY pin timing (confirmed from datasheet Section 6, "按键"/Button):**

| IP_EN LOW duration / pattern | IP5328 action                                                     |
|-------------------------------|--------------------------------------------------------------------|
| < 30 ms                       | Ignored (debounce)                                                 |
| 60 ms – 2 s (short press)      | Turns on battery-level LEDs + boost output. Forces VOUT1 on regardless of load; VOUT2/USB-C only if a load is present. If VOUT2/USB-C is in fast-charge mode, a short press first disables fast-charge (a 2nd short press >1s later forces VOUT1 on instead). |
| > 2 s (long press)             | Toggles WLED flashlight on/off                                    |
| Two short presses within 1 s   | Turns off boost output, LED display, and WLED                     |
| ≥ 10 s                         | Full IP5328 reset                                                  |

Electrical timing: short-press wake/debounce (`T_OnDebounce`) typ. 500 ms (min 60 ms); WLED
turn-on time (`T_Keylight`) typ. 2 s (range 1.2–3 s). In the locked/low-battery-lockout state, KEY
cannot wake the chip at all — it must first enter charging mode to become active again.

**History — how this was found and fixed:**
1. The original vendor firmware pulsed this pin periodically in patterns commented as
   "retrigger the button" and "flicker-disconnect," which read as KEY-pin button-press emulation
   rather than a charger-enable gate.
2. An earlier revision of *this* firmware held `IP_EN` LOW for the entire `CHARGER_STATE_ABSENT`
   duration (continuously, for as long as the unit ran on battery with no charger connected),
   reasoning there was no charger path to gate anyway. Under the KEY-pin model that held a
   spurious "button pressed" state indefinitely — worse than a brief pulse, since ABSENT can last
   indefinitely and would repeatedly force output-on clicks, WLED toggles, and eventually (past
   10 s continuously LOW) a full IP5328 reset. Fixed by holding `IP_EN` HIGH during `ABSENT` too.
3. The firmware also periodically forced `IP_EN` LOW for 1.5 s (`CHARGER_STATE_FORCED_OFF_WINDOW`,
   every `sample_period_minutes` while charging) intending to disconnect the charger path for
   true-VBAT sampling (Section 6, old). The IP5328 datasheet's KEY timing table above shows a
   1.5 s pulse is a "short press," which only forces the boost output on and lights the
   battery-level LEDs — it never touches the charge input path. A live measurement confirmed
   this: `battery_current_mA` sampled immediately before vs. during the pulse tracked each other
   within normal measurement noise across multiple pulses while charging (e.g. +294 mA → +291 mA;
   later, after the load shifted, +267 mA → +269 mA) — IP_EN going LOW produced no measurable
   change. **Fixed by removing `FORCED_OFF_WINDOW`/true-VBAT-via-disconnect entirely** (Section
   6, 7): `IP_EN` is now held HIGH unconditionally outside of the Section 9.1 reset pulse, and
   percent/protection decisions use the raw ADC VBAT reading directly instead of a disconnected
   sample. `CHARGER_STATE_FORCED_OFF_WINDOW` and `LEARNING_ACTIVE` remain declared (unreachable)
   for Factory Testing ABI stability (Section 12).

### 9.1 Periodic IP5328 Reset (Charge Safety-Timer Avoidance)

**Rationale.** The IP5328 datasheet specifies a charge safety timer (`TEND`) of 20/24/27 hours
(min/typ/max): if a charge cycle does not register the battery-side charge current dropping below
the ~300 mA termination threshold (`ISTOP`) before this elapses, the datasheet documents no
automatic recovery path. On this board the RPi (and its TPS61088 boost converter) is powered from
IP5328's `VREG` pin, which draws continuously regardless of charge state; this continuous
downstream load is suspected to prevent the IP5328 from ever observing a clean battery-side
termination current, so a charge session can run out the `TEND` timer instead of completing
normally. Once that happens, the datasheet gives no indication the charger recovers on its own —
observed field behavior is that the charger silently stops accepting/relaying input power even
though the charger remains physically connected, and the battery is then drawn down by the
continuing RPi load with no replenishment, eventually reaching the low-battery protection
threshold and forcing an RPi shutdown. This is a known workaround pattern in other IP5328-based
UPS firmware (periodically resetting the chip whenever "charger present, not charging" is
detected) — here it is applied proactively on a timer rather than reactively on detection, since
this firmware has no visibility into IP5328's internal charge-complete/fault status (its I2C bus
is not connected — Section 4, hardware notes).

**Mechanism.** The datasheet's own KEY-pin timing table above already documents that holding KEY
(i.e. `IP_EN`) LOW for **≥ 10 s** triggers a full IP5328 reset. This mechanism reuses that
confirmed, existing chip behavior for a new purpose — it does **not** reintroduce
`FORCED_OFF_WINDOW` (Section 6), which used a much shorter 1.5 s pulse for an unrelated,
already-disproven purpose (VBAT sampling via charger disconnect).

**Timing.**
- A countdown timer runs only while `charger_state == PRESENT` (Section 5.2).
- `ABSENT → PRESENT`: countdown (re)starts at **20 hours**.
- `PRESENT → ABSENT`: countdown is cancelled; no partial credit carries into the next session.
- On expiry (20 continuous hours in `PRESENT`): the reset pulse (below) is performed, then the
  countdown immediately restarts at 20 hours — repeating every 20h for as long as the charger
  remains continuously connected.
- **Margin note:** 20h equals the datasheet's `TEND` *minimum* spec, not a safety margin below it.
  A unit whose IP5328 happens to sit at the low end of part-to-part tolerance could reach its own
  internal timeout at, or an instant before, this firmware's reset fires. If false negatives (a
  `TEND` fault slipping through before the scheduled reset) are observed in the field, tighten
  this constant (e.g. 18h) to restore margin; 20h is used here to match the explicit requirement
  driving this feature.

**Reset pulse sequence.**
1. Drive `IP_EN` LOW.
2. Hold LOW for `IP5328_RESET_HOLD_MS` = 10,500 ms (10 s datasheet minimum + 500 ms margin, since
   the datasheet gives no upper tolerance on the "≥10 s" threshold and hold time is not expected
   to be timing-critical).
3. Drive `IP_EN` back HIGH (the pin's normal idle state).
4. `charger_state` is **not** force-transitioned by this action (Section 5.2) — it continues to be
   derived independently from the connector-referenced voltage measurement, so a
   still-physically-connected charger reads `PRESENT` again as soon as the countdown restarts.

**Acknowledged side effects (per the KEY timing table above).** The 10.5 s hold necessarily passes
through the short-press window (forces VOUT1 boost output on, lights battery-level LEDs) and the
long-press/WLED-toggle window en route to the full-reset threshold. On this board:
- **RPi power is unaffected:** it is sourced from IP5328's `VREG` pin via the TPS61088 boost
  converter — documented as always-on and independent of the KEY/boost-output switching logic —
  **not** from VOUT1/VOUT2/VBUS, which are what the short/long press actions above actually
  toggle. No load is present on VOUT1/VOUT2/VBUS on this board, so that part of the sequence has
  no externally visible effect.
- **The battery-level LED bar (L1/L2/L3) is confirmed populated on this board, and is lit
  continuously and automatically by IP5328 while charging** — independent of any KEY action; see
  Section 9.2. Because this reset only ever fires while `charger_state == PRESENT` (Timing,
  above), the LED bar will almost always be actively displaying charge progress at the moment the
  reset runs. **Correction from an earlier draft of this section:** this is therefore an
  externally visible event, not a silent one. Expect the charging-progress LED display to glitch,
  blank, or briefly show a discharge-style pattern during the 10.5 s hold, then resume its normal
  charging display once IP5328 has restarted and re-detects the charger. This happens once per
  20h cycle and is accepted as a tradeoff for avoiding the `TEND` fault, but should not be assumed
  invisible to anyone looking at the unit when it occurs.
- Any populated WLED flashlight LED may also blip during the long-press window; treated as
  cosmetic on the same basis.

**Open validation item.** It has not been confirmed on hardware whether `VREG` itself is
interrupted for any portion of an IP5328-internal reset triggered this way. The datasheet's
"always-on" language for VREG describes its exemption from load-detection/KEY-press
*output-switching* logic specifically, not necessarily its behavior across a full chip reset. If
VREG does briefly drop during the reset pulse, this mechanism would cause exactly the RPi power
interruption it exists to avoid, and `MT_EN` sequencing would need to be revisited (e.g. holding
`MT_EN` regardless, and/or scoping actual RPi 5V-rail continuity during a manually-triggered test
pulse before relying on this in the field).

**Battery current side effect and external visibility (register `0x17` bit1).** If IP5328's
internal input-path gating (Q1/Q2, VING/VBUSG) briefly interrupts the charger→battery path as part
of its own reset sequence, the continuous downstream load on `VREG` (RPi, via TPS61088) has to be
sourced from somewhere in that instant — plausibly the battery itself, momentarily. This would
show up as `battery_current_mA` swinging to a discharge (negative) reading for some portion of the
10.5 s hold, even though the charger remains physically connected and `charger_state` never leaves
`PRESENT`. This reading is genuinely valid (a real, if transient, physical current), not a fault or
a stale/invalid sample — `battery_current_valid` is deliberately left unaffected by this mechanism.
Because this could otherwise look like an anomaly to anything polling `battery_current_mA` (e.g.
NUT, a monitoring dashboard), register `0x17` bit1 is set for the duration of the reset hold so an
external observer can attribute the blip to this known, expected cause. See Section 5.3 and
Section 4.1 for the register-level detail (this bit was repurposed from an already-permanently-dead
legacy "calibration window active" flag) and Section 12 for the bit's numeric position.

**Guard conditions.** The reset pulse is only initiated when `charger_state == PRESENT` (per
Timing above). It does not depend on `power_state` and never drives `MT_EN` — it is orthogonal to
the Power State Machine (Section 5.1). See Section 9.3 for arbitration with the other
firmware-initiated drivers of `IP_EN`.

### 9.2 Battery-Level LED Check (Short KEY Press or Periodic, While Discharging)

**Purpose.** With the L1/L2/L3 battery-level LED bar confirmed populated on this board, a brief
KEY (`IP_EN`) short press can be used to trigger IP5328's native battery-level LED display on
demand — reusing datasheet-defined chip behavior instead of building a separate display driver in
firmware. Two independent triggers request the same pulse: a manual button press, and an automatic
periodic timer (below).

**Why gated on `charger_state == ABSENT`.** While charging, the LED bar is already lit
continuously by IP5328's own autonomous charging-mode display logic (solid segments per completed
~25% tier, current tier blinking at 1.5 Hz — independent of any KEY action, see Section 9.1's
corrected side-effects note). A KEY press in that state would be redundant, and would needlessly
force VOUT1's boost stage on for no benefit. This feature therefore only triggers while
`charger_state == ABSENT`, i.e. genuinely running on battery, where the LED bar is otherwise off
and the datasheet's discharge-mode display table applies instead.

**Manual trigger.** Case Button (PB1) short press while `power_state == RPI_ON` **and**
`charger_state == ABSENT` — previously an unspecified/no-op case; see Section 5.4 for the full
Button FSM.
- Short press while `power_state == RPI_ON` **and** `charger_state == PRESENT`: remains a defined
  no-op (LEDs are already visible; nothing is triggered).
- Short press while `power_state == LOAD_ON_DELAY`: out of scope for this feature -- it instead
  triggers the manual power-on override in Section 5.1/5.4 (skips the remaining delay), not a
  KEY pulse.

**Periodic trigger.** In addition to the manual button press, the LED check also fires
automatically on a timer while `power_state == RPI_ON` **and** `charger_state == ABSENT`, once
every `sample_period_minutes` (registers `0x15-0x16`; validated 1-1440, floor-clamped to
`DEFAULT_SAMPLE_PERIOD_MIN` = 2 minutes) -- giving a user a periodic, passive glance at battery
level without needing to press the button, purely by reusing this otherwise-idle register (see
Section 3).
- The interval timer arms (starts counting from a full interval) whenever the
  `RPI_ON` + `ABSENT` condition is freshly (re-)entered, whether via `power_state` transitioning
  to `RPI_ON` or `charger_state` transitioning to `ABSENT`. No partial credit carries over from a
  prior charging or powered-off period, mirroring Section 9.1's baseline-arming semantics.
- The timer does not advance while the condition does not hold (charging, or RPi off); it neither
  fires nor loses progress during that time -- it simply pauses, and the next entry into the
  condition restarts a fresh interval (it does not resume a partially-elapsed one).
- Both triggers request the same underlying pulse and share the same arbitration (Section 9.3): if
  the periodic timer's interval elapses while a manual (or the other) request already owns `IP_EN`,
  the periodic request is dropped for that occurrence (not queued) and simply retried next
  interval, same as a dropped manual button press.

**Mechanism** (applies to either trigger).
1. Drive `IP_EN` LOW.
2. Hold for `KEY_LED_CHECK_HOLD_MS` = 200 ms (comfortably inside the datasheet's 60 ms–2 s
   short-press window, clear of both the 30 ms debounce floor and the 2 s long-press boundary).
3. Drive `IP_EN` back HIGH.
4. No further firmware action is taken — IP5328's own discharge-mode LED table takes over display
   of the current battery level.

**Self-clearing; no explicit "off" step.** The LED display rides on the same VOUT1 boost
activation the short press forces. Since nothing loads VOUT1 on this board, IP5328's own
light-load auto-shutdown (~16–32 s, see Section 9.1's rationale) closes that port again on its
own, taking the LED display down with it. Firmware does not need to issue a follow-up
double-short-press "turn off" sequence.

**No effect in the low-battery locked state.** As noted in Section 9, KEY cannot wake IP5328 out
of its locked state — it must first enter charging mode to become active again. This feature is
gated on `charger_state == ABSENT`, so it would not normally be invoked while the chip is locked
(low-battery lockout implies `power_state == PROTECTION_LATCHED`, not `RPI_ON`), but if it ever
is, the pulse silently does nothing chip-side. This is expected, not a bug.

**IP_EN arbitration.** See Section 9.3 — this is the second of three firmware-initiated drivers of
`IP_EN` and must share the same mutual-exclusion mechanism as Section 9.1's periodic reset (Section
10's HardFault safe state is also a driver of `IP_EN` but, as Section 9.3 explains, bypasses this
mechanism rather than sharing it).

### 9.3 IP_EN Arbitration

`IP_EN` has three independent firmware-initiated drivers:
- Section 9.1: periodic IP5328 reset (~10.5 s LOW, every 20h while charging).
- Section 9.2: battery-level LED check (~200 ms LOW, on button press or the periodic timer while
  discharging).
- Section 10: HardFault safe-state handler (~10.5 s LOW hold -- the same full-reset duration as
  Section 9.1, forced directly by the fault handler itself -- immediately followed by MCU reset).

The main-loop drivers (Sections 9.1/9.2) must go through a single shared ownership mechanism (e.g.
a main-loop `ip_en_busy` flag or equivalent) so overlapping requests cannot interleave and corrupt
each other's hold timing — e.g. a button press arriving mid-reset must not shorten, extend, or
restart the reset pulse's hold time, and vice versa. Recommended arbitration policy:
- A Section 9.2 request (manual or periodic) arriving while `IP_EN` is busy is **dropped, not
  queued** — a missed LED check is harmless; the user can press again, or the periodic timer will
  simply try again next interval.
- A Section 9.1 reset that becomes due while `IP_EN` is busy (e.g. with an in-progress LED check)
  should proceed as soon as the bus frees, not be indefinitely postponed — do not let a nuisance
  button press delay the `TEND`-avoidance reset.
- Section 10's HardFault handler does **not** go through this main-loop mechanism at all — the
  main loop cannot run during a HardFault, so the handler drives IP_EN and holds it directly
  itself (feeding the IWDG in place of the main loop for the duration of the hold; see Section 10)
  and executes unconditionally regardless of any pending arbitration state. It resets the MCU
  immediately afterward, so no other driver can meaningfully be "in progress" across a HardFault
  in practice, and the main-loop `ip_en_owner`/equivalent state is simply reinitialized on reboot.

Given Section 9.1 fires at most once per 20h and completes in ~10.5 s, and Section 10 immediately
resets the MCU, real overlap is expected to be extremely rare — but the guard should exist
regardless of how unlikely.

---

## 10. Reliability and Fail-Safe Behavior

- **Independent Watchdog (IWDG):** Timeout ~8 s (LSI-based). Refreshed **once per main-loop
  iteration**, after critical work (scheduler, I2C processing, INA probe, flash save,
  protection/GPIO). **Never** refreshed in ISRs (e.g. I2C ISR); a main-loop hang or I2C deadlock
  cannot keep the watchdog alive. If the main loop does not complete within the timeout, the
  device resets. **Sole exception:** the Section 10 HardFault handler, which deliberately feeds
  the IWDG itself while it holds `IP_EN` LOW for ~10.5 s -- otherwise the ~8 s timeout would fire
  mid-hold and reset the MCU before the IP5328's reset threshold is reached, cutting the pulse
  short. This is safe because a HardFault handler that reaches this point is already committed to
  resetting the MCU immediately afterward regardless -- feeding the watchdog here cannot mask a
  hung main loop, it only sequences the fault handler's own bounded, fixed-length cleanup.
- **HardFault safe state (force IP5328 reset, prioritize Pi uptime):** On HardFault the handler
  holds `IP_EN` LOW for ~10.5 s -- the same full-reset duration and mechanism as Section 9.1's
  periodic reset (KEY held >= 10 s per the IP5328 datasheet's Section 6 timing table) -- timed via
  the free-running TIM3 counter (unaffected by interrupt masking) and feeding the IWDG directly
  during the hold (see above), then releases `IP_EN` back HIGH before triggering an immediate
  system reset. `PWR_EN` is kept HIGH throughout (MCU hold-up); the handler **does not force
  MT_EN LOW** -- MT_EN is left unchanged to avoid unnecessarily power-cycling the Raspberry Pi if
  it is otherwise running normally. This deliberately forces the IP5328 itself back to a known
  state after a HardFault, on the reasoning that a fault severe enough to reach this handler may
  also have left the power-management path in an indeterminate state; it supersedes an earlier
  revision of this handler that drove `IP_EN` LOW only momentarily (too brief to register as any
  IP5328 KEY action, and originally written under the mistaken belief that `IP_EN` gated the
  charger -- see Section 9). This bypasses the Section 9.3 main-loop arbitration mechanism
  entirely (the main loop cannot run during a HardFault); see Section 9.3 for why that is safe.
  Note: if the application's protection logic later determines the battery is below the
  protection threshold, it will still perform the normal shutdown sequence (attempt flash save,
  then cut MT_EN).
- **Reset cause:** The bootloader (which we do not have access to) clears the RCC reset flags before starting the application, so the actual reset cause cannot be read by the app. The firmware does not record or expose reset cause via I2C; the corresponding flash fields are reserved (written as 0) for layout compatibility.

---

## 11. Testing and Validation

- Automated I2C test script: `tools/testing/upsplus_i2c_test.py`.
- Manual hardware tests still required for charger transitions and protection latching.
- Architecture validation tests:
  - I2C coherence under snapshot swaps.
  - ADC gating correctness.
- `tools/testing/upsplus_i2c_test.py` and `tools/testing/upsplus_state_machine_interactive.py`
  both contain measurement-window/`FORCED_OFF_WINDOW`-specific assertions and a stale
  reset-cause-at-selector-0x08/0x09 test predating this firmware's removal of that feature; these
  test scripts have not yet been updated to match Sections 6/9's removal of the measurement
  window and should be revisited. They also predate Section 9.1's reuse of selector 0x08 for the
  periodic-reset diagnostics page and will need updating for that as well.
- **New tests needed for Section 9.1:** countdown start/cancel on `ABSENT↔PRESENT` transitions;
  reset pulse fires at exactly 20h of continuous `PRESENT` and restarts the countdown afterward;
  `IP_EN` hold duration measured at ≥10.5 s; `charger_state` and `MT_EN` are unaffected by the
  pulse; selector 0x08 minutes-remaining counts down correctly and the reset-count byte
  increments and saturates; register `0x17` bit1 reads 1 for the duration of the reset hold and 0
  otherwise (including during a Section 9.2 LED-check pulse, which must **not** set this bit);
  `battery_current_valid` remains set (unaffected) throughout the reset even if
  `battery_current_mA` reads a discharge value during the hold.
- **New tests needed for Section 9.2:** short press while `RPI_ON`+`ABSENT` produces a ~200 ms
  `IP_EN` LOW pulse; short press while `RPI_ON`+`PRESENT` produces no pulse; short press while
  `LOAD_ON_DELAY` produces no `IP_EN` pulse (it instead triggers the Section 5.1/5.4 manual
  power-on override, skipping the remaining delay — verify `power_state` transitions to `RPI_ON`
  and no KEY action occurs); battery-level LED display (manual/visual check) clears on its own
  within the light-load auto-shutdown window without further firmware action; a press issued
  while IP5328 is in its low-battery locked state produces the `IP_EN` pulse (firmware cannot
  detect chip-internal lock state) but has no visible chip-side effect — document as expected, not
  a failure; the periodic timer produces a pulse at exactly `sample_period_minutes` after entering
  `RPI_ON`+`ABSENT` (and again every `sample_period_minutes` thereafter) with no button involved;
  the timer does not fire and does not accumulate progress while charging or while the RPi is off;
  re-entering `RPI_ON`+`ABSENT` after leaving it restarts a full interval rather than resuming a
  partially-elapsed one; a write to `sample_period_minutes` while an interval is already counting
  down does not retroactively change that interval (the new value is picked up only the next time
  the timer re-arms: on the next periodic fire or the next fresh entry into `RPI_ON`+`ABSENT`).
- **New tests needed for Section 9.3:** a button press arriving while the Section 9.1 reset pulse
  is in progress is dropped, not interleaved, and does not alter the reset pulse's hold time; a
  due Section 9.1 reset proceeds as soon as `IP_EN` frees rather than being postponed indefinitely
  by repeated button presses.
- **New tests needed for Section 10 (HardFault):** a forced HardFault (e.g. via a deliberate
  invalid memory access in a debug build) results in an `IP_EN` LOW hold measured at ≥10.5 s
  followed by `IP_EN` returning HIGH and an MCU reset; the IWDG does not fire mid-hold (confirms
  the handler's direct feed keeps it fed); `MT_EN` is left unchanged throughout; `PWR_EN` remains
  HIGH throughout.

---

## 12. Enumerations (ABI-Stable Values)

- `power_state_t`:  
  - `RPI_OFF=0`, `RPI_ON=1`, `PROTECTION_LATCHED=2`, `LOAD_ON_DELAY=3`
- `charger_state_t`:  
  - `ABSENT=0`, `PRESENT=1`, `FORCED_OFF_WINDOW=2` (unreachable at runtime, see Section 9; kept
    for ABI stability)
- `learning_mode_t`:  
  - `INACTIVE=0`, `ACTIVE=1` (`ACTIVE` unreachable since `FORCED_OFF_WINDOW` is, see Section 5.3)
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
| Device Settings / Persistence | `0x08003C00 – 0x08003FFF` | Saved to RAM before erase; restored after (OTA flag cleared) |

- Flash page size is **1 KB (0x400 bytes)**.
- OTA erase operations cover all 14 pages from `0x08000800` through `0x08003FFF`.
- The device settings page is erased as part of this operation but is saved to RAM first and restored afterwards; see Section 14.5.2.

---

### 14.3 Boot-Time Mode Selection

On reset, the bootloader determines whether to enter OTA mode or boot the application using the following conditions, evaluated **in order**:

1. OTA request flag present at `0x08003C64 == 0x7F` → **enter OTA mode**
2. Force-boot button asserted (GPIOB IDR bit 1 low) → **enter OTA mode**
3. Application MSP at `0x08000800` fails SRAM validation → **enter OTA mode**
4. All checks pass → **boot the application**

#### 14.3.1 Force-Bootloader Button

- A dedicated GPIO input (GPIOB IDR bit 1, active-low) is sampled during boot.
- If asserted (pin low), the device **enters OTA mode**.
- If not asserted, normal application boot is allowed.

#### 14.3.2 OTA Request Flag (Flash)

- The OTA request flag is a single byte at flash address `0x08003C64` (device settings page, offset `0x64`).
- The flag value `0x7F` requests OTA mode; any other value is ignored.
- If the flag is set at boot, the bootloader **enters OTA mode**.
- The flag is **cleared** as part of OTA entry: the bootloader reads the settings page into RAM, zeroes the flag byte (`settings_buf[0x32] = 0`), erases the flash page, then writes the settings back. The flag does not persist after the first OTA boot.

#### 14.3.3 Application Vector Table Validation

Before booting the application, the bootloader validates the application image:

- Reads initial MSP from `0x08000800`
- MSP must satisfy `(MSP & 0x2FFE0000) == 0x20000000` (confirms address is within valid SRAM)
- Reads reset handler from `0x08000804`
- If validation fails, the device remains in OTA mode
- **Note:** erased flash reads as `0xFFFFFFFF`; `0xFFFFFFFF & 0x2FFE0000 = 0x2FFE0000 ≠ 0x20000000`, so a device with no programmed application loops in OTA mode indefinitely until a valid image is received.

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

#### 14.5.2 Flash Erase and Settings Preservation

Upon entering OTA mode, the bootloader executes the following sequence:

1. Reads the device settings page (`0x08003C00`, 255 bytes) into RAM using its stride-2 format (LSByte of each halfword), **zeroing the OTA flag byte** (buffer index `0x32`, flash address `0x08003C64`).
2. Unlocks the FLASH peripheral (keys `0x45670123` / `0xCDEF89AB`).
3. Sequentially erases all 14 × 1 KB flash pages from `0x08000800` through `0x08003FFF`, including the device settings page.
4. Writes the saved settings back to `0x08003C00` (with OTA flag cleared to `0x00`).
5. Sets the flash write pointer to `0x08000800` and begins the OTA receive loop.

Erased flash locations read back as `0xFF`. The OTA request flag is zeroed in RAM before the erase begins, so it does not persist regardless of erase outcome. Device settings (battery thresholds, calibration, etc.) are **preserved** across OTA through this save-restore mechanism.

---

### 14.6 I²C OTA Programming Interface

- The device operates as an I²C slave at address `0x18`, 100 kHz standard mode.
- Programming data is delivered using a framed block protocol.

Block format (17 bytes total):

| Byte | Value | Description |
|------|-------|-------------|
| 0 | `0xFA` | Block marker |
| 1–16 | data | 8 × 16-bit halfwords, **LSB-first** per halfword |

High-level behaviour:

- Each programming block begins with the marker byte `0xFA`, followed by exactly 16 data bytes.
- Each block programs **16 bytes (8 halfwords)** of flash starting at the current write pointer.
- Flash programming is performed using **16-bit halfword writes**; the write pointer auto-increments by 2 after each halfword.
- Programming starts at `0x08000800` and continues sequentially.
- The bootloader accepts blocks continuously while valid frames are received; it returns to the application boot check after a long inactivity timeout.

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
- Register `0x00` is set to `0xC8` when OTA mode is active.
- Internal OTA receive counters are exposed for diagnostics.
- Registers `0xF0–0xFB` expose the MCU's 96-bit unique device ID (12 bytes read from hardware UID registers at `0x1FFFF7AC`).
- Registers `0xFC–0xFF` are unused by the bootloader (read as `0x00`).
- No register functions as a command mailbox for reset or application boot.

---

### 14.9 Design Implications

- OTA workflows **must include a reset or power-cycle step** after programming.
- Application images **must be linked for base address `0x08000800`**.
- The application vector table must remain valid post-OTA.
- Device settings stored in the last flash page are **preserved across OTA**: the bootloader saves the page to RAM before erasing and restores it after programming, with only the OTA flag cleared. The application does not need to reinitialize settings after OTA unless its own CRC/version validation fails.
- Once OTA mode is entered the application is immediately erased. If OTA programming is interrupted or the device is power-cycled before a complete valid image is received, the device will remain in OTA mode on subsequent boots (MSP validation will fail for erased flash). A complete image must be programmed before normal boot can resume.
---

## 15. Glossary

- **true VBAT** (historical): battery voltage intended to be sampled while the charger path was
  disconnected. No mechanism to disconnect the charger path exists (Section 9), so this concept
  has been removed from the firmware; percent/protection use the raw ADC VBAT reading directly at
  all times.
- **charger present**: Physical charger voltage above presence threshold with stability.
- **charger influencing VBAT**: charger is actively charging (`charger_state = PRESENT`) at ADC
  sample time; used to gate percent-update direction and empty-voltage learning, not to select a
  different VBAT source (there is only one VBAT source now).
- **calibration window active (legacy field `learning_mode_t`)** (historical): previously ACTIVE
  while the charger was forced off for true-VBAT sampling. That mechanism is gone (Section 9), so
  this always reads INACTIVE; the field is retained only for Factory Testing ABI stability
  (selector `0x01` byte 3, Section 4.4). Its former I2C-visible signal, register `0x17` bit1, has
  been repurposed for an unrelated meaning -- see Section 5.3 and Section 9.1.
- **snapshot**: Coherent, double-buffered register image used for I2C reads.
- **TEND** (IP5328 datasheet term): the IP5328's charge safety timer, 20/24/27h (min/typ/max). If
  a charge cycle never registers battery-side current below `ISTOP` (~300 mA) before this
  elapses, the datasheet documents no automatic recovery. Mitigated in this firmware by the
  Section 9.1 periodic IP5328 reset.
- **ISTOP** (IP5328 datasheet term): the battery-side charge-current threshold (~300 mA typ.)
  below which the IP5328 considers a charge cycle complete and stops charging. Relevant to
  Section 9.1's rationale: a continuous downstream load on `VREG` may prevent this threshold from
  ever being observed.

---

## 16. Change Impact Map

- If the scheduler timebase (10 ms) source changes (e.g. SysTick vs TIM), update Section 3 and all comments referring to the interrupt source.
- If ADC cadence changes, revisit: charger stability counters, protection sample count.
- If register map changes, revisit: Factory Testing ABI, test scripts, and external tools.
- If snapshot frequency changes, revisit: I2C coherence assumptions and staleness guarantees.
- If protection logic changes, revisit: power-cut ordering and flash save semantics.
- If reliability features change (IWDG timeout, HardFault safe outputs), revisit Section 10 and any tools that depend on factory test selectors.
- If the IP5328 periodic reset period, hold duration, or trigger condition changes, revisit
  Section 3 (timing model), Section 9.1, Section 4.4 selector 0x08, and the Section 11 test list.
  If IP5328's actual `TEND` behavior is ever confirmed via register-level access (would require
  wiring its I2C bus to the MCU, a hardware change), reassess whether a fixed timer is still
  needed or whether reactive detection becomes possible.
- If the battery-level LED check's trigger condition or hold duration changes, or if `IP_EN`
  gains a fourth firmware-initiated driver, revisit Section 3, Section 5.4, Section 9.2,
  Section 9.3 (arbitration policy), and the Section 11 test list.

---

## 17. Spec Authority

In the event of conflict, this Behavior Specification is authoritative over code comments,
test scripts, and historical behavior.
