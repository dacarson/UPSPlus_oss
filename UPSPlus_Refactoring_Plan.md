# UPSPlus Firmware Refactoring Plan

## Document Purpose
This document outlines the implementation plan for refactoring the UPSPlus firmware according to the specified rules. This plan does not modify any code but provides a structured approach to implementing the required changes.

## Table of Contents
1. [Architecture Overview](#architecture-overview)
2. [State Machine Design](#state-machine-design)
3. [I2C Register Map Implementation](#i2c-register-map-implementation)
4. [Data Flow and Snapshot Management](#data-flow-and-snapshot-management)
5. [Timing and Scheduling](#timing-and-scheduling)
6. [Flash Persistence](#flash-persistence)
7. [Pin Behavior and GPIO Management](#pin-behavior-and-gpio-management)
8. [Battery Management](#battery-management)
9. [Button Handling](#button-handling)
10. [Implementation Phases](#implementation-phases)

---

## Architecture Overview

### Core Principles
1. **Single Source of Truth**: One authoritative state structure that is atomically snapshotted for I2C reads
2. **Explicit State Machine**: All system behavior driven by documented state transitions
3. **Canonical Scheduler**: TIM1 10ms interrupt serves as the single timebase for all periodic operations
4. **ISR Safety**: No flash writes in ISRs; minimal work in ISRs; defer to main loop
5. **Data Coherence**: Multi-byte registers must be read from a single atomic snapshot

**Future extensibility guard**: Any future features added to this firmware MUST:
- Consume ADC data only via `adc_sample_seq`
- Expose new multi-byte registers only through the snapshot mechanism
- Avoid adding logic to ISRs beyond flag setting

This prevents architectural erosion.

### System Components
- **I2C Slave**: Register map interface (configuration must not change)
- **ADC + DMA**: Battery voltage/temperature sensing (configuration must not change)
- **TIM1**: 10ms canonical scheduler tick (configuration must not change)
- **SysTick**: Free-running counter for uptime tracking (modifiable)
- **GPIO**: 3 outputs (IP_EN, MT_EN, PWR_EN), 1 input (button on PB1)
- **EXTI**: Button interrupt (configuration must not change)

---

## State Machine Design

### System State Dimensions

The system uses **orthogonal state dimensions** to avoid contradictory transitions. The state is composed of three independent dimensions that can coexist:

#### 1. Power State (RPi Power Control)

Controls whether power is delivered to the RPi (MT_EN pin).

```c
typedef enum {
    POWER_STATE_RPI_OFF,           // RPi power disabled (MT_EN = LOW)
    POWER_STATE_RPI_ON,             // RPi power enabled (MT_EN = HIGH)
    POWER_STATE_PROTECTION_LATCHED, // Protection voltage reached, power latched off
    POWER_STATE_LOAD_ON_DELAY       // Waiting for delay before enabling power
} power_state_t;
```

**Transitions**:
- `RPI_OFF` → `LOAD_ON_DELAY`: Battery % > Low Battery % AND AutoPowerOn enabled AND (charger present OR battery sufficient)
  - **"Battery sufficient"** means `battery_percent > low_battery_percent` computed from the most recent non-charging voltage sample
  - **Deterministic staleness rule**: If `charger_state == CHARGER_STATE_PRESENT` and the last true-VBAT sample age exceeds 10 minutes, the system must force a measurement window using the following scheduling rule:
    - **Forced measurement window scheduling**: When staleness criteria are met, the system MUST schedule a measurement window at the next scheduler opportunity where:
      - No window is currently active, and
      - `charger_state == CHARGER_STATE_PRESENT`
    - The window MUST begin no later than the next `tick_10ms` evaluation cycle.
    - Defer any auto-power-on decision until a fresh true-VBAT sample is captured and battery_percent is updated from it.
  - **Battery percent staleness enforcement**: Any power decision that depends on percent must also require `last_true_vbat_sample_tick` to be "fresh enough" if the charger is present. This is a hard gate in `CheckPowerOnConditions()`.
- `LOAD_ON_DELAY` → `RPI_ON`: Delay elapsed AND conditions still met
- `LOAD_ON_DELAY` → `RPI_OFF`: Conditions no longer met (battery dropped, charger disconnected)
- `RPI_ON` → `PROTECTION_LATCHED`: Battery voltage ≤ Protection Voltage (with hysteresis, N consecutive samples)
- `RPI_ON` → `RPI_OFF`: Button short press (toggle)
- `PROTECTION_LATCHED` → `LOAD_ON_DELAY`: Charger connected AND battery % > threshold AND AutoPowerOn enabled
- `PROTECTION_LATCHED` → `RPI_OFF`: Charger disconnected or conditions not met

#### 2. Charger State (Input Power Control)

Controls whether charger input is enabled (IP_EN pin) and measurement windows.

```c
typedef enum {
    CHARGER_STATE_ABSENT,              // Physical absent and path disabled (IP_EN default LOW, may be overridden after characterization if required for stable operation)
    CHARGER_STATE_PRESENT,              // Physical present and path enabled (IP_EN = HIGH)
    CHARGER_STATE_FORCED_OFF_WINDOW     // Physical present but path disabled (IP_EN = LOW for 1.5s during measurement window)
    // Note: FORCED_OFF_WINDOW is the only state where a charger may be physically plugged in (VBUS high) while IP_EN is LOW, allowing true VBAT measurement
} charger_state_t;
```

**Charger present vs charger enabled**: The `charger_state_t` enum blends physical detection and path control:
- `ABSENT`: Physical absent and path disabled
- `PRESENT`: Physical present and path enabled
- `FORCED_OFF_WINDOW`: Physical present but path disabled

**Implementation separation**: In code, physical detection is the input, not the state itself:
- `charger_present_bool` derived from ADC of VBUS/USBIN via thresholds + stability
- A separate "window manager" decides whether IP_EN is forced low
- This prevents the subtle bug where "unplug during window" must be remembered until window ends

**Canonical VBAT truth rule**: A "true VBAT" sample exists if and only if `charger_state != CHARGER_STATE_PRESENT`. This condition is defined by charger path influence, not by VBUS voltage level or RPi load state.

**Transitions**:
- `ABSENT` → `PRESENT`: Charger voltage > present_on threshold (with hysteresis and stability)
- `PRESENT` → `FORCED_OFF_WINDOW`: Sample period elapsed AND charger_state == PRESENT
  - **Note**: Measurement window active is equivalent to `charger_state == CHARGER_STATE_FORCED_OFF_WINDOW`. No separate boolean needed.
  - **Implementation**: Transition only evaluated when `charger_state == PRESENT`, so cannot retrigger while already in window (state machine enforces this).
  - **Measurement window scheduling rule**: Only schedule a new window if:
    - `charger_state == PRESENT`
    - AND `window_due == 1`
    - AND `charger_state_entry_ticks` etc. indicate no active window
    - Set `window_due=0` only when you actually transition into `FORCED_OFF_WINDOW`
    - This prevents constantly reasserting "due" every 10ms (window storm prevention)
  - **Note**: Measurement windows are required whenever a charger is connected and would otherwise influence VBAT. True VBAT can only be measured when the charger is not influencing the battery node—i.e., either (a) no charger is connected (CHARGER_STATE_ABSENT), or (b) a charger is connected but the charger path is disabled during the measurement window (CHARGER_STATE_FORCED_OFF_WINDOW). RPi load state is irrelevant to VBAT accuracy.
  - **Semantic invariant**: `CHARGER_STATE_FORCED_OFF_WINDOW` is the only state where a charger may be physically connected (VBUS/USBIN ADC reads high) while IP_EN is driven LOW, enabling a true-VBAT sample even though VBUS remains present.
  - **Measurement window semantics**:
    - Measurement windows are never interruptible
    - Measurement windows are never nested
    - Measurement windows are never restarted early, even if:
      - Charger is unplugged mid-window
      - Auto-power-on conditions change
      - A config write arrives
    - **Edge case**: If a charger is unplugged during `CHARGER_STATE_FORCED_OFF_WINDOW`, the state machine MUST:
      - Complete the window timing
      - Transition to `CHARGER_STATE_ABSENT` afterward
      - Treat all samples during the window as valid true-VBAT samples
- `FORCED_OFF_WINDOW` → `PRESENT`: 1.5s elapsed, restore charger
- `PRESENT` → `ABSENT`: Charger voltage < present_off threshold (with hysteresis and stability)

**Charger Detection Thresholds**:
- `present_on_threshold`: 4200mV (charger detected when voltage exceeds this)
- `present_off_threshold`: 3800mV (charger removed when voltage drops below this)
- `stability_samples`: 3 consecutive samples required before state change

**Threshold Units & Scaling**: `present_on_threshold` and `present_off_threshold` are expressed in the same units as `uVBUSVolt` / `uUSBINVolt` after the existing ADC scaling (i.e., mV at the connector input as computed by the current conversion macros). Because `uVBUSVolt`/`uUSBINVolt` use different scaling than `uVBATVolt` (e.g., *4 vs *2), the divider ratios must be confirmed so the thresholds are applied to the correct physical voltage domain. This is a documentation/verification step only and does not change behavior.

**Threshold invariant**: All charger thresholds (4200mV / 3800mV) are defined in post-scaled, connector-referenced millivolts as produced by the existing ADC conversion macros. Under no circumstances may these thresholds be applied to raw ADC counts or pre-scaled values.

**Stability sample qualification (canonical)**: Stability counters must increment only on new processed ADC samples. Implementation must be independent of task ordering by using a per-consumer last-seen sequence:
- `uint32_t charger_last_seq;`
- On each evaluation:
  - If `adc_sample_seq != charger_last_seq`, set `charger_last_seq = adc_sample_seq` and evaluate/advance `charger_stability_count`.
  - Otherwise, do nothing (same ADC sample; do not count).

**Charger Detection Logic** (run every 10ms in main loop, but count only on ADC updates):

**Charger detection channel rule**: Charger detection MUST use the maximum of `uVBUSVolt` and `uUSBINVolt` (post-scaling). The presence of either input above threshold is sufficient to indicate charger presence.

```c
// In charger detection logic (main loop):
static uint32_t charger_last_seq = 0;
if (adc_sample_seq != charger_last_seq) {
    charger_last_seq = adc_sample_seq;
    
    uint16_t charger_voltage_mv = MAX(uVBUSVolt, uUSBINVolt);  // Use higher of the two
    
    // evaluate thresholds and update charger_stability_count
    if (charger_state == CHARGER_STATE_ABSENT) {
        if (charger_voltage_mv > present_on_threshold) {
            system_state.charger_stability_count++;
            if (system_state.charger_stability_count >= stability_samples) {
                charger_state = CHARGER_STATE_PRESENT;
                system_state.charger_stability_count = 0;
            }
        } else {
            system_state.charger_stability_count = 0;  // Reset on change
        }
    } else if (charger_state == CHARGER_STATE_PRESENT) {
        if (charger_voltage_mv < present_off_threshold) {
            system_state.charger_stability_count++;
            if (system_state.charger_stability_count >= stability_samples) {
                charger_state = CHARGER_STATE_ABSENT;
                system_state.charger_stability_count = 0;
            }
        } else {
            system_state.charger_stability_count = 0;  // Reset on change
        }
    }
    // apply hysteresis and state transitions when stability_samples reached
    // FORCED_OFF_WINDOW transitions are handled by measurement window timing logic
}
```

#### 3. Battery Learning Mode (Flag)

Indicates whether the system is actively learning battery parameters.

```c
typedef enum {
    LEARNING_INACTIVE,  // Not learning (Battery Parameters Self-Programmed = 1)
    LEARNING_ACTIVE     // Learning battery Full/Empty voltages (Battery Parameters Self-Programmed = 0)
} learning_mode_t;
```

**Transitions**:
- `INACTIVE` → `ACTIVE`: Battery Parameters Self-Programmed (0x2A) = 0
- `ACTIVE` → `INACTIVE`: Battery Parameters Self-Programmed (0x2A) = 1 (user override only)

**Note**: Learning mode becomes active immediately when 0x2A = 0, but actual learning updates (updating Full/Empty voltage values) only occur when non-charging voltage samples are available (during measurement windows or when charger absent).

### Combined State Machine Structure

```c
typedef struct {
    power_state_t power_state;
    charger_state_t charger_state;
    learning_mode_t learning_mode;
    
    // State timing
    uint32_t power_state_entry_ticks;
    uint32_t charger_state_entry_ticks;
    uint32_t measurement_window_start_ticks;
    
    // Protection state
    uint8_t protection_sample_count;  // Consecutive samples below threshold
    uint8_t charger_stability_count;  // Consecutive samples for charger detection
    uint32_t charger_last_seen_seq;  // Last seen adc_sample_seq for charger detection (for independent tracking)
    
    // Factory reset flag (temporary, not a persistent state)
    uint8_t factory_reset_requested;
} system_state_t;
```

### Register 0x17 Power Status Derivation

The I2C register 0x17 value is derived from the combined state:

```c
uint8_t GetPowerStatusRegisterValue(system_state_t *state) {
    // Precedence: calibration mode (0x2A=0) takes precedence over power state
    // 0x17 reports operation mode, not purely "power state"
    if (state->learning_mode == LEARNING_ACTIVE) {
        return 0x2;  // Learning/Calibration enabled (0x2A=0): device periodically takes charger-off measurements for true VBAT
        // Note: Returns 0x2 even if MT_EN is HIGH (RPi powered) when calibration mode is enabled
    } else if (state->power_state == POWER_STATE_RPI_ON) {
        return 0x1;  // Power to RPi
    } else {
        return 0x0;  // No power to RPi
    }
}
```

**Client Interpretation**: Register value 0x2 means **Learning/Calibration enabled** (0x2A = 0), not "measuring right now". In this mode, the device periodically enters a charger-off measurement window to capture true battery terminal voltage. Those samples are used to:
- Update battery percentage (only computed from non-charging samples)
- Optionally learn full/empty voltage bounds over time (if Battery Parameters Self-Programmed = 0)
- Maintain measurement accuracy (periodic VREFINT-based VDD calibration; windowed VBAT sampling for battery calculations)

**Precedence Rule**: Register 0x17 reports operational mode, not instantaneous electrical state. It MUST NOT be interpreted as "measurement in progress" or "charger disabled at this instant." If calibration mode is enabled (0x2A=0), return 0x2 even if MT_EN is HIGH (RPi powered). 

**Surprising case documentation**: Register 0x2 is returned even if MT_EN is HIGH. This is correct per the "mode" definition, but ensure client code (Pi side) won't treat 0x2 as "power off". If you later want both signals:
- Keep 0x17 as "mode"
- Add a new status register bitfield for:
  - MT_EN state
  - charger_state
  - window active

Clients should display "Learning/Calibration Mode" status when reading 0x2 from register 0x17. Note that 0x2 indicates the mode is enabled, not that a measurement window is currently active at this exact instant.

---

## I2C Register Map Implementation

### Register Map Structure

All registers must be implemented according to the specification table. Key requirements:

1. **Little-Endian Encoding**: Multi-byte registers store LSB at lower address
2. **Read-Only Enforcement**: RO registers must ignore writes
3. **Bounds Checking**: Invalid writes must be rejected
4. **Atomic Snapshot**: Multi-byte registers read from single snapshot
5. **No Read Side Effects**: I2C read operations must never modify authoritative state, counters, timers, or flash persistence. Reads are strictly passive.

### Register Implementation Plan

#### Register Categories

**Voltage Measurements (RO)**
- 0x01-0x02: MCU Voltage (uAVDDVolt)
- 0x03-0x04: Pogopin Bottom Voltage (uPIVCCVolt)
- 0x05-0x06: Battery Terminal Voltage (uVBATVolt)
- 0x07-0x08: USB-C Voltage (uVBUSVolt)
- 0x09-0x0A: MicroUSB Voltage (uUSBINVolt)

**Temperature (RO)**
- 0x0B-0x0C: Battery Temperature (uADCdegC)

**Battery Parameters (RW)**
- 0x0D-0x0E: Full Voltage (uVBATMax) - Range: 0-4500mV, Default: 4200mV
- 0x0F-0x10: Empty Voltage (uVBATMin) - Range: 0-4500mV, Default: 3000mV
- 0x11-0x12: Protection Voltage (uVBATProtect) - Range: 2800-4500mV, Default: 2800mV

**Note**: When Battery Parameters Self-Programmed (0x2A) = 0, the system learns Full/Empty voltages from actual measurements. Until learning completes, the defaults are used. Learning only occurs during non-charging voltage samples (during measurement windows).

**Status (RO)**
- 0x13-0x14: Battery Remaining % (uVBATPrecentReal)
- 0x17: Power Status/Operation Mode - Flags: 0x0 = no power to RPi, 0x1 = power to RPi, 0x2 = Learning/Calibration Enabled (Battery Parameters Self-Programmed = 0). This indicates the device will periodically perform charger-off measurement windows to obtain true-VBAT samples; it does not mean a measurement window is active at this instant.

**Configuration (RW)**
- 0x15-0x16: Sample Period - Range: 1-1440 minutes, Default: 2 minutes
- 0x18: Shutdown Countdown - Range: 0 (inactive) or 10-255 seconds. **When active, shows current countdown value** (decrements on tick_1s)
- 0x19: Back-To-AC Auto Power up - Range: 0/1, Default: 0
- 0x1A: Restart Countdown - Range: 0 (inactive) or 10-255 seconds. **When active, shows current countdown value** (decrements on tick_1s)
- 0x1B: Reset to Factory Defaults - Range: 0/1, Default: 0
- 0x2A: Battery Parameters self-programmed - Range: 0/1, Default: 0
- 0x2B: Low Battery Percentage - Range: 0-100%, Default: 10%
- 0x2C-0x2D: Load On Delay - Range: 0-65535 seconds (16-bit), Default: 60 seconds. **When countdown is active, shows remaining seconds** (decrements on tick_1s)

**Note**: This is a 2-byte register (little-endian), so the maximum value is 65535 seconds, not 2147483647.

**Runtime Counters (RO)**
- 0x1C-0x1F: Cumulative Running Time (RuntimeAtAll)
- 0x20-0x23: Accumulated Charging Time (RuntimePowerOn)
- 0x24-0x27: Running Time (RuntimeOnetime)

**Version and Serial (RO)**
- 0x28-0x29: Version
- 0xF0-0xFB: Serial Number (Device UID)

**Reserved**
- 0x2E-0xEF: Reserved (RW but no defined function)
- 0xFC-0xFF: Factory Testing (RW)

**I2C reserved regions write behavior**: If accepting writes to reserved regions, decide:
- Do they go into a scratch buffer?
- Do they persist?
- Do they affect snapshot?

**Recommended behavior** (simplest and safest):
- Reads return 0x00 (or whatever the legacy returns)
- Writes are accepted but discarded (ACK but no storage)

This preserves compatibility without storing unknown state.

### Register Validation Functions

```c
// Bounds checking for each register type
bool ValidateFullVoltage(uint16_t value);      // 0-4500mV
bool ValidateEmptyVoltage(uint16_t value);     // 0-4500mV, must be >= Protection
bool ValidateProtectionVoltage(uint16_t value); // 2800-4500mV
bool ValidateSamplePeriod(uint16_t value);     // 1-1440 minutes
bool ValidateCountdown(uint8_t value);         // 0 or 10-255
bool ValidateLowBatteryPercent(uint8_t value); // 0-100%
bool ValidateLoadOnDelay(uint16_t value);      // 0-65535 seconds (16-bit)
```

### Register Read Helper Functions

The `GetPowerStatusRegisterValue()` function is defined in the State Machine section (see Register 0x17 Power Status Derivation).

### Register Write Handler

**Authoritative State Ownership Rule**: Only the main loop updates the authoritative state. I2C ISR never directly modifies authoritative state.

**I2C Write Flow**:
1. **I2C ISR**: Receives write data, stores bytes into a small "pending write" buffer, sets `i2c_write_pending` flag
2. **Main Loop**: Checks `i2c_write_pending` flag, applies the write:
   - Validates bounds/clamps values
   - Checks RO enforcement
   - Updates authoritative state (not snapshot)
   - Marks register as dirty for flash persistence
   - Updates snapshot
   - Clears `i2c_write_pending` flag

**Multi-byte Register Handling**:
- For 16-bit registers (e.g., 0x0D-0x0E), both bytes are stored in pending buffer
- Main loop validates and applies both bytes atomically
- This prevents race conditions from partially-written registers

```c
// Pending write buffer (in I2C handler)
typedef struct {
    uint8_t reg_addr;
    uint8_t data[4];  // Max 4 bytes for any register
    uint8_t length;
    volatile uint8_t pending;
} i2c_pending_write_t;

// Main loop processes pending writes
void ProcessI2CWrites(void) {
    if (i2c_pending_write.pending) {
        // Validate, clamp, apply to authoritative state
        // Update snapshot
        i2c_pending_write.pending = 0;
    }
}
```

### I2C RW Register Write Semantics

Special registers have specific write semantics beyond simple bounds checking:

**0x18 - Shutdown Countdown**:
- **Read behavior**: Returns current countdown value (10-255) if active, or 0 if inactive
- Write 0: Cancels any active countdown immediately (register reads back as 0)
- Write 10-255: Starts countdown from that value (seconds), or updates if already running
- Write 1-9: Ignored (invalid, minimum is 10)
- Countdown decrements on `tick_1s` (once per second), when it reaches 1, power is turned off and register reads back as 0

**0x1A - Restart Countdown**:
- **Read behavior**: Returns current countdown value (10-255) if active, or 0 if inactive
- Write 0: Cancels any active countdown immediately (register reads back as 0)
- Write 10-255: Starts countdown from that value (seconds), or updates if already running
- Write 1-9: Ignored (invalid, minimum is 10)
- When countdown reaches 5: Power cycle RPi (MT_EN LOW for 5s, then HIGH)
- When countdown reaches 1: Power on RPi if off, then register reads back as 0

**0x1B - Reset to Factory Defaults**:
- Write 0: No action (normal operation)
- Write 1: Triggers factory reset immediately (all parameters reset to defaults, flash updated)
- Register auto-clears to 0 after reset completes
- This is the only write that bypasses rate limiting

**0x2A - Battery Parameters Self-Programmed**:
- Write 0: Enables auto-learning mode (system learns Full/Empty voltages)
- Write 1: Disables learning, uses user-provided Full/Empty values
- Changing from 1→0 resets learned values and starts fresh learning
- Changing from 0→1 preserves current learned values as user-set values

**0x2C-0x2D - Load On Delay** (16-bit, little-endian):
- **Read behavior**: 
  - When countdown inactive (power_state != LOAD_ON_DELAY): Returns `load_on_delay_config_sec` (configured delay value, 0-65535)
  - When countdown active (power_state == LOAD_ON_DELAY): Returns `load_on_delay_remaining_sec` (remaining seconds, decrements on tick_1s)
- Write 0: No delay, power on immediately when conditions met
- Write 1-65535: Sets `load_on_delay_config_sec` in seconds before power-on when conditions are met
- Takes effect immediately (next state machine evaluation)
- If countdown is already active, new value resets the countdown (sets `load_on_delay_remaining_sec` = new value)
- When countdown completes, power_state changes and register reads back as `load_on_delay_config_sec`

---

## Data Flow and Snapshot Management

### Authoritative State Structure

```c
typedef struct {
    // Voltage measurements (updated by main loop from raw ADC data)
    uint16_t mcu_voltage_mv;
    uint16_t pogopin_voltage_mv;
    uint16_t battery_voltage_mv;
    uint16_t usbc_voltage_mv;
    uint16_t microusb_voltage_mv;
    uint16_t temperature_celsius;
    
    // Battery parameters
    uint16_t full_voltage_mv;
    uint16_t empty_voltage_mv;
    uint16_t protection_voltage_mv;
    uint8_t battery_percent;  // Published value
    uint8_t battery_percent_last_true_vbat_tick;  // For debug/staleness/logic (tick when percent was last updated from true-VBAT)
    uint8_t battery_params_self_programmed;
    uint8_t low_battery_percent;
    
    // Configuration
    uint16_t sample_period_minutes;
    uint8_t shutdown_countdown_sec;  // Current countdown value (0=inactive, 10-255=active countdown in seconds)
    uint8_t auto_power_on;
    uint8_t restart_countdown_sec;   // Current countdown value (0=inactive, 10-255=active countdown in seconds)
    uint16_t load_on_delay_config_sec;    // Configured delay value (0-65535 seconds)
    uint16_t load_on_delay_remaining_sec;  // Remaining seconds when countdown active (0 when inactive)
    
    // Status
    uint8_t power_status;  // Maps to I2C register 0x17: 0x0=no power, 0x1=power on, 0x2=calibration mode enabled
    // Note: Only a minimal, stable subset of system_state_t is mirrored into the snapshot for I2C.
    // Internal FSM state (entry ticks, counters, flags) is NOT exposed via snapshot to prevent
    // treating internal implementation details as API.
    system_state_t operation_mode;
    
    // Runtime counters
    uint32_t cumulative_runtime_sec;
    uint32_t charging_time_sec;
    uint32_t current_runtime_sec;
    
    // Version
    uint16_t version;
    
    // Timestamps for snapshot
    uint32_t snapshot_tick;  // TIM1 tick when snapshot was taken
    uint32_t last_true_vbat_sample_tick;  // TIM1 ticks when last true-VBAT sample was captured
} authoritative_state_t;
```

### Snapshot Buffer

```c
// Double-buffered snapshot for I2C reads
typedef struct {
    authoritative_state_t buffer[2];
    volatile uint8_t active_buffer;  // 0 or 1
    volatile uint32_t snapshot_version;  // Monotonic snapshot counter (debug-only, not used for coherence)
} snapshot_buffer_t;
```

### Snapshot Update Strategy

**Chosen Approach: Double Buffering with Version Counter**

The snapshot mechanism uses double buffering to provide lock-free, atomic reads for I2C register access:

- **Two Snapshot Buffers**: Maintain two copies of the authoritative state
- **Active Buffer Pointer**: Volatile pointer indicates which buffer I2C reads from
- **Version Counter**: Monotonic snapshot counter (debug-only, not used for coherence)
- **Snapshot version counter non-role**: `snapshot_version` exists for debugging and diagnostics only. It MUST NOT be used for correctness, coherence, or retry logic in production code.
- **Writer Process**: Updates inactive buffer, then atomically swaps active buffer pointer
- **Reader Process**: **I2C coherence rule**: On address match / start of an I2C transaction, latch `snapshot_buf.active_buffer` into a transaction-local variable (e.g., `latched_buf`) and serve all bytes for that transaction from `snapshot_buf.buffer[latched_buf]` until STOP. This guarantees multi-byte register coherence without version checking.

**I2C snapshot lifetime**:
- The active snapshot buffer MUST be latched:
  - On ADDR match (preferred), or
  - On receipt of the register index byte (acceptable fallback)
- The latched buffer MUST remain in use until STOPF
- Repeated-start without STOP MUST NOT re-latch

**STM32F0 I2C transaction robustness**: On STM32F0 I2C slave, use explicit transaction variables in the I2C ISR:
- `uint8_t latched_snapshot_idx;` - Buffer index latched at transaction start
- `uint8_t in_transaction;` - Flag indicating active transaction
- `uint8_t reg_ptr;` - Current register index (persists across repeated-start)
- `uint8_t addr_phase_seen;` - Flag to detect ADDR event

**Latch moment**: On ADDR event when address matched (best). Clear moment: On STOPF (STOP detected). Do not re-latch on repeated starts unless you also saw STOP. If supporting "write register index then repeated-start read", ensure the reg pointer persists across repeated-start.

**100ms periodic snapshot update requirement**: Even if the main loop swaps `active_buffer` mid-I2C read, the ISR must keep serving bytes from the latched buffer. That's the whole ballgame for multi-byte register coherence.
- **Advantages**: 
  - No interrupt disable needed during reads
  - Lock-free I2C reads (no blocking)
  - Guaranteed atomicity for multi-byte register reads
  - Minimal latency impact
- **Implementation**: Writer updates inactive buffer, then atomically swaps pointer (single write operation)

#### Double Buffering Implementation

```c
// Global snapshot buffer instance
static snapshot_buffer_t snapshot_buf;

// Update snapshot: called from main loop (not ISR)
void UpdateSnapshot(void) {
    // Determine inactive buffer (opposite of active)
    uint8_t inactive_buffer = 1 - snapshot_buf.active_buffer;
    
    // Copy authoritative state to inactive buffer
    memcpy(&snapshot_buf.buffer[inactive_buffer], &authoritative_state, 
           sizeof(authoritative_state_t));
    
    // Update snapshot metadata
    snapshot_buf.buffer[inactive_buffer].snapshot_tick = GetCurrentTick();
    
    // Atomically swap active buffer pointer (single write operation)
    // This is safe because it's a single byte write on ARM Cortex-M0
    snapshot_buf.active_buffer = inactive_buffer;
    
    // Increment version counter to signal update
    snapshot_buf.snapshot_version++;
}

// Read from snapshot: called from I2C ISR
uint8_t ReadSnapshotRegister(uint8_t reg_addr) {
    // Read from active buffer
    uint8_t active = snapshot_buf.active_buffer;
    authoritative_state_t *snapshot = &snapshot_buf.buffer[active];
    
    // Map register address to state field and return value
    // (Implementation details in I2C handler)
    return MapRegisterToSnapshot(snapshot, reg_addr);
}

// Read multi-byte register (ensures coherence for I2C transaction)
// For registers spanning multiple bytes (e.g., 0x01-0x02):
// Latch active_buffer on ADDR match or first register-index byte, keep until STOP.
// On STM32 slave, reads can span multiple interrupts; use same latched buffer for whole transaction.
uint16_t ReadSnapshotRegisterMultiByte(uint8_t reg_addr_low, uint8_t reg_addr_high) {
    // Latch active_buffer once at start of I2C transaction (on ADDR match or first register byte)
    // This latched value persists for the entire I2C transaction until STOP
    uint8_t latched_buf = snapshot_buf.active_buffer;
    authoritative_state_t *snapshot = &snapshot_buf.buffer[latched_buf];
    
    // Read both bytes from the same latched buffer instance
    uint16_t value_low = MapRegisterToSnapshot(snapshot, reg_addr_low);
    uint16_t value_high = MapRegisterToSnapshot(snapshot, reg_addr_high);
    
    // Combine bytes (little-endian)
    return value_low | (value_high << 8);
}

// Alternative defensive pattern (if version checking desired):
// uint16_t ReadSnapshotRegisterMultiByteDefensive(uint8_t reg_addr_low, uint8_t reg_addr_high) {
//     uint32_t version_before, version_after;
//     uint16_t value_low, value_high;
//     uint8_t latched_buf;
//     
//     do {
//         version_before = snapshot_buf.snapshot_version;
//         latched_buf = snapshot_buf.active_buffer;  // Latch once per attempt
//         authoritative_state_t *snapshot = &snapshot_buf.buffer[latched_buf];
//         value_low = MapRegisterToSnapshot(snapshot, reg_addr_low);
//         value_high = MapRegisterToSnapshot(snapshot, reg_addr_high);
//         version_after = snapshot_buf.snapshot_version;
//     } while (version_before != version_after);
//     
//     return value_low | (value_high << 8);
// }

// Check snapshot freshness (optional, for validation)
bool IsSnapshotFresh(uint32_t max_age_ticks) {
    uint8_t active = snapshot_buf.active_buffer;
    uint32_t age = GetCurrentTick() - snapshot_buf.buffer[active].snapshot_tick;
    return (age <= max_age_ticks);
}
```

**Key Implementation Notes**:
- Buffer swap is atomic because `active_buffer` is a single byte (uint8_t)
- ARM Cortex-M0 guarantees atomic byte writes
- I2C ISR reads from `active_buffer` without locks
- Main loop updates inactive buffer, then swaps atomically
- Multi-byte register reads: Latch `active_buffer` on ADDR match or first register-index byte, keep until STOP. On STM32 slave, reads can span multiple interrupts; use same latched buffer for whole transaction. This guarantees coherence without version checking.

### Snapshot Update Points

Snapshot updates occur in the main loop only (never in ISRs):

1. **After ADC conversion processed** (DMA ISR sets `adc_ready` flag, main loop converts raw ADC → scaled voltages → updates authoritative state → updates snapshot)
2. **After configuration change processed** (I2C write handler sets `config_changed` flag, main loop validates → updates authoritative state → updates snapshot)
3. **Periodically** (every 100ms minimum via tick_100ms flag)
   - The periodic 100ms snapshot update copies the current authoritative state even if no ADC update occurred, ensuring freshness without re-sampling hardware.
4. **On state machine transitions** (when power state, charger state, or learning mode changes)

**Important**: I2C ISR never triggers snapshot updates. Snapshot freshness: Status registers may be up to ~100-200ms stale (depending on main-loop latency), well within the ~1 second requirement.

### Coherence Rules

- Multi-byte registers (e.g., 0x01-0x02) must be read from same snapshot
- Snapshot freshness: Status registers may be up to ~100-200ms stale (depending on main-loop latency)
- Configuration registers must read back immediately after write (next snapshot update, typically <100ms)

---

## ADC and DMA Handling

### ADC/DMA Data Flow

**Critical Rule**: Only the main loop updates the authoritative state. ISRs only set flags.

**ADC Sample Processed Definition**: An ADC sample is considered "processed" only after:
1. Raw ADC DMA data has been converted to scaled voltages using the current VREF calibration
2. Authoritative state has been updated
3. Snapshot has been updated
4. `adc_sample_seq` has been incremented

Any logic that depends on "new ADC data" MUST key exclusively off `adc_sample_seq`.

**DMA ISR Responsibilities** (DMA1_CH1_IRQHandler):
- Clear DMA transfer complete flag
- Set `adc_ready` flag to indicate new raw ADC data is available
- **DO NOT** convert voltages or update authoritative state

**Main Loop Responsibilities**:
- Check `adc_ready` flag
- Convert raw ADC values to scaled voltages (using VREF calibration)
- Update authoritative state with new voltage measurements
- Update snapshot buffer
- Clear `adc_ready` flag

**Raw ADC Buffer**:
```c
// Raw ADC data from DMA (read-only in main loop)
extern volatile uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];
volatile uint8_t adc_ready = 0;  // Set by DMA ISR, cleared by main loop
// Monotonic sequence number incremented once per processed ADC sample.
// This is the canonical "new ADC sample available" signal for all consumers.
// Each consumer tracks its own last-seen value so ordering never matters.
volatile uint32_t adc_sample_seq = 0;
```

**Conversion Process** (main loop):
```c
if (adc_ready) {
    // Periodic VREF calibration (if requested)
    if (vref_calibration_needed) {
        // Calculate new VREF from internal reference channel
        uint16_t new_vref_mv = __LL_ADC_CALC_VREFANALOG_VOLTAGE(
            aADCxConvertedData[5], LL_ADC_RESOLUTION_12B);
        
        // Update VREF using moving average (smooth out noise)
        uAVDDVolt = (new_vref_mv + uAVDDVolt) / 2;
        
        vref_calibration_needed = 0;  // Clear flag
    }
    
    // Convert raw ADC to voltages (using current VREF calibration)
    uPIVCCVolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(uAVDDVolt * 2, aADCxConvertedData[0], LL_ADC_RESOLUTION_12B);
    uVBATVolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(uAVDDVolt * 2, aADCxConvertedData[1], LL_ADC_RESOLUTION_12B);
    uVBUSVolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(uAVDDVolt * 4, aADCxConvertedData[2], LL_ADC_RESOLUTION_12B);
    uUSBINVolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(uAVDDVolt * 4, aADCxConvertedData[3], LL_ADC_RESOLUTION_12B);
    uADCdegC = __LL_ADC_CALC_TEMPERATURE(uAVDDVolt, aADCxConvertedData[4], LL_ADC_RESOLUTION_12B);
    
    // Update authoritative state
    authoritative_state.mcu_voltage_mv = uAVDDVolt;
    authoritative_state.battery_voltage_mv = uVBATVolt;
    // ... etc
    
    // Update snapshot
    UpdateSnapshot();
    
    // After updating authoritative_state and updating snapshot:
    adc_sample_seq++;   // signals a new ADC sample was processed
    adc_ready = 0;
}
```

**New ADC sample gating rule**: Any logic that must "count only on new ADC updates" (charger detection stability, protection sample qualification, etc.) must use `adc_sample_seq` and a per-consumer `last_seen_seq`. No shared boolean flags may be used for this purpose, to prevent ordering dependence.

**Hard Rule - adc_sample_seq as ONLY Gating Primitive**:

**Forbidden pattern**: No logic may use boolean flags such as `adc_ready`, `adc_updated`, `new_sample`, or timing heuristics (e.g., "every 500ms") to qualify sample-based counting.

**Required pattern**: All sample-qualified logic MUST use:
```c
if (adc_sample_seq != last_seen_seq) {
    last_seen_seq = adc_sample_seq;
    // consume exactly one sample
}
```

**Consumers explicitly required to use this pattern**:
- Charger detection stability
- Protection voltage qualification
- Any future learning/stability counters
- Any future averaging / filtering logic

**Periodic VREF Calibration**:

The ADC reference voltage (VREF/VDD) must be periodically calibrated to maintain accuracy, as it can drift with temperature and time. Calibration is triggered by setting `vref_calibration_needed` flag:

**Calibration Triggers**:
1. **On startup**: Calibrate once during initialization
2. **After measurement window completes**: Calibrate after each measurement window (when charger is restored)
3. **Periodically when idle**: Calibrate every ~40 seconds when system is idle (no charger, RPi off)

**Calibration Method**:
- Uses internal VREFINT channel (channel 5 in ADC sequence)
- Calculates actual VDD voltage from VREFINT reading
- Updates VREF using moving average: `new_vref = (calculated_vref + old_vref) / 2`
- This smooths out noise while tracking slow drift

**Implementation**:
- Set `vref_calibration_needed = 1` at trigger points (main loop, not ISR)
- Calibration happens in main loop when processing ADC data (not in DMA ISR)
- VREF calibration uses the VREFINT channel reading from the same ADC conversion

---

## Timing and Scheduling

### Canonical Scheduler: TIM1 (10ms tick)

TIM1 ISR is kept **minimal** - it only sets periodic flags. All actual work runs in the main loop.

**TIM1 ISR Responsibilities** (TIM1_BRK_UP_TRG_COM_IRQHandler):
- Increment tick counters
- Set periodic flags (`tick_10ms`, `tick_100ms`, `tick_500ms`, etc.)
- **DO NOT** run state machine logic, button debounce, or countdowns in ISR

**Main Loop Responsibilities**:
- Check periodic flags set by TIM1 ISR
- Run all scheduler tasks based on flags
- Evaluate state machine transitions
- Process button debounce
- Update countdown timers

#### Periodic Flags Structure

```c
typedef struct {
    volatile uint8_t tick_10ms;    // Set every 10ms, cleared by main loop
    volatile uint8_t tick_100ms;   // Set every 100ms, cleared by main loop
    volatile uint8_t tick_500ms;   // Set every 500ms, cleared by main loop
    volatile uint8_t tick_1s;      // Derived from tick_100ms (10 pulses = 1 second), cleared by main loop
    volatile uint32_t tick_counter; // Free-running counter
} scheduler_flags_t;
```

**Derived Ticks**: Main loop derives `tick_1s` from `tick_100ms` (10 pulses = 1 second). All countdown timers that operate in seconds decrement on `tick_1s`.

#### Scheduler Tasks (run in main loop based on flags)

| Task | Trigger Flag | Description |
|------|--------------|-------------|
| ADC Trigger | tick_500ms | Request ADC conversion (LL_ADC_REG_StartConversion) |

**ADC triggering and sample sequencing rules**:
- Only one conversion in flight at a time
- DMA TC sets `adc_ready=1`
- Main loop clears `adc_ready` only after it copies & processes `aADCxConvertedData[]`
- Main loop must not start a new conversion if `adc_ready==1` (or if ADC is still busy)
- Otherwise you can overwrite the DMA buffer while you're still processing it
| Button Debounce | tick_10ms | Update button state machine |
| Countdown Timers | tick_1s | Decrement shutdown/restart countdowns (seconds)
| Measurement Window | tick_10ms | Manage charger-off window timing |
| Sample Period | tick_10ms | Check if sample period elapsed, trigger measurement window |
| Load On Delay | tick_10ms | Check conditions and state transitions; countdown decrements on tick_1s |
| Protection Check | tick_10ms | Check battery voltage vs protection threshold (using last ADC reading) |
| State Machine | tick_10ms | Evaluate state transitions for all state dimensions |
| Snapshot Update | tick_100ms | Update snapshot buffer (ensures freshness) |
| VREF Calibration | On trigger | Calibrate ADC reference voltage (after measurement window, periodically when idle) |

### SysTick Modifications

**Current Usage**: Tracks RuntimeAtAll, RuntimePowerOn, RuntimeOnetime

**Proposed Changes**:
- Keep as free-running millisecond counter
- Use only for uptime tracking (non-scheduling)
- Remove any scheduling logic from SysTick_Handler
- Convert to use TIM1 ticks for consistency (optional)

### Timing Function Structure

```c
typedef struct {
    uint32_t adc_trigger_counter;      // Counts to 50 (500ms)
    uint32_t countdown_timer_counter;  // Counts to 10 (100ms)
    uint32_t measurement_window_counter; // Counts to 150 (1.5s)
    uint32_t sample_period_counter;    // Counts in minutes
    uint32_t load_on_delay_counter;   // Counts in seconds
    uint32_t protection_check_counter; // For hysteresis/debouncing
} scheduler_counters_t;
```

---

## Flash Persistence

### Flash Write Rules

1. **Never in ISRs**: No flash writes in I2C ISR, DMA ISR, SysTick, or TIM1
2. **Rate Limiting**: Minimum interval between commits (e.g., 5 seconds)
3. **Integrity Protection**: CRC and version number
4. **Safe Update Method**: A/B slots or equivalent to prevent corruption

### Flash Storage Structure

```c
#define FLASH_PAGE_SIZE 1024  // STM32F030F4Px (1KB pages)
#define FLASH_STORAGE_START 0x08003C00  // Start of flash storage area
#define FLASH_STORAGE_END   0x08003FFF  // Last usable address (0x08004000 is end of 16KB flash)
#define FLASH_SLOT_A_ADDR 0x08003C00    // Slot A: first 512 bytes of storage page
#define FLASH_SLOT_B_ADDR 0x08003E00    // Slot B: second 512 bytes of storage page
#define FLASH_SLOT_SIZE 512              // Each slot is 512 bytes (half a page)

**Important**: Verify flash size matches STM32F030F4Px (16KB) in linker script and part ID before using these addresses. Different SKUs may have different flash sizes, and using wrong addresses could brick the device.

**Flash layout verification requirements** (critical bricking prevention):
- Confirm bootloader occupies which region
- Confirm your application + storage do not overlap
- Confirm linker script reserves storage page
- Confirm you aren't erasing a page that contains live code/consts

This is the #1 bricking risk in the whole refactor.

typedef struct {
    uint32_t version;           // Structure version
    uint32_t crc32;             // CRC of data section
    uint32_t magic;             // Magic number (e.g., 0x55505350 "UPSP")
    uint16_t sequence_number;   // Monotonic sequence number (increments on each write, wraps at 65535)
    
    // Persistent state (only RW configuration and learned values)
    uint16_t full_voltage_mv;
    uint16_t empty_voltage_mv;
    uint16_t protection_voltage_mv;
    uint16_t sample_period_minutes;
    // Note: shutdown_countdown_sec and restart_countdown_sec removed from flash persistence.
    // Registers 0x18 and 0x1A are runtime-only (current countdown values, not defaults).
    // After reboot they read 0. If "default countdown start value" is needed, add a new persistent config register later.
    uint8_t auto_power_on;
    uint8_t battery_params_self_programmed;
    uint8_t low_battery_percent;
    uint16_t load_on_delay_config_sec;  // 16-bit, 0-65535 seconds
    
    // Runtime state (optional, for recovery)
    uint8_t power_status;
    uint32_t cumulative_runtime_sec;
    uint32_t charging_time_sec;
} flash_persistent_data_t;
```

### Flash Update Strategy

**Countdown persistence rule**: Shutdown and restart countdown current values are runtime-only and MUST NOT be restored after reboot. Only configuration values (e.g., enable flags, delays, initial countdown values) are persisted. The current countdown values (`shutdown_countdown_sec`, `restart_countdown_sec` in authoritative state) are reset to 0 on boot and only become active when explicitly triggered.

**A/B Slot Method**:
1. Always write to inactive slot
2. Verify CRC after write
3. On boot, active slot is the valid slot with highest sequence number (determined by sequence number + CRC validation)
4. If corrupted, try other slot
5. If both corrupted, use factory defaults

**Flash Page Erase Algorithm**:

Since STM32F030F4Px requires erasing the entire 1KB page before writing, and both slots are in the same page, the A/B update algorithm is:

1. **Read both slots into RAM**:
   - Read Slot A (0x08003C00) into `ram_slot_a`
   - Read Slot B (0x08003E00) into `ram_slot_b`

2. **Determine active slot**:
   - Check CRC and sequence number for both slots
   - Active slot = highest valid sequence number with valid CRC (using wraparound-aware comparison)
   - If both invalid, use factory defaults

3. **Prepare new record**:
   - Copy current authoritative state to `ram_slot_new`
   - Increment sequence number (monotonic, wraps at 65535)
   - Calculate CRC32 of data section
   - Set magic number (0x55505350 "UPSP")

4. **Erase page** (entire 1KB page):
   - Unlock flash
   - Set PER (Page Erase) bit
   - Set page address (0x08003C00)
   - Start erase (STRT bit)
   - Wait for completion

5. **Write new record to inactive slot**:
   - Write `ram_slot_new` to inactive slot address
   - Verify write (read back and compare)

6. **If verification fails**:
   - **Recovery strategy**: Keep running with RAM state, mark flash as dirty for retry
   - On next flash write attempt, will try again (may succeed if failure was transient)
   - **Note**: After page erase, old active slot data is already gone. Recovery writeback would require reprogramming from RAM backup, which might fail for the same reason. More realistic approach: accept the failure, continue with RAM state, retry on next write cycle.
   - On next boot: If both slots invalid (due to failed write), fall back to factory defaults (already stated in step 2)

7. **Lock flash**

**Sequence Number**:
- Each slot includes a 16-bit monotonic sequence number
- On boot, slot with highest sequence number (with valid CRC) is active
- Sequence number increments on each write, wraps at 65535
- This provides automatic "newest wins" logic without explicit "active" marker
- **Wraparound handling**: Sequence number comparison handles wraparound. Wraparound comparison must treat sequence numbers modulo 65536; a signed delta comparison is acceptable. In practice, wraparound after ~65k writes is effectively never, but proper comparison ensures correctness.

### Flash Write Triggers

1. **Explicit User Actions**:
   - Factory reset (immediate)
   - Configuration change (rate-limited, minimum 5s since last write)

2. **Periodic Saves**:
   - After measurement window completes (if state changed)
   - Before protection shutdown
   - Maximum interval: Every 60 seconds if dirty

3. **Rate Limiting**:
   - Track last flash write time
   - Skip writes if < 5 seconds since last write (except factory reset)

### Flash Write Implementation

```c
void FlashWritePersistentData(void);
// - Check rate limit (skip if too soon)
// - Select inactive slot (opposite of last written slot, or scan to find inactive)
// - Calculate CRC
// - Increment sequence number
// - Write to flash (with interrupts disabled during critical section)
// - Verify write
// - Update in-RAM last_written_slot (optional, for next write selection)
// - Clear dirty flags
```

**Note**: There is no persistent "active slot pointer" stored in flash. The active slot is determined at boot by scanning both slots and selecting the valid slot with the highest sequence number. During runtime, only an in-RAM `last_written_slot` variable (if used) tracks which slot was last written, to help select the inactive slot for the next write.

---

## Pin Behavior and GPIO Management

### Pin Characterization Required

Before refactoring, document current behavior:

#### IP_EN (PA5, Output)
- **Function**: Controls power from USB-C/microUSB to charger (charger enable gate)
- **States**: HIGH = charger enabled, LOW = charger disabled
- **Usage**: 
  - Set LOW during measurement window (FORCED_OFF_WINDOW state)
  - Set HIGH when charger is present (CHARGER_STATE_PRESENT)
  - Set LOW when charger is absent (CHARGER_STATE_ABSENT) - default behavior
  - **Note**: IP_EN is strictly a "charger enable gate", not a general power path control. When charger is absent, IP_EN defaults to LOW. After characterization, this may be overridden if required for stable operation (e.g., if RPi power path requires IP_EN HIGH even without charger).

#### MT_EN (PA6, Output)
- **Function**: Controls power to RPi
- **States**: HIGH = RPi powered, LOW = RPi off
- **Usage**: Controlled by state machine and protection logic

#### PWR_EN (PA7, Output)
- **Function**: Controls power to MCU (must be HIGH for MCU to receive power)
- **States**: HIGH = MCU powered (required), LOW = MCU loses power
- **Usage**: MUST always be set HIGH during normal operation. This pin controls the MCU's own power supply.

#### Button (PB1, Input, EXTI)
- **Function**: Physical button for toggling RPi power or factory reset
- **Debounce**: Software debounce in main loop, triggered by tick_10ms flag from TIM1 ISR
- **Short Press**: < 10 seconds, toggles RPi power
  - **Note**: 10 seconds is intentionally long for "short press" to avoid accidental factory reset. Most systems use <2s for short press, but this design prioritizes safety.
- **Long Press**: ≥ 10 seconds, triggers factory reset

### GPIO State Machine

```c
typedef struct {
    uint8_t ip_en_state;      // 0=LOW, 1=HIGH
    uint8_t mt_en_state;      // 0=LOW, 1=HIGH
} gpio_state_t;
```

### Pin Update Logic

- **IP_EN**: Updated based on charger state:
  - HIGH when charger_state == CHARGER_STATE_PRESENT
  - LOW when charger_state == CHARGER_STATE_ABSENT or CHARGER_STATE_FORCED_OFF_WINDOW
- **MT_EN**: Updated based on power state:
  - HIGH when power_state == POWER_STATE_RPI_ON
  - LOW otherwise
- **PWR_EN**: MUST always be HIGH (critical: MCU loses power if LOW)

---

## Battery Management

### Battery Voltage Measurement

**Charging vs Discharging Detection** (with hysteresis):

- **Charger Present Thresholds**: See "Charger Detection Thresholds" in Charger State section (canonical definition).
- **Note**: Charger present changes may take up to 1.5s to be recognized due to stability requirement (3 consecutive ADC samples at 500ms intervals). This is a design characteristic:
  - With `stability_samples=3` and ADC at 500ms, charger present/absent decision can take up to ~1.5 seconds
  - Auto-power-on decisions might be delayed by up to 1.5s after plugging in
  - "Charger unplugged mid-window" edge case becomes fuzzier
  - This is acceptable given the constraint that ADC configuration must not change

- **Charging State**:
  - Charging: USB-C voltage > present_on_threshold OR microUSB voltage > present_on_threshold (with stability)
  - Discharging: Both USB-C and microUSB < present_off_threshold (with stability)

**Charging Detection Function**:
```c
bool is_charging(void) {
    // True means "charger connected and charger path enabled", not "battery is charging"
    // Function name reflects legacy usage; semantically this is "charger_influencing_vbat()"
    return (charger_state == CHARGER_STATE_PRESENT);
}
```

**Measurement Strategy**:
- During charging (charger_state == PRESENT): Voltage = charging voltage (not accurate for percentage)
- During measurement window (charger_state == FORCED_OFF_WINDOW): Voltage = true battery voltage (accurate)
- When charger absent (charger_state == ABSENT): Voltage = true battery voltage (accurate)
- **Canonical true-VBAT definition**: A "true VBAT" sample is any ADC sample taken when `charger_state != CHARGER_STATE_PRESENT` (i.e., charger path not influencing the battery node). This includes:
  - `CHARGER_STATE_ABSENT` (no charger connected), and
  - `CHARGER_STATE_FORCED_OFF_WINDOW` (charger connected but charger path disabled via IP_EN).
- Therefore battery percent updates and VBAT-based learning must use only samples taken when `charger_state == ABSENT` or `charger_state == FORCED_OFF_WINDOW`
- Battery percentage calculated only when NOT charging (i.e., when charger_state != PRESENT)

**Note**: True VBAT condition is defined by charger path state (`charger_state != PRESENT`), not by VBUS voltage reading. A charger may be physically plugged in (VBUS reads high) but IP_EN is LOW during measurement window, allowing true VBAT measurement.

**True-VBAT age tracking**: `last_true_vbat_sample_tick` is updated when `charger_state != PRESENT` at the moment the ADC sample is processed. True-VBAT age is computed from `now - last_true_vbat_sample_tick` (TIM1 ticks) or converted to seconds.

### Battery Learning Algorithm

**When Battery Parameters Self-Programmed = 0**:

The system sets learning_mode = LEARNING_ACTIVE (register 0x17 = 0x2) when actively learning battery parameters.

**Learning Mode Exit Criteria**:
- Learning is active whenever Battery Parameters Self-Programmed (0x2A) = 0
- Learning becomes inactive only when user sets 0x2A = 1 (user override)
- **Deterministic rule**: Learning never automatically exits. It remains active as long as 0x2A = 0, allowing continuous adaptation to battery aging over time.
- **Stabilization tracking**: The system may track stabilization internally (e.g., for reporting or optimization), but this does not affect whether learning is active. Learning continues even after parameters stabilize.

```c
void UpdateBatteryMinMax(void) {
    if (battery_params_self_programmed == 0) {
        // Set learning mode active
        if (system_state.learning_mode != LEARNING_ACTIVE) {
            system_state.learning_mode = LEARNING_ACTIVE;
        }
        
        // Learning mode: update full/empty from non-charging voltage samples
        // Only learn during measurement windows (true battery voltage)
        if (charger_state == CHARGER_STATE_FORCED_OFF_WINDOW || charger_state == CHARGER_STATE_ABSENT) {
            if (battery_voltage_mv > full_voltage_mv) {
                full_voltage_mv = battery_voltage_mv;  // Learn max
                learning_stability_count = 0;  // Reset stability counter on change
            }
            if (battery_voltage_mv < empty_voltage_mv && 
                battery_voltage_mv > protection_voltage_mv) {
                empty_voltage_mv = battery_voltage_mv;  // Learn min
                learning_stability_count = 0;  // Reset stability counter on change
            }
            // Ensure empty never goes below protection
            if (empty_voltage_mv < protection_voltage_mv) {
                empty_voltage_mv = protection_voltage_mv;
            }
            
            // Track stabilization (for internal use, does not affect learning active state)
            if (abs(battery_voltage_mv - last_learning_voltage) < 50) {
                learning_stability_count++;
                // Stabilization tracked but learning remains active
            } else {
                learning_stability_count = 0;  // Reset on change
            }
            last_learning_voltage = battery_voltage_mv;
        }
    } else {
        // User override: disable learning
        if (system_state.learning_mode == LEARNING_ACTIVE) {
            system_state.learning_mode = LEARNING_INACTIVE;
        }
    }
}
```

**When Battery Parameters Self-Programmed = 1**:
- Use user-provided Full and Empty voltages
- Clip measured voltage to range [Empty, Full] if needed

### Battery Percentage Calculation

```c
#define MIN_VOLTAGE_DELTA_MV 50  // Minimum voltage difference for valid calculation

void UpdateBatteryPercentage(void) {
    // Only calculate when NOT charging (during measurement window or discharging)
    // Note: Percentage updates are not forced every ADC update; percentage may remain constant
    // across multiple snapshots if voltage hasn't changed significantly (hysteresis applied)
    if (is_charging()) {
        // Don't update percentage while charging
        return;
    }
    
    // Guard against divide-by-zero: ensure valid voltage range
    int32_t voltage_range = (int32_t)full_voltage_mv - (int32_t)empty_voltage_mv;
    if (voltage_range < MIN_VOLTAGE_DELTA_MV) {
        // Invalid range - use conservative value or hold last
        // If range is too small, report 0% or 100% based on which bound is closer
        if (voltage_range <= 0) {
            battery_percent = (battery_voltage_mv >= full_voltage_mv) ? 100 : 0;
        } else {
            // Range too small but positive - hold last value or use conservative estimate
            return;  // Don't update, keep last known good value
        }
        return;
    }
    
    // Calculate percentage using signed arithmetic to handle edge cases
    int32_t voltage_offset = (int32_t)battery_voltage_mv - (int32_t)empty_voltage_mv;
    int32_t percentage = (voltage_offset * 100) / voltage_range;
    
    // Clamp to 0-100%
    if (percentage > 100) percentage = 100;
    if (percentage < 0) percentage = 0;
    
    // Apply hysteresis to prevent rapid changes
    // Only update if change is significant (e.g., > 1%)
    int32_t change = percentage - (int32_t)battery_percent;
    if (change > 1 || change < -1) {
        battery_percent = (uint8_t)percentage;
    }
}
```

### Protection Voltage Logic

**Hysteresis and Sample Qualification**:
- **Shutdown threshold**: Protection Voltage (from register 0x11-0x12)
- **Hysteresis**: 50mV
- **Shutdown condition**: Battery voltage ≤ Protection Voltage for N consecutive **ADC updates** (not 10ms ticks)
- **Power-on condition**: Battery voltage > (Protection Voltage + Hysteresis) AND other conditions met (charger present, battery % > threshold, AutoPowerOn enabled)

**Important**: Protection checks occur every 10ms using the **last measured battery voltage** (from most recent ADC conversion). However, the sample count is incremented only on **new ADC updates** (every 500ms). This prevents false triggers from transient measurements while still providing responsive protection.

```c
#define PROTECTION_HYSTERESIS_MV 50
#define PROTECTION_SAMPLES_REQUIRED 3  // N consecutive ADC updates (at 500ms intervals = 1.5s)

typedef struct {
    uint8_t below_threshold_count;  // Counts ADC updates, not 10ms ticks
    uint8_t protection_active;       // Protection is latched active
    uint16_t last_adc_battery_mv;   // Last battery voltage from ADC
    uint32_t last_seen_seq;         // Last seen adc_sample_seq (for independent tracking)
} protection_state_t;
```

**Protection sample qualification (canonical)**: The N-consecutive-samples requirement increments only on new processed ADC samples using a per-consumer last-seen sequence:
- `uint32_t protect_last_seq;`
- Increment `below_threshold_count` only when `adc_sample_seq != protect_last_seq`, then set `protect_last_seq = adc_sample_seq`.

**Protection Check Logic** (run every 10ms in main loop, but count only on ADC updates):
```c
// In protection logic (main loop):
static uint32_t protect_last_seq = 0;

if (power_state != POWER_STATE_PROTECTION_LATCHED) {
    uint16_t battery_mv = authoritative_state.battery_voltage_mv;
    
    if (battery_mv <= protection_voltage_mv) {
        if (adc_sample_seq != protect_last_seq) {
            protect_last_seq = adc_sample_seq;
            protection_state.below_threshold_count++;
            if (protection_state.below_threshold_count >= PROTECTION_SAMPLES_REQUIRED) {
                // enter PROTECTION_LATCHED, request flash persist, etc.
                power_state = POWER_STATE_PROTECTION_LATCHED;
                protection_state.protection_active = 1;
                flash_write_requested = 1;
                flash_write_bypass_rate_limit = 1;
            }
        }
    } else if (battery_mv > (protection_voltage_mv + PROTECTION_HYSTERESIS_MV)) {
        protection_state.below_threshold_count = 0;
    }
} else {
    // Already in protection - only check for recovery
    uint16_t battery_mv = authoritative_state.battery_voltage_mv;
    if (battery_mv > (protection_voltage_mv + PROTECTION_HYSTERESIS_MV)) {
        // Voltage recovered - clear protection latch
        protection_state.below_threshold_count = 0;
        protection_state.protection_active = 0;
        // Note: power_state transition out of PROTECTION_LATCHED is handled by state machine
        // based on other conditions (charger present, battery %, AutoPowerOn, etc.)
    }
}
```

**Note**: Flash write is implemented as a request flag (`flash_write_requested`) that main loop processes, not called inline from protection check. This ensures flash writes never occur in any ISR context.

**Flash write during protection timing - deterministic ordering rule**: On entering `POWER_STATE_PROTECTION_LATCHED`, do:
1. Set `flash_write_requested=1` (bypass rate limit)
2. Set a `pending_power_cut` flag
3. Main loop runs `FlashCommitIfRequested()`
4. Only after commit attempt (success or failure), cut MT_EN LOW

This avoids the trap where you cut power and then never get to write.

### Load On Delay Logic

**Conditions**:
1. Battery percentage > Low Battery Percentage
2. Auto Power On enabled
3. Charger connected OR battery sufficient
   - **"Battery sufficient"** means `battery_percent > low_battery_percent` computed from the most recent non-charging voltage sample
   - **Staleness rule**: If `charger_state == PRESENT` and `last_true_vbat_age > 10 minutes`, force a measurement window immediately (or ASAP) and don't allow auto-power-on until it completes. Defer decision until a fresh true-VBAT sample is captured.
   - **Battery percent staleness enforcement**: Any power decision that depends on percent must also require `last_true_vbat_sample_tick` to be "fresh enough" if the charger is present. This is a hard gate in `CheckPowerOnConditions()`.

**Behavior**:
- Start countdown when conditions met
- Countdown duration: Load On Delay seconds
- If conditions change (battery drops, charger disconnected), reset countdown
- When countdown completes, enable RPi power if conditions still met

---

## Button Handling

**Canonical button model**: EXTI is only a wake-up/edge notification. After an edge, the main-loop button FSM debounces and then tracks press duration using periodic sampling (tick_10ms) while the GPIO reads "pressed." Long-press timing must not depend on receiving additional EXTI edges or on any "button_flag_set" remaining asserted.

**EXTI rule**: EXTI is a wake-up signal only. It MUST NOT be used for:
- Timing
- Duration measurement
- Determining press vs release

All button semantics are derived from periodic GPIO sampling in the main loop.

**EXTI Semantics**: EXTI is edge-triggered and only signals that a button transition occurred; press/release level is determined by reading the GPIO pin in the main loop. The EXTI ISR does not distinguish press vs release—it only sets `button_flag_set` (or `button_activity`) to indicate an edge was detected.

**Key rule**: Once a press is confirmed (debounced), hold-time increments every 10ms while GPIO reads pressed, independent of further EXTI edges.

**Long-press action guard**: Long-press action MUST trigger exactly once per physical press, regardless of how long the button remains held. This prevents factory reset loops.

### Button State Machine

```c
typedef enum {
    BUTTON_IDLE,
    BUTTON_PRESSED,
    BUTTON_HELD,
    BUTTON_RELEASED_SHORT,
    BUTTON_RELEASED_LONG
} button_state_t;

typedef struct {
    button_state_t state;
    uint32_t press_start_tick;  // TIM1 tick when pressed
    uint32_t hold_duration_ticks; // Duration held (max 1000 ticks = 10s)
    uint8_t debounce_counter;    // For debounce filtering
} button_handler_t;
```

### Debounce Logic

**Debounce Window**: 50ms (5 scheduler ticks at 10ms each)
- Button must be stable for 5 ticks before state change recognized
- Use counter in main-loop button handler (invoked by tick_10ms flag)
- TIM1 ISR never reads the button pin and never runs debounce logic

### Button Press Detection

**Short Press** (< 10 seconds):
- Press detected: EXTI interrupt sets flag
- Debounce: Wait 5 ticks (50ms) for stability - presses < 50ms are ignored
- Release detected: Button released before reaching exactly 1000 ticks (10s)
- **Boundary**: Exactly 10s (1000 ticks) counts as long press, not short press
- Action: Toggle RPi power state

**Long Press** (≥ 10 seconds):
- Press detected: EXTI interrupt sets flag
- Hold detected: Button still pressed when hold_time reaches exactly 1000 ticks (10s)
- **Boundary**: Exactly 10s (1000 ticks) counts as long press
- Action: Trigger factory reset immediately (same as writing 1 to register 0x1B)
- Factory reset executes: All parameters reset to defaults, flash updated, system continues operation

### Button Handler Implementation

```c
void ProcessButtonState(void) {
    // Called every 10ms scheduler tick in main loop (when tick_10ms flag is set)
    // EXTI sets button_activity = 1 on edge (wakes debounce FSM)
    // Main loop runs ProcessButtonState() every tick_10ms regardless, but it only enters
    // debounce/press-tracking if: button_activity == 1 or FSM is already in PRESSED/HELD states.
    // Clear button_activity after the debounce FSM has latched the initial press/release.
    
    // Check current GPIO state (not just when flag is set)
    if (button_pin_low) {
        // Button is currently pressed
        if (hold_time < 1000) {  // 10 seconds = 1000 ticks at 10ms
            hold_time++;  // Increment every 10ms while GPIO reads pressed
        } else if (hold_time == 1000) {
            // Long press detected (10 seconds) - trigger once
            button_click = LONG_PRESS;
            hold_time++;  // Increment to prevent re-triggering while button still held
            // Trigger factory reset (updates RAM state, marks flash dirty)
            FactoryReset();
            // Request flash write with rate limit bypass (factory reset is critical)
            flash_write_requested = 1;
            flash_write_bypass_rate_limit = 1;  // Allow immediate write
            // Main loop persistence task will perform the actual flash write
        }
        // Clear EXTI flag if set (edge already processed)
        if (button_activity) {
            button_activity = 0;
        }
    } else {
        // Button released
        if (hold_time >= 5 && hold_time < 1000) {
            // Short press (≥ 50ms and < 10 seconds)
            // Exact boundaries: 5 ticks (50ms) minimum, 999 ticks (9.99s) maximum
            button_click = SHORT_PRESS;
            // Toggle RPi power state
            if (power_state == POWER_STATE_RPI_ON) {
                power_state = POWER_STATE_RPI_OFF;
            } else if (power_state == POWER_STATE_RPI_OFF) {
                // Check conditions before enabling
                if (CheckPowerOnConditions()) {
                    power_state = POWER_STATE_LOAD_ON_DELAY;
                }
            }
        }
        // Reset hold time on release
        hold_time = 0;
        if (button_activity) {
            button_activity = 0;
        }
    }
}
```

---

## Implementation Phases

### Phase 1: Foundation and Characterization ✓ COMPLETE
**Goal**: Understand current behavior, establish foundation

1. **Pin Characterization** ✓
   - Create truth table for all pins per system state

2. **State Machine Design** ✓
   - Document current implicit states
   - Design explicit state machine
   - Define all state transitions

3. **Data Structure Design** ✓
   - Design authoritative state structure
   - Design snapshot buffer mechanism
   - Design flash persistence structure

**Implementation Notes (Phase 1)**:
Created `Inc/ups_state.h` containing:
- All state machine enumerations (`power_state_t`, `charger_state_t`, `learning_mode_t`, `button_state_t`)
- System state structure (`system_state_t`) with all fields from plan
- Authoritative state structure (`authoritative_state_t`) - single source of truth
- Snapshot buffer structure (`snapshot_buffer_t`) for double-buffered I2C coherence
- Flash persistence structure (`flash_persistent_data_t`) with A/B slot support
- Scheduler structures (`scheduler_flags_t`, `scheduler_counters_t`)
- I2C pending write structure (`i2c_pending_write_t`)
- Protection state structure (`protection_state_t`)
- Button handler structure (`button_handler_t`)
- GPIO state structure (`gpio_state_t`)
- All constants and thresholds from the plan
- Complete register map definitions
- Pin behavior truth table as documentation comment
- Function prototypes for later phases

### Phase 2: Core Infrastructure ✓ COMPLETE
**Goal**: Implement data structures and snapshot mechanism

1. **Authoritative State** ✓
   - Create authoritative_state_t structure
   - Migrate all state variables to structure
   - Update all code to use structure

2. **Snapshot Mechanism** ✓
   - Implement double-buffered snapshot
   - Implement atomic snapshot update
   - Test snapshot coherence

3. **I2C Register Map** ✓
   - Implement register validation functions
   - Implement bounds checking
   - Implement RO register enforcement
   - Update I2C handler to use snapshot

**Implementation Notes (Phase 2)**:
- **Authoritative state** (`main.c`): `state` (`authoritative_state_t`) and `sys_state` (`system_state_t`) are the single source of truth. All former globals (uXXXVolt, counters, mode flags) removed as authoritative; only timing helpers remain (`CountDownPowerOffTicks`, `CountDownRebootTicks`, `sKeyFlag`, etc.) with comments that real countdown/state lives in `state`/`sys_state`.
- **Snapshot mechanism**: Double-buffered register image `reg_image[2][256]` and `volatile uint8_t active_reg_image`. `Snapshot_Update()` builds into `reg_image[inactive]` via `StateToRegisterBuffer()`, then atomically flips `active_reg_image`. `aReceiveBuffer` is used for flash load (`GetRegValue`) only; I2C reads use `reg_image` only. `Snapshot_Update()` is called only on state changes (ADC processed, 1s tick, I2C write applied, flash save, factory reset, OTA, countdown actions, button toggle)—no unconditional per-loop update.
- **I2C coherence**: In `I2C_Slave.c`, on ADDR+READ the ISR latches `latched_reg_image = active_reg_image`; TX transmits from `reg_image[latched_reg_image][uI2CRegIndex++]` for the whole transaction, ensuring coherent multi-byte reads.
- **I2C register map**: `StateToRegisterBuffer()` maps `state`/`sys_state` to the 256-byte register image; validation helpers (`Validate_FullVoltage`, etc.) and `ProcessI2CPendingWrite()` apply and validate writes. RO registers enforced in `ProcessI2CPendingWrite()` (writes to RO/Reserved discarded).
- **I2C semantics**: First byte of write sets both `uI2CRegIndex` and `i2c_pending_write.reg_addr` (write-then-repeated-start-read works). STOP sets `i2c_pending_write.pending` only when `length > 0` (pointer-only writes do not pend). Overwrite protection: if `pending != 0` on ADDR+WRITE, set `ignore_write` and drop bytes until STOP. `i2c_pending_write_t` fields and `active_reg_image`/`latched_reg_image` are `volatile` where shared with ISR.
- **Init**: `InitAuthoritativeStateFromDefaults()`, `InitAuthoritativeStateFromBuffer()` (from `aReceiveBuffer` after flash load), `Snapshot_Init()` (both `reg_image` buffers + snapshot).

### Phase 3: Timing and Scheduling
**Goal**: Consolidate timing to TIM1 canonical scheduler

1. **Scheduler Refactoring**
   - Move timing logic to main-loop scheduler driven by TIM1 flags; keep TIM1 ISR flag-only
   - Remove scheduling from SysTick (keep only uptime counter)
   - Implement scheduler counters structure

2. **Periodic Tasks**
   - ADC trigger (500ms)
   - Measurement window (1.5s)
   - Sample period timing
   - Countdown timers

### Phase 4: State Machine Implementation
**Goal**: Implement explicit state machine

1. **State Machine Core**
   - Implement state enumeration
   - Implement state transition logic
   - Implement state entry/exit actions

2. **State-Specific Logic**
   - Charging state
   - Measurement window state
   - Protection shutdown state
   - Load on delay state

### Phase 5: Battery Management
**Goal**: Implement battery learning and protection

1. **Battery Learning**
   - Implement learning algorithm
   - Implement bounds checking
   - Test learning behavior

2. **Battery Protection**
   - Implement protection voltage logic with hysteresis
   - Implement protection shutdown
   - Test protection behavior

3. **Battery Percentage**
   - Implement percentage calculation
   - Only update when not charging
   - Implement hysteresis

### Phase 6: Flash Persistence
**Goal**: Implement safe flash persistence

1. **Flash Structure**
   - Implement A/B slot mechanism
   - Implement CRC calculation
   - Implement version checking

2. **Flash Write Logic**
   - Implement rate limiting
   - Implement write triggers
   - Test power-loss recovery

3. **Flash Read Logic**
   - Implement boot-time read
   - Implement corruption recovery
   - Test default fallback

### Phase 7: Button Handling
**Goal**: Implement robust button handling

1. **Button State Machine**
   - Implement debounce logic
   - Implement short/long press detection
   - Document long press behavior

2. **Button Actions**
   - Implement power toggle
   - Implement long press action (factory reset)

### Phase 8: Integration and Testing
**Goal**: Integrate all components and test

1. **Integration**
   - Integrate all components
   - Verify state machine transitions
   - Verify I2C register map

2. **Testing**
   - Test all state transitions
   - Test I2C read/write operations
   - Test flash persistence and recovery
   - Test button handling
   - Test battery learning and protection
   - Test measurement window timing

3. **Documentation**
   - Document state machine
   - Document pin behavior
   - Document I2C register map
   - Document timing requirements

---

## Critical Implementation Notes

### Configuration That Must Not Change
- I2C Slave configuration (MX_I2C1_Slave_Init)
- ADC configuration (MX_ADC_Init)
- DMA configuration (MX_DMA_Init)
- TIM1 configuration (frequency, interrupt setup)
- GPIO pin configuration (MX_GPIO_Init)
- EXTI configuration for button

### Code That Can Be Modified
- TIM1_BRK_UP_TRG_COM_IRQHandler implementation (keep config, change logic)
- SysTick_Handler implementation (modify for better timing)
- Main loop logic (complete refactor allowed)
- State management code
- Flash persistence code

### Endianness Requirements
- All multi-byte I2C registers: Little-endian (LSB at lower address)
- Example: 16-bit value 0x1234 stored as [0x34, 0x12] at addresses [N, N+1]

### Atomic Operations
- Snapshot updates must be atomic
- **Double-buffering approach**: Writer updates inactive buffer, then atomically swaps active buffer pointer (single byte write is atomic on Cortex-M0)
- Multi-byte register reads must be from same snapshot (guaranteed by reading from same active buffer)

### Measurement Window Atomicity
- Measurement window (1.5s charger-off) is atomic operation
- Configuration changes take effect after window completes OR at next scheduler tick
- Document which approach is used

---

## Testing Checklist

### I2C Testing
- [ ] All registers readable with correct values
- [ ] RO registers ignore writes
- [ ] RW registers validate bounds
- [ ] Multi-byte registers are coherent
- [ ] Configuration writes take effect correctly
- [ ] Register map matches specification

### State Machine Testing
- [ ] All state transitions work correctly
- [ ] State entry/exit actions execute
- [ ] State machine handles edge cases
- [ ] State machine documentation is accurate

### Timing Testing
- [ ] ADC triggers every 500ms
- [ ] Measurement window is exactly 1.5s
- [ ] Sample period timing is correct
- [ ] Countdown timers work correctly
- [ ] Load on delay timing is correct

### Battery Management Testing
- [ ] Battery learning works correctly
- [ ] Battery percentage calculation is accurate
- [ ] Protection voltage shutdown works with hysteresis
- [ ] Load on delay conditions are correct
- [ ] Charging vs discharging detection works

### Flash Persistence Testing
- [ ] Flash writes work correctly

### Architecture Validation Tests (Nasty Tests)

These tests specifically validate the new architecture's coherence and gating rules:

1. **I2C coherence torture test**:
   - Read a 16-bit register repeatedly while forcing snapshot swaps at 100ms
   - Ensure the two bytes never come from different samples (no mixed endian or mixed buffer)

2. **adc_sample_seq gating correctness test**:
   - Force the main loop to run "charger detection" multiple times between ADC samples
   - Ensure stability counters don't advance without `adc_sample_seq` increment

3. **Window atomicity under edge cases test**:
   - Start a forced-off window, then unplug charger mid-window
   - Confirm:
     - Window runs full 1.5s
     - Samples counted as true-VBAT
     - Final state becomes ABSENT after window finishes
- [ ] Flash reads work on boot
- [ ] Power loss during write doesn't corrupt data
- [ ] A/B slot mechanism works
- [ ] CRC validation works
- [ ] Factory reset works

### Button Testing
- [ ] Short press toggles power
- [ ] Long press triggers factory reset
- [ ] Debounce works correctly
- [ ] Button doesn't trigger false presses

### GPIO Testing
- [ ] IP_EN controls charger correctly
- [ ] MT_EN controls RPi power correctly

---

## Open Questions

1. **Long Press Behavior**: **RESOLVED** - Long press is ≥ 10 seconds and triggers factory reset (same as writing 1 to register 0x1B).

2. **Measurement Window Timing**: **RESOLVED** - Configuration writes take effect at next scheduler tick, but do not interrupt an in-progress measurement window. (Deterministic, simple.)

5. **Snapshot Update Frequency**: **RESOLVED** - Update on ADC processed + config processed, and also at least every 100ms (since we already have that heartbeat). No "on-demand in ISR."

6. **Protection Hysteresis Value**: **RESOLVED** - 50mV hysteresis is used.

7. **Protection Sample Count**: **RESOLVED** - 3 consecutive ADC updates (at 500ms intervals = 1.5s total). Protection checks occur every 10ms using last measured voltage, but sample count increments only on new ADC updates.

---

## Plan Closure Addendum

This addendum "locks" the plan so implementation can start without further surprise requirements. It defines (A) invariants, (B) explicit decisions, (C) deferred items, and (D) acceptance criteria. Any future review item must be classified into exactly one of those buckets.

⸻

### A) Invariants (MUST be true)

#### A1. Architectural invariants
1. **Authoritative state ownership**: Only the main loop may modify authoritative state. All ISRs (I2C/DMA/TIM1/SysTick/EXTI) are flag-only except for hardware register service required by the peripheral.
2. **adc_sample_seq is the only sample gate**: Any logic that "counts," "debounces," or "qualifies" based on ADC updates MUST be gated exclusively by `adc_sample_seq` with a per-consumer `last_seen_seq`.
3. **No flash in ISR**: Flash erase/program never runs in any ISR context.
4. **No read side effects**: I2C reads must never change state, counters, timers, or persistence flags.

#### A2. I2C coherence invariants
5. **Transaction snapshot latch**: On I2C transaction start (ADDR match preferred; reg-index byte acceptable fallback), latch `snapshot_buf.active_buffer` into a transaction-local variable. Use that same buffer for all bytes until STOPF.
6. **No relatch on repeated-start**: Repeated-start without STOP must not relatch the snapshot buffer.
7. **Multi-byte registers are coherent**: Any multi-byte register read must return bytes from the same latched snapshot instance.

#### A3. Measurement window invariants
8. **Definition of true-VBAT**: A "true VBAT" sample exists iff `charger_state != CHARGER_STATE_PRESENT` at the moment the ADC sample is processed.
9. **Window atomicity**: Measurement windows are never interruptible, never nested, never restarted early—regardless of unplug/replug, config writes, or state changes.
10. **Unplug mid-window rule**: If charger is unplugged during `FORCED_OFF_WINDOW`, the window completes its timing; samples are treated as valid true-VBAT; state transitions to `ABSENT` after the window ends.

#### A4. Scheduler invariants
11. **TIM1 10ms is canonical**: TIM1 ISR sets flags only. All logic (FSM, countdowns, debounce, window manager, protection counting) runs in main loop.
12. **ADC single in-flight**: Never start a new ADC conversion while prior DMA data is unprocessed (`adc_ready==1`), or while ADC is busy.

#### A5. Persistence invariants
13. **Countdowns are runtime-only**: Registers 0x18 and 0x1A are current countdown values and MUST NOT be restored after reboot. After boot they read 0 until explicitly started by a write.
14. **Reserved register writes**: Reserved regions (0x2E–0xEF, 0xFC–0xFF) accept writes but discard them; reads return 0x00 (or the legacy value if characterization proves otherwise). No storage, no persistence, no snapshot impact.

#### A6. Protection cut ordering invariant
15. **Deterministic protection ordering**: On entering `POWER_STATE_PROTECTION_LATCHED`:
    - Set `flash_write_requested=1` with bypass
    - Set `pending_power_cut=1`
    - Attempt `FlashCommitIfRequested()` in main loop
    - After the attempt (success or failure), then cut MT_EN LOW

⸻

### B) Decisions Locked (no longer ambiguous)

#### B1. Configuration-write timing vs window atomicity

**Decision**: Configuration writes take effect at the next scheduler tick (10ms granularity) except they do not interrupt an in-progress measurement window. They may change what happens after the window ends.

#### B2. Countdown semantics (0x18 / 0x1A / 0x2C–0x2D)

**Decision**:
- 0x18 and 0x1A always represent current remaining seconds (runtime), not "default start values." They read back immediately as current state once applied by main loop and snapshot updated (≤100ms typical).
- 0x2C–0x2D:
  - When `power_state != LOAD_ON_DELAY`, reads return `load_on_delay_config_sec`.
  - When `power_state == LOAD_ON_DELAY`, reads return `load_on_delay_remaining_sec`.
  - Writes update `load_on_delay_config_sec` immediately; if countdown active, remaining resets to new value.

#### B3. Charger detect channel selection

**Decision**: Charger detection uses `MAX(uVBUSVolt, uUSBINVolt)` after existing scaling, and thresholds are defined in those same post-scaled connector-referenced mV units.

#### B4. Charger state modeling

**Decision**: External interface remains the documented `charger_state_t` (ABSENT/PRESENT/FORCED_OFF_WINDOW). Internally, physical detection is represented by `charger_present_bool` and window control is managed by the window manager; state machine must preserve "unplug mid-window completes window" behavior.

#### B5. Learning mode semantics

**Decision**: Learning mode is strictly tied to 0x2A:
- 0x2A=0 ⇒ learning active (0x17 returns 0x2), indefinitely.
- 0x2A=1 ⇒ learning inactive.
No "auto exit."

#### B6. Power status register meaning (0x17)

**Decision**: 0x17 is operational mode:
- 0x2 indicates learning/calibration enabled (0x2A=0), regardless of MT_EN.
- 0x1 indicates RPi powered and not in learning mode.
- 0x0 otherwise.
Clients must not interpret 0x2 as "power off."

#### B7. Reserved-region behavior

**Decision**: ACK writes but discard; reads return 0x00 unless characterization proves legacy differs.

⸻

### C) Deferred Items (explicitly out of scope)

These items are permitted to exist as future enhancements and MUST NOT block implementation now:

1. **Additional status bitfield register** (e.g., MT_EN actual state, charger_state, window active). The plan documents how to add later; not required now.
2. **Persisting default countdown start values** via new registers. Not part of current API; current countdowns remain runtime-only.
3. **Optimizing charger detect latency** beyond the existing ADC cadence + stability sample design. Accept up to ~1.5s reaction time as a constraint of "ADC config must not change."
4. **Changing any peripheral configuration** listed under "Configuration That Must Not Change."

⸻

### D) Acceptance Criteria for "Plan Complete"

Implementation should start only when each criterion below can be answered unambiguously from the document.

#### D1. Register-map completeness checklist

For every register (including special and multi-byte):
- Access type (RO/RW)
- Units and scaling domain (raw vs post-scaled mV)
- Bounds (valid/invalid behavior)
- Write semantics (what happens, when, and what reads back)
- Persistence policy (persisted vs runtime-only)
- Snapshot source (which authoritative field)
- Special cases (e.g., 0x17 precedence, countdown transitions)

#### D2. State machine completeness checklist

For each state dimension (power/charger/learning):
- Entry actions
- Exit actions
- Transition conditions (including staleness gating)
- Timing sources and counters used
- Explicit precedence rules where applicable (e.g., 0x17)

#### D3. Snapshot/I2C transaction checklist
- Exact latch moment (ADDR preferred; reg-index fallback)
- STOPF clear moment
- Repeated-start handling behavior
- Register pointer behavior across repeated-start
- Guarantee that multi-byte reads do not mix buffers

#### D4. ADC cadence and gating checklist
- ADC trigger policy (500ms) and "single in-flight" rule
- Definition of "processed ADC sample"
- Exact point where `adc_sample_seq++` occurs (after authoritative + snapshot update)
- Per-consumer gating requirement documented for charger stability and protection sample counting

#### D5. Flash persistence checklist
- Flash layout verification steps explicitly performed before writing:
  - Confirm SKU flash size and linker layout
  - Confirm bootloader region
  - Confirm storage page not overlapping code/consts
- A/B slot algorithm defined with:
  - Boot-time slot selection
  - Runtime commit trigger & rate limit
  - Factory reset bypass rule
  - Protection ordering rule (commit attempt before power cut) documented and treated as invariant

#### D6. "Nasty tests" must be executable as written
- I2C coherence torture (no mixed bytes under snapshot swaps)
- `adc_sample_seq` gating correctness (no counter advance without new seq)
- Window atomicity edge-case (unplug mid-window behavior)

⸻

### E) Closure Record: Known "Must Verify" Items (not gotchas, explicit verification tasks)

These are required verification steps, but they do not change the plan unless verification fails:

1. **Flash address/layout verification** against actual linker script and bootloader footprint (bricking prevention).
   - **Status**: Deferred to Phase 6 (gated behind UPS_ENABLE_FLASH_PERSISTENCE and UPS_FLASH_LAYOUT_VERIFIED)

2. **ADC scaling confirmation** for VBUS/USBIN thresholds (confirm that the existing macros' scaling corresponds to connector mV as assumed).
   - **Status**: ✓ VERIFIED (Phase 1)
   - **Result**: ADC scaling confirmed - __LL_ADC_CALC_DATA_TO_VOLTAGE macros for VBUS/USBIN produce connector-referenced millivolts. Thresholds (4200mV/3800mV) are correct.

3. **Legacy reserved-region read behavior** (confirm whether legacy returns 0x00; if not, mirror legacy readback while still discarding writes).
   - **Status**: ✓ VERIFIED (Phase 1)
   - **Result**: Legacy firmware returns 0x00 for reserved register reads. Snapshot mapping matches this behavior.

4. **Battery percent MSB (register 0x14) behavior** (confirm whether MSB is always 0x00).
   - **Status**: ✓ VERIFIED (Phase 1)
   - **Result**: Register 0x14 (MSB) always returns 0x00 in legacy firmware. Register mapping contract confirmed.

5. **Temperature units** (confirm __LL_ADC_CALC_TEMPERATURE scaling).
   - **Status**: ✓ VERIFIED (Phase 1)
   - **Result**: Temperature is integer degrees Celsius (°C). Confirmed via Phase 1 characterization.

If any of these verifications contradict the plan, the contradiction must be resolved by updating the plan in section B (Decisions Locked)—not by ad hoc code changes.

⸻

### F) Change-Control Rule (prevents future "surprise must-fix")

Any proposed review item after this addendum MUST be categorized as one of:
- **Invariant violation** (plan bug; must fix in plan)
- **Decision missing/changed** (update section B)
- **Deferred enhancement** (add to section C)
- **Verification finding** (update section E + resolve in B if needed)

No other category is allowed. This is what stops the endless "new gotchas" loop.

---

## Conclusion

This plan provides a comprehensive roadmap for refactoring the UPSPlus firmware according to all specified rules. The implementation should proceed phase by phase, with thorough testing at each stage. Critical configurations must be preserved, while the logic and structure can be completely refactored to meet the requirements.

The key architectural changes are:
1. Explicit state machine
2. Atomic snapshot mechanism for I2C
3. Canonical scheduler (TIM1)
4. Safe flash persistence
5. Robust battery management
6. Proper button handling

Each phase builds on the previous one, allowing for incremental development and testing.
