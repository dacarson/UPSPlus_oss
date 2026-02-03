# INA219 Current Measurement Plan (Firmware-Only)

## 1. Purpose and Scope
Implement reliable current measurement for:
- Output / RPi rail (INA219 at 0x40)
- Battery charge/discharge path (INA219 at 0x45)

This plan is firmware-only and designed to preserve the existing behavior rules and
I2C slave priority described in `UPSPlus_Behavior_Spec.md`.

## 2. System Constraints (Must Respect)
- STM32 is primarily an I2C **slave** to the Raspberry Pi.
- Single I2C peripheral shared with two INA219 devices.
- Master and slave roles cannot be active simultaneously.
- No changes to hardware or bus topology.
- No side effects on I2C reads; snapshots remain the only read path.

## 3. Measurement Model
- Read **shunt voltage register only** (signed 16-bit).
- Ignore INA bus voltage, current, power, and calibration math.
- Scaling rule (normative):
  - `current_mA = shunt_raw`
  - 1 LSB = 1 mA (10 uV / 10 mOhm).
- Signed value is meaningful and stable; keep sign.

## 4. Boot-Time Master Phase (Before Slave Enable)
1. Configure I2C as **master @ 100 kHz**.
2. Program INA219 configuration register (0x00) for deterministic shunt accuracy:
   - Use PGA = /4 (±160 mV) to tolerate current spikes above 8 A without clipping.
   - Bitfield definition:
     - BRNG = 16 V
     - PGA = /4 (±160 mV)
     - BADC = 12-bit (532 us)
     - SADC = 12-bit (532 us)
     - MODE = shunt continuous (bus conversions disabled; no dependency on bus voltage)
   - Example hex value:
     - BRNG=16 V: **0x0B9D**
3. Optional sanity reads of shunt register to confirm presence.
   - If a device does not respond at boot, mark its valid flag 0 and continue;
     periodic reads will retry and may recover later.
   - A device absent at boot may recover later via periodic reads.
   - Periodic reads continue in the alternating cadence; a successful read flips
     valid=1 and resets age.
   - If a device remains absent, its age stays saturated (0xFFFF) and valid
     remains 0 until the first successful read.
   - Optional: read back config register 0x00 and verify it matches the write;
     if it does not match, treat the device as absent for that boot.
4. Reconfigure I2C as **slave**.
5. Enable slave address acknowledgement only after step 4 completes.

## 5. Runtime Master Window (Time-Sliced)
### 5.1 Entry Conditions (All Required)
Only enter master mode if:
1. No active slave transaction is in progress.
   - An active slave transaction is defined from ADDR match until STOPF.
   - ADDR match means an ADDR interrupt for the STM32's own slave address.
   - Implementation MUST maintain `i2c_slave_txn_active` (set on ADDR match,
     cleared on STOPF for own address) and check it before entering master mode.
2. A STOP ending a transaction **addressed to the STM32 slave address** has been observed.
   - Detect via the I2C peripheral STOPF flag (or equivalent) for the STM32's own address.
   - Only count STOPF if an ADDR match was seen since the last STOPF
     (track `last_addr_event_ticks` and `last_stop_event_ticks`).
   - STOPs from other bus transactions are not considered.
   - STOPF must be recent: observed within the last **50 ms**.
   - This is an intentional, ultra-conservative gate: if the Pi is idle longer
     than this window, INA sampling pauses until a new STM32-addressed
     transaction occurs.
   - This eliminates collision risk with non-STM32 traffic; if sampling while
     Pi is idle is later required, increase this window and revalidate.
3. A guard time of **500 us** elapsed since STOP.
4. Bus idle stable for guard window:
   - SDA = HIGH
   - SCL = HIGH
   - Stability check: take exactly 50 samples at 10 us spacing across the 500 us
     guard window; if any sample is low, the check fails.
   - Sampling schedule: t = 0..490 us inclusive (50 samples).

If any condition fails, skip and retry at the next period.

### 5.2 Window Actions (Hard Timeout <= 2 ms)
1. Disable slave mode by disabling the I2C peripheral (PE=0); do not leave ACK
   enabled while in master mode.
2. Configure I2C as master @ 100 kHz.
   - Master and slave use independent timing configs; role switch must load the
     correct timing each time.
3. Read INA219 shunt voltage register (INA219 register `0x01`) as **signed 16-bit big-endian**.
   - Transaction shape: write pointer `0x01`, then repeated-start read 2 bytes.
   - Convert to `int16_t` for `current_mA = shunt_raw`.
   - If shunt voltage exceeds ±160 mV (±16 A on 10 mOhm), readings clip/saturate.
     Valid flag still reflects freshness, not accuracy beyond this range.
   - Clipped readings are still reported as the saturated int16 value.
   - Future: an optional "clipped" status bit could be added if needed.
4. Restore slave configuration immediately.
5. Enforce timeout for the **entire window** (including slave/master re-init);
   on timeout, disable the I2C peripheral (PE=0) to release SDA/SCL, then
   reinit slave mode.
6. Implementation MUST use a fast role-switch path; it must not call heavy init
   routines that routinely exceed 2 ms.
7. Role switching must not include blocking waits other than bounded polling for
   immediate flags; all loops must check remaining time budget.
8. Implementations must not relax the <= 2 ms window without updating this plan.

## 6. Sampling Policy (Least Intrusive)
- Base period: 500 ms.
- Alternate channels each period:
  - Tick N: read 0x40 (output current)
  - Tick N+1: read 0x45 (battery current)
- Each channel updates once per second.
- Scheduling driven by 10 ms main-loop tick (no ISR work beyond flags).

## 7. Data Model and Snapshot Integration
Add to authoritative state:
- `int16_t output_current_mA`
- `int16_t battery_current_mA`
- `uint8_t output_current_valid`
- `uint8_t battery_current_valid`
- `uint16_t output_current_age_10ms`
- `uint16_t battery_current_age_10ms`

Rules:
- Update cached values **only** on successful reads.
- If a read is skipped or fails, do not overwrite cached values.
- Snapshot copies cached values and validity metadata only.
- I2C reads must not mutate state.
- Valid flag means **fresh within the last 2 seconds** (age <= 200 * 10 ms).
- Age counters increment every 10 ms tick in the main loop.
- `*_age_10ms` counts in 10 ms units (increment by 1 every 10 ms tick).
- Internal age counters **saturate and must not wrap**.
- Freshness is based on the **last successful INA read timestamp**, not attempts.
- Boot semantics: valid flags start at 0; ages start at **0xFFFF** until first success.

## 8. I2C Register Map Additions (Current Values)
Expose current values in the main register map (little-endian, RO):
- `0x2E–0x2F`: `output_current_mA` (int16, signed, 1 mA/LSB)
- `0x30–0x31`: `battery_current_mA` (int16, signed, 1 mA/LSB)
- `0x32`: `current_valid_flags` (RO bitfield)
  - bit0: output_current_valid
  - bit1: battery_current_valid
  - bits2–7: reserved (0)
  - Serialization: int16 values are little-endian (LSB at lower address).

If a read is skipped or fails, cached current values are not overwritten; the valid
flag indicates whether the cached value is fresh. When valid=0, registers still
return the last cached value; consumers must consult valid.

Phase 1 allocates registers only; current values remain 0 until phases 2-4
implement INA sampling and snapshot population.

Expose boot presence via Factory Testing selector:
- Selector `0x06`: INA boot presence page
  - `0xFD`: bit0=output(0x40) present, bit1=battery(0x45) present
  - `0xFE`: 0
  - `0xFF`: 0


Expose age via Factory Testing selector:
- Selector `0x07`: Current age page
  - `0xFD`: `output_current_age_10ms` (uint8, age in 10 ms units, saturates at 255)
  - `0xFE`: `battery_current_age_10ms` (uint8, age in 10 ms units, saturates at 255)
  - `0xFF`: reserved (0)
  - Mapping rule: export `min(age_10ms, 255)` (no wrap/truncation).
- Factory selector reads are snapshot-based and do not update mid-transaction.

## 9. Error Handling and Recovery
On any of:
- NACK
- Arbitration loss
- Unexpected START/STOP during a master window (peripheral error flags, e.g. NACKF/ARLO/BERR/OVR)
- Bus fault

Then:
1. Abort transaction.
2. Restore slave mode immediately.
3. Do not update cached values.
4. Retry on next scheduled interval.

Required bus recovery (guarded):
- Attempt recovery only if:
  - SDA is low continuously for > **5 ms**, and
  - No slave transaction is active, and
  - SCL is HIGH (not being actively clocked), and
  - No STM32-addressed transaction start/stop observed in the last **5 ms**
    (start = ADDR match, stop = STOPF; track `last_addr_event_ticks` and
    `last_stop_event_ticks`; this is a best-effort quiet-bus heuristic).
- Recovery action: clock SCL (GPIO open-drain) ~9 pulses.
- Restore I2C peripheral and slave mode after recovery.

## 10. Invariants (Must Always Hold)
- STM32 remains a responsive I2C slave.
- Master window is short, bounded, and time-sliced.
- No ISR logic beyond flags; all timing in main loop.
- Current scaling is always 1 mA per LSB.
- INA calibration register is not required at runtime.

## 11. Implementation Phases
### Phase 1: Plan and Register Allocation
- Confirm register addresses do not collide with existing map.
- Decide validity/age metadata exposure (flags + Factory Testing age page).
- Document Factory Testing selector 0x06 (INA boot presence) when implemented.

Status: Phase 1 complete (register allocation, docs/test updates, reserved-region alignment).

### Phase 2: Boot-Time Master Setup
- Add master-mode init before slave enable.
- Program INA219 configuration register (0x00) to a known mode (bitfield defined above).
- Sanity reads to verify both devices respond.
- Expose boot presence via Factory Test selector 0x06 (bit0=0x40, bit1=0x45).

Status: Phase 2 complete (boot-time master setup and presence exposure).

### Phase 3: Runtime Master Window
- Implement guard-time + bus-idle checks using **TIM3** for sub-ms timing.
- Configure TIM3 as a free-running 1 MHz counter (1 us ticks).
- TIM3 runs free; **no TIM3 ISR** is used; the master-window state machine runs
  **in the main loop only**.
- Switch slave->master->slave with hard timeout.
- Alternate devices every 500 ms tick.
  - Temporary implementation: run guarded boot-probe on 500 ms tick to validate
    guard window under NUT traffic; replace with shunt reads in Phase 3/4.

Status: Phase 3 complete (runtime master window and timed guard logic).

### Phase 4: Snapshot Integration
- Add cached values + validity to authoritative state.
- Copy into snapshot without side effects on read.
- Ensure multi-byte atomicity in snapshot layout.

Status: Phase 4 complete (current cache/validity/age + snapshot mapping + factory age page).

### Phase 5: Fault Handling + Recovery
- Handle NACK/arbitration/bus faults by abort + restore slave.
- Optional GPIO clock-out recovery for stuck SDA.
- Verify timeout always restores slave mode.

Status: Phase 5 complete (guarded recovery + timeout restore).

### Phase 6: Validation
- Verify bus responsiveness under repeated INA reads.
- Confirm sign correctness and 1 mA/LSB scaling.
- Validate alternating cadence and valid-flag behavior.

Status: Phase 6 complete (validation checks added to `tools/testing/upsplus_i2c_test.py`).

## 12. Validation Checklist
- I2C slave remains responsive during repeated INA reads.
- Signed current direction matches physical flow.
- 1 mA/LSB confirmed against known load.
- Alternating cadence verified (each channel updated every 1s).
- Timeout path restores slave mode and preserves cached values.
