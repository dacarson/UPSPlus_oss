# MCU Reliability Features Implementation Plan

This document outlines the implementation plan for STM32F030F4 hardware features that improve reliability with minimal (or zero) flash impact.

---

## Current State Summary

| Item | Status |
|------|--------|
| **HardFault handler** | Default infinite loop; no safe outputs, no reset |
| **Reset cause** | Not captured or exported via I2C |
| **CRC** | Software CRC32 in `Flash_ComputeCrc32()` (~25 lines) |
| **I2C** | LL driver; no explicit analog/digital filter or timeout configuration |
| **Main loop** | Structured `while(1)` with scheduler flags; ideal for IWDG petting location |
| **Factory test region** | Pages 0x00–0x07 in use; can add reset-cause page |

---

## Phase 1: IWDG (Independent Watchdog) — Highest ROI

**Goal:** Add a hardware safety net that recovers from hangs.

### Tasks

1. **Add IWDG init** (early in `main()`, after clock config):
   - Unlock IWDG (`IWDG->KR = 0x5555`)
   - Set prescaler and reload for timeout
   - Start watchdog (`IWDG->KR = 0xCCCC`)

   **Timeout calculation:**
   - LSI can vary significantly (temp/voltage); use conservative margin
   - Choose timeout **larger than worst-case main-loop latency** (flash save, INA probe, etc.)
   - **Default: 8 s** unless you have a reason for faster recovery
   - If you have long blocking operations, either (a) break into chunks, or (b) refresh IWDG only after “healthy state reached” (plan already places refresh at end of loop)

2. **Pet IWDG in main loop** — exactly once per iteration, **after** critical work:
   - Place `IWDG->KR = 0xAAAA` at end of `while(1)`, after:
     - `Scheduler_Tick10ms()` / `tick_100ms` / `tick_500ms` / `tick_1s`
     - `ProcessI2CPendingWrite()`
     - INA219 probe
     - Flash save attempt
     - `Protection_ForceCutIfTimedOut()` and `ApplyGPIOFromState()`
   - **Never** pet in I2C ISR or other ISRs (so an I2C storm or deadlock can’t keep it alive)

3. **Verify option bytes**:
   - Ensure IWDG is not disabled in option bytes (default: enabled)
   - Optionally enable HW watchdog on Stop/Standby if needed

**Files:** `Src/main.c`  
**Flash impact:** ~20–30 bytes (init + one KR write)  
**Dependencies:** None

---

## Phase 2: SRAM Parity & HardFault Handler

**Goal:** Ensure parity is not defeated; make HardFault fail-safe (safe outputs + reset).

### HardFault Safe State Definition

Define safe outputs in terms of actual GPIO levels and board-side effects. The handler must set pins **without** relying on HAL, LL init, or function calls that could fault. Use direct register writes only.

| Pin | Port | Active Level | Safe Output | Board Effect |
|-----|------|--------------|-------------|--------------|
| **MT_EN** (PA6) | GPIOA | High = RPi powered | **LOW** | Power off RPi immediately |
| **IP_EN** (PA5) | GPIOA | High = charger enabled | **LOW** | Disable charger path immediately |
| **PWR_EN** (PA7) | GPIOA | High = MCU powered | **HIGH** | Keep MCU alive for reset (hold-up allowed briefly). *Verify on your board:* if MCU is always powered (PWR_EN doesn't gate MCU supply), mark N/A. If N/A, don't touch it in HardFault (avoid unexpected side effects). |

**Implementation constraints:**
- Pins may be in analog/input state, or already misconfigured, if fault occurs after partial init
- **Enable GPIOA clock first**: write `RCC->AHBENR |= RCC_AHBENR_GPIOAEN` (direct RCC register) before touching GPIO registers
- **Set MODER, OTYPER, OSPEEDR, PUPDR explicitly** for each pin (output mode, push-pull, low/med speed, no-pull) before output level, so you don't inherit a prior bad config
- **Set outputs using `GPIOx->BSRR`** (set and reset bits) — avoid ODR read-modify-write, which can be risky if something else touched ODR or you fault mid-write
- No hold-up delay: set safe levels immediately; then trigger reset

### Tasks

1. **Verify parity is enabled**:
   - Cortex-M0 has no parity; STM32F0 SRAM parity is hardware (check if any config disables it)
   - Typically enabled by default

2. **Implement deterministic HardFault handler**:
   - Enable GPIOA clock (direct `RCC->AHBENR` write) if not guaranteed already
   - For each pin: set MODER=output, OTYPER=push-pull, OSPEEDR=low/med, PUPDR=no-pull (don't inherit bad config)
   - Drive MT_EN LOW, IP_EN LOW, PWR_EN HIGH (or omit PWR_EN if N/A) per table above using `GPIOA->BSRR` (not ODR)
   - Optionally write a flag to backup SRAM or known flash location (if available)
   - Trigger reset via **direct register write** (zero dependencies):  
     `SCB->AIRCR = (0x5FAu << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;`  
     Then `for (;;) { __NOP(); }` — wait forever in case reset request is delayed; keeps behavior deterministic.  
     (Do not call `NVIC_SystemReset()` — it's a function call that may rely on CMSIS state.)
   - Handler must be minimal: no HAL, no LL init, no function calls that could fault

3. **Guard against recursive faults**:
   - Keep handler minimal (no function calls that could fault)
   - Consider simple flag: if already in HardFault, just loop (avoid double-fault)
   - **Verification:** Confirm handler does not depend on healthy stack; consider `__attribute__((naked))` or minimal stack use if needed

---

## Phase 3: Reset Cause Logging & NRST Hygiene

**Goal:** Export reset cause for diagnostics; document NRST hardware robustness.

### Reset Cause Persistence: Decision

**Chosen: Option B (persist in flash)** unless flash-write minimization is a hard goal.

- **A. Boot-only (simple):** Populate I2C-exported reset-cause bytes during early init and **never overwrite them**. Reset-cause bytes are written once during early init and never touched again. Static variable survives; Pi reads whenever. Validation: confirm reset-cause bytes are available immediately after I2C comes up. Risk: if snapshot buffers or factory test pages are re-initialized/overwritten, reset cause could be lost. HardFault path may reset before snapshot export; ensure init populates before any fault can occur.

- **B. Persist in flash (recommended):** Add 1 byte `last_reset_cause` + 1 byte `last_reset_seq` (or integrate into existing `flash_persistent_data_t`), saved during normal flash save (not in fault handler). Survives across resets; best for UPS diagnostics.

### Tasks

1. **Capture reset cause at boot**:
   - Read `RCC->CSR` immediately after clock config
   - **Order matters:** snapshot the flags *before* clearing
   - Extract flags: `LPWRRSTF`, `WWDGRSTF`, `IWDGRSTF`, `SFTRSTF`, `PORRSTF`, `PINRSTF`, `BORRSTF`
   - Store in static variable (e.g., `last_reset_cause`)

2. **Clear reset flags**:
   - Set `RCC->CSR |= RCC_CSR_RMVF` *after* you snapshot them, so next reset reflects fresh cause

3. **Export via I2C** — chosen encoding (5 bytes):
   - Add factory test page (e.g., 0x08). **Layout:**
     - **Byte0:** Normalized primary cause (one-hot: 0x01=LPWR, 0x02=WWDG, 0x04=IWDG, 0x08=SFT, 0x10=POR, 0x20=PIN, 0x40=BOR). When multiple flags are set, prefer watchdog (IWDG/WWDG) > software > POR/PIN > BOR > LPWR.
     - **Bytes 1–4:** Full 32-bit `RCC->CSR` snapshot (LSB first), most debuggable
   - Update `Snapshot_UpdateDerived()` / `StateToRegisterBuffer()` and register map
   - If boot-only: populate during early init and never overwrite; validate availability after I2C up

4. **NRST hardware** (documentation only):
   - Add note in hardware docs or README:
     - Datasheet-recommended RC on NRST
     - Optional filtering cap for noise immunity
   - No firmware change; schematic/PCB review

**Optional: Fault reason byte (recommended):** Alongside reset cause, consider 1 byte `last_fault_code` (e.g., 0=none, 1=assert, 2=hardfault, 3=stack overflow, 4=protection-triggered reset). Optionally 1 byte `last_fault_detail` (subcode). Export via factory test page. Even if not persisted, having it in RAM on boot helps diagnostics.

**Files:** `Src/main.c`, `Inc/ups_state.h` (register map), behavior spec docs  
**Flash impact:** ~40–60 bytes  
**Dependencies:** Phase 1 complete if you want IWDG resets visible in logs

---

## Phase 4: CRC Peripheral for Flash Persistence

**Goal:** Replace software CRC with hardware CRC (potentially smaller flash, more reliable).

### CRC Compatibility Decision (Do Before Implementation)

Explicit decision gate — pick one to avoid mid-implementation stuck state:

| Option | Condition | Action |
|--------|-----------|--------|
| **A. Internal only** | Persistence format has no external CRC expectation (Pi/tools don't verify CRC) | Switch to HW CRC; bump record version; HW CRC is non-reflected (poly 0x04C11DB7). Flash neutral or smaller. |
| **B. Must stay identical** | External tools or format require identical CRC bytes | Keep software CRC32 (reflected, 0xEDB88320). HW CRC with reflection tricks on F0 is usually not worth it. |
| **C. Migration** | Willing to migrate tools | Change algorithm, bump version, update any Pi-side validation. |

**Prefer LL CRC, not HAL:** `HAL_CRC_MODULE_ENABLED` may increase flash by pulling in HAL. Use `stm32f0xx_ll_crc.h` and direct register access to keep flash neutrality.

### Tasks

1. **Use LL CRC (do not enable HAL CRC)**:
   - Include `stm32f0xx_ll_crc.h` and `stm32f0xx_ll_bus.h` for CRC clock
   - Enable CRC clock: `LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC)`
   - No change to `stm32f0xx_hal_conf.h`

2. **Configure CRC** (after compatibility decision):
   - STM32F0 CRC: polynomial 0x04C11DB7, init 0xFFFFFFFF, no reflection
   - Software CRC uses 0xEDB88320 (reflected) — incompatible. Apply decision above.

3. **Replace `Flash_ComputeCrc32`** (only if Option A or C):
   - Reset CRC (`LL_CRC_ResetCRCCalculation`)
   - Feed data 32-bit aligned; handle trailing bytes if `FLASH_CRC_SIZE` not multiple of 4
   - Return `LL_CRC_ReadReg(CRC, DR)`

   **If Option B (keep SW CRC):** No code change; keep `Flash_ComputeCrc32()`. Add a known-vector test: store a fixed test buffer + expected CRC constant (documented); run at boot in debug builds or via a factory-test selector.

4. **Validate**:
   - Compare HW vs SW CRC on known record (after migration)
   - Run full flash load/save cycle to confirm integrity

**Files:** `Src/main.c`  
**Flash impact:** Neutral or slightly smaller (removes SW CRC loop); HAL avoided  
**Dependencies:** None (can parallel with other phases)

---

## Phase 5: I2C Analog/Digital Filters & Timeouts

**Goal:** Improve I2C robustness in noisy environments (long wires, Pi noise).

### I2C Timeout & Errata Verification (Do First)

- **Verify STM32F030 I2C1 timeout support:** Check the STM32F030 Reference Manual — timeout features (TIMEOUTEN, TIMEOUT register) vary by I2C peripheral revision. Some F0 parts lack full timeout machinery or behave differently than later STM32 lines.
- **If timeout not supported or unreliable:** Define alternative "stuck-bus recovery" strategy, e.g.:
  - Reinit I2C on STOP timeout detected in software
  - Bus-clear by toggling SCL (e.g., existing `I2C_RunRecovery()`-style logic)
- **Check STM32F0 I2C errata:** Review errata for your specific silicon revision (clock stretching, TXIS behavior, etc.). Matters for reliability work.

### Filter Config Order

Configure analog and digital filter settings **before** `LL_I2C_Enable(I2C1)`. Some filter bits are safer to set while the peripheral is disabled.

### Tasks

1. **Analog filter** (before enabling I2C):
   - Default is enabled; confirm `LL_I2C_EnableAnalogFilter(I2C1)` (or that it’s not disabled)
   - Add explicit enable if unsure

2. **Digital filter** (before enabling I2C):
   - Add `LL_I2C_SetDigitalFilter(I2C1, 1)` — **default 1** I2C clock; tune if bus is marginal

3. **I2C timeout** (only if verified supported):
   - Enable `TIMEOUTEN` and set reasonable timeout
   - Avoids “stuck bus” holding peripheral in bad state indefinitely
   - If not supported: document and implement alternative stuck-bus recovery

4. **Validate**:
   - Test with long cables or noisy setup if available
   - Confirm I2C still works correctly under repeated-start and short reads after filter changes (common regression point)
   - **No-regression for recovery path:** Verify repeated-start read, mid-transfer STOP, and master abort still behave and don't wedge the peripheral

**Files:** `Src/I2C_Slave.c` — add filter config in `MX_I2C1_Slave_Init()` (the function that configures I2C1 timing/addressing; see `I2C_Slave.c`). If init is refactored, apply to the actual I2C1 config entrypoint.  
**Flash impact:** ~10–20 bytes (register config)  
**Dependencies:** None

---

## Implementation Order

| Order | Phase | Est. Effort | Risk |
|-------|-------|-------------|------|
| 1 | IWDG | 1–2 hrs | Low |
| 2 | HardFault | 1–2 hrs | Low |
| 3 | Reset cause | 1–2 hrs | Low |
| 4 | CRC | 2–3 hrs | Medium (format compatibility) |
| 5 | I2C filters | ~1 hr | Low |

Phases 4 and 5 can be implemented in parallel.

---

## Verification Checklist

- [ ] IWDG resets device when main loop artificially blocked
- [ ] HardFault drives outputs safe and triggers reset
- [ ] HardFault path does not depend on healthy stack (keep handler naked/minimal if needed)
- [ ] Reset cause (especially IWDG) visible via I2C after watchdog reset
- [ ] Reset-cause bytes correct across: POR, NRST pin reset, software reset, IWDG reset
- [ ] Flash persistence still validates after CRC migration (if changed)
- [ ] I2C stable with filters/timeout enabled
- [ ] I2C still works correctly under repeated-start and short reads after filter changes
- [ ] I2C recovery: repeated-start read, mid-transfer STOP, master abort don't wedge peripheral

---

## References

- STM32F030x4/x6 Reference Manual — IWDG, CRC, I2C, RCC_CSR
- STM32F030x4/x6 Datasheet — NRST recommendations, electrical characteristics
- STM32F0xx Errata Sheet — I2C (clock stretching, TXIS, etc.)
