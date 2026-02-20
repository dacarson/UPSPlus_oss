# SysTick Migration Plan: TIM1 → SysTick for 10 ms Scheduler

This plan moves the canonical 10 ms scheduler tick from TIM1 to SysTick to reduce flash usage. SysTick remains at 1 ms (as set by `LL_Init1msTick`); the 10 ms scheduler is triggered every 10 SysTick ticks. Behavior (10 ms tick, flag-only ISR, main-loop processing) and `LL_mDelay()` remain unchanged.

---

## Phase 1: Prepare and Baseline

**Goal:** Lock down current behavior, locate the real SysTick handler, and measure flash so you can verify savings.

1. **Build and record baseline**
   - Build Release (or your shipping) configuration.
   - Record flash usage from the map file (e.g. `.map` total or `.text` size).
   - Optionally note TIM1-related symbols/sizes in the map for later comparison.

2. **Locate the real SysTick_Handler**
   - `main.c` does **not** define `SysTick_Handler`. The definition is in **`Src/stm32f0xx_it.c`** (Cube IT file) or, if not overridden, the weak `Default_Handler` in the startup file.
   - Find and note the file/line of the actual `SysTick_Handler` definition. You will modify that implementation in Phase 2; otherwise you risk adding a second handler or editing the wrong file and seeing no behavior change.

3. **Confirm SysTick is already enabled**
   - `LL_Init1msTick(48000000)` and `LL_SYSTICK_EnableIT()` are already called in `SystemClock_Config()` in `main.c`. SysTick stays at 1 ms; the 10 ms scheduler will run every 10th tick. No code changes in this phase.
   - **Note:** In this project's STM32F0 LL driver, `LL_mDelay()` uses the **hardware** SysTick COUNTFLAG (`SysTick->CTRL`), not a software tick. It does not require `LL_IncTick()` or any code in `SysTick_Handler` to run. The current empty `SysTick_Handler` is therefore correct for `LL_mDelay()`.

### Phase 1 Results (completed)

- **Baseline flash (Release, from `Release/UPSPlus_oss.map`):**
  - `.text`: 0x31a0 (12,640 bytes)
  - Total program flash (load end): ~0x32c8 (12,968 bytes) from FLASH origin 0x08000800
  - **TIM1 (for later comparison):** `TIM1_BRK_UP_TRG_COM_IRQHandler` at 0x0800199c; `SysTick_Handler` at 0x08001a8c
- **Real `SysTick_Handler`:**
  - **Definition:** `Src/stm32f0xx_it.c` at **line 139** (the body is empty; USER CODE blocks only). This is the file to modify in Phase 2.
  - Declaration: `Inc/stm32f0xx_it.h` line 54. Startup `Startup/startup_stm32f030f4px.s` has weak `SysTick_Handler` → `Default_Handler`; the strong symbol in `stm32f0xx_it.c` overrides it.
- **SysTick enabled:** Confirmed in `main.c`: `LL_Init1msTick(48000000)` and `LL_SYSTICK_EnableIT()` are called in `SystemClock_Config()` at lines 1506–1507. No code changes in Phase 1.

---

## Phase 2: Move Scheduler Logic to SysTick and Disable TIM1

**Goal:** Implement the 10 ms tick and flags in SysTick, disable TIM1 so only one timebase drives the flags, then verify. Do **not** run both TIM1 and SysTick in parallel—that would double-count.

1. **Naming and semantics**
   - **`Scheduler_ISR_Tick10ms(void)`** = the current TIM1 ISR body only: increment `tick_counter`, set `tick_10ms`, update downcounters `ticks_until_100ms` / `ticks_until_500ms`, set `tick_100ms` / `tick_500ms` when they hit zero. This is the minimal, flag-only work the ISR does today.
   - **`Scheduler_Tick10ms()`** (or your existing equivalent) = the **main-loop** work that runs when `sched_flags.tick_10ms` is set. Keep this as-is; it is not moved into the ISR.
   - Implementing `Scheduler_ISR_Tick10ms()` and calling it from SysTick keeps current semantics intact.

2. **Expose ISR tick to the IT module**
   - Implement `Scheduler_ISR_Tick10ms()` in `main.c` (same logic as the current TIM1 ISR body). Declare it in a header included by `stm32f0xx_it.c` (e.g. `ups_state.h` or a small `scheduler_tick.h`) so `SysTick_Handler` can call it.
   - **Keep the SysTick ISR extremely small**, like the TIM1 ISR today: set flags and return. Avoid extra logic or heavy function calls that might pull in more code; sometimes a call costs more flash than inlining, so measure if unsure. If flash savings are smaller than expected, consider making `Scheduler_ISR_Tick10ms()` `static inline` in a header so the compiler can inline it into `SysTick_Handler`.

3. **Implement 10 ms in the real SysTick_Handler**
   - In the **actual** `SysTick_Handler` (in `stm32f0xx_it.c`), use a **0..9** counter so exactly one 10 ms tick runs per 10 SysTicks (avoids off-by-one and drift). For example:
     - `static uint8_t div10 = 0;`
     - `div10++; if (div10 >= 10) { div10 = 0; Scheduler_ISR_Tick10ms(); }`
   - Do **not** reconfigure SysTick; it stays at 1 ms. (No `LL_IncTick()` is needed—`LL_mDelay()` in this driver uses the SysTick hardware COUNTFLAG, not a software tick.)
   - **Optional (belt-and-suspenders):** If you ever see jitter concerns, you can explicitly set SysTick priority (only if you’re already managing priorities elsewhere). On F0 this usually isn’t necessary if your handler stays flag-only.

4. **Disable TIM1 (do not run both timebases)**
   - Add the SysTick-driven 10 ms call as above, then **disable TIM1** so only one source drives the scheduler flags:
     - **Stop the counter:** call `LL_TIM_DisableCounter(TIM1)` or remove the init-block line that enables it (`LL_TIM_EnableCounter(TIM1)`)—so the counter is off, not the enable call accidentally left in place.
     - **Disable the update interrupt:** call `LL_TIM_DisableIT_UPDATE(TIM1)` or remove the line that enables it (`LL_TIM_EnableIT_UPDATE(TIM1)`). Then clear the UIF flag once: `LL_TIM_ClearFlag_UPDATE(TIM1);`—prevents weirdness if you later re-enable TIM1 for other reasons during development.
     - **Disable the TIM1 NVIC line:** `NVIC_DisableIRQ(TIM1_BRK_UP_TRG_COM_IRQn)` (or equivalent). If UIF were ever left set with NVIC still enabled, the vector could still fire; after Phase 3 the handler would be `Default_Handler` (infinite loop). Disabling NVIC makes that failure mode much harder to hit.
     - **Clear pending TIM1 IRQ (belt-and-suspenders):** `NVIC_ClearPendingIRQ(TIM1_BRK_UP_TRG_COM_IRQn)`. Cheap and avoids a stale pending bit firing during the transition.
   - **Correctness:** After Phase 3, TIM1 must **never** generate an interrupt: counter off, update IT off, NVIC off. The vector will point to `Default_Handler`, which is an infinite loop—so TIM1 must not fire.
   - Verify behavior (10 ms / 100 ms / 500 ms / 1 s, LED, ADC, countdowns, button debounce, I2C timing if applicable). Then proceed to Phase 3 to delete TIM1 code.

### Phase 2 Results (completed)

- **`Scheduler_ISR_Tick10ms()`** implemented in `main.c` (flag-only: tick_counter, tick_10ms, downcounters for tick_100ms/tick_500ms). Declared in `Inc/ups_state.h`; `Src/stm32f0xx_it.c` includes `ups_state.h` and calls it from `SysTick_Handler`.
- **SysTick_Handler** (`Src/stm32f0xx_it.c`): static 0..9 divider; every 10th tick calls `Scheduler_ISR_Tick10ms()`. SysTick left at 1 ms; no `LL_IncTick()`.
- **TIM1 never enabled in Phase 2:** TIM1 init block keeps clock + prescaler + ARR only; no `LL_TIM_EnableIT_UPDATE`, no `NVIC_EnableIRQ(TIM1_...)`, no `LL_TIM_EnableCounter`, no `LL_TIM_GenerateEvent_UPDATE`. Defensive disable/clear block runs once after config: `LL_TIM_DisableCounter`, `LL_TIM_DisableIT_UPDATE`, `LL_TIM_ClearFlag_UPDATE`, `NVIC_DisableIRQ(TIM1_BRK_UP_TRG_COM_IRQn)`, `NVIC_ClearPendingIRQ(TIM1_BRK_UP_TRG_COM_IRQn)`.
- **TIM1_BRK_UP_TRG_COM_IRQHandler** left in `main.c` (dead code until Phase 3 removal). Only SysTick drives scheduler flags.
- **Comments added:** downcounters marked ISR-only; `Scheduler_ISR_Tick10ms()` documents Cortex-M0 atomic 32-bit tick_counter. Main loop and `LL_mDelay()` unchanged.

---

## Phase 3: Remove TIM1 from Scheduler

**Goal:** Remove all TIM1 usage for the 10 ms tick and clean up. You **cannot** drop `stm32f0xx_ll_tim.h` entirely—TIM3 is still used (e.g. `I2C_Slave.c`, `main.c` for microsecond timing). Savings come only from TIM1-specific code.

1. **Remove TIM1 initialization**
   - In `main.c`, delete the **exact** TIM1 init block present in your code. Do not rely on a generic macro name: **search for `PERIPH_TIM1`** and remove the TIM1 clock enable and the rest of the TIM1 block: prescaler, auto-reload, update interrupt enable, **NVIC enable for TIM1** (`NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn)` or equivalent), counter enable, and any update-event generate call. (Phase 2 already disabled the TIM1 NVIC; removal here eliminates the enable call so TIM1 can never fire after this phase.)

2. **Remove TIM1 ISR**
   - Remove `TIM1_BRK_UP_TRG_COM_IRQHandler` from `main.c`.
   - Remove its declaration from `Inc/stm32f0xx_it.h`. The startup file will still have the vector; it can point to the weak `Default_Handler` so the binary remains valid.

3. **Ensure only SysTick calls the ISR tick**
   - Confirm `Scheduler_ISR_Tick10ms()` is only called from `SysTick_Handler`. No further changes if already done in Phase 2.

4. **Verify**
   - Build Release again; confirm no link errors and that TIM1 is no longer referenced for the scheduler.
   - Record new flash usage; compare to Phase 1 baseline. Savings = TIM1 init/config, NVIC for TIM1, TIM1 ISR, and TIM1-specific LL calls only (~80–150 bytes typical).

### Phase 3 Results (completed)

- **TIM1 initialization removed** from `main.c`: PERIPH_TIM1 clock enable, prescaler, auto-reload, and defensive disable/clear block deleted from the ADC/TIM init function.
- **TIM1 ISR removed:** `TIM1_BRK_UP_TRG_COM_IRQHandler` deleted from `main.c`.
- **TIM1 handler declaration removed** from `Inc/stm32f0xx_it.h`; vector in startup still points to weak `Default_Handler`.
- **Scheduler_ISR_Tick10ms()** is only called from `SysTick_Handler` in `stm32f0xx_it.c` (confirmed). LL TIM retained for TIM3 (I2C_Slave, main.c microsecond timing).
- **Flash (from `Release/UPSPlus_oss.map` after user build):**
  - `.text`: 0x3114 (12,564 bytes). Phase 1 baseline was 0x31a0 (12,640 bytes) → **saving 76 bytes** in code.
  - Program load end: _sidata + .data = 0x080039dc + 0x58 → 0x08003a34; size from FLASH origin 0x08000800 = 0x3234 (12,852 bytes). Phase 1 baseline ~0x32c8 (12,968 bytes) → **saving ~116 bytes** total program flash.
- **Symbols:** `TIM1_BRK_UP_TRG_COM_IRQHandler` at 0x08003940 = `Default_Handler` (weak, from startup); no custom TIM1 handler. `SysTick_Handler` at 0x08001a30; `Scheduler_ISR_Tick10ms` from main.o (called only from SysTick).

---

## Phase 4: Update Documentation and Spec (completed)

**Goal:** Keep the behavior spec and code comments accurate and timebase-agnostic.

1. **Behavior spec** (`documents/UPSPlus_Behavior_Spec.md`)
   - Section 3 (Timing Model): Change “TIM1 ISR sets flags only” to “canonical 10 ms tick (generated from SysTick) sets flags only” or similar—so it’s clear SysTick remains 1 ms and the 10 ms tick is derived by dividing. Keep “Canonical scheduler tick: 10 ms” and all derived rates unchanged.

2. **Code comments and headers (timebase-agnostic)**
   - Replace **literal** “TIM1” references with “canonical 10 ms tick” (or “scheduler tick”) so they stay true. Examples to update:
     - “Canonical scheduler – TIM1 sets flags only” → “Canonical scheduler – canonical 10 ms tick (generated from SysTick) sets flags only”
     - `state.snapshot_tick = sched_flags.tick_counter;  /* canonical TIM1 tick */` → `/* canonical 10 ms tick */`
   - `Inc/ups_state.h`: Update TICK_PERIOD_MS comment, `scheduler_flags_t`, `snapshot_tick`, `press_start_tick`, `IsTrueVbatSampleFresh` docs from “TIM1 tick” to “canonical 10 ms tick” or “scheduler tick.” Adjust “MUST NOT use SysTick milliseconds” only if still needed; otherwise “MUST use canonical 10 ms tick for freshness.”

3. **Change impact**
   - In the spec’s Change Impact Map (Section 16), add: “If the scheduler timebase (10 ms) source changes (e.g. SysTick vs TIM), update Section 3 and all comments referring to the interrupt source.”

### Phase 4 Results (completed)

- **Behavior spec** (`documents/UPSPlus_Behavior_Spec.md`): Section 3 (Timing Model) updated to "canonical 10 ms tick generated from SysTick sets flags only"; Section 16 (Change Impact Map) updated with scheduler timebase change-impact bullet.
- **main.c**: Comments updated—timing/button helpers, canonical scheduler (two places), snapshot_tick, and "run tasks from scheduler flags."
- **Inc/ups_state.h**: Core Principles (canonical 10 ms tick from SysTick); TICK_PERIOD_MS block and define comment; state timing, press_start_tick, snapshot_tick (canonical 10 ms tick); scheduler_flags_t doc (canonical 10 ms tick); IsTrueVbatSampleFresh doc (canonical 10 ms tick, MUST use for freshness).

---

## Summary Checklist

- [x] Phase 1: Baseline flash recorded; real `SysTick_Handler` location noted.
- [x] Phase 2: `Scheduler_ISR_Tick10ms()` in `SysTick_Handler` via 0..9 counter; TIM1 never enabled, defensive disable/clear; behavior verified.
- [x] Phase 3: TIM1 init (search `PERIPH_TIM1`) and TIM1 ISR removed; flash saving confirmed; LL TIM retained for TIM3.
- [x] Phase 4: Behavior spec and code comments updated to “canonical 10 ms tick” / timebase-agnostic wording.

---

## Risk and Rollback

- **Risk:** SysTick priority or timing might interact with other interrupts (I2C, DMA, EXTI). Keep the 10 ms handler minimal (flags only); compare NVIC priorities if you see odd behavior.
- **Rollback:** Revert to the pre-migration commit; or re-add the TIM1 init and TIM1 ISR and remove the SysTick 10 ms logic to restore the previous design.
