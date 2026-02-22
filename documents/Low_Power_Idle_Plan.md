# Low-Power Idle Implementation Plan

This document describes a plan to add sleep-on-idle behavior, optionally reduce wake frequency, and measure power on the UPSPlus firmware (STM32F0).

---

## Summary

**Constraint:** Prefer no new subsystems or files. Implement changes as small edits in existing files; avoid new abstraction layers or helper frameworks.

The firmware currently uses a **busy main loop** with no sleep. The canonical **10 ms scheduler** is driven by SysTick firing every **1 ms** (every 10th tick triggers the scheduler). This plan adds:

1. An **idle path** that enters sleep when there’s nothing to do.
2. **Changing SysTick** from 1 ms to 10 ms: Phase 4 replace `LL_mDelay()` with TIM3-based `WaitMs()`; Phase 5 reconfigure SysTick; Phase 6 update Behavior Spec.
3. A **measurement approach** for before/after power on the 3.3 V rail or MCU VDD.

---

## 1. Add Idle Path When There’s Nothing to Do

### Current State

- The main loop in `main.c` (approx. lines 648–807) is a tight `while(1)` that processes scheduler flags, I2C writes, ADC, flash, protection, GPIO, and refreshes the IWDG. There is no sleep anywhere; the CPU spins continuously.

### Implementation

**Step 1.1 – Define “idle”**

Enter sleep only when no work is pending. Idle when **all** of the following hold:

- No scheduler flags to process: `!sched_flags.tick_10ms && !sched_flags.tick_100ms && !sched_flags.tick_500ms && !sched_flags.tick_1s`
- No ADC result waiting: `!adc_ready`
- Flash not eligible now: not (`flash_save_requested && now_sec >= flash_next_retry_sec`). Idle is **allowed** when flash is pending but backoff has not elapsed (`flash_save_requested && now_sec < flash_next_retry_sec`), because you’re intentionally waiting. Use `now_sec = state.cumulative_runtime_sec` (1 s resolution) when evaluating flash retry eligibility. *Assumption:* `state.cumulative_runtime_sec` continues to advance while the MCU is sleeping (driven by SysTick interrupts). Do not disable SysTick as a wake source in Phase 1–5, or flash retry timing will break.
- No I2C pending write: `!i2c_pending_write.pending`
- No ina_probe due: `!ina_probe_requested` (can loosen later if desired)
- No factory reset requested: `!sys_state.factory_reset_requested`
- No I2C slave transaction active: `I2C1_GetSlaveTxnActive() == 0` (avoids sleeping in the middle of servicing a burst; I2C IRQs will wake, but this improves responsiveness; important because read-only bursts may not set i2c_pending_write.pending)
- No button pending: `sKeyFlag == 0` and `button_handler.pending_click == BUTTON_CLICK_NONE` (ensures prompt processing of button activity)

All of this keeps existing behavior; you only add a branch that calls the sleep sequence when the above conditions are true.

**FLASH note:** Prefer `if ((...) ) { sleep; }` unless `idle_ok` measurably reduces code size under LTO. Avoid helper functions; they can add call/return overhead on Cortex-M0 if not inlined.

**Size-first fallback:** If FLASH is tight, start with only these gates: scheduler flags, `adc_ready`, `i2c_pending_write`, `factory_reset`, and flash-backoff eligibility. Exclude ina_probe, I2C-txn-active, and button-pending gates until proven necessary. If I2C-txn-active is excluded, rely on I2C ISR wake + fast loop for responsiveness; if any host sees latency/NACKs during bursts, re-enable the txn-active gate. Reads don’t set a pending flag; responsiveness relies on I2C IRQ wake + fast loop.

**Step 1.2 – Insert sleep in main loop**

Process all work → kick IWDG → then sleep if idle.

**Rule:** Only enter sleep when interrupts are enabled (PRIMASK == 0). Do not place the sleep call inside any critical section that disables interrupts. Place the sleep block at the very end of the main loop, after all critical sections and after IWDG refresh. Because the sleep block is at the end of the main loop and outside critical sections, PRIMASK should already be 0. No runtime PRIMASK check is required as long as the sleep block remains outside all critical sections.

```c
/* ... process all work ... */
IWDG->KR = 0xAAAAu;   /* Key: reload watchdog counter */
/* Idle path: enter sleep when nothing to do; wake on interrupts (SysTick/DMA/I2C/EXTI); SEV/WFE/WFE avoids lost-wake race */
if (/* idle conditions */)
{
    __SEV(); __WFE(); __WFE();  /* event-safe: clears stale event, then waits for next */
}
```

**Step 1.3 – Wake sources**

We use SEV/WFE/WFE (WFE-based sleep). In practice, any enabled interrupt will wake the CPU from WFE; SEV/WFE/WFE avoids the lost-wake race. Current interrupt sources:

- **SysTick** (1 ms or, if changed, 10 ms) – drives scheduler
- **DMA** (ADC complete) – sets `adc_ready`
- **I2C** (address match / RX / TX / STOPF, etc.) – wakes CPU; write paths may also set `i2c_pending_write.pending`
- **EXTI** (button) – sets `sKeyFlag`
- **TIM3** – used for `WaitUs()` only; no interrupt; not a wake source

Ensure SysTick, DMA, I2C, and EXTI remain enabled so the CPU wakes when work arrives.

**Step 1.4 – Event-safe sleep pattern**

There is a classic sleep race: main checks idle (true) → an interrupt fires and sets work pending → main executes sleep after the interrupt has already completed → CPU may sleep until the next interrupt, delaying work. This usually won’t break correctness (SysTick will wake soon), but it can add latency (e.g. I2C responsiveness if host timing is tight). It’s easy to avoid.

**Requirement:** Use the event-safe sleep pattern `__SEV(); __WFE(); __WFE();` – clears any pending event, then waits for the next one. This ensures that if work was set pending between the idle check and sleep, the CPU does not unnecessarily sleep.

---

## 2. Prepare for and switch SysTick to 10 ms (Phases 4–6)

### Current State

- `SystemClock_Config()` in `main.c` calls `LL_Init1msTick(48000000)` – SysTick fires every **1 ms**.
- `SysTick_Handler` in `stm32f0xx_it.c` uses a static counter 0..9 and calls `Scheduler_ISR_Tick10ms()` on the 10th tick.
- So the **scheduler** effectively runs every **10 ms**; the 1 ms resolution is only used to derive that.
- `LL_mDelay()` uses SysTick; if SysTick becomes 10 ms, all `LL_mDelay(n)` calls would wait 10× longer (e.g. `LL_mDelay(100)` → 1 s). All `LL_mDelay()` usages must be replaced before changing SysTick.

### Implementation

**Step 2.1 – Replace `LL_mDelay()` with TIM3-based `WaitMs()` (Phase 4)**

The project already has TIM3 for `WaitUs()` (1 MHz, 1 µs resolution). Add `WaitMs()` (or equivalent) and replace every `LL_mDelay()` call:

- `Src/main.c`: 1 call – `LL_mDelay(100)` at startup
- `Src/I2C_Slave.c`: 11 calls – `LL_mDelay(1)`, `LL_mDelay(10)`, `LL_mDelay(100)` for I2C recovery and probe delays

`WaitMs()` is a busy loop (does not sleep); use only for startup and I2C recovery/probe delays. Do not call it in periodic runtime paths or it will undermine Phase 1 power savings. Ensure `WaitMs()` is available where needed (e.g. declare in a shared header if `I2C_Slave.c` uses it).

**FLASH note:** Implement `WaitMs()` as a loop of `WaitUs(1000)` to avoid overflow and keep code small. Keep the loop counter as an unsigned local and avoid division/mod; this should compile very small under -Os. Keep it `static` in the C file(s) where used unless shared use clearly reduces total size. Avoid formatting, printing, logging, and generalized timeout utilities. Replace only the exact `LL_mDelay()` call sites; do not introduce new delays. If both main.c and I2C_Slave.c need it, prefer a single shared implementation only if it does not require pulling in extra headers or new compilation units. If `WaitMs()` is duplicated in two files, keep the implementations identical and `static` to avoid accidental linkage growth.

**Step 2.2 – Configure SysTick for 10 ms (Phase 5)**

In `SystemClock_Config()`:

- Stop calling `LL_Init1msTick(48000000)`.
- Configure SysTick manually for 10 ms (e.g. reload = (SystemCoreClock / 100) - 1 for 10 ms). Ensure SysTick uses HCLK (not HCLK/8) before applying the computed reload value. Verify SysTick->CTRL CLKSOURCE selects processor clock (HCLK) before setting the reload. Reload must be computed from the actual HCLK used by SystemClock_Config(). Either call `SysTick_Config(reload + 1)` or program LOAD/VAL/CTRL manually (do not do both).

**Step 2.3 – Simplify SysTick_Handler (Phase 5)**

In `stm32f0xx_it.c`, remove the 0..9 divider. Call `Scheduler_ISR_Tick10ms()` every SysTick interrupt.

**FLASH note:** Do not enable HAL timebase or `HAL_GetTick()` support; keep SysTick handler minimal and only call `Scheduler_ISR_Tick10ms()`.

**Step 2.4 – Tick unit semantics, audit, and doc update**

**Phase 5 (code correctness):** `sched_flags.tick_counter` becomes a **10 ms unit** tick (no longer derived from 1 ms). Any constant named `TICKS_PER_*` must remain consistent with that unit. Before changing SysTick, audit for any use of HAL/LL tick-based timeouts or assumptions of 1 ms tick (e.g. `HAL_GetTick()`, timeout-in-ms logic); ensure nothing relies on SysTick as a millisecond source.

**Phase 6 (docs):** Update only Timing Model + any outdated "1 ms SysTick" references; no other edits. No new diagrams, tables, or prose expansions. Micro-checklist: replace "SysTick 1 ms" → "SysTick 10 ms"; state `tick_counter` unit = 10 ms; ensure `TICKS_PER_*` values match the unit. Doc edit method: search for "1 ms", "1ms", and "LL_Init1msTick" and update only the relevant timing lines. See Phase 6 in Recommended Order of Work and Files to Modify.

---

## 3. Measure Before/After on 3.3 V Rail or MCU VDD

### Current State

- **MCU VDD** is not measured by a separate ADC channel; the MCU is typically powered from the 3.3 V rail.
- **VREFINT** is used in the ADC sequence (channel 5); `state.mcu_voltage_mv` is derived from it (`__LL_ADC_CALC_VREFANALOG_VOLTAGE`), which reflects the actual VDD (supply voltage to the MCU). So `mcu_voltage_mv` is a reasonable proxy for **MCU VDD** (and thus the 3.3 V rail if that’s what feeds VDD).
- **PI_VCC_SENSE** (ADC channel 0) is used for **pogopin voltage**, not the 3.3 V rail directly.

### Measurement Protocol

**Step 3.1 – Before/after setup**

1. **Baseline (before changes)**  
   - Build current firmware (no sleep, 1 ms SysTick).  
   - Power the board from a stable supply (e.g. battery or bench supply).  
   - With **RPi off** and **charger absent** (or minimal load):  
     - Measure **current** on the 3.3 V rail (e.g. in series with a multimeter or µCurrent Gold).  
     - Optionally record **voltage** on the 3.3 V rail with a multimeter (I2C register 0x01–0x02 is not available when the RPi is off, since the RPi is the I2C host).  
   - Let the system run for at least 1 minute and note average idle current.

2. **After adding idle path**  
   - Same conditions.  
   - Measure again.  
   - Compare average idle current before vs after.

3. **After SysTick change (Phase 5)**  
   - Same conditions.  
   - Measure again and compare.

**Step 3.2 – In-firmware (optional)**

- Register **0x01–0x02** (MCU voltage) exposes a VREFINT-derived value. When the RPi (or another I2C host) is on, you can read this over I2C to confirm VDD is stable. For idle current measurement with RPi off, use an external meter; I2C register read is not available in that configuration.

**Step 3.3 – Notes**

- If the 3.3 V rail is shared (MCU + other loads), measure at the MCU VDD pin if possible for MCU-only current.
- VREFINT-based `mcu_voltage_mv` is suitable for “MCU VDD” in the spec; it does not measure current.
- **Realism note:** If 3.3 V rail current is dominated by regulators, pullups, IP5328, etc., sleep may show only a modest improvement. The MCU improvement can be real but masked by board-level quiescent current.
- Record whether pullups / I2C host / regulator enable states are unchanged between runs to keep comparisons honest.

---

## Recommended Order of Work

| Phase | Task | Risk |
|-------|------|------|
| 0 | Measure baseline idle current (3.3 V or MCU VDD) before any firmware change | None |
| 1 | Add sleep idle path (event-safe pattern, after IWDG refresh) | Low – easy to revert |
| 2 | Optional: README note that low-power idle is enabled (WFE sleep-on-idle) | None |
| 3 | Verify behavior (scheduler, I2C, button, ADC, flash); re-measure idle current | Low |
| 4 | Replace `LL_mDelay()` with TIM3-based `WaitMs()` in main.c and I2C_Slave.c | Low |
| 5 | Change SysTick to 10 ms; simplify SysTick_Handler | Medium – touch timing |
| 6 | Required: Behavior Spec update. Only timing model description + references to 1 ms tick; do not touch register map or state machine sections | None |
| 7 | Verify behavior; measure idle current again | None |

---

## Files to Modify

| File | Changes |
|------|---------|
| `Src/main.c` | Add idle condition and sleep in main loop (Phase 1); add `WaitMs()`, replace `LL_mDelay()` (Phase 4); configure SysTick for 10 ms (Phase 5) |
| `Src/I2C_Slave.c` | Replace `LL_mDelay()` with `WaitMs()` (Phase 4) |
| `Src/stm32f0xx_it.c` | Simplify `SysTick_Handler` to call `Scheduler_ISR_Tick10ms()` every time (Phase 5) |
| `documents/UPSPlus_Behavior_Spec.md` | Update only Section 3 (Timing Model) + references to 1 ms tick; document `tick_counter` and `TICKS_PER_*` as 10 ms units. Do not touch register map or state machine sections (Phase 6) |
| `README.md` | Optional brief note about sleep-on-idle (Phase 2) |

---

## Validation Checklist

- [ ] **Phase 1:** Idle path: sleep only when no work is pending (flags, ADC, flash, I2C, ina_probe, factory reset, I2C slave not active, sKeyFlag, button pending); use event-safe sleep pattern (SEV/WFE/WFE).
- [ ] Scheduler: 10 ms, 100 ms, 500 ms, 1 s behavior unchanged.
- [ ] IWDG: main loop (and thus IWDG refresh) still runs within timeout after wake. Confirm worst-case 'work' loop execution + longest sleep interval remains comfortably under IWDG timeout.
- [ ] I2C: host can still read/write; I2C interrupt wakes CPU. Run continuous I2C read bursts while MCU idles; confirm no NACKs or timing issues. Verify I2C host can initiate a transaction immediately after a long idle period (no first-transaction delay / NACK burst). Test both read-only polling bursts and mixed read/write bursts.
- [ ] Button: EXTI wakes CPU; button actions still work. Ensure `pending_click` is cleared promptly after `Button_DispatchActions()` so a sticky value cannot block sleep indefinitely.
- [ ] Flash save and ADC processing unchanged.
- [ ] **Phase 2 (optional):** README includes a brief note about sleep-on-idle (if we choose to document it).
- [ ] **Phase 4:** All `LL_mDelay()` replaced with `WaitMs()`; delays behave as intended.
- [ ] **Phase 5:** SysTick 10 ms; `SysTick_Handler` calls `Scheduler_ISR_Tick10ms()` every tick.
- [ ] **Phase 6 (required):** Behavior Spec: only timing model + 1 ms tick references; do not touch register map or state machine sections.
- [ ] Power: before/after current on 3.3 V (or MCU VDD) measured and recorded.

---

## Risks and Mitigations

1. **Sleep too deep / missed events**  
   Sleep (WFE) keeps CPU halted until an interrupt or event. If an expected wake source (e.g. SysTick) is disabled or delayed, the loop could run late. Keep SysTick (and other IRQs) enabled; test button and I2C in low-activity scenarios.

2. **SysTick change**  
   Any code that assumes 1 ms ticks (e.g. `LL_mDelay`) will behave differently. Audit uses of `LL_mDelay`, HAL ticks, and timing assumptions before changing SysTick.

3. **IWDG**  
   Ensure the loop (including after sleep, SEV/WFE/WFE) runs often enough to refresh the watchdog. With 1 ms or 10 ms SysTick, wake-up is frequent; with 10 ms the margin is larger but still safe if the loop completes within a few milliseconds.

**Doc rule:** Phase 1 sleep-on-idle is an implementation detail → spec change optional. Phase 5 SysTick timebase change is a timing model change → spec change required (Phase 6).
