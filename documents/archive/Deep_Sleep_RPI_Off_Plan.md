# Deeper Sleep When RPi Power Is Off and Running on Battery

## Context

`documents/Low_Power_Idle_Plan.md` Section 4 describes a future enhancement: when the UPSPlus is running on battery with the RPi powered off (`power_state ∈ {RPI_OFF, PROTECTION_LATCHED}` and `charger_state == ABSENT`), there's no I2C host to stay responsive for, so the MCU can drop into a much deeper sleep than the existing per-10ms-tick `WFE` idle path (Phase 1, already implemented at `Src/main.c:1171-1189`). The doc explicitly left the wake mechanism unscoped ("RTC vs. a slow-counted SysTick"). This plan scopes and implements it.

Goal: when idle and in that battery/RPi-off condition, enter Cortex-M0 **Stop mode** (clocks halted, µA-range current vs. mA-range in Sleep mode), waking only on a periodic charger-presence check (~5s) or a button press, then resume normal operation exactly as today.

## Key finding that changed the mechanism

The originally-assumed RTC Wakeup Timer **does not exist on this silicon** (STM32F030x6 — confirmed via `Drivers/CMSIS/Device/ST/STM32F0xx/Include/stm32f030x6.h`: no `WUTR`/`RTC_CR_WUTE`/EXTI line 20; that block only exists on the "xC" density F0 parts). The verified alternative, using hardware already present on this part:

**RTC Alarm A with all four match-mask bits set**, combined with a slowed-down calendar prescaler (`RTC->PRER`) so the alarm's `ck_spre` tick — and therefore `ALRAF`/EXTI17 — fires every ~5s, forever, with **no rearming needed** between wakes (set once at boot). This is small (register writes only, no BCD/calendar arithmetic at runtime) and fits the project's existing flash-conscious, hand-rolled-register style (matches `IWDG`/`WaitUs` precedent).

Clocked from the existing LSI (already enabled for IWDG at `Src/main.c:901`). The wake period tracks LSI tolerance (28-56kHz → 3.57-7.14s, nominal 5s at 40kHz) but its **ratio to the IWDG timeout is LSI-frequency-independent** (both clocked from the same oscillator): `199936 / 447744 ≈ 44.7%`, i.e. always ~2.2x margin under the watchdog regardless of actual LSI frequency — no watchdog risk across the whole tolerance band.

## Implementation

### 1. One-time RTC/Alarm init — `Src/main.c`, right after the IWDG start sequence (after line 911, before `MX_TIM3_Init();` at line 913)

```c
/* RTC Alarm A, all fields masked, as a free-running ~5s periodic wake source for
 * deep sleep (see Section 4 of Low_Power_Idle_Plan.md). This part has no RTC
 * Wakeup Timer block -- an all-masked Alarm A compare against a slowed ck_spre
 * is the equivalent: fires every ck_spre tick forever, no rearm needed. */
PWR->CR |= PWR_CR_DBP;                              /* unlock backup domain writes */
{
    uint32_t lsi_wait = 100000u;                    /* bounded; proceed regardless, same idiom as IWDG start */
    while (!(RCC->CSR & RCC_CSR_LSIRDY_Msk) && --lsi_wait) { }
}
RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_RTCSEL_Msk) | RCC_BDCR_RTCSEL_LSI;
RCC->BDCR |= RCC_BDCR_RTCEN_Msk;
RTC->WPR = 0xCAu;
RTC->WPR = 0x53u;
RTC->ISR |= RTC_ISR_INIT_Msk;
{
    uint32_t init_wait = 100000u;
    while (!(RTC->ISR & RTC_ISR_INITF_Msk) && --init_wait) { }
}
RTC->PRER = (127u << 16) | 1561u;                   /* PREDIV_A=127, PREDIV_S=1561 -> ck_spre ~5s @ 40kHz LSI */
RTC->ALRMAR = 0x80808080u;                          /* MSK4|MSK3|MSK2|MSK1: ignore date/hr/min/sec, fire every ck_spre tick */
RTC->ISR &= ~RTC_ISR_INIT_Msk;
RTC->CR |= RTC_CR_ALRAE_Msk | RTC_CR_ALRAIE_Msk;
RTC->WPR = 0xFFu;                                   /* re-lock */
EXTI->RTSR |= EXTI_RTSR_TR17;
EXTI->IMR  |= EXTI_IMR_MR17;
NVIC_EnableIRQ(RTC_IRQn);
NVIC_SetPriority(RTC_IRQn, 0);
```

Add `#define DEEP_SLEEP_PERIOD_SEC 5u` near the `IWDG_PR_256`/`IWDG_RLR_VAL` defines (`Src/main.c:56-58`), with a comment noting it's nominal (actual period varies 3.57-7.14s with LSI tolerance, same imprecision already accepted for IWDG) and used only for `cumulative_runtime_sec` catch-up.

Add `volatile uint8_t rtc_wake_pending = 0;` next to `sKeyFlag` (`Src/main.c:51`).

### 2. `RTC_IRQHandler` — `Src/stm32f0xx_it.c`, after `DMA1_CH1_IRQHandler` (after line 223)

```c
void RTC_IRQHandler(void)
{
    if (RTC->ISR & RTC_ISR_ALRAF_Msk)
    {
        RTC->ISR &= ~RTC_ISR_ALRAF_Msk;   /* no WPR unlock needed for ISR flag clears */
        EXTI->PR = EXTI_PR_PR17;          /* write-1-to-clear */
        rtc_wake_pending = 1;
    }
}
```
Declare `extern volatile uint8_t rtc_wake_pending;` alongside the existing `extern volatile uint8_t adc_ready;`-style externs at the top of this file.

### 3. Deep-sleep entry/exit — `Src/main.c`, nested inside the existing idle block (lines 1174-1189)

Add an inner branch right where the shallow `__SEV();__WFE();__WFE();` currently sits (1186-1188): if the deep-sleep condition also holds, do the Stop-mode variant instead; otherwise keep today's shallow sleep unchanged.

```c
    if (/* existing Phase-1 idle condition, unchanged, lines 1174-1181 */)
    {
        if ((sys_state.power_state == POWER_STATE_RPI_OFF || sys_state.power_state == POWER_STATE_PROTECTION_LATCHED) &&
            sys_state.charger_state == CHARGER_STATE_ABSENT)
        {
            /* Deep sleep: Stop mode. Only RTC Alarm A (~5s) and the button EXTI can wake the
             * core here -- ADC/I2C/DMA clocks (HSI/HSI14) stop, but that's fine: the RPi is
             * off in this state so there's no I2C host and nothing pending (guaranteed by the
             * outer idle check above). */
            PWR->CR = (PWR->CR & ~PWR_CR_PDDS_Msk) | PWR_CR_LPDS_Msk;  /* Stop mode, low-power regulator */
            SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
            __SEV(); __WFE(); __WFE();
            SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;   /* must clear before any later shallow WFE below */

            SystemClock_Config();                  /* Stop always resumes on un-multiplied HSI; restore
                                                        48MHz PLL, SysTick reload, I2C1 clock source */
            if (rtc_wake_pending)
            {
                rtc_wake_pending = 0;
                state.cumulative_runtime_sec += DEEP_SLEEP_PERIOD_SEC;
            }
            /* sKeyFlag (button wake) needs no extra handling: normal loop processing below
             * already dispatches it via the existing Button FSM. */
        }
        else
        {
            __SEV(); __WFE(); __WFE();
        }
    }
```

No other change is needed for charger re-detection: on any wake (RTC or button), control returns to the top of the outer `while(1)` naturally, where `IWDG->KR` refreshes, the scheduler/ADC/button code runs exactly as it does today, and `ChargerStateMachine_Step()`/`CheckPowerOnConditions()` pick up fresh ADC data on the next `tick_500ms` — reusing all existing debounce/state-machine logic unchanged. If the charger is still absent and nothing else is pending, the very next loop iteration re-enters this same deep-sleep branch.

### Why no other counters need catch-up
`current_runtime_sec` only increments when `power_state == RPI_ON`, and `charging_time_sec` only when `Charger_IsInfluencingVBAT()` — both false for the entire duration of deep sleep by construction (entry condition requires `RPI_OFF`/`PROTECTION_LATCHED` + `ABSENT`). Only `cumulative_runtime_sec` would have advanced had SysTick kept ticking, so it's the only one caught up.

### Why `PROTECTION_LATCHED` recovery isn't broken by skipped ticks
Checked `CheckPowerOnConditions()` (`Src/main.c:1198-1249`): `PROTECTION_LATCHED` only clears via `charger_present && battery_ok && battery_above_protection` (line 1208) — an event-driven check, not a tick-based timeout. No tick-based auto-recovery logic runs while latched, so deep sleep can't cause it to miss a timeout it doesn't have.

## Files to modify
- `Src/main.c` — RTC/Alarm init, `DEEP_SLEEP_PERIOD_SEC`/`rtc_wake_pending` decls, nested deep-sleep branch in the idle block.
- `Src/stm32f0xx_it.c` — new `RTC_IRQHandler`, extern decl for `rtc_wake_pending`.
- `documents/Low_Power_Idle_Plan.md` — update Section 4 from "future enhancement, unscoped" to describe the implemented Alarm-A-based mechanism (brief edit, not a rewrite); optionally append a Measurements Log row once hardware-tested.

## Verification (user builds/flashes/tests — not done by Claude per standing instruction)
- Bench: force `RPI_OFF`/`PROTECTION_LATCHED` + charger absent, confirm current drops to µA range (vs the mA-range baseline already logged in Section 3.4) and current on the 3.3V rail/MCU VDD shows the periodic ~5s wake blip.
- Confirm periodic wake cadence roughly matches ~5s (scope EXTI17/RTC activity or watch a debug toggle).
- Confirm charger plug-in while deep-asleep is detected within one period and the RPi powers back up via the existing `RPI_OFF → LOAD_ON_DELAY` path.
- Confirm button press while deep-asleep wakes immediately and follows the existing short-press safety check (only powers on if battery above protection threshold).
- Confirm IWDG does not reset the board during extended deep-sleep dwell (leave it in this state for several minutes).
- Confirm normal (non-deep) idle behavior — I2C bursts, button response, ADC/flash — is unaffected when `RPI_ON` or charger present, i.e. the existing Phase 1 shallow-sleep path is untouched when the new inner condition is false.
