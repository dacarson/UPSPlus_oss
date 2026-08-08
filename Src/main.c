#include "pindef.h"

#include "stm32f0xx_ll_adc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_crs.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_i2c.h"
#include "stm32f0xx_ll_iwdg.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_rtc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_crc.h"

#include "I2C_Slave.h"
#include "stm32f0xx.h"
#include "ups_state.h"

#include <string.h>

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_DMA_Init(void);

void UpdateBatteryPercentage(void);
void UpdateBatteryMinMax(void);
void FactoryReset(void);
void CheckPowerOnConditions(void);

/* Set to 1 to re-enable factory-test selectors 0x0A-0x0E (raw per-channel ADC
 * codes + TS_CAL1/TS_CAL2 readback; read with tools/upsplus_adc_diag.py).
 * Left at 0 normally: this part's flash is nearly full, and these registers
 * are only needed when re-investigating this class of ADC/temperature bug. */
#ifndef UPS_ADC_FACTORY_DIAG_ENABLED
#define UPS_ADC_FACTORY_DIAG_ENABLED 0
#endif

/* Everything below needs to be mapped to registers */

/* Raw ADC buffer; DMA fills, main loop processes into state. DMA runs
 * one-shot (see MX_DMA_Init/tick_500ms re-arm), so this is stable and idle
 * between bursts -- safe to read directly. */
__IO uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];

/* Timing/button helpers (canonical 10 ms tick/EXTI). Authoritative state is state/sys_state only;
 * no shadow globals (uXXXVolt, counters, mode flags) remain as authoritative. */
__IO uint8_t sKeyFlag = 0;         /* Button activity (EXTI edge) */
volatile uint8_t rtc_wake_pending = 0;  /* Set by RTC_IRQHandler on Alarm A (deep-sleep periodic wake) */
/* Countdowns decremented in main loop on tick_1s (removed from ISR) */

#define BL_START_ADDRESS 0x8000000

/* IWDG: LL_IWDG_PRESCALER_256 (÷256), RLR chosen for ~8 s minimum at 56 kHz LSI (~17 s at 26 kHz). Main loop must complete within timeout. */
#define IWDG_RLR_VAL  1748u

/* Deep-sleep periodic wake (RTC Alarm A, all fields masked -- see Section 4 of
 * documents/Low_Power_Idle_Plan.md). Nominal period at 40 kHz LSI; actual varies
 * ~3.57-7.14 s across the documented 28-56 kHz LSI tolerance (same imprecision
 * already accepted for IWDG above). Used only to catch up cumulative_runtime_sec. */
#define DEEP_SLEEP_PERIOD_SEC  5u

/* I2C stuck-bus recovery: if ADDR flag set for this many seconds, re-init slave (clock-stretch watchdog).
 * Keep this at 1s so the master (RPi bcm2835) gets a bus error promptly rather than waiting
 * through its own kernel-level timeout (~15-20s), which causes sustained I2C stall storms. */
#define I2C_STUCK_ADDR_RECOVERY_SEC  1u

typedef void (*pFunction)(void);
__IO pFunction JumpToAplication;

/*===========================================================================*/
/* Authoritative state, system state, snapshot (single source)              */
/* State machine runtime - window manager, charger physical,                */
/* protection (single place for transitions + entry/exit actions)           */
/*===========================================================================*/
static authoritative_state_t state;
static system_state_t sys_state;
static snapshot_buffer_t snapshot;
static charger_physical_state_t charger_physical;
static protection_state_t prot_state;
static button_handler_t button_handler;
static uint8_t button_last_level = 0; /* 1=pressed, 0=released */
static uint8_t i2c_stuck_addr_sec = 0; /* Seconds ADDR flag has been set; used for stuck-bus recovery */

/* Double-buffered register image. Snapshot_Update fills reg_image for I2C TX. */
uint8_t reg_image[2][256];
volatile uint8_t active_reg_image = 0;

/* ADC flags: DMA sets adc_ready; main loop processes and increments adc_sample_seq */
volatile uint8_t adc_ready = 0;
volatile uint32_t adc_sample_seq = 0;

/* Canonical scheduler - canonical 10 ms tick (generated from SysTick) sets flags only; main loop runs all tasks.
 * Flag race: small chance of losing an event if ISR sets a flag between main's check and clear.
 * For coarse 100ms/500ms tasks usually acceptable; for never-miss semantics consider
 * counters (increment in ISR) or copy+clear in one short critical section in main. */
static scheduler_flags_t sched_flags;
/* Main loop derives tick_1s from tick_100ms (10 pulses = 1s) */
static uint8_t tick_100ms_count = 0;
/* Downcounters for 100ms/500ms flags (avoid modulo in ISR). ISR-only: do not modify from main. */
static uint8_t ticks_until_100ms = TICKS_PER_100MS;
static uint8_t ticks_until_500ms = TICKS_PER_500MS;

/* Flash save requested; main loop snapshots then commits (rate-limited unless bypassed). */
static volatile uint8_t flash_save_requested = 0;
static volatile uint8_t flash_save_bypass = 0;
static uint8_t flash_dirty = 0;
static uint16_t flash_sequence = 0;
static uint32_t flash_last_write_sec = 0;
static uint32_t flash_next_retry_sec = 0;
/* Flash/persistence debug (factory test page 0x05) */
static uint8_t flash_record_valid_boot = 0;
static uint8_t flash_save_attempted = 0;
static uint8_t flash_last_save_success = 0;
static uint8_t flash_auto_power_on_loaded = 0;
/* OTA request sub-state: request latched → flash save requested → on save-success (for that save) reboot.
 * flash_ota_flag_next_save: next save writes 0x7F at bootloader_ota_flag (cleared inside FillRecord).
 * ota_reboot_after_save: we reboot only after the save that actually wrote the OTA flag (gated in main loop). */
static volatile uint8_t flash_ota_flag_next_save = 0;
static volatile uint8_t ota_reboot_after_save = 0;
static volatile uint8_t ina_probe_requested = 0;
static uint32_t ina_probe_due_ticks = 0;
static uint8_t ina_probe_is_output = 1u;
static uint8_t ina_next_is_output = 1u;
/* A failed master-mode INA219 read (measured: consistently BERR/ARLO/OVR, i.e. a genuine
 * collision with another master) is normal, recoverable multi-master I2C behavior, not
 * evidence the channel is stuck. The guard's pre-check can reduce collision odds but can't
 * eliminate them, since NUT's poll timer runs independently and doesn't coordinate with the
 * STM32 in real time. Retry the
 * SAME channel shortly after a failure instead of waiting for the next full alternation (up to
 * 500ms later, and only after the other channel's turn) -- bounded so a persistently
 * unresponsive device still falls back to normal alternation rather than retrying forever. */
#define INA_PROBE_MAX_RETRIES        3u
#define INA_PROBE_RETRY_DELAY_TICKS  5u  /* ~50ms: long enough for a colliding transaction to finish */
static uint8_t ina_probe_retry_count = 0u;

static void MX_TIM3_Init(void);
static uint8_t I2C1_GuardWindowReady(void);
static uint8_t I2C1_BusIdleStable500us(void);
static void WaitUs(uint16_t us);
static void WaitMs(uint16_t ms);

/* Battery plateau full detection (self-programming enabled) */
static uint16_t filtered_vbat_mv = 0;
static uint16_t plateau_samples[PLATEAU_WINDOW_SAMPLES];
static uint16_t plateau_sample_count = 0;
static uint16_t plateau_sample_index = 0;
static uint8_t plateau_suppressed = 0;
static uint8_t plateau_persisted_this_session = 0;
static charger_state_t plateau_last_charger_state = CHARGER_STATE_ABSENT;
static uint8_t plateau_eval_counter = 0;
/* Empty learning: minimum battery voltage seen while load was on during this discharge (0xFFFF = none yet) */
static uint16_t min_vbat_while_load_on_mv = 0xFFFF;
static uint8_t empty_persisted_this_session = 0;
/* Debounce: only commit empty on load_off after sustained load_off (0 = not in load_off, else tick when load_off started) */
static uint32_t empty_learn_load_off_start_tick = 0u;
static uint8_t battery_full = 0;
static uint32_t battery_full_clear_start_ticks = 0;
static uint32_t taper_hold_ticks = 0;   /* Seconds (in ticks) battery current <= I_TAPER_MA with valid; for taper gate */

/* Restart power-cycle state machine: MT_EN low for 5s then RPI_ON (non-blocking) */
#define RESTART_POWER_OFF_TICKS  (5u * TICKS_PER_1S)  /* 5 seconds in 10ms ticks */
static uint8_t restart_phase = 0;       /* 0=idle, 1=MT_EN low, counting down */
static uint16_t restart_remaining_ticks = 0;

/* Section 9.3: IP_EN arbitration -- single shared ownership so the Section 9.1 periodic reset
 * and Section 9.2 battery-level LED check pulses cannot interleave or corrupt each other's hold
 * timing. Section 10's HardFault handler deliberately bypasses this (see stm32f0xx_it.c): it
 * runs in a dedicated fault context and resets the MCU immediately afterward, so it can never
 * meaningfully overlap with either driver here. */
typedef enum {
    IP_EN_OWNER_NONE = 0,
    IP_EN_OWNER_PERIODIC_RESET,
    IP_EN_OWNER_LED_CHECK
} ip_en_owner_t;
static ip_en_owner_t ip_en_owner = IP_EN_OWNER_NONE;
static uint32_t ip_en_release_tick = 0;        /* tick_counter value at which to release IP_EN HIGH */

/* Section 9.1: periodic IP5328 reset. See ups_state.h for why only a baseline snapshot of
 * charging_time_sec is needed instead of a dedicated countdown. */
static uint32_t ip5328_reset_baseline_sec = 0; /* charging_time_sec value at last arm/rearm */
static uint8_t ip5328_reset_pending = 0;       /* Due; consumed by IPEnArbitration_Tick10ms */
static uint8_t ip5328_reset_pulse_count = 0;   /* Saturating; factory-test selector 0x08 */

/* Section 9.2: battery-level LED check request (set by button dispatch or the periodic timer
 * below, consumed by arbitration) */
static uint8_t led_check_requested = 0;

/* Section 9.2 periodic auto-trigger: reuses sample_period_minutes (register 0x15-0x16,
 * otherwise unused -- see UPSPlus_Behavior_Spec.md Section 3) as the interval between automatic
 * LED checks while genuinely discharging (RPI_ON && charger ABSENT), on top of the existing
 * manual button trigger. Armed to a fresh interval whenever that condition is (re)entered; no
 * partial credit carries over from a prior charging/off session (mirrors Section 9.1's baseline
 * semantics). */
static uint32_t led_check_periodic_remaining_sec = 0;
static uint8_t led_check_periodic_active_prev = 0;

static void InitAuthoritativeStateFromDefaults(void);
static void ProcessI2CPendingWrite(void);
static void Scheduler_Tick10ms(void);
static void Protection_ForceCutIfTimedOut(void);
static void PowerStateMachine_OnTick1s(void);
static void Button_Init(void);
static void Button_ProcessTick10ms(void);
static void Button_DispatchActions(void);
static void BatteryPlateau_Init(void);
static void BatteryPlateau_OnAdcSample(uint16_t raw_vbat_mv);
static void BatteryPlateau_Tick1s(void);
static void CurrentAge_Tick10ms(void);
static void IPEnArbitration_Tick10ms(void);
static void LedCheckPeriodic_OnTick1s(void);

/*===========================================================================*/
/* Validation functions (register bounds, RO enforcement in apply)           */
/*===========================================================================*/
uint8_t Validate_FullVoltage(uint16_t value)
{
    return (value <= VBAT_FULL_MAX_MV) ? 1u : 0u;
}

uint8_t Validate_EmptyVoltage(uint16_t value)
{
    return (value <= VBAT_EMPTY_MAX_MV) ? 1u : 0u;
}

uint8_t Validate_ProtectionVoltage(uint16_t value)
{
    return (value >= VBAT_PROTECT_MIN_MV && value <= VBAT_PROTECT_MAX_MV) ? 1u : 0u;
}

uint8_t Validate_SamplePeriod(uint16_t value)
{
    return (value >= 1u && value <= 1440u) ? 1u : 0u;
}

uint8_t Validate_Countdown(uint8_t value)
{
    return (value == 0u || (value >= 10u && value <= 255u)) ? 1u : 0u;
}

uint8_t Validate_LowBatteryPercent(uint8_t value)
{
    return (value <= 100u) ? 1u : 0u;
}

uint8_t Validate_LoadOnDelay(uint16_t value)
{
    (void)value;
    return 1u; /* 0-65535, no upper clamp required for 16-bit */
}

/*===========================================================================*/
/* State machine helpers (declared in ups_state.h)                            */
/*===========================================================================*/
uint8_t Charger_IsInfluencingVBAT(const system_state_t *s)
{
    return (s->charger_state == CHARGER_STATE_PRESENT) ? 1u : 0u;
}

static uint16_t ClampU16(uint16_t value, uint16_t min_value, uint16_t max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

static uint16_t AbsDiffU16(uint16_t a, uint16_t b)
{
    return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static void BatteryPlateau_ResetWindow(void)
{
    plateau_sample_count = 0;
    plateau_sample_index = 0;
    plateau_eval_counter = 0;
}

static void BatteryPlateau_ResetSession(void)
{
    plateau_persisted_this_session = 0;
    plateau_suppressed = 0;
    taper_hold_ticks = 0;
    BatteryPlateau_ResetWindow();
}

static void BatteryPlateau_Init(void)
{
    filtered_vbat_mv = 0;
    battery_full = 0;
    battery_full_clear_start_ticks = 0;
    plateau_last_charger_state = CHARGER_STATE_ABSENT;
    BatteryPlateau_ResetSession();
}

static void BatteryPlateau_OnAdcSample(uint16_t raw_vbat_mv)
{
    if (filtered_vbat_mv == 0u)
        filtered_vbat_mv = raw_vbat_mv;
    else
    {
        uint32_t num = (uint32_t)(VBAT_FILTER_ALPHA_DEN - VBAT_FILTER_ALPHA_NUM) * (uint32_t)filtered_vbat_mv +
                       (uint32_t)VBAT_FILTER_ALPHA_NUM * (uint32_t)raw_vbat_mv +
                       (uint32_t)(VBAT_FILTER_ALPHA_DEN / 2u);
        filtered_vbat_mv = (uint16_t)(num / (uint32_t)VBAT_FILTER_ALPHA_DEN);
    }
}

/*===========================================================================*/
/* Register 0x17 derivation and register image fill                          */
/*===========================================================================*/
uint8_t GetPowerStatusRegisterValue(const authoritative_state_t *auth_state, const system_state_t *state)
{
    (void)auth_state;
    uint8_t value = 0x00u;
    /* Bit0 reflects effective MT_EN assertion; restart_phase forces MT_EN LOW. */
    if (state->power_state == POWER_STATE_RPI_ON && restart_phase == 0)
        value |= 0x01u; /* Power to RPi */
    /* Bit1: Section 9.1 periodic IP5328 reset in progress. Repurposed -- this bit previously
     * reported learning_mode's "calibration window active" flag, which has been permanently
     * unreachable (always LEARNING_INACTIVE) since the true-VBAT-via-disconnect mechanism was
     * removed (see UPSPlus_Behavior_Spec.md Section 9); learning_mode itself is retained only for
     * Factory Testing ABI stability (selector 0x01 byte 3) and is no longer surfaced here. This
     * bit flags the ~10.5s window so an external observer of battery_current_mA can attribute a
     * transient discharge reading to the expected reset side effect rather than a real event --
     * the charger remains physically connected and charger_state stays PRESENT throughout. */
    if (ip_en_owner == IP_EN_OWNER_PERIODIC_RESET)
        value |= 0x02u;
    return value;
}

/* Little-endian store helpers to shrink StateToRegisterBuffer code. */
static inline void StoreU16LE(uint8_t *buf, uint8_t off, uint16_t v) {
    buf[off]     = (uint8_t)(v & 0xFFu);
    buf[off + 1] = (uint8_t)((v >> 8) & 0xFFu);
}
static inline void StoreU32LE(uint8_t *buf, uint8_t off, uint32_t v) {
    buf[off]     = (uint8_t)(v & 0xFFu);
    buf[off + 1] = (uint8_t)((v >> 8) & 0xFFu);
    buf[off + 2] = (uint8_t)((v >> 16) & 0xFFu);
    buf[off + 3] = (uint8_t)((v >> 24) & 0xFFu);
}

/**
 * Fill 256-byte register image from authoritative state + system state (for 0x17).
 * Little-endian multi-byte registers; reserved/factory regions filled with 0x00.
 */
static void StateToRegisterBuffer(const authoritative_state_t *auth, const system_state_t *sys,
                                  uint8_t *buf)
{
    uint16_t i;
    /* Snapshot local copies for debug/diagnostic pages (main-loop owned state). */
    const button_handler_t button_snapshot = button_handler;
    const charger_physical_state_t charger_snapshot = charger_physical;
    const protection_state_t protection_snapshot = prot_state;

    StoreU16LE(buf, REG_MCU_VOLTAGE_L, auth->mcu_voltage_mv);
    StoreU16LE(buf, REG_POGOPIN_VOLTAGE_L, auth->pogopin_voltage_mv);
    StoreU16LE(buf, REG_BATTERY_VOLTAGE_L, auth->battery_voltage_mv);
    StoreU16LE(buf, REG_USBC_VOLTAGE_L, auth->usbc_voltage_mv);
    StoreU16LE(buf, REG_MICROUSB_VOLTAGE_L, auth->microusb_voltage_mv);
    StoreU16LE(buf, REG_TEMPERATURE_L, auth->temperature_raw);
    StoreU16LE(buf, REG_FULL_VOLTAGE_L, auth->full_voltage_mv);
    StoreU16LE(buf, REG_EMPTY_VOLTAGE_L, auth->empty_voltage_mv);
    StoreU16LE(buf, REG_PROTECT_VOLTAGE_L, auth->protection_voltage_mv);
    buf[REG_BATTERY_PERCENT_L]   = auth->battery_percent;
    buf[REG_BATTERY_PERCENT_H]   = 0x00u;
    StoreU16LE(buf, REG_SAMPLE_PERIOD_L, auth->sample_period_minutes);
    buf[REG_POWER_STATUS]       = GetPowerStatusRegisterValue(auth, sys);
    buf[REG_SHUTDOWN_COUNTDOWN]  = auth->shutdown_countdown_sec;
    buf[REG_AUTO_POWER_ON]       = auth->auto_power_on;
    buf[REG_RESTART_COUNTDOWN]   = auth->restart_countdown_sec;
    buf[REG_FACTORY_RESET]       = 0x00u; /* RO read; write path handled separately */
    StoreU32LE(buf, REG_RUNTIME_ALL_0, auth->cumulative_runtime_sec);
    StoreU32LE(buf, REG_RUNTIME_CHARGING_0, auth->charging_time_sec);
    StoreU32LE(buf, REG_RUNTIME_CURRENT_0, auth->current_runtime_sec);
    StoreU16LE(buf, REG_VERSION_L, auth->version);
    buf[REG_BATTERY_SELF_PROG]   = auth->battery_params_self_programmed;
    buf[REG_LOW_BATTERY_PERCENT] = auth->low_battery_percent;
    StoreU16LE(buf, REG_LOAD_ON_DELAY_L,
               (auth->load_on_delay_remaining_sec != 0u) ? auth->load_on_delay_remaining_sec : auth->load_on_delay_config_sec);
    StoreU16LE(buf, REG_OUTPUT_CURRENT_L, (uint16_t)auth->output_current_mA);
    StoreU16LE(buf, REG_BATTERY_CURRENT_L, (uint16_t)auth->battery_current_mA);
    buf[REG_CURRENT_VALID_FLAGS] = (uint8_t)((auth->output_current_valid ? 0x01u : 0x00u) |
                                             (auth->battery_current_valid ? 0x02u : 0x00u));
    for (i = REG_RESERVED_START; i <= REG_RESERVED_END; i++)
        buf[i] = 0x00u;
    StoreU32LE(buf, REG_SERIAL_START + 0,  LL_GetUID_Word0());
    StoreU32LE(buf, REG_SERIAL_START + 4,  LL_GetUID_Word1());
    StoreU32LE(buf, REG_SERIAL_START + 8, LL_GetUID_Word2());
    for (i = REG_FACTORY_TEST_START; i <= REG_FACTORY_TEST_END; i++)
        buf[i] = 0x00u;
    if (sys->factory_test_selector != 0u) {
        uint8_t selector = sys->factory_test_selector;
        buf[REG_FACTORY_TEST_START] = selector;
        switch (selector) {
        case 0x01u:
            buf[REG_FACTORY_TEST_START + 1] = (uint8_t)sys->power_state;
            buf[REG_FACTORY_TEST_START + 2] = (uint8_t)sys->charger_state;
            buf[REG_FACTORY_TEST_START + 3] = (uint8_t)sys->learning_mode;
            break;
        case 0x02u:
            buf[REG_FACTORY_TEST_START + 1] = (uint8_t)button_snapshot.state;
            buf[REG_FACTORY_TEST_START + 2] = (uint8_t)button_snapshot.pending_click;
            buf[REG_FACTORY_TEST_START + 3] = (uint8_t)(button_snapshot.hold_ticks & 0xFFu);
            break;
        case 0x03u:
            /* Bytes 2-3 (window_active/window_due) are reserved-zero: the measurement window
             * they described has been removed (IP_EN cannot gate the charge path -- see
             * UPSPlus_Behavior_Spec.md Section 9). */
            buf[REG_FACTORY_TEST_START + 1] = (uint8_t)(charger_snapshot.charger_physically_present != 0u);
            buf[REG_FACTORY_TEST_START + 2] = 0x00u;
            buf[REG_FACTORY_TEST_START + 3] = 0x00u;
            break;
        case 0x04u:
            buf[REG_FACTORY_TEST_START + 1] = (uint8_t)(protection_snapshot.protection_active != 0u);
            buf[REG_FACTORY_TEST_START + 2] = (uint8_t)protection_snapshot.below_threshold_count;
            buf[REG_FACTORY_TEST_START + 3] = (uint8_t)(sys->pending_power_cut != 0u);
            break;
        case 0x05u:
        {
            uint8_t flash_status = 0u;
            uint8_t auto_power_info = 0u;
            if (flash_record_valid_boot) flash_status |= 0x01u;
            if (flash_save_attempted) flash_status |= 0x02u;
            if (flash_last_save_success) flash_status |= 0x04u;
            if (flash_auto_power_on_loaded) auto_power_info |= 0x01u;
            if (auth->auto_power_on != 0u) auto_power_info |= 0x02u;
            buf[REG_FACTORY_TEST_START + 1] = flash_status;
            buf[REG_FACTORY_TEST_START + 2] = auto_power_info;
            buf[REG_FACTORY_TEST_START + 3] = (uint8_t)(flash_sequence & 0xFFu);
            break;
        }
        case 0x06u:
            /* INA219 boot presence: bit0=output(0x40), bit1=battery(0x45). */
            buf[REG_FACTORY_TEST_START + 1] = (uint8_t)((I2C1_GetInaProbeOutputPresent() ? 0x01u : 0x00u) |
                                                       (I2C1_GetInaProbeBatteryPresent() ? 0x02u : 0x00u));
            buf[REG_FACTORY_TEST_START + 2] = 0x00u;
            buf[REG_FACTORY_TEST_START + 3] = 0x00u;
            break;
        case 0x07u:
            /* INA219 current age (10ms units), saturated to 8-bit for factory test readout. */
            buf[REG_FACTORY_TEST_START + 1] = (uint8_t)((auth->output_current_age_10ms <= 255u) ?
                                                        auth->output_current_age_10ms : 255u);
            buf[REG_FACTORY_TEST_START + 2] = (uint8_t)((auth->battery_current_age_10ms <= 255u) ?
                                                        auth->battery_current_age_10ms : 255u);
            buf[REG_FACTORY_TEST_START + 3] = 0x00u;
            break;
        case 0x08u:
        {
            /* Section 9.1 periodic-reset diagnostics. Minutes remaining is derived from the
             * charging_time_sec elapsed-basis (see ups_state.h) rather than a stored countdown;
             * 0 whenever charger_state != PRESENT (timer not running, per behavior spec). */
            uint16_t minutes_remaining = 0u;
            if (sys->charger_state == CHARGER_STATE_PRESENT)
            {
                uint32_t elapsed_sec = auth->charging_time_sec - ip5328_reset_baseline_sec;
                uint32_t remaining_sec = (elapsed_sec < IP5328_RESET_PERIOD_SEC) ?
                    (IP5328_RESET_PERIOD_SEC - elapsed_sec) : 0u;
                minutes_remaining = (uint16_t)(remaining_sec / 60u);
            }
            buf[REG_FACTORY_TEST_START + 1] = (uint8_t)(minutes_remaining & 0xFFu);
            buf[REG_FACTORY_TEST_START + 2] = (uint8_t)(minutes_remaining >> 8);
            buf[REG_FACTORY_TEST_START + 3] = ip5328_reset_pulse_count;
            break;
        }
        /* 0x09 previously held guard-open/read-success counts and reset cause, then briefly held
         * a charge-disconnect current diagnostic (entry/during-window battery current around the
         * since-removed FORCED_OFF_WINDOW pulse). Both uses were removed to reclaim flash once
         * they'd served their purpose and conclusively answered their question: the first traced
         * a chronic STM32 I2C reliability problem to an external root cause (a NUT driver bug
         * aliasing a reserved core variable name) rather than anything in this firmware; the
         * second confirmed via live measurement that IP_EN has no effect on battery current, i.e.
         * cannot gate the charge path (see UPSPlus_Behavior_Spec.md Section 9), which is why
         * FORCED_OFF_WINDOW itself was then removed. 0x10 (retry effectiveness) and 0x11
         * (internal guard-window quiet-time tracking) were removed for the same original reason
         * and remain free. 0x0F (INA219 failure-reason breakdown) was removed earlier for the
         * same reason: its question was conclusively answered by testing (consistently
         * bus_error, never NACK/timeout) before removal, not left unanswered. */
#if UPS_ADC_FACTORY_DIAG_ENABLED
        case 0x0Au: /* raw ADC code, battery channel (idx 1), pre-scaling */
        case 0x0Bu: /* raw ADC code, USB-C/VBUS channel (idx 2), pre-scaling */
        case 0x0Cu: /* raw ADC code, TEMPSENSOR channel (idx 4), pre-formula */
        {
            /* Shared body: selector 0x0A/B/C map to aADCxConvertedData[1]/[2]/[4]
             * (indices are 1<<offset, avoids a lookup table). Compare against
             * registers 0x05/0x07/0x0B to recompute the corresponding
             * __LL_ADC_CALC_* formula by hand. */
            uint16_t raw = aADCxConvertedData[1u << (selector - 0x0Au)];
            buf[REG_FACTORY_TEST_START + 1] = (uint8_t)(raw & 0xFFu);
            buf[REG_FACTORY_TEST_START + 2] = (uint8_t)(raw >> 8);
            buf[REG_FACTORY_TEST_START + 3] = 0x00u;
            break;
        }
        case 0x0Du: /* factory TS_CAL1, raw ADC code at 30 DegC, Vref=3.3V */
        case 0x0Eu: /* factory TS_CAL2, raw ADC code at 110 DegC, Vref=3.3V */
        {
            /* If either looks unprogrammed (0x0000/0xFFFF) or CAL2<=CAL1, the
             * temperature formula's calibration inputs are bad on this part,
             * independent of anything ADC/DMA related. */
            uint16_t cal = (selector == 0x0Du) ? *TEMPSENSOR_CAL1_ADDR : *TEMPSENSOR_CAL2_ADDR;
            buf[REG_FACTORY_TEST_START + 1] = (uint8_t)(cal & 0xFFu);
            buf[REG_FACTORY_TEST_START + 2] = (uint8_t)(cal >> 8);
            buf[REG_FACTORY_TEST_START + 3] = 0x00u;
            break;
        }
#endif /* UPS_ADC_FACTORY_DIAG_ENABLED */
        default:
            break;
        }
    }
}

uint8_t Snapshot_ReadRegister(uint8_t reg_addr)
{
    uint8_t a = active_reg_image;
    return reg_image[a][reg_addr];
}

void Snapshot_Init(void)
{
    StateToRegisterBuffer(&state, &sys_state, reg_image[0]);
    StateToRegisterBuffer(&state, &sys_state, reg_image[1]);
    active_reg_image = 0;
    snapshot.buffer[0] = state;
    snapshot.buffer[1] = state;
    snapshot.active_buffer = 0;
    snapshot.snapshot_version = 0;
}

static void UpdateDerivedState(void)
{
    sys_state.learning_mode = (sys_state.charger_state == CHARGER_STATE_FORCED_OFF_WINDOW)
        ? LEARNING_ACTIVE
        : LEARNING_INACTIVE;
}

void Snapshot_Update(void)
{
    state.snapshot_tick = sched_flags.tick_counter;  /* canonical 10 ms tick */
    uint8_t inactive = (uint8_t)(1u - active_reg_image);
    StateToRegisterBuffer(&state, &sys_state, reg_image[inactive]);
    active_reg_image = inactive;
    snapshot.buffer[inactive] = state;
    snapshot.active_buffer = inactive;
    snapshot.snapshot_version++;
}

static void Snapshot_UpdateDerived(void)
{
    UpdateDerivedState();
    Snapshot_Update();
}

static void RequestFlashSave(uint8_t bypass, uint8_t mark_dirty)
{
    flash_save_requested = 1;
    if (bypass)
        flash_save_bypass = 1;
    if (mark_dirty)
        flash_dirty = 1;
}

static void BootBackoff_OnPowerOn(void)
{
    sys_state.boot_backoff_active = 1u;
    sys_state.boot_attempt_start_ticks = sched_flags.tick_counter;
}

static void BootBackoff_OnProtectionTrigger(void)
{
    if (!sys_state.boot_backoff_active)
        return;
    if (sys_state.power_state != POWER_STATE_RPI_ON)
        return;

    uint32_t elapsed = sched_flags.tick_counter - sys_state.boot_attempt_start_ticks;
    if (elapsed <= BOOT_BROWNOUT_WINDOW_TICKS)
    {
        uint32_t new_delay = (uint32_t)state.load_on_delay_config_sec + BOOT_BACKOFF_INCREMENT_SEC;
        if (new_delay > BOOT_BACKOFF_MAX_SEC)
            new_delay = BOOT_BACKOFF_MAX_SEC;
        if ((state.load_on_delay_config_sec < BOOT_BACKOFF_MAX_SEC) && (state.load_on_delay_config_sec != (uint16_t)new_delay)) 
        {
            state.load_on_delay_config_sec = (uint16_t)new_delay;
            RequestFlashSave(0, 1);
            Snapshot_UpdateDerived();
        }
    }

    sys_state.boot_backoff_active = 0u;
}

static void BootBackoff_CheckSuccess(void)
{
    if (!sys_state.boot_backoff_active)
        return;
    if (sys_state.power_state != POWER_STATE_RPI_ON)
    {
        sys_state.boot_backoff_active = 0u;
        return;
    }
    if ((sched_flags.tick_counter - sys_state.boot_attempt_start_ticks) >= BOOT_BROWNOUT_WINDOW_TICKS)
        sys_state.boot_backoff_active = 0u;
}

/* RO register addresses: reject writes (ignore, don't apply) */
static uint8_t IsReadOnlyRegister(uint8_t reg)
{
    if (reg <= REG_TEMPERATURE_H) return 1; /* 0x01-0x0C */
    if (reg >= REG_BATTERY_PERCENT_L && reg <= REG_BATTERY_PERCENT_H) return 1; /* 0x13-0x14 */
    if (reg == REG_POWER_STATUS) return 1; /* 0x17 */
    if (reg >= REG_RUNTIME_ALL_0 && reg <= REG_RUNTIME_CURRENT_3) return 1; /* 0x1C-0x27 */
    if (reg >= REG_VERSION_L && reg <= REG_VERSION_H) return 1; /* 0x28-0x29 */
    if (reg >= REG_OUTPUT_CURRENT_L && reg <= REG_CURRENT_VALID_FLAGS) return 1; /* 0x2E-0x32 */
    if (reg >= REG_SERIAL_START && reg <= REG_SERIAL_END) return 1; /* 0xF0-0xFB */
    return 0;
}

static void ProcessI2CPendingWrite(void)
{
    uint8_t reg = i2c_pending_write.reg_addr;
    uint8_t len = i2c_pending_write.length;
    uint16_t u16;
    uint8_t u8;
    uint8_t changed = 0;
    uint8_t persist_changed = 0;
    uint8_t persist_bypass = 0;  /* set for auto_power_on so save is not rate-limited (issue 3) */

    if (!i2c_pending_write.pending) return;

    /* Reserved: ACK but discard */
    if (IS_RESERVED_REG(reg) || (IS_FACTORY_TEST_REG(reg) && reg != REG_FACTORY_TEST_START)) {
        i2c_pending_write.pending = 0;
        return;
    }
    if (IsReadOnlyRegister(reg)) {
        i2c_pending_write.pending = 0;
        return;
    }

    switch (reg) {
    case REG_FULL_VOLTAGE_L:
        if (len >= 2) {
            u16 = (uint16_t)i2c_pending_write.data[0] | ((uint16_t)i2c_pending_write.data[1] << 8);
            if (Validate_FullVoltage(u16)) {
                int32_t range = (int32_t)u16 - (int32_t)state.empty_voltage_mv;
                if (range >= MIN_VOLTAGE_DELTA_MV && state.full_voltage_mv != u16) {
                state.full_voltage_mv = u16;
                changed = 1;
                persist_changed = 1;
                }
            }
        }
        break;
    case REG_EMPTY_VOLTAGE_L:
        if (len >= 2) {
            u16 = (uint16_t)i2c_pending_write.data[0] | ((uint16_t)i2c_pending_write.data[1] << 8);
            if (Validate_EmptyVoltage(u16) && u16 >= state.protection_voltage_mv) {
                int32_t range = (int32_t)state.full_voltage_mv - (int32_t)u16;
                if (range >= MIN_VOLTAGE_DELTA_MV && state.empty_voltage_mv != u16) {
                    state.empty_voltage_mv = u16;
                    changed = 1;
                    persist_changed = 1;
                }
            }
        }
        break;
    case REG_PROTECT_VOLTAGE_L:
        if (len >= 2) {
            u16 = (uint16_t)i2c_pending_write.data[0] | ((uint16_t)i2c_pending_write.data[1] << 8);
            if (Validate_ProtectionVoltage(u16)) {
                if (state.protection_voltage_mv != u16) {
                    state.protection_voltage_mv = u16;
                    changed = 1;
                    persist_changed = 1;
                }
                if (state.empty_voltage_mv < u16) {
                    state.empty_voltage_mv = u16;
                    changed = 1;
                    persist_changed = 1;
                }
                if (state.full_voltage_mv < u16) {
                    state.full_voltage_mv = u16;
                    changed = 1;
                    persist_changed = 1;
                }
            }
        }
        break;
    case REG_SAMPLE_PERIOD_L:
        if (len >= 2) {
            u16 = (uint16_t)i2c_pending_write.data[0] | ((uint16_t)i2c_pending_write.data[1] << 8);
            if (Validate_SamplePeriod(u16) && state.sample_period_minutes != u16) {
                state.sample_period_minutes = u16;
                changed = 1;
                persist_changed = 1;
            }
        }
        break;
    case REG_SHUTDOWN_COUNTDOWN:
        if (len >= 1) {
            u8 = i2c_pending_write.data[0];
            if (Validate_Countdown(u8) && state.shutdown_countdown_sec != u8) {
                state.shutdown_countdown_sec = u8;
                changed = 1;
            }
        }
        break;
    case REG_AUTO_POWER_ON:
        if (len >= 1) {
            u8 = (i2c_pending_write.data[0] != 0) ? 1u : 0u;
            if (state.auto_power_on != u8) {
                state.auto_power_on = u8;
                changed = 1;
                persist_changed = 1;
                persist_bypass = 1;  /* commit to flash on next loop, not rate-limited */
            }
        }
        break;
    case REG_RESTART_COUNTDOWN:
        if (len >= 1) {
            u8 = i2c_pending_write.data[0];
            if (Validate_Countdown(u8) && state.restart_countdown_sec != u8) {
                state.restart_countdown_sec = u8;
                changed = 1;
            }
        }
        break;
    case REG_FACTORY_RESET:
        if (len >= 1 && i2c_pending_write.data[0] == 1) sys_state.factory_reset_requested = 1;
        break;
    case REG_FACTORY_TEST_START:
        if (len >= 1) {
            u8 = i2c_pending_write.data[0];
            if (u8 == 0x7Fu) {
                /* OTA command (consumed): latch request → save → reboot. break so selector is never updated. */
                flash_ota_flag_next_save = 0x7Fu;
                ota_reboot_after_save = 1u;
                RequestFlashSave(1, 1);  /* bypass=1, mark_dirty=1: next Flash_Save uses bypass and runs soon */
                break;
            }
            if (sys_state.factory_test_selector != u8) {
                sys_state.factory_test_selector = u8;
                changed = 1;
            }
        }
        break;
    case REG_BATTERY_SELF_PROG:
        if (len >= 1) {
            uint8_t new_value = (i2c_pending_write.data[0] != 0) ? 1u : 0u;
            if (state.battery_params_self_programmed != new_value) {
                state.battery_params_self_programmed = new_value;
                if (new_value == 0u) {
                    state.full_voltage_mv = DEFAULT_VBAT_FULL_MV;
                    state.empty_voltage_mv = DEFAULT_VBAT_EMPTY_MV;
                    if (state.empty_voltage_mv < state.protection_voltage_mv)
                        state.empty_voltage_mv = state.protection_voltage_mv;
                }
                changed = 1;
                persist_changed = 1;
            }
        }
        break;
    case REG_LOW_BATTERY_PERCENT:
        if (len >= 1) {
            u8 = i2c_pending_write.data[0];
            if (Validate_LowBatteryPercent(u8) && state.low_battery_percent != u8) {
                state.low_battery_percent = u8;
                changed = 1;
                persist_changed = 1;
            }
        }
        break;
    case REG_LOAD_ON_DELAY_L:
        if (len >= 2) {
            u16 = (uint16_t)i2c_pending_write.data[0] | ((uint16_t)i2c_pending_write.data[1] << 8);
            if (Validate_LoadOnDelay(u16)) {
                if (state.load_on_delay_config_sec != u16) {
                    state.load_on_delay_config_sec = u16;
                    changed = 1;
                    persist_changed = 1;
                }
                if (sys_state.power_state == POWER_STATE_LOAD_ON_DELAY)
                    state.load_on_delay_remaining_sec = u16;
            }
        }
        break;
    default:
        break;
    }

    if (changed)
        Snapshot_UpdateDerived();
    if (persist_changed)
        RequestFlashSave(persist_bypass, 1);
    i2c_pending_write.pending = 0;
}

static void InitAuthoritativeStateFromDefaults(void)
{
    state.mcu_voltage_mv = 3300;
    state.pogopin_voltage_mv = 0;
    state.battery_voltage_mv = 0;
    state.usbc_voltage_mv = 0;
    state.microusb_voltage_mv = 0;
    state.temperature_raw = 0;
    state.full_voltage_mv = DEFAULT_VBAT_FULL_MV;
    state.empty_voltage_mv = DEFAULT_VBAT_EMPTY_MV;
    state.protection_voltage_mv = DEFAULT_VBAT_PROTECT_MV;
    state.battery_percent = 0;
    state.sample_period_minutes = DEFAULT_SAMPLE_PERIOD_MIN;
    state.shutdown_countdown_sec = 0;
    state.auto_power_on = DEFAULT_AUTO_POWER_ON;
    state.restart_countdown_sec = 0;
    state.battery_params_self_programmed = 0;
    state.low_battery_percent = DEFAULT_LOW_BATTERY_PERCENT;
    state.load_on_delay_config_sec = DEFAULT_LOAD_ON_DELAY_SEC;
    state.load_on_delay_remaining_sec = 0;
    state.output_current_mA = 0;
    state.battery_current_mA = 0;
    state.output_current_valid = 0;
    state.battery_current_valid = 0;
    state.output_current_age_10ms = 0xFFFFu;
    state.battery_current_age_10ms = 0xFFFFu;
    state.cumulative_runtime_sec = 0;
    state.charging_time_sec = 0;
    state.current_runtime_sec = 0;
    state.version = 30;
    state.snapshot_tick = 0;

    sys_state.power_state = POWER_STATE_RPI_OFF;
    sys_state.charger_state = CHARGER_STATE_ABSENT;
    sys_state.learning_mode = LEARNING_INACTIVE;
    sys_state.power_state_entry_ticks = 0;
    sys_state.charger_state_entry_ticks = 0;
    sys_state.factory_reset_requested = 0;
    sys_state.factory_test_selector = 0;
    sys_state.pending_power_cut = 0;
    sys_state.pending_power_cut_start_ticks = 0;
    sys_state.boot_backoff_active = 0;
    sys_state.boot_attempt_start_ticks = 0;

    /* State machine runtime state */
    charger_physical.charger_physically_present = 0;
    charger_physical.charger_stability_count = 0;
    charger_physical.charger_last_seen_seq = 0;
    prot_state.below_threshold_count = 0;
    prot_state.protection_active = 0;
    prot_state.last_adc_battery_mv = 0;
    prot_state.last_seen_seq = 0;

    Button_Init();
    BatteryPlateau_Init();
}

static void Button_Init(void)
{
    button_handler.state = BUTTON_IDLE;
    button_handler.press_start_tick = 0;
    button_handler.hold_ticks = 0;
    button_handler.long_press_fired = 0;
    button_handler.debounce_counter = 0;
    button_handler.pending_click = BUTTON_CLICK_NONE;
    button_last_level = 0;
}

void NVIC_SetVectorTable(void)
{
    uint8_t i;
    extern const uint32_t __vector_table_ram_start__;
    extern const uint32_t __vector_table_flash_start__;
    uint32_t *pVecTab = (uint32_t *)&__vector_table_ram_start__;
    for (i = 0; i < 48; i++)
    {
        *(pVecTab++) = *(__IO uint32_t *)((uint32_t)&__vector_table_flash_start__ + (i << 2));
    }
    LL_SYSCFG_SetRemapMemory(LL_SYSCFG_REMAP_SRAM);
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    //Program to be updated via OTA.
    NVIC_SetVectorTable();

    SystemClock_Config();

    /* Clear RCC reset flags so they do not accumulate (bootloader already cleared them before app run). */
    RCC->CSR |= RCC_CSR_RMVF;

    /* IWDG: LSI 26–56 kHz; PR=256, RLR=1748 → ~8 s at 56 kHz, ~17 s at 26 kHz. Main loop must finish within timeout.
     * We do not block on LSI ready: if LSI never starts, the watchdog simply won’t tick, but firmware still runs.
     * Option bytes: verify SW-start mode and IWDG not frozen in documentation/manufacturing; do not program to disable.
     * Starting here puts init under the watchdog; if init grows, consider moving start to just before while(1). */
    LL_RCC_LSI_Enable();
    LL_IWDG_EnableWriteAccess(IWDG);
    LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_256);
    LL_IWDG_SetReloadCounter(IWDG, IWDG_RLR_VAL);
    {
        uint32_t sr_wait = 0xFFFFu;
        while ((LL_IWDG_IsActiveFlag_PVU(IWDG) || LL_IWDG_IsActiveFlag_RVU(IWDG)) && sr_wait != 0u)   /* Wait for PR/RLR latch */
            sr_wait--;
        /* If SR doesn't clear, start anyway; timeout may be shorter than expected. */
    }
    LL_IWDG_Enable(IWDG);

    /* RTC Alarm A, all fields masked, as a free-running ~5 s periodic wake source for
     * deep sleep (see Section 4 of Low_Power_Idle_Plan.md). This part (STM32F030x6) has
     * no RTC Wakeup Timer block -- an all-masked Alarm A compare against a slowed ck_spre
     * is the equivalent: fires every ck_spre tick forever, no rearm needed after this. */
    LL_PWR_EnableBkUpAccess();                          /* unlock backup domain writes */
    {
        uint32_t lsi_wait = 100000u;                    /* bounded; proceed regardless, same idiom as IWDG start above */
        while (!LL_RCC_LSI_IsReady() && --lsi_wait) { }
    }
    LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
    LL_RCC_EnableRTC();
    LL_RTC_DisableWriteProtection(RTC);
    LL_RTC_EnterInitMode(RTC);                          /* polls RTC->ISR INITF with a bounded internal timeout */
    LL_RTC_SetAsynchPrescaler(RTC, 127u);
    LL_RTC_SetSynchPrescaler(RTC, 1561u);               /* PREDIV_A=127, PREDIV_S=1561 -> ck_spre ~5s @ 40kHz LSI */
    LL_RTC_ALMA_SetMask(RTC, LL_RTC_ALMA_MASK_ALL);     /* ignore date/hr/min/sec: fire every ck_spre tick */
    LL_RTC_ExitInitMode(RTC);
    LL_RTC_ALMA_Enable(RTC);
    LL_RTC_EnableIT_ALRA(RTC);
    LL_RTC_EnableWriteProtection(RTC);                  /* re-lock */
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_17);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_17);
    NVIC_EnableIRQ(RTC_IRQn);
    NVIC_SetPriority(RTC_IRQn, 0);

    MX_TIM3_Init();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC_Init();
    MX_I2C1_ProbeMasterSetup();
    MX_I2C1_Slave_Init();

    /* Single source of truth; load from defaults then from flash */
    InitAuthoritativeStateFromDefaults();
    Flash_Init();
    Flash_Load();
    Snapshot_Init();
    sys_state.factory_reset_requested = 0;

    /* Allow power rails / I2C pull-ups to settle before main loop; remove or reduce if not required on target hardware. */
    WaitMs(100);

    while (1)
    {
        /* Canonical scheduler - run tasks from scheduler flags, then clear flags */
        if (sched_flags.tick_10ms)
        {
            Scheduler_Tick10ms();
            sched_flags.tick_10ms = 0;
        }
        if (sched_flags.tick_100ms)
        {
            tick_100ms_count++;
            if (tick_100ms_count >= 10u)
            {
                sched_flags.tick_1s = 1;
                tick_100ms_count = 0;
            }
            Snapshot_UpdateDerived();  /* 100ms periodic snapshot (ensures freshness) */
            sched_flags.tick_100ms = 0;
        }
        if (sched_flags.tick_500ms)
        {
            if (!adc_ready)
            {
                /* DMA runs NORMAL (one-shot): each burst's completion leaves
                 * NDTR at 0, so it must be explicitly re-armed before the next
                 * trigger, every cycle (not just after a deadlock). */
                LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
                LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ADC_CONVERTED_DATA_BUFFER_SIZE);
                LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

                if (!LL_ADC_IsEnabled(ADC1))
                {
                    /* ADEN=0 with ADSTART=1 is a deadlock: RS bits prevent clearing
                     * ADSTART by writing 0. Use APB peripheral reset to restore all
                     * ADC registers to reset state, then re-initialise (which also
                     * re-triggers conversion). Should be rare now that DMA no longer
                     * runs the CIRC+LIMITED combination suspected to cause this. */
                    LL_APB1_GRP2_ForceReset(LL_APB1_GRP2_PERIPH_ADC1);
                    LL_APB1_GRP2_ReleaseReset(LL_APB1_GRP2_PERIPH_ADC1);
                    MX_ADC_Init();
                }
                else
                {
                    LL_ADC_REG_StartConversion(ADC1);
                }
            }
            if (!ina_probe_requested)
            {
                ina_probe_requested = 1;
                ina_probe_due_ticks = sched_flags.tick_counter + 1u; /* ~10 ms */
                ina_probe_is_output = ina_next_is_output;
                ina_next_is_output = (ina_next_is_output == 0u) ? 1u : 0u;
            }
            sched_flags.tick_500ms = 0;
        }
        if (sched_flags.tick_1s)
        {
            state.cumulative_runtime_sec++;
            if (sys_state.power_state == POWER_STATE_RPI_ON)
                state.current_runtime_sec++;
            else
                state.current_runtime_sec = 0;
            if (Charger_IsInfluencingVBAT(&sys_state))
            {
                state.charging_time_sec++;
                /* Section 9.1: elapsed-charging basis for the periodic IP5328 reset (see
                 * ups_state.h) -- reuses this counter instead of a separate countdown. */
                if ((state.charging_time_sec - ip5328_reset_baseline_sec) >= IP5328_RESET_PERIOD_SEC)
                {
                    ip5328_reset_pending = 1u;
                    ip5328_reset_baseline_sec = state.charging_time_sec;
                }
            }
            PowerStateMachine_OnTick1s();
            LedCheckPeriodic_OnTick1s();
            BatteryPlateau_Tick1s();
            if (flash_dirty &&
                (state.cumulative_runtime_sec - flash_last_write_sec) >= FLASH_DIRTY_MAX_INTERVAL_SEC)
            {
                RequestFlashSave(0, 0);
            }
            /* I2C stuck-bus recovery: if ADDR flag set for >= 5 s (clock stretch), re-init slave. */
            if (I2C1_IsAddrFlagSet())
                i2c_stuck_addr_sec++;
            else
                i2c_stuck_addr_sec = 0;
            if (i2c_stuck_addr_sec >= I2C_STUCK_ADDR_RECOVERY_SEC)
            {
                MX_I2C1_Slave_Init();
                i2c_stuck_addr_sec = 0;
            }
            Snapshot_UpdateDerived();
            sched_flags.tick_1s = 0;
        }

        /* Apply I2C writes to authoritative state (RO rejected, bounds checked) */
        ProcessI2CPendingWrite();

        if (ina_probe_requested)
        {
            uint32_t now_ticks = sched_flags.tick_counter;
            if ((uint32_t)(now_ticks - ina_probe_due_ticks) < 0x80000000u &&
                I2C1_GuardWindowReady())
            {
                int16_t current_mA = 0;
                ina_probe_requested = 0;
                if (I2C1_ReadIna219Current(ina_probe_is_output, &current_mA))
                {
                    ina_probe_retry_count = 0u;
                    if (ina_probe_is_output)
                    {
                        state.output_current_mA = current_mA;
                        state.output_current_age_10ms = 0u;
                        state.output_current_valid = 1u;
                    }
                    else
                    {
                        state.battery_current_mA = current_mA;
                        state.battery_current_age_10ms = 0u;
                        state.battery_current_valid = 1u;
                    }
                    Snapshot_UpdateDerived();
                }
                else if (ina_probe_retry_count < INA_PROBE_MAX_RETRIES)
                {
                    /* Likely transient collision (BERR/ARLO/OVR) -- retry the same channel
                     * shortly rather than waiting for the next full alternation. */
                    ina_probe_retry_count++;
                    ina_probe_requested = 1u;
                    ina_probe_due_ticks = sched_flags.tick_counter + INA_PROBE_RETRY_DELAY_TICKS;
                }
                else
                {
                    /* Exhausted retries -- give up on this channel for now and fall back to
                     * normal alternation (next tick_500ms picks up the other channel). */
                    ina_probe_retry_count = 0u;
                }
            }
        }

        /* Invariant A6: attempt flash commit first; only then cut MT_EN (set PROTECTION_LATCHED) */
        if (flash_save_requested)
        {
            uint32_t now_sec = state.cumulative_runtime_sec;
            if (now_sec < flash_next_retry_sec)
            {
                /* Backoff after failed flash save to avoid tight retry loop */
            }
            else
            {
                uint8_t ota_save_this_round = (ota_reboot_after_save && (flash_ota_flag_next_save == 0x7Fu));
                Snapshot_UpdateDerived();
                if (Flash_Save(flash_save_bypass))
                {
                    flash_save_requested = 0;
                    flash_save_bypass = 0;
                    flash_next_retry_sec = 0;
                    if (ota_save_this_round)
                    {
                        ota_reboot_after_save = 0;
                        SCB->AIRCR = (0x5FAu << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
                        for (;;) { __NOP(); }
                    }
                    if (sys_state.pending_power_cut)
                    {
                        sys_state.power_state = POWER_STATE_PROTECTION_LATCHED;
                        sys_state.power_state_entry_ticks = sched_flags.tick_counter;
                        sys_state.pending_power_cut = 0;
                        sys_state.pending_power_cut_start_ticks = 0;
                    }
                }
                else
                {
                    uint32_t backoff = flash_save_bypass ? FLASH_RETRY_BACKOFF_SEC_BYPASS : FLASH_RETRY_BACKOFF_SEC;
                    flash_next_retry_sec = now_sec + backoff;
                }
            }
        }

        /* Factory reset requested via I2C 0x1B */
        if (sys_state.factory_reset_requested)
        {
            FactoryReset();
            Snapshot_UpdateDerived();
            flash_save_bypass = 1;
            flash_dirty = 1;
            if (Flash_Save(1u))
            {
                flash_save_requested = 0;
                flash_save_bypass = 0;
            }
            else
            {
                flash_save_requested = 1;
            }
            sys_state.factory_reset_requested = 0;
        }

        /* Protection fallback: force cut if flash save stalls */
        Protection_ForceCutIfTimedOut();

        /* Sample period clamp */
        if (state.sample_period_minutes < DEFAULT_SAMPLE_PERIOD_MIN)
            state.sample_period_minutes = DEFAULT_SAMPLE_PERIOD_MIN;

        /* ADC processing in main loop (DMA sets adc_ready only) */
        if (adc_ready)
        {
            /* VREFINT is sampled every ADC round-robin scan (~500ms, see LL_ADC_REG_SetSequencerChAdd
             * call for LL_ADC_CHANNEL_VREFINT), so recalibrate VDDA every cycle rather than on a slow
             * timer: the pogopin/battery/USB voltage conversions below are ratiometric against it, and
             * a stale VDDA estimate biases every one of them. The blend below still smooths single-
             * sample ADC noise, but now settles in ~1-2s instead of tens of seconds, fast enough to
             * track a real VDDA sag (e.g. as the battery nears empty) instead of reading back as
             * spurious voltage growth on those channels. */
            state.mcu_voltage_mv = (__LL_ADC_CALC_VREFANALOG_VOLTAGE(aADCxConvertedData[5], LL_ADC_RESOLUTION_12B) + state.mcu_voltage_mv) / 2;
            state.pogopin_voltage_mv = __LL_ADC_CALC_DATA_TO_VOLTAGE(state.mcu_voltage_mv * 2, aADCxConvertedData[0], LL_ADC_RESOLUTION_12B);
            state.battery_voltage_mv = __LL_ADC_CALC_DATA_TO_VOLTAGE(state.mcu_voltage_mv * 2, aADCxConvertedData[1], LL_ADC_RESOLUTION_12B);
            state.usbc_voltage_mv = __LL_ADC_CALC_DATA_TO_VOLTAGE(state.mcu_voltage_mv * 4, aADCxConvertedData[2], LL_ADC_RESOLUTION_12B);
            state.microusb_voltage_mv = __LL_ADC_CALC_DATA_TO_VOLTAGE(state.mcu_voltage_mv * 4, aADCxConvertedData[3], LL_ADC_RESOLUTION_12B);
            state.temperature_raw = __LL_ADC_CALC_TEMPERATURE(state.mcu_voltage_mv, aADCxConvertedData[4], LL_ADC_RESOLUTION_12B);
            uint16_t min_valid_mv = ClampU16(VBAT_MIN_VALID_MV, state.protection_voltage_mv, 0xFFFFu);
            state.battery_voltage_mv = ClampU16(state.battery_voltage_mv, min_valid_mv, 0xFFFFu);
            BatteryPlateau_OnAdcSample(state.battery_voltage_mv);
            UpdateBatteryMinMax();
            UpdateBatteryPercentage();
            /* Protection: store last ADC reading; counting is done in Protection_Step with adc_sample_seq gating. */
            prot_state.last_adc_battery_mv = state.battery_voltage_mv;
            Snapshot_UpdateDerived();
            adc_sample_seq++;
            adc_ready = 0;
        }

        CheckPowerOnConditions();

        /* Button actions (short/long press) */
        Button_DispatchActions();

        /* IWDG: single refresh per loop, after all critical work (scheduler, I2C, INA, flash, protection, GPIO). */
        LL_IWDG_ReloadCounter(IWDG);   /* Reload watchdog counter */

        /* Idle path: sleep when there is no pending work. Flash is eligible to idle
         * even while flash_save_requested is set, as long as its retry backoff hasn't
         * elapsed yet (now_sec < flash_next_retry_sec) -- that wait is intentional. */
        if (!sched_flags.tick_10ms && !sched_flags.tick_100ms && !sched_flags.tick_500ms && !sched_flags.tick_1s &&
            !adc_ready &&
            !(flash_save_requested && state.cumulative_runtime_sec >= flash_next_retry_sec) &&
            !i2c_pending_write.pending &&
            !ina_probe_requested &&
            !sys_state.factory_reset_requested &&
            I2C1_GetSlaveTxnActive() == 0 &&
            sKeyFlag == 0 && button_handler.pending_click == BUTTON_CLICK_NONE)
        {
            if ((sys_state.power_state == POWER_STATE_RPI_OFF || sys_state.power_state == POWER_STATE_PROTECTION_LATCHED) &&
                sys_state.charger_state == CHARGER_STATE_ABSENT)
            {
                /* Deep sleep: Stop mode. Only RTC Alarm A (~5s, see DEEP_SLEEP_PERIOD_SEC) and the
                 * button EXTI can wake the core here -- ADC/I2C/DMA clocks (HSI/HSI14) stop, but
                 * that's fine: the RPi is off in this state so there's no I2C host and nothing
                 * pending (guaranteed by the outer idle check above). */
                LL_PWR_SetPowerMode(LL_PWR_MODE_STOP_LPREGU);  /* Stop mode, low-power regulator */
                LL_LPM_EnableDeepSleep();
                __SEV();
                __WFE();
                __WFE();
                LL_LPM_EnableSleep();   /* clear SLEEPDEEP before any later shallow WFE below */

                SystemClock_Config();   /* Stop always resumes on un-multiplied HSI; restore 48MHz
                                            PLL, SysTick reload, I2C1 clock source */
                if (rtc_wake_pending)
                {
                    rtc_wake_pending = 0;
                    state.cumulative_runtime_sec += DEEP_SLEEP_PERIOD_SEC;
                }
                /* sKeyFlag (button wake) needs no extra handling here: normal loop processing
                 * on the next iteration dispatches it via the existing Button FSM. */
            }
            else
            {
                /* Event-safe sleep: SEV then WFE/WFE clears any stale event before waiting,
                 * avoiding the race where an interrupt sets pending work between the idle
                 * check above and entering sleep. Wakes on SysTick/DMA/I2C/EXTI. */
                __SEV();
                __WFE();
                __WFE();
            }
        }
    }
}

/**
 * Check power-on conditions with explicit state transitions.
 * RPI_OFF -> LOAD_ON_DELAY when conditions met and charger present.
 * LOAD_ON_DELAY -> RPI_OFF when conditions no longer met.
 */
void CheckPowerOnConditions(void)
{
    uint32_t now_ticks = sched_flags.tick_counter;
    uint8_t battery_ok = (state.battery_percent > state.low_battery_percent);
    uint8_t battery_above_protection = (state.battery_voltage_mv > (uint16_t)(state.protection_voltage_mv + PROTECTION_HYSTERESIS_MV));
    uint8_t charger_present = Charger_IsInfluencingVBAT(&sys_state);

    /* Clear PROTECTION_LATCHED when charger present and battery recovered (even if !auto_power_on). */
    if (sys_state.power_state == POWER_STATE_PROTECTION_LATCHED)
    {
        uint8_t recovered = (charger_present && battery_ok && battery_above_protection);
        sys_state.power_state = (recovered && state.auto_power_on) ? POWER_STATE_LOAD_ON_DELAY : POWER_STATE_RPI_OFF;
        sys_state.power_state_entry_ticks = now_ticks;
        if (sys_state.power_state == POWER_STATE_LOAD_ON_DELAY)
            state.load_on_delay_remaining_sec = state.load_on_delay_config_sec;
        sys_state.pending_power_cut = 0;
        sys_state.pending_power_cut_start_ticks = 0;
        prot_state.below_threshold_count = 0;
        return;
    }

    if (!state.auto_power_on)
        return;

    if (sys_state.power_state == POWER_STATE_LOAD_ON_DELAY)
    {
        /* Cancel if any required condition fails (plan: battery % must be > low for power decisions) */
        if (!battery_ok)
        {
            /* Below protection threshold: latch to prevent immediate re-entry into LOAD_ON_DELAY */
            sys_state.power_state = battery_above_protection ? POWER_STATE_RPI_OFF : POWER_STATE_PROTECTION_LATCHED;
            sys_state.power_state_entry_ticks = now_ticks;
            state.load_on_delay_remaining_sec = 0;
        }
        return;
    }

    if (sys_state.power_state != POWER_STATE_RPI_OFF)
        return;

    /* Do not auto power-on from battery-only; require charger present */
    if (!charger_present)
        return;

    /* Allow transition when battery_ok (battery % > low) */
    if (battery_ok && battery_above_protection)
    {
        sys_state.power_state = POWER_STATE_LOAD_ON_DELAY;
        sys_state.power_state_entry_ticks = now_ticks;
        state.load_on_delay_remaining_sec = state.load_on_delay_config_sec;
    }
}

void FactoryReset(void)
{
    /* Set authoritative state to defaults (single source of truth) */
    state.full_voltage_mv = DEFAULT_VBAT_FULL_MV;
    state.empty_voltage_mv = DEFAULT_VBAT_EMPTY_MV;
    state.protection_voltage_mv = DEFAULT_VBAT_PROTECT_MV;
    state.sample_period_minutes = DEFAULT_SAMPLE_PERIOD_MIN;
    state.low_battery_percent = DEFAULT_LOW_BATTERY_PERCENT;
    state.load_on_delay_config_sec = DEFAULT_LOAD_ON_DELAY_SEC;
    state.auto_power_on = DEFAULT_AUTO_POWER_ON;
    state.battery_params_self_programmed = 0;
    state.battery_percent = 0;
    state.shutdown_countdown_sec = 0;
    state.restart_countdown_sec = 0;
    state.load_on_delay_remaining_sec = 0;
    state.cumulative_runtime_sec = 0;
    state.charging_time_sec = 0;
    state.current_runtime_sec = 0;
    sys_state.power_state = POWER_STATE_RPI_OFF;
    sys_state.factory_test_selector = 0;

    /* Reset runtime/state-machine variables so FSM is in known-safe state */
    sys_state.charger_state = CHARGER_STATE_ABSENT;
    sys_state.charger_state_entry_ticks = 0;
    sys_state.power_state_entry_ticks = 0;
    sys_state.pending_power_cut = 0;
    sys_state.pending_power_cut_start_ticks = 0;
    sys_state.learning_mode = LEARNING_INACTIVE;
    sys_state.boot_backoff_active = 0;
    sys_state.boot_attempt_start_ticks = 0;
    charger_physical.charger_physically_present = 0;
    charger_physical.charger_stability_count = 0;
    charger_physical.charger_last_seen_seq = 0;
    prot_state.below_threshold_count = 0;
    prot_state.protection_active = 0;
    prot_state.last_adc_battery_mv = 0;
    prot_state.last_seen_seq = 0;
    restart_phase = 0;
    restart_remaining_ticks = 0;
    /* Section 9.1/9.2/9.3 runtime state: fresh slate. charger_state was just reset to ABSENT
     * above, so the next ABSENT->PRESENT transition will re-arm the baseline regardless, but
     * resetting explicitly here avoids relying on that ordering and matches the rest of this
     * function's treatment of runtime/session state. */
    ip5328_reset_baseline_sec = 0;
    ip5328_reset_pending = 0;
    ip5328_reset_pulse_count = 0;
    led_check_requested = 0;
    led_check_periodic_remaining_sec = 0;
    led_check_periodic_active_prev = 0;
    ip_en_owner = IP_EN_OWNER_NONE;
    ip_en_release_tick = 0;
    BatteryPlateau_Init();
    min_vbat_while_load_on_mv = 0xFFFFu;
    empty_persisted_this_session = 0;
}

extern const uint8_t __flash_storage_start__[];
extern const uint8_t __flash_storage_end__[];
static inline const uint8_t *FlashStorageStartPtr(void)
{
    return __flash_storage_start__;
}
static inline const uint8_t *FlashStorageEndPtr(void)
{
    return __flash_storage_end__;
}

/* Hardware CRC (STM32F0): polynomial 0x04C11DB7, init 0xFFFFFFFF, no reflection.
 * Replaces software CRC32 to reduce flash; record version bumped so old records are discarded. */
static uint32_t Flash_ComputeCrc32(const uint8_t *data, uint32_t len)
{
    uint32_t i;
    uint32_t last = 0u;

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);
    LL_CRC_ResetCRCCalculationUnit(CRC);

    for (i = 0u; i < (len >> 2u); i++)
        LL_CRC_FeedData32(CRC, *(const uint32_t *)(data + (i << 2u)));

    if (len & 3u) {
        memcpy(&last, data + (len & ~3u), len & 3u);
        LL_CRC_FeedData32(CRC, last);
    }

    return LL_CRC_ReadData32(CRC);
}

static uint8_t Flash_RecordIsValid(const flash_persistent_data_t *rec)
{
    uint32_t computed;
    if (rec->magic != FLASH_MAGIC_NUMBER)
        return 0;
    if (rec->version != FLASH_STRUCTURE_VERSION)
        return 0;
    computed = Flash_ComputeCrc32(((const uint8_t *)rec) + FLASH_CRC_START_OFFSET, FLASH_CRC_SIZE);
    return (computed == rec->crc32) ? 1u : 0u;
}

static void Flash_ApplyRecordToState(const flash_persistent_data_t *rec)
{
    state.full_voltage_mv = rec->full_voltage_mv;
    state.empty_voltage_mv = rec->empty_voltage_mv;
    state.protection_voltage_mv = rec->protection_voltage_mv;
    state.sample_period_minutes = rec->sample_period_minutes;
    state.auto_power_on = (rec->auto_power_on != 0u) ? 1u : 0u;
    state.battery_params_self_programmed = (rec->battery_params_self_programmed != 0u) ? 1u : 0u;
    state.low_battery_percent = rec->low_battery_percent;
    state.load_on_delay_config_sec = rec->load_on_delay_config_sec;
    /* rec->last_true_vbat_mv / last_true_vbat_age_sec intentionally not read: reserved, see
     * Flash_FillRecordFromState. */
    state.cumulative_runtime_sec = rec->cumulative_runtime_sec;
    state.charging_time_sec = rec->charging_time_sec;

    /* Clamp to valid ranges (same policy as InitAuthoritativeStateFromBuffer) */
    if (!Validate_FullVoltage(state.full_voltage_mv)) state.full_voltage_mv = DEFAULT_VBAT_FULL_MV;
    if (!Validate_EmptyVoltage(state.empty_voltage_mv)) state.empty_voltage_mv = DEFAULT_VBAT_EMPTY_MV;
    if (!Validate_ProtectionVoltage(state.protection_voltage_mv)) state.protection_voltage_mv = DEFAULT_VBAT_PROTECT_MV;
    state.empty_voltage_mv = ClampU16(state.empty_voltage_mv, state.protection_voltage_mv, VBAT_EMPTY_MAX_MV);
    if (!Validate_SamplePeriod(state.sample_period_minutes)) state.sample_period_minutes = DEFAULT_SAMPLE_PERIOD_MIN;
    if (state.low_battery_percent > 100u) state.low_battery_percent = DEFAULT_LOW_BATTERY_PERCENT;

    /* Runtime-only: countdowns must not restore from flash; zero after load */
    state.shutdown_countdown_sec = 0;
    state.restart_countdown_sec = 0;
    state.current_runtime_sec = 0;
}

static void Flash_FillRecordFromState(flash_persistent_data_t *rec, uint16_t seq)
{
    memset(rec, 0, sizeof(*rec));
    rec->magic = FLASH_MAGIC_NUMBER;
    rec->version = FLASH_STRUCTURE_VERSION;
    rec->sequence_number = seq;
    rec->reserved_header = 0;

    rec->full_voltage_mv = state.full_voltage_mv;
    rec->empty_voltage_mv = state.empty_voltage_mv;
    rec->protection_voltage_mv = state.protection_voltage_mv;
    rec->sample_period_minutes = state.sample_period_minutes;
    rec->auto_power_on = (state.auto_power_on != 0u) ? 1u : 0u;
    rec->battery_params_self_programmed = (state.battery_params_self_programmed != 0u) ? 1u : 0u;
    rec->low_battery_percent = state.low_battery_percent;
    rec->load_on_delay_config_sec = state.load_on_delay_config_sec;
    /* last_true_vbat_mv / last_true_vbat_age_sec: reserved, left at the memset-zero above. The
     * true-VBAT-via-disconnect mechanism these backed has been removed (IP_EN cannot gate the
     * charge path -- see UPSPlus_Behavior_Spec.md Section 9); the fields are kept in the struct,
     * unused, to preserve flash layout and the bootloader_ota_flag offset (FLASH_OTA_FLAG_OFFSET
     * assert) rather than resizing/reordering the record. */
    rec->cumulative_runtime_sec = state.cumulative_runtime_sec;
    rec->charging_time_sec = state.charging_time_sec;
    /* Bootloader OTA byte at 0x08003C64: 0 = run app, 0x7F = enter OTA (set when factory test 0xFC = 0x7F) */
    rec->bootloader_ota_flag = (flash_ota_flag_next_save == 0x7Fu) ? 0x7Fu : 0u;
    if (flash_ota_flag_next_save == 0x7Fu)
        flash_ota_flag_next_save = 0;

    rec->crc32 = Flash_ComputeCrc32(((const uint8_t *)rec) + FLASH_CRC_START_OFFSET, FLASH_CRC_SIZE);
}

static uint8_t Flash_ErasePage(uint32_t address)
{
    uint32_t Timeout = 48000000;
    if ((address & 0x3FFu) != 0u)
        return 0;
    if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET)
    {
        WRITE_REG(FLASH->KEYR, FLASH_KEY1);
        WRITE_REG(FLASH->KEYR, FLASH_KEY2);
    }
    /* Clear error and EOP flags */
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
    SET_BIT(FLASH->CR, FLASH_CR_PER);
    WRITE_REG(FLASH->AR, address);
    SET_BIT(FLASH->CR, FLASH_CR_STRT);
    while (((FLASH->SR) & (FLASH_SR_BSY)) == (FLASH_SR_BSY))
    {
        if (Timeout-- == 0)
        {
            CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
            SET_BIT(FLASH->CR, FLASH_CR_LOCK);
            return 0;
        }
    }
    CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
    if (FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR))
    {
        SET_BIT(FLASH->CR, FLASH_CR_LOCK);
        return 0;
    }
    return 1;
}

static uint8_t Flash_ProgramBuffer(uint32_t address, const uint8_t *data, uint32_t len)
{
    uint32_t Timeout;
    uint32_t i = 0;
    uint32_t addr = address;
    uint16_t halfword;

    if ((address & 1u) != 0u)
        return 0;
    if (len == 0u || len > FLASH_PAGE_SIZE)
        return 0;

    /* Stride-2 format: each struct byte occupies one 16-bit halfword (LSByte = data,
     * MSByte = 0xFF). Matches the bootloader's read/write format so settings survive OTA.
     *
     * IRQs are disabled only for the atomic write+BSY-poll per halfword (~40µs each).
     * Re-enabling between halfwords lets the I2C ISR fire in the gaps, preventing
     * sustained I2C unresponsiveness during the program phase. */
    while (i < len)
    {
        uint32_t primask;
        halfword = (uint16_t)data[i] | 0xFF00u;

        primask = __get_PRIMASK();
        __disable_irq();

        FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
        SET_BIT(FLASH->CR, FLASH_CR_PG);
        *(__IO uint16_t *)addr = halfword;

        Timeout = 48000000;
        while (((FLASH->SR) & (FLASH_SR_BSY)) == (FLASH_SR_BSY))
        {
            if (Timeout-- == 0)
            {
                CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
                SET_BIT(FLASH->CR, FLASH_CR_LOCK);
                if (!primask)
                    __enable_irq();
                return 0;
            }
        }
        CLEAR_BIT(FLASH->CR, FLASH_CR_PG);

        if (!primask)
            __enable_irq();

        if (FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR))
        {
            SET_BIT(FLASH->CR, FLASH_CR_LOCK);
            return 0;
        }

        addr += 2u;
        i += 1u;
    }
    SET_BIT(FLASH->CR, FLASH_CR_LOCK);
    return 1;
}

void Flash_Init(void)
{
    flash_dirty = 0;
    flash_sequence = 0;
    flash_last_write_sec = 0;
    flash_record_valid_boot = 0;
    flash_save_attempted = 0;
    flash_last_save_success = 0;
    flash_auto_power_on_loaded = 0;
}

void Flash_Load(void)
{
    flash_persistent_data_t rec;
    size_t i;
    const uint8_t *flash_base = (const uint8_t *)FlashStorageStartPtr();
    size_t storage_size = (size_t)(FlashStorageEndPtr() - FlashStorageStartPtr());
    /* Stride-2 format: each struct byte is the LSByte of a 16-bit halfword in flash. */
    if (storage_size < sizeof(rec) * 2u)
    {
        FactoryReset();
        RequestFlashSave(0, 1);
        flash_sequence = 0;
        return;
    }
    for (i = 0u; i < sizeof(rec); i++)
        ((uint8_t *)&rec)[i] = flash_base[i * 2u];
    flash_record_valid_boot = Flash_RecordIsValid(&rec);
    flash_auto_power_on_loaded = 0u;

    if (flash_record_valid_boot)
    {
        Flash_ApplyRecordToState(&rec);
        flash_auto_power_on_loaded = (rec.auto_power_on != 0u) ? 1u : 0u;
        UpdateDerivedState();
        /* Recompute battery_percent from persisted true-VBAT so auto power-on can proceed after reset (issue 4). */
        UpdateBatteryPercentage();
        flash_sequence = rec.sequence_number;
        flash_dirty = 0;
        flash_last_write_sec = state.cumulative_runtime_sec;
    }
    else
    {
        FactoryReset();
        RequestFlashSave(0, 1);
        flash_sequence = 0;
    }
}

uint8_t Flash_Save(uint8_t bypass)
{
    flash_persistent_data_t rec;
    flash_persistent_data_t verify;
    uint16_t next_seq = (uint16_t)(flash_sequence + 1u);
    uint32_t target_addr = (uint32_t)(uintptr_t)FlashStorageStartPtr();
    uint8_t ok;
    size_t storage_size = (size_t)(FlashStorageEndPtr() - FlashStorageStartPtr());

    if (!flash_dirty && !bypass)
        return 1u;
    if (!bypass && !Flash_CanWrite())
        return 0u;
    /* Stride-2 format: each struct byte occupies one halfword in flash. */
    if (storage_size < sizeof(rec) * 2u)
        return 0u;

    flash_save_attempted = 1u;
    flash_last_save_success = 0u;

    Flash_FillRecordFromState(&rec, next_seq);

    /* No global IRQ disable here. Flash_ErasePage stalls flash reads for ~20ms (hardware
     * constraint on STM32F0 — unavoidable), but Flash_ProgramBuffer re-enables IRQs between
     * each halfword write so the I2C ISR can fire in the ~40µs gaps between writes. */
    ok = Flash_ErasePage(target_addr);
    if (ok)
        ok = Flash_ProgramBuffer(target_addr, (const uint8_t *)&rec, sizeof(rec));
    if (!ok)
        return 0u;

    {
        size_t vi;
        const uint8_t *flash_base = (const uint8_t *)FlashStorageStartPtr();
        for (vi = 0u; vi < sizeof(verify); vi++)
            ((uint8_t *)&verify)[vi] = flash_base[vi * 2u];
    }
    if (!Flash_RecordIsValid(&verify) || verify.sequence_number != next_seq)
        return 0u;

    flash_sequence = next_seq;
    flash_last_write_sec = state.cumulative_runtime_sec;
    flash_dirty = 0;
    flash_last_save_success = 1u;
    return 1u;
}

void Flash_FactoryReset(void)
{
    FactoryReset();
    RequestFlashSave(1, 1);
}

uint8_t Flash_IsDirty(void)
{
    return flash_dirty;
}

uint8_t Flash_CanWrite(void)
{
    uint32_t elapsed = state.cumulative_runtime_sec - flash_last_write_sec;
    return (elapsed >= FLASH_WRITE_RATE_LIMIT_SEC) ? 1u : 0u;
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
    {
    }
    LL_RCC_HSI_Enable();

    /* Wait till HSI is ready */
    while (LL_RCC_HSI_IsReady() != 1)
    {
    }
    LL_RCC_HSI_SetCalibTrimming(16);
    LL_RCC_HSI14_Enable();

    /* Wait till HSI14 is ready */
    while (LL_RCC_HSI14_IsReady() != 1)
    {
    }
    LL_RCC_HSI14_SetCalibTrimming(16);
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1)
    {
    }
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
    {
    }
    /* SysTick at 10 ms (not LL_Init1msTick's 1 ms): scheduler runs directly off SysTick,
     * see SysTick_Handler in stm32f0xx_it.c. Reload computed from the 48 MHz HCLK configured
     * above (AHB prescaler DIV_1, so HCLK == SYSCLK == 48 MHz). CLKSOURCE_Msk selects the
     * processor clock (HCLK), not HCLK/8. */
    SysTick->LOAD = (48000000UL / 100UL) - 1UL;
    SysTick->VAL  = 0UL;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    LL_SYSTICK_EnableIT();
    LL_SetSystemCoreClock(48000000);
    LL_RCC_HSI14_EnableADCControl();
    /* I2C1 timing assumes HSI (8 MHz). Update timing if I2C1SW changes. */
    LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
}

/** Upper bound for empty_voltage_mv (full - MIN_VOLTAGE_DELTA_MV, clamped to VBAT_EMPTY_MAX_MV). */
__attribute__((noinline)) static uint16_t EmptyVoltageMaxMv(void)
{
    uint16_t em = VBAT_EMPTY_MAX_MV;
    if (state.full_voltage_mv >= (uint16_t)MIN_VOLTAGE_DELTA_MV)
    {
        uint16_t u = state.full_voltage_mv - MIN_VOLTAGE_DELTA_MV;
        if (u < em) em = u;
    }
    return em;
}

/**
 * Commit tracked discharge-session minimum to state.empty_voltage_mv (spec: commit at
 * load-off, protection trigger, or output-current validity loss). Call with request_flash=0
 * when the caller will request flash anyway (e.g. protection path).
 * reset_min=1: reset min_vbat_while_load_on_mv (real shutdown: load-off or protection trigger).
 * reset_min=0: keep tracking (validity-loss commit — may be a polling gap, not a true shutdown).
 */
static void EmptyLearn_CommitTrackedMin(uint8_t request_flash, uint8_t reset_min)
{
    if (min_vbat_while_load_on_mv == 0xFFFFu)
        return;
    /* Do not commit if min is too close to full — no meaningful discharge (e.g. I2C stuck near full). */
    if ((uint32_t)min_vbat_while_load_on_mv + (uint32_t)EMPTY_LEARN_MIN_MEANINGFUL_DISCHARGE_MV >= (uint32_t)state.full_voltage_mv)
        return;
    uint16_t new_empty = ClampU16(min_vbat_while_load_on_mv, state.protection_voltage_mv, EmptyVoltageMaxMv());
    uint16_t old_empty = state.empty_voltage_mv;
    state.empty_voltage_mv = new_empty;
    if (reset_min)
        min_vbat_while_load_on_mv = 0xFFFFu;

    if (request_flash &&
        AbsDiffU16(new_empty, old_empty) >= EMPTY_PERSIST_MIN_CHANGE_MV &&
        !empty_persisted_this_session)
    {
        empty_persisted_this_session = 1;
        RequestFlashSave(0, 1);
    }
}

void UpdateBatteryMinMax(void)
{
    if (state.battery_params_self_programmed == 1)
    {
        state.battery_voltage_mv = ClampU16(state.battery_voltage_mv, state.empty_voltage_mv, state.full_voltage_mv);
    }
    else
    {
        /* When charger influences VBAT, discard session minimum so it cannot be committed later. */
        if (Charger_IsInfluencingVBAT(&sys_state))
        {
            min_vbat_while_load_on_mv = 0xFFFFu;
            empty_persisted_this_session = 0;
            empty_learn_load_off_start_tick = 0u;
        }
        /* Only track/commit empty when charger is ABSENT (actual discharge). */
        else if (sys_state.charger_state == CHARGER_STATE_ABSENT)
        {
            /* Track min VBAT when load is on; when INA invalid but RPi on, assume load on (brownout). */
            int16_t out_ma = state.output_current_mA;
            uint8_t load_off = state.output_current_valid &&
                (out_ma >= -(int16_t)EMPTY_LEARN_OUTPUT_CURRENT_THRESHOLD_MA &&
                 out_ma <= (int16_t)EMPTY_LEARN_OUTPUT_CURRENT_THRESHOLD_MA);
            uint8_t load_on = state.output_current_valid &&
                (out_ma < -(int16_t)EMPTY_LEARN_OUTPUT_CURRENT_THRESHOLD_MA ||
                 out_ma > (int16_t)EMPTY_LEARN_OUTPUT_CURRENT_THRESHOLD_MA);
            uint8_t assume_load_on = (sys_state.power_state == POWER_STATE_RPI_ON && !state.output_current_valid);

            if ((load_on || assume_load_on) && state.battery_voltage_mv >= state.protection_voltage_mv)
            {
                empty_learn_load_off_start_tick = 0u;  /* Load is on; clear load_off debounce */
                if (min_vbat_while_load_on_mv == 0xFFFFu ||
                    state.battery_voltage_mv < min_vbat_while_load_on_mv)
                    min_vbat_while_load_on_mv = state.battery_voltage_mv;
            }
            if (load_off && min_vbat_while_load_on_mv != 0xFFFFu)
            {
                uint32_t now = sched_flags.tick_counter;
                if (empty_learn_load_off_start_tick == 0u)
                    empty_learn_load_off_start_tick = now;
                else if ((now - empty_learn_load_off_start_tick) >= EMPTY_LEARN_LOAD_OFF_HOLD_TICKS)
                {
                    EmptyLearn_CommitTrackedMin(1, 1);
                    empty_learn_load_off_start_tick = 0u;
                }
            }
            else if (!load_off)
                empty_learn_load_off_start_tick = 0u;
        }
    }
    state.empty_voltage_mv = ClampU16(state.empty_voltage_mv, state.protection_voltage_mv, EmptyVoltageMaxMv());
}

void UpdateBatteryPercentage(void)
{
    int32_t range = (int32_t)state.full_voltage_mv - (int32_t)state.empty_voltage_mv;
    if (range < MIN_VOLTAGE_DELTA_MV)
        return;
    uint8_t charging = Charger_IsInfluencingVBAT(&sys_state);
    /* No true-VBAT-via-disconnect mechanism exists (IP_EN cannot gate the charge path -- see
     * UPSPlus_Behavior_Spec.md Section 9); use the raw/filtered ADC reading directly whether
     * charging or not, accepting reduced accuracy while a charger is present. */
    int32_t voltage = (int32_t)state.battery_voltage_mv;
    int32_t percentage;
    if (voltage <= (int32_t)state.empty_voltage_mv)
        percentage = 0;
    else if (voltage >= (int32_t)state.full_voltage_mv)
        percentage = 100;
    else
        percentage = (int32_t)(((voltage - (int32_t)state.empty_voltage_mv) * 100) / range);
    if (percentage < 0)
        percentage = 0;
    else if (percentage > 100)
        percentage = 100;

    /* Percent update direction uses charger state (Charger_IsInfluencingVBAT), not raw VBUS. */
    int32_t delta = percentage - (int32_t)state.battery_percent;
    if (delta >= (int32_t)BATTERY_PERCENT_HYSTERESIS || delta <= -(int32_t)BATTERY_PERCENT_HYSTERESIS)
    {
        if (charging && delta > 0)
            state.battery_percent = (uint8_t)percentage;
        else if (!charging && delta < 0)
            state.battery_percent = (uint8_t)percentage;
        else if (state.battery_percent == 0)
            state.battery_percent = (uint8_t)percentage;
    }
}

static void MX_TIM3_Init(void)
{
    /* TIM3 free-running 1 MHz counter (1 us ticks), no ISR. */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
    LL_TIM_DisableCounter(TIM3);
    LL_TIM_SetPrescaler(TIM3, 48u - 1u);
    LL_TIM_SetAutoReload(TIM3, 0xFFFFu);
    LL_TIM_SetCounter(TIM3, 0u);
    LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP);
    LL_TIM_EnableCounter(TIM3);
}

static void WaitUs(uint16_t us)
{
    uint16_t start = (uint16_t)LL_TIM_GetCounter(TIM3);
    while ((uint16_t)(LL_TIM_GetCounter(TIM3) - start) < us)
    {
    }
}

static void WaitMs(uint16_t ms)
{
    while (ms--) WaitUs(1000);
}

static uint8_t I2C1_BusIdleStable500us(void)
{
    uint8_t i;
    for (i = 0u; i < 50u; i++)
    {
        if (!LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_9) ||
            !LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_10))
            return 0u;
        WaitUs(10u);
    }
    return 1u;
}

/* Minimum ticks since the last STM32-addressed STOP before the fine-grained (TIM3,
 * microsecond) proximity check is no longer needed and live bus-idle sensing alone is
 * trusted, with no upper bound. Tick-resolution tracking (I2C1_GetLastStopTick(),
 * I2C1_GetLastAddrTick() — 32-bit, 10 ms resolution) has no wraparound ceiling, unlike TIM3's
 * 16-bit/~65.536 ms range, so this closes the starvation gap a steadily-polling master (e.g.
 * NUT's ~2 s critical_update_interval) used to hit between the old ~50 ms window and an idle
 * bypass: NUT's traffic clusters at one point per poll, leaving the bus genuinely quiet for
 * most of each cycle — this lets that quiet time be used, not just a sliver right after the
 * last transaction. Below this threshold, true elapsed time is guaranteed well under TIM3's
 * wrap range, so the microsecond check remains valid there. */
#define I2C_STOP_SETTLE_TICKS  1u

/* If no STM32-addressed I2C activity has EVER been observed (true standalone operation, or
 * shortly after boot with no RPi present), allow INA219 reads once idle this long. Kept at
 * the original, conservative value: I2C_STOP_SETTLE_TICKS above now closes the steady-polling
 * starvation gap directly, so this branch is only a defense-in-depth fallback for the
 * "no master ever" case, not the primary mechanism. */
#define I2C_MASTER_IDLE_ALLOW_READ_TICKS  (5u * TICKS_PER_1S)

static uint8_t I2C1_GuardWindowReady(void)
{
    uint16_t now_us = (uint16_t)LL_TIM_GetCounter(TIM3);
    uint16_t last_stop_us = I2C1_GetLastStopUs();
    uint8_t txn_active = I2C1_GetSlaveTxnActive();

    if (txn_active)
        return 0u;

    uint32_t now_ticks = sched_flags.tick_counter;
    uint32_t last_stop_tick = I2C1_GetLastStopTick();
    uint32_t last_addr_tick = I2C1_GetLastAddrTick();

    if (last_stop_tick == 0u && last_addr_tick == 0u)
    {
        /* No STM32-addressed activity observed since boot. */
        if (now_ticks <= I2C_MASTER_IDLE_ALLOW_READ_TICKS)
            return 0u;
    }
    else if ((now_ticks - last_stop_tick) < I2C_STOP_SETTLE_TICKS)
    {
        /* Within the same (or immediately preceding) 10 ms tick as the last STM32-addressed
         * STOP: true elapsed time is necessarily well under TIM3's 65.536 ms wrap range, so
         * fall back to the fine microsecond check for the mandatory 500 us settle time
         * (design plan Sec 5.1.3) that tick resolution alone can't guarantee. */
        if ((uint16_t)(now_us - last_stop_us) < 500u)
            return 0u;
    }

    if (!LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_9) ||
        !LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_10))
        return 0u;

    if (!I2C1_BusIdleStable500us())
        return 0u;

    return 1u;
}

static void BatteryPlateau_UpdateBatteryFullClear(uint32_t now_ticks)
{
    /* Clear FULL only on physical charger removal (ABSENT). */
    if (sys_state.charger_state == CHARGER_STATE_ABSENT)
    {
        battery_full = 0;
        battery_full_clear_start_ticks = 0;
        return;
    }
    if (filtered_vbat_mv < VBAT_FULL_RESET_MV)
    {
        if (battery_full_clear_start_ticks == 0u)
            battery_full_clear_start_ticks = now_ticks;
        if ((now_ticks - battery_full_clear_start_ticks) >= (uint32_t)(FULL_RESET_HOLD_SEC * TICKS_PER_1S))
            battery_full = 0;
    }
    else
    {
        battery_full_clear_start_ticks = 0;
    }
}

static void BatteryPlateau_Tick1s(void)
{
    uint32_t now_ticks = sched_flags.tick_counter;
    /* Only reset when charger physically absent. */
    uint8_t charger_connected = (sys_state.charger_state != CHARGER_STATE_ABSENT) ? 1u : 0u;

    if (state.battery_params_self_programmed != 0u)
    {
        battery_full = 0;
        battery_full_clear_start_ticks = 0;
        BatteryPlateau_ResetSession();
        plateau_last_charger_state = sys_state.charger_state;
        return;
    }

    if (!charger_connected)
    {
        battery_full = 0;
        battery_full_clear_start_ticks = 0;
        BatteryPlateau_ResetSession();
        plateau_last_charger_state = sys_state.charger_state;
        return;
    }

    /* Reset window only when transitioning from unplugged to connected. */
    if (plateau_last_charger_state == CHARGER_STATE_ABSENT && sys_state.charger_state != CHARGER_STATE_ABSENT)
        BatteryPlateau_ResetSession();
    plateau_last_charger_state = sys_state.charger_state;

    BatteryPlateau_UpdateBatteryFullClear(now_ticks);

    /* Taper accumulation: |battery_current_mA| <= I_TAPER_MA with fresh reading; else reset.
     * Bidirectional: slight discharge draws (negative current) count -- normal operation draws
     * small loads from the battery even while charging. Stale/invalid readings reset the timer. */
    if (state.battery_current_valid &&
        state.battery_current_mA >= -(int16_t)I_TAPER_MA &&
        state.battery_current_mA <= (int16_t)I_TAPER_MA)
        taper_hold_ticks += (uint32_t)TICKS_PER_1S;
    else
        taper_hold_ticks = 0;

    /* Taper path: independent of plateau window stability.
     * Declares FULL if current has been within +/-I_TAPER_MA for TAPER_HOLD_SEC and VBAT is at
     * near-top voltage. No learned-full update from this path (no plateau mean available). */
    if (taper_hold_ticks >= (uint32_t)(TAPER_HOLD_SEC * TICKS_PER_1S) &&
        filtered_vbat_mv >= VBAT_FULL_MIN_MV)
    {
        battery_full = 1;
        battery_full_clear_start_ticks = 0;
        if (!plateau_suppressed)
            plateau_suppressed = 1;
    }

    /* Plateau path: stable 30-min voltage window at near-top voltage.
     * Also updates the learned full-voltage calibration value. */
    plateau_eval_counter++;
    if (plateau_eval_counter < PLATEAU_EVAL_PERIOD_SEC)
        return;
    plateau_eval_counter = 0;

    plateau_samples[plateau_sample_index] = filtered_vbat_mv;
    if (plateau_sample_count < PLATEAU_WINDOW_SAMPLES)
        plateau_sample_count++;
    plateau_sample_index++;
    if (plateau_sample_index >= PLATEAU_WINDOW_SAMPLES)
        plateau_sample_index = 0;

    if (plateau_sample_count < PLATEAU_WINDOW_SAMPLES)
        return;

    uint16_t min_v = 0xFFFFu;
    uint16_t max_v = 0u;
    uint32_t sum = 0u;
    for (uint16_t i = 0; i < plateau_sample_count; i++)
    {
        uint16_t v = plateau_samples[i];
        if (v < min_v)
            min_v = v;
        if (v > max_v)
            max_v = v;
        sum += v;
    }

    if ((uint16_t)(max_v - min_v) > PLATEAU_DELTA_MV)
        return;

    uint16_t plateau_mean = (uint16_t)(sum / plateau_sample_count);
    if (plateau_mean < PLATEAU_MEAN_MIN_MV)
        return;

    /* Plateau path requires VBAT at near-top voltage; taper is handled independently above. */
    if (filtered_vbat_mv < VBAT_FULL_MIN_MV)
        return;

    if (state.full_voltage_mv < LEARNED_FULL_MIN_MV || state.full_voltage_mv > LEARNED_FULL_MAX_MV)
        state.full_voltage_mv = VBAT_FULL_MIN_MV;

    uint16_t learned_full = state.full_voltage_mv;
    uint16_t old_full = state.full_voltage_mv;

    {
        uint32_t num = (uint32_t)learned_full * (uint32_t)(PLATEAU_ALPHA_DEN - PLATEAU_ALPHA_NUM) +
                       (uint32_t)plateau_mean * (uint32_t)PLATEAU_ALPHA_NUM +
                       (uint32_t)(PLATEAU_ALPHA_DEN / 2u);
        uint16_t new_full = (uint16_t)(num >> 2u);  /* PLATEAU_ALPHA_DEN == 4 */
        uint16_t min_allowed = (uint16_t)(state.empty_voltage_mv + MIN_VOLTAGE_DELTA_MV);
        if (new_full < min_allowed)
            new_full = min_allowed;
        new_full = ClampU16(new_full, LEARNED_FULL_MIN_MV, LEARNED_FULL_MAX_MV);

        if (new_full != state.full_voltage_mv)
        {
            state.full_voltage_mv = new_full;
            UpdateBatteryPercentage();
            Snapshot_UpdateDerived();
            if (!plateau_persisted_this_session &&
                AbsDiffU16(old_full, new_full) >= PLATEAU_PERSIST_MIN_CHANGE_MV)
            {
                RequestFlashSave(0, 1);
                plateau_persisted_this_session = 1;
            }
        }
    }

    battery_full = 1;
    battery_full_clear_start_ticks = 0;
    if (!plateau_suppressed)
        plateau_suppressed = 1;
}


/* Scheduler 10 ms tick: flag-only work, called from SysTick every 10th tick.
 * On Cortex-M0, aligned 32-bit write to tick_counter is atomic; main reads it for deltas. */
void Scheduler_ISR_Tick10ms(void)
{
    sched_flags.tick_counter++;
    I2C1_SetTickCounter(sched_flags.tick_counter);
    sched_flags.tick_10ms = 1;
    if (--ticks_until_100ms == 0u)
    {
        ticks_until_100ms = TICKS_PER_100MS;
        sched_flags.tick_100ms = 1;
    }
    if (--ticks_until_500ms == 0u)
    {
        ticks_until_500ms = TICKS_PER_500MS;
        sched_flags.tick_500ms = 1;
    }
}

/* MT_EN: restart phase overrides (MT_EN low for 5s); else driven by power_state.
 * Protection cut happens only after flash commit in main loop; see pending_power_cut. */
static void ApplyGPIOFromState(void)
{
    if (restart_phase != 0)
        LL_GPIO_ResetOutputPin(GPIOA, MT_EN);
    else if (sys_state.power_state == POWER_STATE_RPI_ON)
        LL_GPIO_SetOutputPin(GPIOA, MT_EN);
    else
        LL_GPIO_ResetOutputPin(GPIOA, MT_EN);
    /* IP_EN: HIGH except during a brief, deliberate KEY pulse owned by Section 9.1 (periodic
     * IP5328 reset) or Section 9.2 (battery-level LED check) -- see ip_en_owner, arbitrated in
     * IPEnArbitration_Tick10ms(). IP_EN physically drives the IP5328's KEY (button-detect) pin,
     * confirmed against the datasheet and a live measurement (UPSPlus_Behavior_Spec.md Section
     * 9) -- it is NOT a charger-enable gate; KEY only controls the output boost stage
     * (VOUT1/VOUT2/USB-C), the battery-level LED display, and the WLED flashlight. An earlier
     * revision pulsed IP_EN LOW for 1.5s periodically (CHARGER_STATE_FORCED_OFF_WINDOW), intending
     * to disconnect the charger path for true-VBAT sampling; measured battery current
     * before/during that pulse was unchanged, confirming the pulse did nothing but risk spurious
     * IP5328 button clicks. That mechanism was removed (see Section 6/7 history) in favor of
     * holding IP_EN HIGH unconditionally; the two deliberate KEY-press mechanisms above were
     * added later, reusing the confirmed KEY behavior on purpose instead of trying to avoid it. */
    if (ip_en_owner != IP_EN_OWNER_NONE)
        LL_GPIO_ResetOutputPin(GPIOA, IP_EN);
    else
        LL_GPIO_SetOutputPin(GPIOA, IP_EN);
    LL_GPIO_SetOutputPin(GPIOA, PWR_EN);
}

static void Protection_Step(void)
{
    if (sys_state.power_state != POWER_STATE_RPI_ON)
        return;
    /* Use charger state, not voltage heuristic (plan: true-VBAT / protection when not charging) */
    if (Charger_IsInfluencingVBAT(&sys_state))
    {
        prot_state.below_threshold_count = 0;
        return;
    }
    if (adc_sample_seq != prot_state.last_seen_seq)
    {
        prot_state.last_seen_seq = adc_sample_seq;
        if (prot_state.last_adc_battery_mv <= state.protection_voltage_mv)
        {
            if (prot_state.below_threshold_count < PROTECTION_SAMPLES_REQUIRED)
                prot_state.below_threshold_count++;
        }
        else if (prot_state.last_adc_battery_mv > state.protection_voltage_mv + PROTECTION_HYSTERESIS_MV)
        {
            prot_state.below_threshold_count = 0;
        }
    }
    if (prot_state.below_threshold_count < PROTECTION_SAMPLES_REQUIRED)
        return;
    /* Invariant A6: request flash, set pending_power_cut; main loop commits then sets PROTECTION_LATCHED */
    if (sys_state.pending_power_cut == 0u)
    {
        BootBackoff_OnProtectionTrigger();
        sys_state.pending_power_cut_start_ticks = sched_flags.tick_counter;
        /* Empty learning: commit tracked min before power cut (spec commit condition (2)). */
        if (state.battery_params_self_programmed == 0)
            EmptyLearn_CommitTrackedMin(0, 1);
    }
    RequestFlashSave(1, 1);
    sys_state.pending_power_cut = 1;
}

static void Protection_ForceCutIfTimedOut(void)
{
    if (sys_state.pending_power_cut == 0u)
        return;
    if ((sched_flags.tick_counter - sys_state.pending_power_cut_start_ticks) < PROTECTION_FORCE_CUT_TIMEOUT_TICKS)
        return;
    sys_state.power_state = POWER_STATE_PROTECTION_LATCHED;
    sys_state.power_state_entry_ticks = sched_flags.tick_counter;
    sys_state.pending_power_cut = 0;
    sys_state.pending_power_cut_start_ticks = 0;
}

/* CHARGER_STATE_FORCED_OFF_WINDOW and LEARNING_ACTIVE are kept as enum constants (their numeric
 * values are part of the Factory Testing ABI, selector 0x01) but this state machine now never
 * transitions into them: the measurement window they represented pulsed IP_EN LOW to try to
 * disconnect the charger path for true-VBAT sampling, which datasheet review and a live current
 * measurement both confirmed IP_EN cannot do (see UPSPlus_Behavior_Spec.md Section 9). charger_state
 * is therefore only ever ABSENT or PRESENT now; sys->learning_mode (derived below, unchanged)
 * consequently always reads LEARNING_INACTIVE. */
static void ChargerStateMachine_Step(void)
{
    uint32_t tick = sched_flags.tick_counter;
    uint16_t charger_mv = UpsChargerVoltageMv(&state);

    if (adc_sample_seq != charger_physical.charger_last_seen_seq)
    {
        charger_physical.charger_last_seen_seq = adc_sample_seq;
        if (charger_mv > CHARGER_PRESENT_ON_MV)
            charger_physical.charger_physically_present = 1;
        else if (charger_mv < CHARGER_PRESENT_OFF_MV)
            charger_physical.charger_physically_present = 0;

        if (sys_state.charger_state == CHARGER_STATE_ABSENT)
        {
            if (charger_mv > CHARGER_PRESENT_ON_MV)
            {
                charger_physical.charger_stability_count++;
                if (charger_physical.charger_stability_count >= CHARGER_STABILITY_SAMPLES)
                {
                    sys_state.charger_state = CHARGER_STATE_PRESENT;
                    sys_state.charger_state_entry_ticks = tick;
                    charger_physical.charger_stability_count = 0;
                    charger_physical.charger_physically_present = 1;
                    /* Section 9.1: (re)arm the periodic-reset elapsed-charging baseline. */
                    ip5328_reset_baseline_sec = state.charging_time_sec;
                }
            }
            else
                charger_physical.charger_stability_count = 0;
        }
        else if (sys_state.charger_state == CHARGER_STATE_PRESENT)
        {
            if (charger_mv < CHARGER_PRESENT_OFF_MV)
            {
                charger_physical.charger_stability_count++;
                if (charger_physical.charger_stability_count >= CHARGER_STABILITY_SAMPLES)
                {
                    sys_state.charger_state = CHARGER_STATE_ABSENT;
                    sys_state.charger_state_entry_ticks = tick;
                    charger_physical.charger_stability_count = 0;
                    charger_physical.charger_physically_present = 0;
                    /* Section 9.1: cancel countdown, no partial credit into the next session.
                     * A reset that became due but hadn't yet fired through arbitration is
                     * dropped too -- it can only be initiated while charger_state == PRESENT. */
                    ip5328_reset_pending = 0;
                }
            }
            else
                charger_physical.charger_stability_count = 0;
        }
    }
}

/**
 * Restart power-cycle step: when in restart phase, count down 10ms ticks;
 * at 0 transition to RPI_ON (entry: reset protection count). Runs every 10ms.
 */
static void RestartPowerCycle_Step(void)
{
    if (restart_phase == 0)
        return;
    if (restart_remaining_ticks == 0)
        return;
    restart_remaining_ticks--;
    if (restart_remaining_ticks == 0)
    {
        restart_phase = 0;
        sys_state.power_state = POWER_STATE_RPI_ON;
        sys_state.power_state_entry_ticks = sched_flags.tick_counter;
        prot_state.below_threshold_count = 0;
        BootBackoff_OnPowerOn();
    }
}

static void PowerStateMachine_Step(void)
{
    RestartPowerCycle_Step();
    Protection_Step();
}

/**
 * Power FSM tick_1s - countdown decrements and transitions (single place).
 * Entry to RPI_ON resets protection sample count so N samples are observed while on.
 */
static void PowerStateMachine_OnTick1s(void)
{
    /* Shutdown countdown: action at 1, then clear; else decrement */
    if (state.shutdown_countdown_sec == 1)
    {
        sys_state.power_state = POWER_STATE_RPI_OFF;
        sys_state.power_state_entry_ticks = sched_flags.tick_counter;
        state.shutdown_countdown_sec = 0;
    }
    else if (state.shutdown_countdown_sec > 1)
        state.shutdown_countdown_sec--;

    /* Restart countdown: at 1 start non-blocking power-cycle (MT_EN low 5s then RPI_ON) */
    if (state.restart_countdown_sec == 1)
    {
        state.restart_countdown_sec = 0;
        restart_phase = 1;
        restart_remaining_ticks = RESTART_POWER_OFF_TICKS;
        sys_state.power_state = POWER_STATE_RPI_OFF;  /* 0x17 reports no power during cycle */
        sys_state.power_state_entry_ticks = sched_flags.tick_counter;
    }
    else if (state.restart_countdown_sec > 1)
        state.restart_countdown_sec--;

    /* Load-on-delay: decrement; at 0 and battery_ok -> RPI_ON with entry action */
    if (state.load_on_delay_remaining_sec != 0)
    {
        state.load_on_delay_remaining_sec--;
        if (state.load_on_delay_remaining_sec == 0 &&
            state.battery_percent > state.low_battery_percent)
        {
            sys_state.power_state = POWER_STATE_RPI_ON;
            sys_state.power_state_entry_ticks = sched_flags.tick_counter;
            prot_state.below_threshold_count = 0;
            BootBackoff_OnPowerOn();
        }
    }

    BootBackoff_CheckSuccess();
}

static void CurrentAge_Tick10ms(void)
{
    uint8_t prev_output_valid = state.output_current_valid;
    uint8_t prev_battery_valid = state.battery_current_valid;

    if (state.output_current_age_10ms < 0xFFFFu) {
        state.output_current_age_10ms++;
    }
    if (state.battery_current_age_10ms < 0xFFFFu) {
        state.battery_current_age_10ms++;
    }

    state.output_current_valid =
        (((uint32_t)state.output_current_age_10ms) <= CURRENT_VALID_AGE_TICKS) ? 1u : 0u;
    state.battery_current_valid =
        (((uint32_t)state.battery_current_age_10ms) <= CURRENT_VALID_AGE_TICKS) ? 1u : 0u;

    /* Empty-learning brownout detection (formerly "condition (3)") was removed: it inferred a
     * dead Pi from output-current validity loss and/or I2C master silence, but neither signal
     * actually indicates that. Validity loss usually means the bus is busy (a live master is
     * actively transacting). I2C silence alone is also unsafe: it's indistinguishable from
     * "NUT isn't running" while the Pi is otherwise perfectly healthy. The one signal that
     * actually reflects whether the Pi has lost power is the measured output current itself,
     * which condition (1) (`load_off`, below) already uses directly — and now that
     * I2C_STOP_SETTLE_TICKS lets the INA219 probe get prompt, valid readings regardless of
     * whether NUT is polling, condition (1) alone is sufficient: a live-but-NUT-less Pi reads
     * back normal current (no false trigger); a genuinely dead Pi reads back near-zero current
     * (correctly triggers). */

    if (state.output_current_valid != prev_output_valid ||
        state.battery_current_valid != prev_battery_valid)
        Snapshot_UpdateDerived();
}

/**
 * Section 9.2 periodic auto-trigger: while genuinely discharging (RPI_ON && charger ABSENT),
 * automatically request a battery-level LED check every sample_period_minutes, in addition to
 * the manual button trigger in Button_DispatchActions(). Both routes funnel into the same
 * led_check_requested flag and share the same arbitration/hold mechanism, so an automatic and a
 * manual request can never interleave.
 */
static void LedCheckPeriodic_OnTick1s(void)
{
    uint8_t active = (sys_state.power_state == POWER_STATE_RPI_ON &&
                       sys_state.charger_state == CHARGER_STATE_ABSENT);

    if (active && !led_check_periodic_active_prev)
    {
        /* Just started discharging on battery: begin a fresh interval (no partial credit from
         * a prior charging/off session, mirroring Section 9.1's baseline semantics). */
        led_check_periodic_remaining_sec = (uint32_t)state.sample_period_minutes * 60u;
    }
    led_check_periodic_active_prev = active;

    if (!active)
        return;

    if (led_check_periodic_remaining_sec > 0u)
        led_check_periodic_remaining_sec--;
    if (led_check_periodic_remaining_sec == 0u)
    {
        led_check_requested = 1u;
        led_check_periodic_remaining_sec = (uint32_t)state.sample_period_minutes * 60u;
    }
}

/**
 * Section 9.3 IP_EN arbitration: single shared ownership so the Section 9.1 periodic reset and
 * Section 9.2 battery-level LED check pulses cannot interleave or corrupt each other's hold
 * timing. A due periodic reset always proceeds as soon as IP_EN frees (never postponed by a
 * nuisance button press); a concurrent LED-check request is dropped, not queued, if IP_EN is
 * already busy -- a missed LED check is harmless and the user can just press again.
 */
static void IPEnArbitration_Tick10ms(void)
{
    uint32_t now = sched_flags.tick_counter;

    if (ip_en_owner != IP_EN_OWNER_NONE && (int32_t)(now - ip_en_release_tick) >= 0)
        ip_en_owner = IP_EN_OWNER_NONE;

    if (ip5328_reset_pending && ip_en_owner == IP_EN_OWNER_NONE &&
        sys_state.charger_state == CHARGER_STATE_PRESENT)
    {
        ip_en_owner = IP_EN_OWNER_PERIODIC_RESET;
        ip_en_release_tick = now + IP5328_RESET_HOLD_TICKS;
        ip5328_reset_pending = 0;
        if (ip5328_reset_pulse_count < 255u)
            ip5328_reset_pulse_count++;
    }

    if (led_check_requested)
    {
        led_check_requested = 0;  /* Consumed either way; dropped (not queued) if IP_EN is busy */
        if (ip_en_owner == IP_EN_OWNER_NONE)
        {
            ip_en_owner = IP_EN_OWNER_LED_CHECK;
            ip_en_release_tick = now + KEY_LED_CHECK_HOLD_TICKS;
        }
    }
}

static void Scheduler_Tick10ms(void)
{
    PowerStateMachine_Step();
    ChargerStateMachine_Step();
    IPEnArbitration_Tick10ms();
    /* Single place for GPIO: reflects power state, charger state, and IP_EN ownership after
     * all three of the above */
    ApplyGPIOFromState();

    /* Button handling (tick_10ms) */
    Button_ProcessTick10ms();
    CurrentAge_Tick10ms();
}  /* end Scheduler_Tick10ms() */

static void Button_ProcessTick10ms(void)
{
    uint8_t button_pressed = (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_1) == 0u);

    if (button_handler.state == BUTTON_IDLE && sKeyFlag == 0u)
        return;

    switch (button_handler.state)
    {
    case BUTTON_IDLE:
        button_handler.state = BUTTON_PRESSED;
        button_handler.debounce_counter = 0;
        button_handler.hold_ticks = 0;
        button_handler.long_press_fired = 0;
        button_handler.pending_click = BUTTON_CLICK_NONE;
        button_handler.press_start_tick = sched_flags.tick_counter;
        button_last_level = button_pressed ? 1u : 0u;
        break;

    case BUTTON_PRESSED:
        if (button_pressed != button_last_level)
        {
            button_last_level = button_pressed ? 1u : 0u;
            button_handler.debounce_counter = 0;
        }
        else if (button_handler.debounce_counter < BUTTON_DEBOUNCE_TICKS)
        {
            button_handler.debounce_counter++;
        }
        if (button_last_level)
        {
            if (button_handler.debounce_counter >= BUTTON_DEBOUNCE_TICKS)
            {
                button_handler.state = BUTTON_HELD;
                button_handler.hold_ticks = 0;
                button_handler.debounce_counter = 0;
                sKeyFlag = 0;
            }
        }
        else
        {
            if (button_handler.debounce_counter >= BUTTON_DEBOUNCE_TICKS)
            {
                button_handler.state = BUTTON_IDLE;
                button_handler.press_start_tick = 0;
                button_handler.debounce_counter = 0;
                sKeyFlag = 0;
            }
        }
        break;

    case BUTTON_HELD:
        if (button_pressed)
        {
            button_last_level = 1;
            button_handler.debounce_counter = 0;
            {
                uint32_t held = sched_flags.tick_counter - button_handler.press_start_tick;
                if (held > (uint32_t)(BUTTON_LONG_PRESS_TICKS + 1u))
                    held = (uint32_t)(BUTTON_LONG_PRESS_TICKS + 1u);
                button_handler.hold_ticks = (uint16_t)held;
            }
            if (!button_handler.long_press_fired &&
                button_handler.hold_ticks >= BUTTON_LONG_PRESS_TICKS)
            {
                button_handler.pending_click = BUTTON_CLICK_LONG;
                button_handler.long_press_fired = 1;
            }
        }
        else
        {
            if (button_last_level != 0u)
            {
                button_last_level = 0u;
                button_handler.debounce_counter = 0;
            }
            else if (button_handler.debounce_counter < BUTTON_DEBOUNCE_TICKS)
            {
                button_handler.debounce_counter++;
            }
            if (button_handler.debounce_counter >= BUTTON_DEBOUNCE_TICKS)
            {
                if (!button_handler.long_press_fired &&
                    button_handler.hold_ticks < BUTTON_LONG_PRESS_TICKS)
                {
                    button_handler.pending_click = BUTTON_CLICK_SHORT;
                }
                button_handler.press_start_tick = 0;
                button_handler.hold_ticks = 0;
                button_handler.long_press_fired = 0;
                button_handler.debounce_counter = 0;
                button_handler.state = BUTTON_IDLE;
                sKeyFlag = 0;
            }
        }
        break;

    default:
        button_handler.state = BUTTON_IDLE;
        button_handler.press_start_tick = 0;
        button_handler.debounce_counter = 0;
        button_handler.hold_ticks = 0;
        button_handler.long_press_fired = 0;
        button_handler.pending_click = BUTTON_CLICK_NONE;
        sKeyFlag = 0;
        break;
    }
}

static void Button_DispatchActions(void)
{
    if (button_handler.pending_click == BUTTON_CLICK_NONE)
        return;

    if (button_handler.pending_click == BUTTON_CLICK_SHORT)
    {
        if (sys_state.power_state == POWER_STATE_RPI_OFF ||
            sys_state.power_state == POWER_STATE_PROTECTION_LATCHED ||
            sys_state.power_state == POWER_STATE_LOAD_ON_DELAY)
        {
            /* A button press is the user explicitly asking to power on now, so it deliberately
             * does NOT gate on low_battery_percent (default 10%) the way automatic power-on does
             * -- that's a "time to shut down soon" warning threshold, not a safety limit, and a
             * user should be able to manually power on at e.g. 5% if they want to. It DOES still
             * require the same protection-voltage+hysteresis margin as the automatic path
             * (CheckPowerOnConditions): a bare battery_voltage_mv > protection_voltage_mv check
             * with no hysteresis let a press power back on right at the empty threshold with no
             * source attached, driving the battery straight back into protection cutoff. */
            uint8_t allow_power_on = (state.battery_voltage_mv > (uint16_t)(state.protection_voltage_mv + PROTECTION_HYSTERESIS_MV));
            if (allow_power_on)
            {
                sys_state.power_state = POWER_STATE_RPI_ON;
                sys_state.power_state_entry_ticks = sched_flags.tick_counter;
                prot_state.below_threshold_count = 0;
                sys_state.pending_power_cut = 0;
                sys_state.pending_power_cut_start_ticks = 0;
                state.load_on_delay_remaining_sec = 0;
                BootBackoff_OnPowerOn();
                Snapshot_UpdateDerived();
            }
        }
        else if (sys_state.power_state == POWER_STATE_RPI_ON &&
                 sys_state.charger_state == CHARGER_STATE_ABSENT)
        {
            /* Section 9.2: battery-level LED check. No-op while charger PRESENT (LEDs already
             * visible via IP5328's own charging-mode display) -- that case simply isn't matched
             * here. Arbitrated (may be dropped if IP_EN is already busy) in
             * IPEnArbitration_Tick10ms(). */
            led_check_requested = 1u;
        }
    }
    else if (button_handler.pending_click == BUTTON_CLICK_LONG)
    {
        if (sys_state.power_state == POWER_STATE_RPI_ON)
        {
            sys_state.power_state = POWER_STATE_RPI_OFF;
            sys_state.power_state_entry_ticks = sched_flags.tick_counter;
            sys_state.pending_power_cut = 0;
            sys_state.pending_power_cut_start_ticks = 0;
            state.load_on_delay_remaining_sec = 0;
            Snapshot_UpdateDerived();
        }
        else if (sys_state.power_state == POWER_STATE_RPI_OFF)
        {
            sys_state.factory_reset_requested = 1;
        }
    }

    button_handler.pending_click = BUTTON_CLICK_NONE;
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void)
{
    LL_ADC_InitTypeDef ADC_InitStruct;
    LL_ADC_REG_InitTypeDef ADC_REG_InitStruct;
    LL_GPIO_InitTypeDef GPIO_InitStruct;

    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    GPIO_InitStruct.Pin = PI_VCC_SENSE;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = VBAT_SENSE;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = VBUS_SENSE;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = USB_IN_SENSE;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_0);
    LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_1);
    LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_2);
    LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_3);
    LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_TEMPSENSOR);
    LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_VREFINT);
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
                                   LL_ADC_PATH_INTERNAL_VREFINT |
                                       LL_ADC_PATH_INTERNAL_TEMPSENSOR);

    ADC_InitStruct.Clock = LL_ADC_CLOCK_ASYNC;
    ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
    ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
    LL_ADC_Init(ADC1, &ADC_InitStruct);

    ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
    ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
    ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
    ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;

    LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
    LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
    LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_239CYCLES_5);

    LL_ADC_StartCalibration(ADC1);
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
    {
    };
    LL_ADC_Enable(ADC1);
    LL_ADC_REG_StartConversion(ADC1);
}

/**
  * Enable DMA controller and interrupt
  * This is used to update the values in the aADCxConvertedData buffer
  * once the ADC conversion is complete.
  */
 static void MX_DMA_Init(void)
 {
    /* Init with LL driver */
    /* DMA controller clock enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    /* DMA interrupt init */
    NVIC_SetPriority(DMA1_Channel1_IRQn, 3);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);

    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);
    /* NORMAL (one-shot), not CIRCULAR: pairs with the ADC's DMATransfer=LIMITED
     * per the reference manual (CIRC=1 with DMACFG=0 is a documented-prohibited
     * combination, and is suspected to be the root cause of the ADEN/ADSTART
     * deadlock this part has hit). Re-armed explicitly each cycle in the
     * tick_500ms handler alongside LL_ADC_REG_StartConversion(). */
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

    LL_DMA_ConfigAddresses(
        DMA1, LL_DMA_CHANNEL_1,
        LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
        (uint32_t)&aADCxConvertedData, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ADC_CONVERTED_DATA_BUFFER_SIZE);

    /* Enable DMA interrupt and channel BEFORE enabling ADC and starting conversion */
    LL_DMA_EnableIT_TC(DMA1,LL_DMA_CHANNEL_1); // Enable DMA transfer complete interrupt
    LL_DMA_EnableIT_TE(DMA1,LL_DMA_CHANNEL_1); // Enable DMA transfer error interrupt

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}
/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    LL_EXTI_InitTypeDef EXTI_InitStruct;
    LL_GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);

    LL_GPIO_SetOutputPin(GPIOA, IP_EN);
    LL_GPIO_ResetOutputPin(GPIOA, MT_EN); /* Disabled */
    LL_GPIO_SetOutputPin(GPIOA, PWR_EN);  /* Must be set high, or else it cannot be turned off. */

    GPIO_InitStruct.Pin = OTA_DETECT;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = IP_EN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MT_EN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PWR_EN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE1);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_1, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT);

    EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_1;
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
    LL_EXTI_Init(&EXTI_InitStruct);

    NVIC_EnableIRQ(EXTI0_1_IRQn);
    NVIC_SetPriority(EXTI0_1_IRQn, 0);
}

void EXTI0_1_IRQHandler(void)
{
    /* Manage Flags */
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_1) != RESET)
    {
        sKeyFlag = 1;
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
    }
}
