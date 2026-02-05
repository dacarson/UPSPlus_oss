#include "pindef.h"

/* RCC_CSR reset-flag byte: bits 31:25 (LPWRRSTF down to BORRSTF); client interprets raw. */
#define RCC_CSR_RESET_FLAGS_SHIFT  25
#define RCC_CSR_RESET_FLAGS_MASK   0x7Fu

#include "stm32f0xx_ll_adc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_crs.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_i2c.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_crc.h"

#include "I2C_Slave.h"
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

/* Everything below needs to be mapped to registers */

#define ADC_CONVERTED_DATA_BUFFER_SIZE 6

/* Raw ADC buffer; DMA fills, main loop processes into state */
__IO uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];

/* Timing/button helpers (TIM1/EXTI). Authoritative state is state/sys_state only;
 * no shadow globals (uXXXVolt, counters, mode flags) remain as authoritative. */
__IO uint8_t sKeyFlag = 0;         /* Button activity (EXTI edge) */
__IO uint16_t sIPIdleTicks = 0;
__IO uint8_t MustRefreshVDD = 1;
/* Countdowns decremented in main loop on tick_1s (removed from ISR) */

#define BL_START_ADDRESS 0x8000000

/* IWDG: PR=6 (÷256), RLR chosen for ~8 s minimum at 56 kHz LSI (~17 s at 26 kHz). Main loop must complete within timeout. */
#define IWDG_PR_256   6u
#define IWDG_RLR_VAL  1748u

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
static window_manager_state_t window_mgr;
static charger_physical_state_t charger_physical;
static protection_state_t prot_state;
static button_handler_t button_handler;
static uint8_t button_last_level = 0; /* 1=pressed, 0=released */

/* Double-buffered register image. Snapshot_Update fills reg_image for I2C TX. */
uint8_t reg_image[2][256];
volatile uint8_t active_reg_image = 0;

/* ADC flags: DMA sets adc_ready; main loop processes and increments adc_sample_seq */
volatile uint8_t adc_ready = 0;
volatile uint32_t adc_sample_seq = 0;

/* Canonical scheduler - TIM1 sets flags only; main loop runs all tasks.
 * Flag race: small chance of losing an event if ISR sets a flag between main's check and clear.
 * For coarse 100ms/500ms tasks usually acceptable; for never-miss semantics consider
 * counters (increment in ISR) or copy+clear in one short critical section in main. */
static scheduler_flags_t sched_flags;
/* Main loop derives tick_1s from tick_100ms (10 pulses = 1s) */
static uint8_t tick_100ms_count = 0;
/* Downcounters for 100ms/500ms flags (avoid modulo in ISR) */
static uint8_t ticks_until_100ms = TICKS_PER_100MS;
static uint8_t ticks_until_500ms = TICKS_PER_500MS;

/* Flash save requested; main loop snapshots then commits (rate-limited unless bypassed). */
static volatile uint8_t flash_save_requested = 0;
static volatile uint8_t flash_save_bypass = 0;
static uint8_t flash_dirty = 0;
static uint16_t flash_sequence = 0;
static uint32_t flash_last_write_sec = 0;
static uint32_t flash_next_retry_sec = 0;
/* Reset cause: captured at boot, exported via factory test 0x08/0x09. */
static uint32_t boot_reset_csr = 0u;
static uint8_t last_persisted_reset_cause = 0u;
static uint8_t last_persisted_reset_seq = 0u;

/* Flash/persistence debug (factory test page 0x05) */
static uint8_t flash_record_valid_boot = 0;
static uint8_t flash_save_attempted = 0;
static uint8_t flash_last_save_success = 0;
static uint8_t flash_auto_power_on_loaded = 0;
static volatile uint8_t ina_probe_requested = 0;
static uint32_t ina_probe_due_ticks = 0;
static uint8_t ina_probe_is_output = 1u;
static uint8_t ina_next_is_output = 1u;

static void MX_TIM3_Init(void);
static uint8_t I2C1_GuardWindowReady(void);
static uint8_t I2C1_BusIdleStable500us(void);
static void WaitUs(uint16_t us);

/* Battery plateau full detection (self-programming enabled) */
static uint16_t filtered_vbat_mv = 0;
static uint16_t plateau_samples[PLATEAU_WINDOW_SAMPLES];
static uint16_t plateau_sample_count = 0;
static uint16_t plateau_sample_index = 0;
static uint8_t plateau_suppressed = 0;
static uint8_t plateau_persisted_this_session = 0;
static charger_state_t plateau_last_charger_state = CHARGER_STATE_ABSENT;
static power_state_t plateau_last_power_state = POWER_STATE_RPI_OFF;
static uint8_t plateau_eval_counter = 0;
/* Empty learning: minimum battery voltage seen while load was on during this discharge (0xFFFF = none yet) */
static uint16_t min_vbat_while_load_on_mv = 0xFFFF;
static uint8_t empty_persisted_this_session = 0;
static uint8_t battery_full = 0;
static uint32_t battery_full_clear_start_ticks = 0;
static uint32_t taper_hold_ticks = 0;   /* Seconds (in ticks) battery current <= I_TAPER_MA with valid; for taper gate */

/* Restart power-cycle state machine: MT_EN low for 5s then RPI_ON (non-blocking) */
#define RESTART_POWER_OFF_TICKS  (5u * TICKS_PER_1S)  /* 5 seconds in 10ms ticks */
static uint8_t restart_phase = 0;       /* 0=idle, 1=MT_EN low, counting down */
static uint16_t restart_remaining_ticks = 0;

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

uint8_t IsTrueVbatSampleFresh(uint32_t now_ticks, uint32_t last_true_vbat_sample_tick)
{
    /* Unsigned wrap gives correct age when tick_counter has wrapped */
    uint32_t age_ticks = now_ticks - last_true_vbat_sample_tick;
    return (age_ticks <= TRUE_VBAT_MAX_AGE_TICKS) ? 1u : 0u;
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
    if (state->learning_mode == LEARNING_ACTIVE)
        value |= 0x02u; /* Learning/Calibration enabled */
    return value;
}

/**
 * Fill 256-byte register image from authoritative state + system state (for 0x17).
 * Little-endian multi-byte registers; reserved/factory regions filled with 0x00.
 */
static void StateToRegisterBuffer(const authoritative_state_t *auth, const system_state_t *sys,
                                  uint8_t *buf)
{
    uint16_t u16;
    uint32_t u32;
    uint16_t i;
    /* Snapshot local copies for debug/diagnostic pages (main-loop owned state). */
    const button_handler_t button_snapshot = button_handler;
    const charger_physical_state_t charger_snapshot = charger_physical;
    const window_manager_state_t window_snapshot = window_mgr;
    const protection_state_t protection_snapshot = prot_state;

    buf[REG_MCU_VOLTAGE_L]       = (uint8_t)(auth->mcu_voltage_mv & 0xFFu);
    buf[REG_MCU_VOLTAGE_H]       = (uint8_t)((auth->mcu_voltage_mv >> 8) & 0xFFu);
    buf[REG_POGOPIN_VOLTAGE_L]   = (uint8_t)(auth->pogopin_voltage_mv & 0xFFu);
    buf[REG_POGOPIN_VOLTAGE_H]   = (uint8_t)((auth->pogopin_voltage_mv >> 8) & 0xFFu);
    buf[REG_BATTERY_VOLTAGE_L]   = (uint8_t)(auth->battery_voltage_mv & 0xFFu);
    buf[REG_BATTERY_VOLTAGE_H]   = (uint8_t)((auth->battery_voltage_mv >> 8) & 0xFFu);
    buf[REG_USBC_VOLTAGE_L]      = (uint8_t)(auth->usbc_voltage_mv & 0xFFu);
    buf[REG_USBC_VOLTAGE_H]      = (uint8_t)((auth->usbc_voltage_mv >> 8) & 0xFFu);
    buf[REG_MICROUSB_VOLTAGE_L]  = (uint8_t)(auth->microusb_voltage_mv & 0xFFu);
    buf[REG_MICROUSB_VOLTAGE_H]  = (uint8_t)((auth->microusb_voltage_mv >> 8) & 0xFFu);
    buf[REG_TEMPERATURE_L]       = (uint8_t)(auth->temperature_raw & 0xFFu);
    buf[REG_TEMPERATURE_H]       = (uint8_t)((auth->temperature_raw >> 8) & 0xFFu);
    buf[REG_FULL_VOLTAGE_L]      = (uint8_t)(auth->full_voltage_mv & 0xFFu);
    buf[REG_FULL_VOLTAGE_H]      = (uint8_t)((auth->full_voltage_mv >> 8) & 0xFFu);
    buf[REG_EMPTY_VOLTAGE_L]     = (uint8_t)(auth->empty_voltage_mv & 0xFFu);
    buf[REG_EMPTY_VOLTAGE_H]     = (uint8_t)((auth->empty_voltage_mv >> 8) & 0xFFu);
    buf[REG_PROTECT_VOLTAGE_L]   = (uint8_t)(auth->protection_voltage_mv & 0xFFu);
    buf[REG_PROTECT_VOLTAGE_H]   = (uint8_t)((auth->protection_voltage_mv >> 8) & 0xFFu);
    buf[REG_BATTERY_PERCENT_L]   = auth->battery_percent;
    buf[REG_BATTERY_PERCENT_H]   = 0x00u;
    buf[REG_SAMPLE_PERIOD_L]     = (uint8_t)(auth->sample_period_minutes & 0xFFu);
    buf[REG_SAMPLE_PERIOD_H]     = (uint8_t)((auth->sample_period_minutes >> 8) & 0xFFu);
    buf[REG_POWER_STATUS]       = GetPowerStatusRegisterValue(auth, sys);
    buf[REG_SHUTDOWN_COUNTDOWN]  = auth->shutdown_countdown_sec;
    buf[REG_AUTO_POWER_ON]       = auth->auto_power_on;
    buf[REG_RESTART_COUNTDOWN]   = auth->restart_countdown_sec;
    buf[REG_FACTORY_RESET]       = 0x00u; /* RO read; write path handled separately */
    u32 = auth->cumulative_runtime_sec;
    buf[REG_RUNTIME_ALL_0]       = (uint8_t)(u32 & 0xFFu);
    buf[REG_RUNTIME_ALL_1]       = (uint8_t)((u32 >> 8) & 0xFFu);
    buf[REG_RUNTIME_ALL_2]       = (uint8_t)((u32 >> 16) & 0xFFu);
    buf[REG_RUNTIME_ALL_3]       = (uint8_t)((u32 >> 24) & 0xFFu);
    u32 = auth->charging_time_sec;
    buf[REG_RUNTIME_CHARGING_0]  = (uint8_t)(u32 & 0xFFu);
    buf[REG_RUNTIME_CHARGING_1]  = (uint8_t)((u32 >> 8) & 0xFFu);
    buf[REG_RUNTIME_CHARGING_2]  = (uint8_t)((u32 >> 16) & 0xFFu);
    buf[REG_RUNTIME_CHARGING_3]  = (uint8_t)((u32 >> 24) & 0xFFu);
    u32 = auth->current_runtime_sec;
    buf[REG_RUNTIME_CURRENT_0]   = (uint8_t)(u32 & 0xFFu);
    buf[REG_RUNTIME_CURRENT_1]   = (uint8_t)((u32 >> 8) & 0xFFu);
    buf[REG_RUNTIME_CURRENT_2]   = (uint8_t)((u32 >> 16) & 0xFFu);
    buf[REG_RUNTIME_CURRENT_3]   = (uint8_t)((u32 >> 24) & 0xFFu);
    buf[REG_VERSION_L]           = (uint8_t)(auth->version & 0xFFu);
    buf[REG_VERSION_H]           = (uint8_t)((auth->version >> 8) & 0xFFu);
    buf[REG_BATTERY_SELF_PROG]   = auth->battery_params_self_programmed;
    buf[REG_LOW_BATTERY_PERCENT] = auth->low_battery_percent;
    u16 = (auth->load_on_delay_remaining_sec != 0u) ? auth->load_on_delay_remaining_sec : auth->load_on_delay_config_sec;
    buf[REG_LOAD_ON_DELAY_L]     = (uint8_t)(u16 & 0xFFu);
    buf[REG_LOAD_ON_DELAY_H]     = (uint8_t)((u16 >> 8) & 0xFFu);
    /* INA219 current registers (cached values + validity). */
    buf[REG_OUTPUT_CURRENT_L]    = (uint8_t)(auth->output_current_mA & 0xFFu);
    buf[REG_OUTPUT_CURRENT_H]    = (uint8_t)(((uint16_t)auth->output_current_mA >> 8) & 0xFFu);
    buf[REG_BATTERY_CURRENT_L]   = (uint8_t)(auth->battery_current_mA & 0xFFu);
    buf[REG_BATTERY_CURRENT_H]   = (uint8_t)(((uint16_t)auth->battery_current_mA >> 8) & 0xFFu);
    buf[REG_CURRENT_VALID_FLAGS] = (uint8_t)((auth->output_current_valid ? 0x01u : 0x00u) |
                                             (auth->battery_current_valid ? 0x02u : 0x00u));
    for (i = REG_RESERVED_START; i <= REG_RESERVED_END; i++)
        buf[i] = 0x00u;
    buf[REG_SERIAL_START + 0]  = (uint8_t)(LL_GetUID_Word0() & 0xFFu);
    buf[REG_SERIAL_START + 1]  = (uint8_t)((LL_GetUID_Word0() >> 8) & 0xFFu);
    buf[REG_SERIAL_START + 2]  = (uint8_t)((LL_GetUID_Word0() >> 16) & 0xFFu);
    buf[REG_SERIAL_START + 3]  = (uint8_t)((LL_GetUID_Word0() >> 24) & 0xFFu);
    buf[REG_SERIAL_START + 4]  = (uint8_t)(LL_GetUID_Word1() & 0xFFu);
    buf[REG_SERIAL_START + 5]  = (uint8_t)((LL_GetUID_Word1() >> 8) & 0xFFu);
    buf[REG_SERIAL_START + 6]  = (uint8_t)((LL_GetUID_Word1() >> 16) & 0xFFu);
    buf[REG_SERIAL_START + 7]  = (uint8_t)((LL_GetUID_Word1() >> 24) & 0xFFu);
    buf[REG_SERIAL_START + 8]  = (uint8_t)(LL_GetUID_Word2() & 0xFFu);
    buf[REG_SERIAL_START + 9]  = (uint8_t)((LL_GetUID_Word2() >> 8) & 0xFFu);
    buf[REG_SERIAL_START + 10] = (uint8_t)((LL_GetUID_Word2() >> 16) & 0xFFu);
    buf[REG_SERIAL_START + 11] = (uint8_t)((LL_GetUID_Word2() >> 24) & 0xFFu);
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
            buf[REG_FACTORY_TEST_START + 1] = (uint8_t)(charger_snapshot.charger_physically_present != 0u);
            buf[REG_FACTORY_TEST_START + 2] = (uint8_t)(window_snapshot.window_active != 0u);
            buf[REG_FACTORY_TEST_START + 3] = (uint8_t)(window_snapshot.window_due != 0u);
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
        case 0x08u: {
            /* CSR byte packing: buf[+1] = RCC_CSR bits 31:25 (reset flags). buf[+2]/buf[+3] = CSR high 16 bits as [23:16], [31:24] (not LE order). */
            uint32_t csr = boot_reset_csr;
            buf[REG_FACTORY_TEST_START + 1] = (uint8_t)((csr >> RCC_CSR_RESET_FLAGS_SHIFT) & RCC_CSR_RESET_FLAGS_MASK);
            buf[REG_FACTORY_TEST_START + 2] = (uint8_t)(csr >> 16);
            buf[REG_FACTORY_TEST_START + 3] = (uint8_t)(csr >> 24);
            break;
        }
        case 0x09u:
            /* Last reset cause persisted in flash (previous run). */
            buf[REG_FACTORY_TEST_START + 1] = last_persisted_reset_cause;
            buf[REG_FACTORY_TEST_START + 2] = last_persisted_reset_seq;
            buf[REG_FACTORY_TEST_START + 3] = 0x00u;
            break;
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
    state.snapshot_tick = sched_flags.tick_counter;  /* canonical TIM1 tick */
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
            /* Selector only; ignore any extra data bytes. */
            u8 = i2c_pending_write.data[0];
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
            if (Validate_LoadOnDelay(u16) && state.load_on_delay_config_sec != u16) {
                state.load_on_delay_config_sec = u16;
                changed = 1;
                persist_changed = 1;
            }
        }
        break;
    default:
        break;
    }

    if (changed)
        Snapshot_UpdateDerived();
    if (persist_changed)
        RequestFlashSave(0, 1);
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
    state.version = 23;
    state.snapshot_tick = 0;
    state.last_true_vbat_sample_tick = 0;
    state.last_true_vbat_mv = 0;

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
    window_mgr.window_due = 0;
    window_mgr.window_active = 0;
    window_mgr.last_window_end_ticks = 0;
    window_mgr.window_start_ticks = 0;
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

    /* Capture reset cause before clearing (order matters). Export via factory test 0x08/0x09. */
    {
        uint32_t csr = RCC->CSR;
        boot_reset_csr = csr;
        RCC->CSR |= RCC_CSR_RMVF;
    }

    /* IWDG: LSI 26–56 kHz; PR=256, RLR=1748 → ~8 s at 56 kHz, ~17 s at 26 kHz. Main loop must finish within timeout.
     * We do not block on LSI ready: if LSI never starts, the watchdog simply won’t tick, but firmware still runs.
     * Option bytes: verify SW-start mode and IWDG not frozen in documentation/manufacturing; do not program to disable.
     * Starting here puts init under the watchdog; if init grows, consider moving start to just before while(1). */
    LL_RCC_LSI_Enable();
    IWDG->KR = 0x5555u;   /* Key: unlock PR and RLR for write */
    IWDG->PR = IWDG_PR_256;
    IWDG->RLR = IWDG_RLR_VAL;
    {
        uint32_t sr_wait = 0xFFFFu;
        while ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0u && sr_wait != 0u)   /* Wait for PR/RLR latch */
            sr_wait--;
        /* If SR doesn't clear, start anyway; timeout may be shorter than expected. */
    }
    IWDG->KR = 0xCCCCu;   /* Key: start watchdog */

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
    LL_mDelay(100);

    while (1)
    {
        /* Canonical scheduler - run tasks from TIM1 flags, then clear flags */
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
                LL_ADC_REG_StartConversion(ADC1);
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
                state.charging_time_sec++;
            PowerStateMachine_OnTick1s();
            BatteryPlateau_Tick1s();
            if (flash_dirty &&
                (state.cumulative_runtime_sec - flash_last_write_sec) >= FLASH_DIRTY_MAX_INTERVAL_SEC)
            {
                RequestFlashSave(0, 0);
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
                int16_t shunt_raw = 0;
                ina_probe_requested = 0;
                if (I2C1_ReadIna219Shunt(ina_probe_is_output, &shunt_raw))
                {
                    if (ina_probe_is_output)
                    {
                        state.output_current_mA = shunt_raw;
                        state.output_current_age_10ms = 0u;
                        state.output_current_valid = 1u;
                    }
                    else
                    {
                        state.battery_current_mA = shunt_raw;
                        state.battery_current_age_10ms = 0u;
                        state.battery_current_valid = 1u;
                    }
                    Snapshot_UpdateDerived();
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
                Snapshot_UpdateDerived();
                if (Flash_Save(flash_save_bypass))
                {
                    flash_save_requested = 0;
                    flash_save_bypass = 0;
                    flash_next_retry_sec = 0;
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
            if (MustRefreshVDD)
            {
                state.mcu_voltage_mv = (__LL_ADC_CALC_VREFANALOG_VOLTAGE(aADCxConvertedData[5], LL_ADC_RESOLUTION_12B) + state.mcu_voltage_mv) / 2;
                MustRefreshVDD = 0;
            }
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
            /* True-VBAT sample tick when charger not influencing (for percent and staleness gating). */
            if (!Charger_IsInfluencingVBAT(&sys_state))
            {
                state.last_true_vbat_sample_tick = sched_flags.tick_counter;
                state.last_true_vbat_mv = state.battery_voltage_mv;
            }
            /* Protection: store last ADC reading; counting is done in Protection_Step with adc_sample_seq gating. */
            prot_state.last_adc_battery_mv = state.battery_voltage_mv;
            Snapshot_UpdateDerived();
            adc_sample_seq++;
            adc_ready = 0;
        }

        if (state.auto_power_on)
            CheckPowerOnConditions();

        /* Button actions (short/long press) */
        Button_DispatchActions();

        /* IWDG: single refresh per loop, after all critical work (scheduler, I2C, INA, flash, protection, GPIO). */
        IWDG->KR = 0xAAAAu;   /* Key: reload watchdog counter */
    }
}

/**
 * Check power-on conditions with explicit state transitions and staleness gating.
 * RPI_OFF -> LOAD_ON_DELAY when conditions met (and true-VBAT
 * fresh if charger present). LOAD_ON_DELAY -> RPI_OFF when conditions no longer met.
 */
void CheckPowerOnConditions(void)
{
    if (!state.auto_power_on)
        return;

    uint32_t now_ticks = sched_flags.tick_counter;
    uint8_t battery_ok = (state.battery_percent > state.low_battery_percent);
    uint8_t battery_above_protection = (state.battery_voltage_mv > (uint16_t)(state.protection_voltage_mv + PROTECTION_HYSTERESIS_MV));
    /* Charger physically connected (PRESENT or FORCED_OFF_WINDOW); distinct from Charger_IsInfluencingVBAT */
    uint8_t charger_present = (sys_state.charger_state == CHARGER_STATE_PRESENT ||
                              sys_state.charger_state == CHARGER_STATE_FORCED_OFF_WINDOW);
    uint8_t true_vbat_fresh = IsTrueVbatSampleFresh(now_ticks, state.last_true_vbat_sample_tick);

    if (sys_state.power_state == POWER_STATE_LOAD_ON_DELAY)
    {
        /* Cancel if any required condition fails (plan: battery % must be > low for power decisions) */
        if (!battery_ok)
        {
            sys_state.power_state = POWER_STATE_RPI_OFF;
            sys_state.power_state_entry_ticks = now_ticks;
            state.load_on_delay_remaining_sec = 0;
        }
        return;
    }

    if (sys_state.power_state == POWER_STATE_PROTECTION_LATCHED)
    {
        /* Staleness: if charger present, require fresh true-VBAT for percent-based decision */
        if (charger_present && battery_ok && battery_above_protection && true_vbat_fresh)
        {
            sys_state.power_state = POWER_STATE_LOAD_ON_DELAY;
            sys_state.power_state_entry_ticks = now_ticks;
            state.load_on_delay_remaining_sec = state.load_on_delay_config_sec;
            sys_state.pending_power_cut = 0;
            sys_state.pending_power_cut_start_ticks = 0;
            prot_state.below_threshold_count = 0;
        }
        else
        {
            sys_state.power_state = POWER_STATE_RPI_OFF;
            sys_state.power_state_entry_ticks = now_ticks;
            sys_state.pending_power_cut = 0;
            sys_state.pending_power_cut_start_ticks = 0;
            prot_state.below_threshold_count = 0;
        }
        return;
    }

    if (sys_state.power_state != POWER_STATE_RPI_OFF)
        return;

    /* Staleness gate: if charger present, require fresh true-VBAT before auto power-on */
    if (charger_present && !true_vbat_fresh)
    {
        /* Plan: force measurement window ASAP so we get fresh true-VBAT */
        if (!window_mgr.window_active)
            window_mgr.window_due = 1;
        return;
    }

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
    state.last_true_vbat_sample_tick = 0;
    state.last_true_vbat_mv = 0;
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
    window_mgr.window_due = 0;
    window_mgr.window_active = 0;
    window_mgr.last_window_end_ticks = 0;
    window_mgr.window_start_ticks = 0;
    charger_physical.charger_physically_present = 0;
    charger_physical.charger_stability_count = 0;
    charger_physical.charger_last_seen_seq = 0;
    prot_state.below_threshold_count = 0;
    prot_state.protection_active = 0;
    prot_state.last_adc_battery_mv = 0;
    prot_state.last_seen_seq = 0;
    restart_phase = 0;
    restart_remaining_ticks = 0;
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
    rec->last_reset_cause = (uint8_t)((boot_reset_csr >> RCC_CSR_RESET_FLAGS_SHIFT) & RCC_CSR_RESET_FLAGS_MASK);
    rec->last_reset_seq = (uint8_t)(flash_sequence & 0xFFu);  /* Save-time sequence context (diagnostic only). */
    rec->cumulative_runtime_sec = state.cumulative_runtime_sec;
    rec->charging_time_sec = state.charging_time_sec;

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

    while (i < len)
    {
        uint8_t low = data[i];
        uint8_t high = 0xFFu;
        if ((i + 1u) < len)
            high = data[i + 1u];
        halfword = (uint16_t)low | ((uint16_t)high << 8);

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
                return 0;
            }
        }
        CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
        if (FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR))
        {
            SET_BIT(FLASH->CR, FLASH_CR_LOCK);
            return 0;
        }

        addr += 2u;
        i += 2u;
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
    size_t storage_size = (size_t)(FlashStorageEndPtr() - FlashStorageStartPtr());
    if (storage_size < sizeof(rec))
    {
        FactoryReset();
        RequestFlashSave(0, 1);
        flash_sequence = 0;
        return;
    }
    memcpy(&rec, (const void *)FlashStorageStartPtr(), sizeof(rec));
    flash_record_valid_boot = Flash_RecordIsValid(&rec);
    flash_auto_power_on_loaded = 0u;

    if (flash_record_valid_boot)
    {
        Flash_ApplyRecordToState(&rec);
        last_persisted_reset_cause = rec.last_reset_cause;
        last_persisted_reset_seq = rec.last_reset_seq;
        flash_auto_power_on_loaded = (rec.auto_power_on != 0u) ? 1u : 0u;
        UpdateDerivedState();
        flash_sequence = rec.sequence_number;
        flash_dirty = 0;
        flash_last_write_sec = state.cumulative_runtime_sec;
    }
    else
    {
        /* 0 = unknown / no prior boot record (invalid or missing flash). */
        last_persisted_reset_cause = 0u;
        last_persisted_reset_seq = 0u;
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
    uint32_t primask;
    uint8_t ok;
    size_t storage_size = (size_t)(FlashStorageEndPtr() - FlashStorageStartPtr());

    if (!flash_dirty && !bypass)
        return 1u;
    if (!bypass && !Flash_CanWrite())
        return 0u;
    if (storage_size < sizeof(rec))
        return 0u;

    flash_save_attempted = 1u;
    flash_last_save_success = 0u;

    Flash_FillRecordFromState(&rec, next_seq);

    primask = __get_PRIMASK();
    __disable_irq();
    ok = Flash_ErasePage(target_addr);
    if (ok)
        ok = Flash_ProgramBuffer(target_addr, (const uint8_t *)&rec, sizeof(rec));
    if (!primask)
        __enable_irq();
    if (!ok)
        return 0u;

    memcpy(&verify, (const void *)FlashStorageStartPtr(), sizeof(verify));
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
    LL_Init1msTick(48000000);
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
 */
static void EmptyLearn_CommitTrackedMin(uint8_t request_flash)
{
    if (min_vbat_while_load_on_mv == 0xFFFFu)
        return;
    uint16_t new_empty = ClampU16(min_vbat_while_load_on_mv, state.protection_voltage_mv, EmptyVoltageMaxMv());
    uint16_t old_empty = state.empty_voltage_mv;
    state.empty_voltage_mv = new_empty;
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
        }
        else
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
                if (min_vbat_while_load_on_mv == 0xFFFFu ||
                    state.battery_voltage_mv < min_vbat_while_load_on_mv)
                    min_vbat_while_load_on_mv = state.battery_voltage_mv;
            }
            if (load_off && min_vbat_while_load_on_mv != 0xFFFFu)
                EmptyLearn_CommitTrackedMin(1);
        }
    }
    state.empty_voltage_mv = ClampU16(state.empty_voltage_mv, state.protection_voltage_mv, EmptyVoltageMaxMv());
}

void UpdateBatteryPercentage(void)
{
    int32_t range = (int32_t)state.full_voltage_mv - (int32_t)state.empty_voltage_mv;
    if (range < MIN_VOLTAGE_DELTA_MV)
        return;
    uint32_t now_ticks = sched_flags.tick_counter;
    uint8_t true_vbat_fresh = IsTrueVbatSampleFresh(now_ticks, state.last_true_vbat_sample_tick);
    uint8_t charging = Charger_IsInfluencingVBAT(&sys_state);
    if (charging && !true_vbat_fresh)
        return;
    int32_t voltage = charging ? (int32_t)state.last_true_vbat_mv
                               : (int32_t)state.battery_voltage_mv;
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

static uint8_t I2C1_GuardWindowReady(void)
{
    uint16_t now_us = (uint16_t)LL_TIM_GetCounter(TIM3);
    uint16_t last_stop_us = I2C1_GetLastStopUs();
    uint16_t last_addr_us = I2C1_GetLastAddrUs();
    uint8_t txn_active = I2C1_GetSlaveTxnActive();

    if (txn_active)
        return 0u;

    if ((uint16_t)(now_us - last_stop_us) > 50000u)
        return 0u;

    if ((uint16_t)(now_us - last_addr_us) > 50000u)
        return 0u;

    if ((uint16_t)(now_us - last_stop_us) < 500u)
        return 0u;

    if (!LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_9) ||
        !LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_10))
        return 0u;

    if (!I2C1_BusIdleStable500us())
        return 0u;

    return 1u;
}

static void BatteryPlateau_UpdateBatteryFullClear(uint32_t now_ticks)
{
    if (sys_state.charger_state != CHARGER_STATE_PRESENT)
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
    uint8_t charger_present = (sys_state.charger_state == CHARGER_STATE_PRESENT) ? 1u : 0u;

    if (state.battery_params_self_programmed != 0u)
    {
        battery_full = 0;
        battery_full_clear_start_ticks = 0;
        BatteryPlateau_ResetSession();
        plateau_last_charger_state = sys_state.charger_state;
        plateau_last_power_state = sys_state.power_state;
        return;
    }

    if (!charger_present)
    {
        battery_full = 0;
        battery_full_clear_start_ticks = 0;
        BatteryPlateau_ResetSession();
        plateau_last_charger_state = sys_state.charger_state;
        plateau_last_power_state = sys_state.power_state;
        return;
    }

    if (plateau_last_charger_state != CHARGER_STATE_PRESENT && charger_present)
        BatteryPlateau_ResetSession();
    plateau_last_charger_state = sys_state.charger_state;

    /* Reset plateau window on power-state transition so the 30-min window is continuous under RPI_ON. */
    if (plateau_last_power_state != POWER_STATE_RPI_ON && sys_state.power_state == POWER_STATE_RPI_ON)
        BatteryPlateau_ResetWindow();
    plateau_last_power_state = sys_state.power_state;

    BatteryPlateau_UpdateBatteryFullClear(now_ticks);

    /* Taper gate: hold time that battery (charging) current <= I_TAPER_MA with valid; else reset */
    if (state.battery_current_valid &&
        state.battery_current_mA >= 0 &&
        state.battery_current_mA <= (int16_t)I_TAPER_MA)
        taper_hold_ticks += (uint32_t)TICKS_PER_1S;
    else
        taper_hold_ticks = 0;

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
    if (plateau_suppressed)
        return;
    if (filtered_vbat_mv < VBAT_FULL_MIN_MV)
        return;
    if (sys_state.power_state != POWER_STATE_RPI_ON || sys_state.power_state_entry_ticks == 0u)
        return;
    if ((now_ticks - sys_state.power_state_entry_ticks) < (uint32_t)(PLATEAU_POWER_STABLE_SEC * TICKS_PER_1S))
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

    /* Optional taper gate: if current is valid, require taper hold; else plateau-only (no stale current) */
    if (state.battery_current_valid &&
        taper_hold_ticks < (uint32_t)(TAPER_HOLD_SEC * TICKS_PER_1S))
        return;

    if (state.full_voltage_mv < LEARNED_FULL_MIN_MV || state.full_voltage_mv > LEARNED_FULL_MAX_MV)
        state.full_voltage_mv = VBAT_FULL_MIN_MV;

    uint16_t plateau_mean = (uint16_t)(sum / plateau_sample_count);
    uint16_t learned_full = state.full_voltage_mv;
    uint16_t old_full = state.full_voltage_mv;

    if (AbsDiffU16(plateau_mean, learned_full) >= PLATEAU_MIN_CHANGE_MV)
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
    plateau_suppressed = 1;
}

/* DMA only sets adc_ready; main loop processes ADC and updates state */
void DMA1_CH1_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TC1(DMA1))
    {
        LL_DMA_ClearFlag_TC1(DMA1);
        adc_ready = 1;
    }
    if (LL_DMA_IsActiveFlag_TE1(DMA1) == 1)
    {
        LL_DMA_ClearFlag_TE1(DMA1);
    }
}

/* TIM1 ISR is flag-only. All timing logic runs in main loop. */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM1) == 1)
    {
        LL_TIM_ClearFlag_UPDATE(TIM1);
        sched_flags.tick_counter++;
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
}

/* Scheduler uses explicit state machine (power, charger, protection). */
#define SAMPLE_PERIOD_TICKS_PER_MIN  ((uint32_t)(60u * TICKS_PER_1S))

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
    /* IP_EN HIGH only when PRESENT; ABSENT and FORCED_OFF_WINDOW both drive IP_EN low (intentional) */
    if (sys_state.charger_state == CHARGER_STATE_PRESENT)
        LL_GPIO_SetOutputPin(GPIOA, IP_EN);
    else
        LL_GPIO_ResetOutputPin(GPIOA, IP_EN);
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
            EmptyLearn_CommitTrackedMin(0);
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

static void ChargerStateMachine_Step(void)
{
    uint32_t tick = sched_flags.tick_counter;
    uint16_t charger_mv = UpsChargerVoltageMv(&state);

    if (sys_state.charger_state == CHARGER_STATE_FORCED_OFF_WINDOW)
    {
        uint32_t elapsed = tick - sys_state.charger_state_entry_ticks;
        if (elapsed >= MEASUREMENT_WINDOW_TICKS)
        {
            window_mgr.window_active = 0;
            window_mgr.last_window_end_ticks = tick;
            /* Unplug mid-window: decide PRESENT vs ABSENT from current voltage at window end.
             * Hysteresis: ABSENT uses > ON_MV for detection; here >= OFF_MV = still present (no 1mV gap). */
            if (charger_mv >= CHARGER_PRESENT_OFF_MV)
                sys_state.charger_state = CHARGER_STATE_PRESENT;
            else
                sys_state.charger_state = CHARGER_STATE_ABSENT;
            sys_state.charger_state_entry_ticks = tick;
            /* Charger/window are runtime-only (B9); only flush if something else is dirty. */
            MustRefreshVDD = 1;
            if (flash_dirty)
                RequestFlashSave(0, 0);
        }
    }
    else
    {
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
                    }
                }
                else
                    charger_physical.charger_stability_count = 0;
            }
        }

        if (sys_state.charger_state == CHARGER_STATE_PRESENT && !window_mgr.window_active)
        {
            uint32_t elapsed_since = tick - window_mgr.last_window_end_ticks;
            uint32_t period_ticks = SAMPLE_PERIOD_TICKS_PER_MIN * (uint32_t)state.sample_period_minutes;
            if (elapsed_since >= period_ticks)
                window_mgr.window_due = 1;
        }

        if (sys_state.charger_state == CHARGER_STATE_PRESENT && window_mgr.window_due && !window_mgr.window_active)
        {
            sys_state.charger_state = CHARGER_STATE_FORCED_OFF_WINDOW;
            sys_state.charger_state_entry_ticks = tick;
            window_mgr.window_due = 0;
            window_mgr.window_active = 1;
            window_mgr.window_start_ticks = tick;
            charger_physical.charger_stability_count = 0;  /* Defensive cleanup on window entry */
        }
    }

    if (sys_state.charger_state != CHARGER_STATE_ABSENT || sys_state.power_state == POWER_STATE_RPI_ON)
        sIPIdleTicks = 0;
    else
    {
        sIPIdleTicks++;
        if (sIPIdleTicks > 0x1000u)
        {
            sIPIdleTicks = 0;
            MustRefreshVDD = 1;
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

    /* Empty learning: commit on loss of output-current validity while RPi on (spec condition (3)). */
    if (prev_output_valid && !state.output_current_valid &&
        sys_state.power_state == POWER_STATE_RPI_ON &&
        state.battery_params_self_programmed == 0)
        EmptyLearn_CommitTrackedMin(1);

    if (state.output_current_valid != prev_output_valid ||
        state.battery_current_valid != prev_battery_valid)
        Snapshot_UpdateDerived();
}

static void Scheduler_Tick10ms(void)
{
    PowerStateMachine_Step();
    ChargerStateMachine_Step();
    /* Single place for GPIO: reflects power state and charger state after both FSM steps */
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
        if (sys_state.power_state == POWER_STATE_RPI_OFF)
        {
            uint8_t allow_power_on = 1;
            if (sys_state.power_state == POWER_STATE_PROTECTION_LATCHED)
                allow_power_on = 0;
            if (Charger_IsInfluencingVBAT(&sys_state) &&
                !IsTrueVbatSampleFresh(sched_flags.tick_counter, state.last_true_vbat_sample_tick))
            {
                allow_power_on = 0;
            }
            if (state.battery_voltage_mv <= state.protection_voltage_mv)
                allow_power_on = 0;
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

    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

    LL_TIM_SetPrescaler(TIM1, __LL_TIM_CALC_PSC(SystemCoreClock, 10000));
    /* Set the frequency to 100 Hz. */
    LL_TIM_SetAutoReload(
        TIM1, __LL_TIM_CALC_ARR(SystemCoreClock, LL_TIM_GetPrescaler(TIM1), 100));
    LL_TIM_EnableIT_UPDATE(TIM1);

    NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 0);
    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);

    LL_TIM_EnableCounter(TIM1);

    LL_TIM_GenerateEvent_UPDATE(TIM1);
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
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
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
