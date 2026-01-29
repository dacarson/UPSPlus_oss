#include "pindef.h"

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

#include "I2C_Slave.h"
#include "ups_state.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_DMA_Init(void);

void StorRegValue(void);
uint8_t GetRegValue(void);  /* Returns 1 if loaded from flash, 0 if factory reset */
uint32_t GetUptimeSeconds(void);
void UpdateBatteryPercentage(void);
void UpdateBatteryMinMax(void);
void FactoryReset(void);
void CheckPowerOnConditions(void);

#define VBAT_PROTECT_VOLTAGE 2800
#define VBAT_LOW_PERCENT 10
#define LOAD_ON_DELAY_TIME 60
#define VBAT_MAX_VOLTAGE 3000
#define VBAT_MIN_VOLTAGE 4200
#define IP_REFRESH_TIME 2

/* Everything below needs to be mapped to registers */

#define ADC_CONVERTED_DATA_BUFFER_SIZE 6

/* Phase 2: Raw ADC buffer; DMA fills, main loop processes into state */
__IO uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];

/* Phase 2: Timing/button helpers (TIM1/SysTick/EXTI). Authoritative state is state/sys_state only;
 * no shadow globals (uXXXVolt, counters, mode flags) remain as authoritative. */
__IO uint8_t sKeyFlag = 0;
__IO uint8_t sKeyClick = 0;        /* 1 = short press, 2 = long press */
__IO uint16_t sIPIdleTicks = 0;
__IO uint8_t sHoldTime = 0;       /* 10 ms units */
__IO uint8_t MustRefreshVDD = 1;
/* Phase 3: Countdowns decremented in main loop on tick_1s (removed from ISR) */
/* Free-running ms since boot (uint32_t, overflows ~49 days). Not "ms within current second";
 * document/use consistently. Phase 3: no longer reset for tick_1s. */
__IO uint32_t uUptimeMsCounter = 0;
__IO uint8_t OTAShot = 0;

#define BL_START_ADDRESS 0x8000000

typedef void (*pFunction)(void);
__IO pFunction JumpToAplication;

/*===========================================================================*/
/* Phase 2: Authoritative state, system state, snapshot (single source)     */
/*===========================================================================*/
static authoritative_state_t state;
static system_state_t sys_state;
static snapshot_buffer_t snapshot;

/* Phase 2.2: Double-buffered register image for coherent I2C multi-byte reads. Snapshot_Update
 * writes into reg_image[inactive], then flips active_reg_image. ISR latches on ADDR+READ and
 * transmits from reg_image[latched]. aReceiveBuffer is used for flash load only, never for I2C reads. */
uint8_t reg_image[2][256];
volatile uint8_t active_reg_image = 0;

/* ADC flags: DMA sets adc_ready; main loop processes and increments adc_sample_seq */
volatile uint8_t adc_ready = 0;
volatile uint32_t adc_sample_seq = 0;

/* Phase 3: Canonical scheduler - TIM1 sets flags only; main loop runs all tasks.
 * Flag race: small chance of losing an event if ISR sets a flag between main's check and clear.
 * For coarse 100ms/500ms tasks usually acceptable; for never-miss semantics consider
 * counters (increment in ISR) or copy+clear in one short critical section in main. */
static scheduler_flags_t sched_flags;
static scheduler_counters_t sched_counters;
/* Main loop derives tick_1s from tick_100ms (10 pulses = 1s) */
static uint8_t tick_100ms_count = 0;

/* Flash save requested (e.g. protection shutdown); main loop does Snapshot_Update then StorRegValue */
static volatile uint8_t flash_save_requested = 0;
/* OTA requested via I2C write 127 to register 0x32 (50) */
static volatile uint8_t ota_triggered = 0;

static void InitAuthoritativeStateFromDefaults(void);
static void ProcessI2CPendingWrite(void);
static void Scheduler_Tick10ms(void);

/*===========================================================================*/
/* Phase 2: Validation functions (register bounds, RO enforcement in apply)  */
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
/* Phase 2: Register 0x17 derivation and register image fill                 */
/*===========================================================================*/
uint8_t GetPowerStatusRegisterValue(const authoritative_state_t *auth_state, const system_state_t *state)
{
    (void)auth_state;
    if (state->learning_mode == LEARNING_ACTIVE)
        return 0x02u; /* Learning/Calibration enabled */
    if (state->power_state == POWER_STATE_RPI_ON)
        return 0x01u; /* Power to RPi */
    return 0x00u;     /* No power to RPi */
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
    uint8_t i;

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

void Snapshot_Update(void)
{
    state.snapshot_tick = sched_flags.tick_counter;  /* Phase 3: canonical TIM1 tick */
    uint8_t inactive = (uint8_t)(1u - active_reg_image);
    StateToRegisterBuffer(&state, &sys_state, reg_image[inactive]);  /* never aReceiveBuffer */
    active_reg_image = inactive;
    snapshot.buffer[inactive] = state;
    snapshot.active_buffer = inactive;
    snapshot.snapshot_version++;
}

/* RO register addresses: reject writes (ignore, don't apply) */
static uint8_t IsReadOnlyRegister(uint8_t reg)
{
    if (reg <= REG_TEMPERATURE_H) return 1; /* 0x01-0x0C */
    if (reg >= REG_BATTERY_PERCENT_L && reg <= REG_BATTERY_PERCENT_H) return 1; /* 0x13-0x14 */
    if (reg == REG_POWER_STATUS) return 1; /* 0x17 */
    if (reg >= REG_RUNTIME_ALL_0 && reg <= REG_RUNTIME_CURRENT_3) return 1; /* 0x1C-0x27 */
    if (reg >= REG_VERSION_L && reg <= REG_VERSION_H) return 1; /* 0x28-0x29 */
    if (reg >= REG_SERIAL_START && reg <= REG_SERIAL_END) return 1; /* 0xF0-0xFB */
    return 0;
}

static void ProcessI2CPendingWrite(void)
{
    uint8_t reg = i2c_pending_write.reg_addr;
    uint8_t len = i2c_pending_write.length;
    uint16_t u16;
    uint8_t u8;

    if (!i2c_pending_write.pending) return;

    /* OTA trigger: write 127 to reg 0x32 (50) - accept before reserved discard */
    if (reg == 50 && len >= 1 && i2c_pending_write.data[0] == 127)
        ota_triggered = 1;

    /* Reserved: ACK but discard */
    if (IS_RESERVED_REG(reg) || IS_FACTORY_TEST_REG(reg)) {
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
            if (Validate_FullVoltage(u16)) state.full_voltage_mv = u16;
        }
        break;
    case REG_EMPTY_VOLTAGE_L:
        if (len >= 2) {
            u16 = (uint16_t)i2c_pending_write.data[0] | ((uint16_t)i2c_pending_write.data[1] << 8);
            if (Validate_EmptyVoltage(u16) && u16 >= state.protection_voltage_mv)
                state.empty_voltage_mv = u16;
        }
        break;
    case REG_PROTECT_VOLTAGE_L:
        if (len >= 2) {
            u16 = (uint16_t)i2c_pending_write.data[0] | ((uint16_t)i2c_pending_write.data[1] << 8);
            if (Validate_ProtectionVoltage(u16)) {
                state.protection_voltage_mv = u16;
                if (state.empty_voltage_mv < u16) state.empty_voltage_mv = u16;
            }
        }
        break;
    case REG_SAMPLE_PERIOD_L:
        if (len >= 2) {
            u16 = (uint16_t)i2c_pending_write.data[0] | ((uint16_t)i2c_pending_write.data[1] << 8);
            if (Validate_SamplePeriod(u16)) state.sample_period_minutes = u16;
        }
        break;
    case REG_SHUTDOWN_COUNTDOWN:
        if (len >= 1) {
            u8 = i2c_pending_write.data[0];
            if (Validate_Countdown(u8)) state.shutdown_countdown_sec = u8;
        }
        break;
    case REG_AUTO_POWER_ON:
        if (len >= 1) state.auto_power_on = (i2c_pending_write.data[0] != 0) ? 1u : 0u;
        break;
    case REG_RESTART_COUNTDOWN:
        if (len >= 1) {
            u8 = i2c_pending_write.data[0];
            if (Validate_Countdown(u8)) state.restart_countdown_sec = u8;
        }
        break;
    case REG_FACTORY_RESET:
        if (len >= 1 && i2c_pending_write.data[0] == 1) sys_state.factory_reset_requested = 1;
        break;
    case REG_BATTERY_SELF_PROG:
        if (len >= 1) {
            state.battery_params_self_programmed = (i2c_pending_write.data[0] != 0) ? 1u : 0u;
            sys_state.learning_mode = (state.battery_params_self_programmed == 0) ? LEARNING_ACTIVE : LEARNING_INACTIVE;
        }
        break;
    case REG_LOW_BATTERY_PERCENT:
        if (len >= 1) {
            u8 = i2c_pending_write.data[0];
            if (Validate_LowBatteryPercent(u8)) state.low_battery_percent = u8;
        }
        break;
    case REG_LOAD_ON_DELAY_L:
        if (len >= 2) {
            u16 = (uint16_t)i2c_pending_write.data[0] | ((uint16_t)i2c_pending_write.data[1] << 8);
            if (Validate_LoadOnDelay(u16)) state.load_on_delay_config_sec = u16;
        }
        break;
    default:
        break;
    }

    Snapshot_Update();
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
    state.battery_params_self_programmed = 1;
    state.low_battery_percent = DEFAULT_LOW_BATTERY_PERCENT;
    state.load_on_delay_config_sec = DEFAULT_LOAD_ON_DELAY_SEC;
    state.load_on_delay_remaining_sec = 0;
    state.cumulative_runtime_sec = 0;
    state.charging_time_sec = 0;
    state.current_runtime_sec = 0;
    state.version = 20;
    state.snapshot_tick = 0;
    state.last_true_vbat_sample_tick = 0;

    sys_state.power_state = POWER_STATE_RPI_OFF;
    sys_state.charger_state = CHARGER_STATE_ABSENT;
    sys_state.learning_mode = (state.battery_params_self_programmed == 0) ? LEARNING_ACTIVE : LEARNING_INACTIVE;
    sys_state.power_state_entry_ticks = 0;
    sys_state.charger_state_entry_ticks = 0;
    sys_state.factory_reset_requested = 0;
    sys_state.pending_power_cut = 0;
}

/* Apply persisted register buffer (from flash) to state; call after GetRegValue() when flash was valid */
static void InitAuthoritativeStateFromBuffer(void)
{
    state.full_voltage_mv       = (uint16_t)aReceiveBuffer[REG_FULL_VOLTAGE_L] | ((uint16_t)aReceiveBuffer[REG_FULL_VOLTAGE_H] << 8);
    state.empty_voltage_mv      = (uint16_t)aReceiveBuffer[REG_EMPTY_VOLTAGE_L] | ((uint16_t)aReceiveBuffer[REG_EMPTY_VOLTAGE_H] << 8);
    state.protection_voltage_mv = (uint16_t)aReceiveBuffer[REG_PROTECT_VOLTAGE_L] | ((uint16_t)aReceiveBuffer[REG_PROTECT_VOLTAGE_H] << 8);
    state.sample_period_minutes = (uint16_t)aReceiveBuffer[REG_SAMPLE_PERIOD_L] | ((uint16_t)aReceiveBuffer[REG_SAMPLE_PERIOD_H] << 8);
    state.auto_power_on         = (aReceiveBuffer[REG_AUTO_POWER_ON] != 0) ? 1u : 0u;
    state.battery_params_self_programmed = (aReceiveBuffer[REG_BATTERY_SELF_PROG] != 0) ? 1u : 0u;
    state.low_battery_percent   = aReceiveBuffer[REG_LOW_BATTERY_PERCENT];
    state.load_on_delay_config_sec = (uint16_t)aReceiveBuffer[REG_LOAD_ON_DELAY_L] | ((uint16_t)aReceiveBuffer[REG_LOAD_ON_DELAY_H] << 8);
    sys_state.power_state       = (aReceiveBuffer[REG_POWER_STATUS] != 0) ? POWER_STATE_RPI_ON : POWER_STATE_RPI_OFF;
    sys_state.learning_mode     = (state.battery_params_self_programmed == 0) ? LEARNING_ACTIVE : LEARNING_INACTIVE;
    /* Clamp to valid ranges */
    if (!Validate_FullVoltage(state.full_voltage_mv)) state.full_voltage_mv = DEFAULT_VBAT_FULL_MV;
    if (!Validate_EmptyVoltage(state.empty_voltage_mv)) state.empty_voltage_mv = DEFAULT_VBAT_EMPTY_MV;
    if (!Validate_ProtectionVoltage(state.protection_voltage_mv)) state.protection_voltage_mv = DEFAULT_VBAT_PROTECT_MV;
    if (state.empty_voltage_mv < state.protection_voltage_mv) state.empty_voltage_mv = state.protection_voltage_mv;
    if (!Validate_SamplePeriod(state.sample_period_minutes)) state.sample_period_minutes = DEFAULT_SAMPLE_PERIOD_MIN;
    if (state.low_battery_percent > 100u) state.low_battery_percent = DEFAULT_LOW_BATTERY_PERCENT;
}

void NVIC_SetVectorTable(void)
{
    uint8_t i;
    uint32_t *pVecTab = (uint32_t *)(0x20000000);
    for (i = 0; i < 48; i++)
    {
        *(pVecTab++) = *(__IO uint32_t *)(0x8000800 + (i << 2));
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

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC_Init();
    MX_I2C1_Slave_Init();

    /* Phase 2: Single source of truth; load from defaults then optionally from flash */
    InitAuthoritativeStateFromDefaults();
    if (GetRegValue())
        InitAuthoritativeStateFromBuffer();
    Snapshot_Init();
    sys_state.factory_reset_requested = 0;

    LL_mDelay(100);

    //__enable_irq(); /* Enable interrupts after initialization */

    while (1)
    {
        /* Phase 3: Canonical scheduler - run tasks from TIM1 flags, then clear flags */
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
            Snapshot_Update();  /* Phase 3: 100ms periodic snapshot (ensures freshness) */
            sched_flags.tick_100ms = 0;
        }
        if (sched_flags.tick_500ms)
        {
            if (!adc_ready)
                LL_ADC_REG_StartConversion(ADC1);
            sched_flags.tick_500ms = 0;
        }
        if (sched_flags.tick_1s)
        {
            state.cumulative_runtime_sec++;
            if (sys_state.power_state == POWER_STATE_RPI_ON)
            {
                state.current_runtime_sec++;
                if (state.usbc_voltage_mv > 4000 || state.microusb_voltage_mv > 4000)
                    state.charging_time_sec++;
            }
            else
            {
                state.current_runtime_sec = 0;
            }
            /* Countdowns: check actions first, then decrement (so == 1 is never skipped) */
            if (state.shutdown_countdown_sec == 1)
            {
                sys_state.power_state = POWER_STATE_RPI_OFF;
                state.shutdown_countdown_sec = 0;
            }
            else if (state.shutdown_countdown_sec > 1)
                state.shutdown_countdown_sec--;
            /* Restart: when countdown reaches 1, power cycle RPi (MT_EN low 5s then high) and clear.
             * Risk: __disable_irq() for full 5s stalls TIM1, I2C; Phase 4: make non-blocking. */
            if (state.restart_countdown_sec == 1)
            {
                __disable_irq();
                LL_GPIO_ResetOutputPin(GPIOA, MT_EN);
                LL_mDelay(5000);
                LL_GPIO_SetOutputPin(GPIOA, MT_EN);
                sys_state.power_state = POWER_STATE_RPI_ON;
                state.restart_countdown_sec = 0;
                __enable_irq();
            }
            else if (state.restart_countdown_sec > 1)
                state.restart_countdown_sec--;
            if (state.load_on_delay_remaining_sec != 0)
            {
                state.load_on_delay_remaining_sec--;
                if (state.load_on_delay_remaining_sec == 0 && state.battery_percent > state.low_battery_percent)
                    sys_state.power_state = POWER_STATE_RPI_ON;
            }
            Snapshot_Update();
            sched_flags.tick_1s = 0;
        }

        /* Phase 2: Apply I2C writes to authoritative state (RO rejected, bounds checked) */
        ProcessI2CPendingWrite();

        if (flash_save_requested)
        {
            flash_save_requested = 0;
            Snapshot_Update();
            StorRegValue();
        }

        /* Factory reset requested via I2C 0x1B */
        if (sys_state.factory_reset_requested)
        {
            FactoryReset();
            Snapshot_Update();
            StorRegValue();
            sys_state.factory_reset_requested = 0;
        }

        /* Sample period clamp */
        if (state.sample_period_minutes < DEFAULT_SAMPLE_PERIOD_MIN)
            state.sample_period_minutes = DEFAULT_SAMPLE_PERIOD_MIN;

        /* Phase 2: ADC processing in main loop (DMA sets adc_ready only) */
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
            if (state.battery_voltage_mv < VBAT_MIN_VALID_MV)
                state.battery_voltage_mv = VBAT_MIN_VALID_MV;
            UpdateBatteryMinMax();
            UpdateBatteryPercentage();
            Snapshot_Update();
            adc_sample_seq++;
            adc_ready = 0;
        }

        if (state.auto_power_on)
            CheckPowerOnConditions();

        if (ota_triggered && OTAShot == 0)
        {
            ota_triggered = 0;
            Snapshot_Update();
            StorRegValue();
            OTAShot = 1;
            while (1)
                ;
        }

        /* Button short press: toggle RPi power */
        if (sKeyClick == 1)
        {
            sKeyClick = 0;
            sys_state.power_state = (sys_state.power_state == POWER_STATE_RPI_ON) ? POWER_STATE_RPI_OFF : POWER_STATE_RPI_ON;
            Snapshot_Update();
        }
    }
}

void CheckPowerOnConditions(void)
{
    uint8_t isCharging = (state.microusb_voltage_mv > 4000 || state.usbc_voltage_mv > 4000);
    uint8_t batteryOk = (state.battery_percent > state.low_battery_percent);

    if (state.load_on_delay_remaining_sec != 0)
    {
        if (!isCharging && !batteryOk)
            state.load_on_delay_remaining_sec = 0;
    }
    else if (isCharging && sys_state.power_state != POWER_STATE_RPI_ON && batteryOk)
    {
        state.load_on_delay_remaining_sec = state.load_on_delay_config_sec;
    }
}

void FactoryReset(void)
{
    /* Phase 2: Set authoritative state to defaults (single source of truth) */
    state.full_voltage_mv = DEFAULT_VBAT_FULL_MV;
    state.empty_voltage_mv = DEFAULT_VBAT_EMPTY_MV;
    state.protection_voltage_mv = DEFAULT_VBAT_PROTECT_MV;
    state.sample_period_minutes = DEFAULT_SAMPLE_PERIOD_MIN;
    state.low_battery_percent = DEFAULT_LOW_BATTERY_PERCENT;
    state.load_on_delay_config_sec = DEFAULT_LOAD_ON_DELAY_SEC;
    state.auto_power_on = DEFAULT_AUTO_POWER_ON;
    state.battery_params_self_programmed = 1;
    sys_state.power_state = POWER_STATE_RPI_OFF;
    sys_state.learning_mode = LEARNING_INACTIVE;
}

void StorRegValue(void)
{
    uint32_t Timeout = 0;
    uint8_t i = 0;
    uint32_t Address = 0x08003C00;
    uint8_t *img = reg_image[active_reg_image];

    if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET)
    {
        WRITE_REG(FLASH->KEYR, FLASH_KEY1);
        WRITE_REG(FLASH->KEYR, FLASH_KEY2);
    }
    SET_BIT(FLASH->CR, FLASH_CR_PER);
    WRITE_REG(FLASH->AR, Address);
    SET_BIT(FLASH->CR, FLASH_CR_STRT);
    Timeout = 48000000;
    while (((FLASH->SR) & (FLASH_SR_BSY)) == (FLASH_SR_BSY))
    {
        if (Timeout-- == 0)
            return;
    }
    CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
    for (i = 0; i < 0xFF; i++)
    {
        SET_BIT(FLASH->CR, FLASH_CR_PG);
        *(__IO uint16_t *)Address = img[i];

        /* Wait for last operation to be completed */
        Timeout = 48000000;
        while (((FLASH->SR) & (FLASH_SR_BSY)) == (FLASH_SR_BSY))
        {
            if (Timeout-- == 0)
            {
                /* Time-out occurred. Set LED2 to blinking mode */
                return;
            }
        }

        /* If the program operation is completed, disable the PG Bit */
        CLEAR_BIT(FLASH->CR, FLASH_CR_PG);

        Address = Address + 2;
    }
    SET_BIT(FLASH->CR, FLASH_CR_LOCK);
}

uint8_t GetRegValue(void)
{
    uint8_t i = 0;
    uint32_t Address = 0x08003C00;

    if (*(__IO uint8_t *)Address != 0xFF)
    {
        for (i = 0; i < 0xFF; i++)
        {
            aReceiveBuffer[i] = *(__IO uint16_t *)Address & 0xFF;
            Address = Address + 2;
        }
        return 1;  /* Loaded from flash; caller applies to state via InitAuthoritativeStateFromBuffer */
    }
    FactoryReset();
    return 0;  /* Factory reset; state already set by FactoryReset */
}

/**
 * @brief Get system uptime in seconds
 * @retval Uptime in seconds since system start
 */
uint32_t GetUptimeSeconds(void)
{
    return state.cumulative_runtime_sec;
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
    LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
}

void UpdateBatteryMinMax(void)
{
    if (state.battery_params_self_programmed == 1)
    {
        if (state.battery_voltage_mv > state.full_voltage_mv)
            state.battery_voltage_mv = state.full_voltage_mv;
        if (state.empty_voltage_mv > state.battery_voltage_mv)
            state.battery_voltage_mv = state.empty_voltage_mv;
    }
    else
    {
        if (state.battery_voltage_mv > state.full_voltage_mv)
            state.full_voltage_mv = state.battery_voltage_mv;
        else if (state.empty_voltage_mv > state.battery_voltage_mv && state.battery_voltage_mv > state.protection_voltage_mv)
            state.empty_voltage_mv = state.battery_voltage_mv;
    }
    if (state.empty_voltage_mv < state.protection_voltage_mv)
        state.empty_voltage_mv = state.protection_voltage_mv;
}

void UpdateBatteryPercentage(void)
{
    uint16_t range = state.full_voltage_mv - state.empty_voltage_mv;
    if (range < MIN_VOLTAGE_DELTA_MV)
        return;
    uint16_t percentage = (uint16_t)(((uint32_t)(state.battery_voltage_mv - state.empty_voltage_mv) * 100u) / (uint32_t)range);
    if (percentage > 100u)
        percentage = 100u;
    if (percentage > 0 && percentage < 100)
    {
        uint8_t charging = (state.microusb_voltage_mv > 4000 || state.usbc_voltage_mv > 4000);
        if (charging && percentage > state.battery_percent)
            state.battery_percent = (uint8_t)percentage;
        else if (!charging && percentage < state.battery_percent)
            state.battery_percent = (uint8_t)percentage;
        else if (state.battery_percent == 0)
            state.battery_percent = (uint8_t)percentage;
    }
}

/* Phase 2: DMA only sets adc_ready; main loop processes ADC and updates state */
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

/* Phase 3: SysTick is uptime only. 1s timing comes from TIM1 (tick_1s derived in main). */
void SysTick_Handler(void)
{
    uUptimeMsCounter++;
}

/* Phase 3: TIM1 ISR is flag-only. All timing logic runs in main loop. */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM1) == 1)
    {
        LL_TIM_ClearFlag_UPDATE(TIM1);
        sched_flags.tick_counter++;
        sched_flags.tick_10ms = 1;
        /* Modulo is acceptable at 100Hz; for higher tick rate or more periodicities consider
         * downcounters or accumulated tick counters to avoid % in ISR. */
        if ((sched_flags.tick_counter % TICKS_PER_100MS) == 0u)
            sched_flags.tick_100ms = 1;
        if ((sched_flags.tick_counter % TICKS_PER_500MS) == 0u)
            sched_flags.tick_500ms = 1;
    }
}

/* Phase 3: Scheduler tasks (run from main loop when flags set). Called when sched_flags.tick_10ms.
 * Measurement window: one active flag + sched_counters.measurement_window_ticks; keep non-nested
 * / non-retriggered when moving to explicit state machine (Phase 4+). */
#define SAMPLE_PERIOD_TICKS_PER_MIN  (6000u)  /* 10ms ticks per minute: 60 * TICKS_PER_1S */
static uint8_t measurement_window_active = 0;

static void Scheduler_Tick10ms(void)
{
    /* Protection check */
    if (state.battery_voltage_mv > 1000 && (state.microusb_voltage_mv + state.usbc_voltage_mv) < 4000)
    {
        if (state.battery_voltage_mv < state.protection_voltage_mv)
        {
            if (sys_state.power_state == POWER_STATE_RPI_ON)
                flash_save_requested = 1;
            sys_state.power_state = POWER_STATE_RPI_OFF;
        }
    }
    if (sys_state.power_state == POWER_STATE_RPI_ON)
        LL_GPIO_SetOutputPin(GPIOA, MT_EN);
    else
        LL_GPIO_ResetOutputPin(GPIOA, MT_EN);

    /* Measurement window and sample period (IP_EN) */
    if (state.usbc_voltage_mv > 3000 || state.microusb_voltage_mv > 3000 || sys_state.power_state == POWER_STATE_RPI_ON)
    {
        sIPIdleTicks = 0;
        if (measurement_window_active)
        {
            sched_counters.measurement_window_ticks++;
            LL_GPIO_ResetOutputPin(GPIOA, IP_EN);
            if (sched_counters.measurement_window_ticks >= MEASUREMENT_WINDOW_TICKS)
            {
                measurement_window_active = 0;
                sched_counters.sample_period_elapsed_ticks = 0;
                LL_GPIO_SetOutputPin(GPIOA, IP_EN);
                flash_save_requested = 1;
                MustRefreshVDD = 1;
            }
        }
        else
        {
            sched_counters.sample_period_elapsed_ticks++;
            if (sched_counters.sample_period_elapsed_ticks > 0 && sched_counters.sample_period_elapsed_ticks < 12000u)
            {
                /* 523 is a prime number; every time interval of this length, re-trigger the button. */
                if (((sched_counters.sample_period_elapsed_ticks + 520) % 523) == 0)
                {
                    LL_GPIO_ResetOutputPin(GPIOA, IP_EN);
                }
                else if (((sched_counters.sample_period_elapsed_ticks + 520) % 523) == 20)
                {
                    LL_GPIO_SetOutputPin(GPIOA, IP_EN);
                }
            }
            /*
            * Periodically perform a brief power interruption for 1.5seconds
            */
            else if (sched_counters.sample_period_elapsed_ticks >= SAMPLE_PERIOD_TICKS_PER_MIN * (uint32_t)state.sample_period_minutes &&
                     sched_counters.sample_period_elapsed_ticks < (SAMPLE_PERIOD_TICKS_PER_MIN * (uint32_t)state.sample_period_minutes) + MEASUREMENT_WINDOW_TICKS)
            {
                if (!measurement_window_active)
                {
                    measurement_window_active = 1;
                    sched_counters.measurement_window_ticks = 0;
                }
                LL_GPIO_ResetOutputPin(GPIOA, IP_EN);
            }
            else if (sched_counters.sample_period_elapsed_ticks >= (SAMPLE_PERIOD_TICKS_PER_MIN * (uint32_t)state.sample_period_minutes) + MEASUREMENT_WINDOW_TICKS)
            {
                sched_counters.sample_period_elapsed_ticks = 0;
                LL_GPIO_SetOutputPin(GPIOA, IP_EN);
                // At this time, also save the registers while we’re at it.
                flash_save_requested = 1;
                MustRefreshVDD = 1;
            }
        }
    }  /* end if (charger present or RPi on) */
    else
    {
        sched_counters.sample_period_elapsed_ticks = 0;
        sIPIdleTicks++;
        if (sIPIdleTicks > 0x1000)
        {
            sIPIdleTicks = 0;
            MustRefreshVDD = 1;
        }
    }

    /* Button debounce (tick_10ms) - still inside Scheduler_Tick10ms() */
    if (sKeyFlag)
    {
        if (!LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_1))
        {
            if (sHoldTime < 200)
                sHoldTime++;
            else
            {
                sHoldTime = 0;
                sKeyClick = 2;
                sKeyFlag = 0;
            }
        }
        else
        {
            if (sHoldTime > 5)
            {
                sHoldTime = 0;
                sKeyClick = 1;
                sKeyFlag = 0;
            }
            else
            {
                sHoldTime = 0;
                sKeyFlag = 0;
            }
        }
    }
}  /* end Scheduler_Tick10ms() */

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
