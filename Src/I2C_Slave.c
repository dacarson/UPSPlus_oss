#include "I2C_Slave.h"

#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_i2c.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx.h"

/* Pending write - ISR stores here; main loop applies to authoritative state */
i2c_pending_write_t i2c_pending_write = {0};

/* I2C Slave state variables */
static uint8_t uI2CRegIndex = 0;             /* Register pointer: set by first byte of write; used for READ */
static uint8_t write_byte_index = 0;         /* 0 = expect reg ptr, 1+ = data bytes */
static uint8_t ignore_write = 0;             /* 1 = pending already set, drop this write until STOP */
static volatile uint8_t latched_reg_image = 0;  /* Latched at ADDR+READ; TX uses reg_image[latched_reg_image] for whole transaction */

/* INA219 probe/master setup (Phase 2 + minimal Phase 3) */
#define INA219_ADDR_OUTPUT      0x40u
#define INA219_ADDR_BATTERY     0x45u
#define INA219_REG_CONFIG       0x00u
#define INA219_CONFIG_VALUE     0x0B9Du
#define I2C_BOOT_TIMEOUT_CYCLES 100000u
/* I2C1 kernel clock source is HSI (8 MHz) per SystemClock_Config().
 * If I2C1SW changes, regenerate timing and update this macro. */
#define I2C1_TIMING_100KHZ_HSI8MHZ __LL_I2C_CONVERT_TIMINGS(1, 4, 1, 19, 19)
#define I2C_PROBE_RETRIES  5u
#define I2C_PROBE_IDLE_CYCLES    200000u
#define I2C_PROBE_RETRY_DELAY_MS 10u
#define I2C_PROBE_INTER_PROBE_DELAY_MS 20u
#define I2C_PROBE_INITIAL_DELAY_MS 100u
#define I2C_RECOVERY_PULSES     12u
#define I2C_RECOVERY_DELAY_MS   1u
#if defined(RCC_CFGR3_I2C1SW)
/* I2C1SW: 0 = HSI, 1 = SYSCLK. If this changes, regenerate timing. */
#define I2C1_CLOCK_IS_HSI() (((RCC->CFGR3 & RCC_CFGR3_I2C1SW) == 0u) ? 1u : 0u)
#else
#define I2C1_CLOCK_IS_HSI() 1u
#endif

static uint8_t ina_probe_present_output = 0u;
static uint8_t ina_probe_present_battery = 0u;

static volatile uint8_t i2c_slave_txn_active = 0u;
static volatile uint8_t i2c_addr_matched_since_stop = 0u;
static volatile uint32_t i2c_last_addr_us = 0u;
static volatile uint32_t i2c_last_stop_us = 0u;


static void I2C1_ClearErrorFlags(void)
{
    if (LL_I2C_IsActiveFlag_NACK(I2C1))
        LL_I2C_ClearFlag_NACK(I2C1);
    if (LL_I2C_IsActiveFlag_BERR(I2C1))
        LL_I2C_ClearFlag_BERR(I2C1);
    if (LL_I2C_IsActiveFlag_ARLO(I2C1))
        LL_I2C_ClearFlag_ARLO(I2C1);
    if (LL_I2C_IsActiveFlag_OVR(I2C1))
        LL_I2C_ClearFlag_OVR(I2C1);
}

static uint8_t I2C1_CheckAndClearErrors(void)
{
    if (LL_I2C_IsActiveFlag_BERR(I2C1))
    {
        LL_I2C_ClearFlag_BERR(I2C1);
        return 1u;
    }
    if (LL_I2C_IsActiveFlag_ARLO(I2C1))
    {
        LL_I2C_ClearFlag_ARLO(I2C1);
        return 1u;
    }
    if (LL_I2C_IsActiveFlag_OVR(I2C1))
    {
        LL_I2C_ClearFlag_OVR(I2C1);
        return 1u;
    }
    return 0u;
}

static void I2C1_ClearAllFlags(void)
{
    if (LL_I2C_IsActiveFlag_STOP(I2C1))
        LL_I2C_ClearFlag_STOP(I2C1);
    if (LL_I2C_IsActiveFlag_NACK(I2C1))
        LL_I2C_ClearFlag_NACK(I2C1);
    if (LL_I2C_IsActiveFlag_BERR(I2C1))
        LL_I2C_ClearFlag_BERR(I2C1);
    if (LL_I2C_IsActiveFlag_ARLO(I2C1))
        LL_I2C_ClearFlag_ARLO(I2C1);
    if (LL_I2C_IsActiveFlag_OVR(I2C1))
        LL_I2C_ClearFlag_OVR(I2C1);
}

static void I2C1_AbortBootTransfer(void)
{
    LL_I2C_Disable(I2C1);
    I2C1_ClearAllFlags();
    LL_I2C_Enable(I2C1);
    I2C1_ClearAllFlags();
    LL_I2C_Disable(I2C1);
}

static void I2C1_BootEnsureEnabled(void)
{
    if (!LL_I2C_IsEnabled(I2C1))
    {
        LL_I2C_Enable(I2C1);
        I2C1_ClearAllFlags();
    }
}

static void I2C1_BusRecovery(void)
{
    uint8_t pulse = 0u;

    LL_I2C_Disable(I2C1);

    /* SCL as GPIO open-drain output; SDA as input pull-up (released). */
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_LOW);

    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);

    /* Ensure SCL high, SDA released before clocking. */
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9);
    LL_mDelay(I2C_RECOVERY_DELAY_MS);

    /* Clock SCL low->high to free a stuck slave. */
    for (pulse = 0u; pulse < I2C_RECOVERY_PULSES; pulse++)
    {
        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9);
        LL_mDelay(I2C_RECOVERY_DELAY_MS);
        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9);
        LL_mDelay(I2C_RECOVERY_DELAY_MS);
    }

    /* Generate STOP: drive SDA low (open-drain), then release high while SCL high. */
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10);
    LL_mDelay(I2C_RECOVERY_DELAY_MS);
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9);
    LL_mDelay(I2C_RECOVERY_DELAY_MS);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);
    LL_mDelay(I2C_RECOVERY_DELAY_MS);

    /* Restore AF4 I2C pins. */
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_9, LL_GPIO_AF_4);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_OPENDRAIN);

    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_10, LL_GPIO_AF_4);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_OPENDRAIN);

    I2C1_ClearAllFlags();
}

static uint8_t I2C1_WaitForBusIdle(uint32_t timeout_cycles)
{
    while (LL_I2C_IsActiveFlag_BUSY(I2C1))
    {
        if (timeout_cycles-- == 0u)
        {
            I2C1_BusRecovery();
            I2C1_BootEnsureEnabled();
            if (LL_I2C_IsActiveFlag_BUSY(I2C1))
                return 0u;
            return 1u;
        }
    }
    return 1u;
}

static uint8_t I2C1_WaitForTXIS(uint32_t timeout_cycles)
{
    while (!LL_I2C_IsActiveFlag_TXIS(I2C1))
    {
        if (I2C1_CheckAndClearErrors())
            return 0u;
        if (LL_I2C_IsActiveFlag_NACK(I2C1))
        {
            LL_I2C_ClearFlag_NACK(I2C1);
            return 0u;
        }
        if (timeout_cycles-- == 0u)
            return 0u;
    }
    return 1u;
}

static uint8_t I2C1_WaitForRXNE(uint32_t timeout_cycles)
{
    while (!LL_I2C_IsActiveFlag_RXNE(I2C1))
    {
        if (I2C1_CheckAndClearErrors())
            return 0u;
        if (LL_I2C_IsActiveFlag_NACK(I2C1))
        {
            LL_I2C_ClearFlag_NACK(I2C1);
            return 0u;
        }
        if (timeout_cycles-- == 0u)
            return 0u;
    }
    return 1u;
}

static uint8_t I2C1_WaitForSTOP(uint32_t timeout_cycles)
{
    while (!LL_I2C_IsActiveFlag_STOP(I2C1))
    {
        if (I2C1_CheckAndClearErrors())
            return 0u;
        if (LL_I2C_IsActiveFlag_NACK(I2C1))
        {
            LL_I2C_ClearFlag_NACK(I2C1);
            return 0u;
        }
        if (timeout_cycles-- == 0u)
            return 0u;
    }
    return 1u;
}

static uint8_t I2C1_WaitForTC(uint32_t timeout_cycles)
{
    while (!LL_I2C_IsActiveFlag_TC(I2C1))
    {
        if (I2C1_CheckAndClearErrors())
            return 0u;
        if (LL_I2C_IsActiveFlag_NACK(I2C1))
        {
            LL_I2C_ClearFlag_NACK(I2C1);
            return 0u;
        }
        if (timeout_cycles-- == 0u)
            return 0u;
    }
    return 1u;
}

static uint8_t I2C1_MasterWriteReg16(uint8_t addr_7bit, uint8_t reg, uint16_t value)
{
    uint8_t msb = (uint8_t)((value >> 8) & 0xFFu);
    uint8_t lsb = (uint8_t)(value & 0xFFu);

    I2C1_ClearAllFlags();
    LL_I2C_HandleTransfer(I2C1, (uint32_t)(addr_7bit << 1), LL_I2C_ADDRSLAVE_7BIT, 3,
                          LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

    if (!I2C1_WaitForTXIS(I2C_BOOT_TIMEOUT_CYCLES))
    {
        I2C1_AbortBootTransfer();
        return 0u;
    }
    LL_I2C_TransmitData8(I2C1, reg);

    if (!I2C1_WaitForTXIS(I2C_BOOT_TIMEOUT_CYCLES))
    {
        I2C1_AbortBootTransfer();
        return 0u;
    }
    LL_I2C_TransmitData8(I2C1, msb);

    if (!I2C1_WaitForTXIS(I2C_BOOT_TIMEOUT_CYCLES))
    {
        I2C1_AbortBootTransfer();
        return 0u;
    }
    LL_I2C_TransmitData8(I2C1, lsb);

    if (!I2C1_WaitForSTOP(I2C_BOOT_TIMEOUT_CYCLES))
    {
        I2C1_AbortBootTransfer();
        return 0u;
    }
    LL_I2C_ClearFlag_STOP(I2C1);
    return 1u;
}

static uint8_t I2C1_MasterReadReg16(uint8_t addr_7bit, uint8_t reg, uint16_t *value)
{
    uint8_t msb = 0u;
    uint8_t lsb = 0u;

    I2C1_ClearAllFlags();
    LL_I2C_HandleTransfer(I2C1, (uint32_t)(addr_7bit << 1), LL_I2C_ADDRSLAVE_7BIT, 1,
                          LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);

    if (!I2C1_WaitForTXIS(I2C_BOOT_TIMEOUT_CYCLES))
    {
        I2C1_AbortBootTransfer();
        return 0u;
    }
    LL_I2C_TransmitData8(I2C1, reg);

    if (!I2C1_WaitForTC(I2C_BOOT_TIMEOUT_CYCLES))
    {
        I2C1_AbortBootTransfer();
        return 0u;
    }

    /* Only clear error/NACK flags between pointer write and repeated-start read. */
    I2C1_ClearErrorFlags();
    LL_I2C_HandleTransfer(I2C1, (uint32_t)(addr_7bit << 1), LL_I2C_ADDRSLAVE_7BIT, 2,
                          LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

    if (!I2C1_WaitForRXNE(I2C_BOOT_TIMEOUT_CYCLES))
    {
        I2C1_AbortBootTransfer();
        return 0u;
    }
    msb = LL_I2C_ReceiveData8(I2C1);

    if (!I2C1_WaitForRXNE(I2C_BOOT_TIMEOUT_CYCLES))
    {
        I2C1_AbortBootTransfer();
        return 0u;
    }
    lsb = LL_I2C_ReceiveData8(I2C1);

    if (!I2C1_WaitForSTOP(I2C_BOOT_TIMEOUT_CYCLES))
    {
        I2C1_AbortBootTransfer();
        return 0u;
    }
    LL_I2C_ClearFlag_STOP(I2C1);

    *value = (uint16_t)(((uint16_t)msb << 8) | (uint16_t)lsb);
    return 1u;
}

void MX_I2C1_ProbeMasterSetup(void)
{
    uint32_t timing_100k;
    uint16_t readback = 0u;
    uint8_t ok = 0u;
    uint8_t attempt = 0u;

    /* GPIOA + I2C1 clocks */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

    /* Configure PA9/PA10 for I2C1 AF4 (master pre-init) */
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_9, LL_GPIO_AF_4);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_OPENDRAIN);

    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_10, LL_GPIO_AF_4);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_OPENDRAIN);

    LL_I2C_Disable(I2C1);
    /* Do not enable own address/ACK or interrupts until slave init runs. */
    LL_I2C_DisableOwnAddress1(I2C1);
    LL_I2C_DisableOwnAddress2(I2C1);
    NVIC_DisableIRQ(I2C1_IRQn);
    LL_I2C_DisableIT_ADDR(I2C1);
    LL_I2C_DisableIT_NACK(I2C1);
    LL_I2C_DisableIT_STOP(I2C1);
    LL_I2C_DisableIT_ERR(I2C1);
    if (!I2C1_CLOCK_IS_HSI())
    {
        I2C1_ClearAllFlags();
        return;
    }
    timing_100k = I2C1_TIMING_100KHZ_HSI8MHZ;
    LL_I2C_SetTiming(I2C1, timing_100k);
    LL_I2C_Enable(I2C1);
    LL_mDelay(I2C_PROBE_INITIAL_DELAY_MS);

    /* Configure INA219s and verify with a readback for probe presence. */
    ok = 0u;
    for (attempt = 0u; attempt < I2C_PROBE_RETRIES && ok == 0u; attempt++)
    {
        I2C1_BootEnsureEnabled();
        if (!I2C1_WaitForBusIdle(I2C_PROBE_IDLE_CYCLES))
        {
            LL_mDelay(I2C_PROBE_RETRY_DELAY_MS);
            continue;
        }
        ok = I2C1_MasterWriteReg16(INA219_ADDR_OUTPUT, INA219_REG_CONFIG, INA219_CONFIG_VALUE);
        if (ok)
        {
            ok = I2C1_MasterReadReg16(INA219_ADDR_OUTPUT, INA219_REG_CONFIG, &readback);
        }
        if (ok && readback != INA219_CONFIG_VALUE)
        {
            ok = 0u;
        }
        if (ok == 0u)
            LL_mDelay(I2C_PROBE_RETRY_DELAY_MS);
    }
    ina_probe_present_output = ok;

    ok = 0u;
    for (attempt = 0u; attempt < I2C_PROBE_RETRIES && ok == 0u; attempt++)
    {
        I2C1_BootEnsureEnabled();
        if (!I2C1_WaitForBusIdle(I2C_PROBE_IDLE_CYCLES))
        {
            LL_mDelay(I2C_PROBE_RETRY_DELAY_MS);
            continue;
        }
        ok = I2C1_MasterWriteReg16(INA219_ADDR_BATTERY, INA219_REG_CONFIG, INA219_CONFIG_VALUE);
        if (ok)
        {
            ok = I2C1_MasterReadReg16(INA219_ADDR_BATTERY, INA219_REG_CONFIG, &readback);
        }
        if (ok && readback != INA219_CONFIG_VALUE)
        {
            ok = 0u;
        }
        if (ok == 0u)
            LL_mDelay(I2C_PROBE_RETRY_DELAY_MS);
    }
    ina_probe_present_battery = ok;

    LL_I2C_Disable(I2C1);
}

uint8_t I2C1_GetInaProbeOutputPresent(void)
{
    return ina_probe_present_output;
}

uint8_t I2C1_GetInaProbeBatteryPresent(void)
{
    return ina_probe_present_battery;
}

uint8_t I2C1_GetSlaveTxnActive(void)
{
    return i2c_slave_txn_active;
}

uint32_t I2C1_GetLastAddrUs(void)
{
    return i2c_last_addr_us;
}

uint32_t I2C1_GetLastStopUs(void)
{
    return i2c_last_stop_us;
}

void I2C1_RunIna219Probe(void)
{
    /* Re-run INA219 probe safely by reinitializing the slave afterward. */
    NVIC_DisableIRQ(I2C1_IRQn);
    LL_I2C_Disable(I2C1);
    MX_I2C1_ProbeMasterSetup();
    MX_I2C1_Slave_Init();
}

/**
 * @brief I2C1 Slave Initialization Function
 * @param None
 * @retval None
 */
void MX_I2C1_Slave_Init(void)
{
    /* GPIOA + I2C1 clocks */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1); // LL_APB1_GRP1_PERIPH_I2C1 == firmware has 0x200000

    /* Now configure PA9/PA10 for I2C1 AF4 */
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_9, LL_GPIO_AF_4);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_OPENDRAIN);

    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_10, LL_GPIO_AF_4);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_OPENDRAIN);

    /* NVIC (enable after flags/ITs are configured) */
    NVIC_SetPriority(I2C1_IRQn, 0);

    // Disable I2C1 before configuring it
    LL_I2C_Disable(I2C1);
    I2C1_ClearAllFlags();

    /* Program TIMINGR 100 kHz timing values */
    uint32_t timing_100k = I2C1_TIMING_100KHZ_HSI8MHZ;
    LL_I2C_SetTiming(I2C1, timing_100k);

    /*
     * Program Own Address 1 
     */
    LL_I2C_SetOwnAddress1(I2C1, 0x17 << 1, LL_I2C_OWNADDRESS1_7BIT);
    LL_I2C_EnableOwnAddress1(I2C1);

    /* Enable interrupts: */
    LL_I2C_EnableIT_ADDR(I2C1);
    LL_I2C_EnableIT_NACK(I2C1);
    LL_I2C_EnableIT_STOP(I2C1);
    LL_I2C_EnableIT_ERR(I2C1);

    /* Enable peripheral */
    LL_I2C_Enable(I2C1);
    NVIC_EnableIRQ(I2C1_IRQn);
}

/**
 * @brief I2C1 Event Interrupt Handler
 * @retval None
 *
 * This handler is automatically registered by the linker, which overrides the weak
 * definition in the startup file (startup_stm32f030f4px.s). No explicit registration needed.
 *
 * Register map:
 * - Write: [SlaveAddr+W] [RegIndex] [Data1] [Data2] ... First byte = register pointer (uI2CRegIndex).
 *   Data bytes go to i2c_pending_write; main loop applies. Pending set only if length > 0.
 *   If pending already set, ignore this write until STOP.
 * - Read: [SlaveAddr+R] [Data1] [Data2] ... Transmit from reg_image[latched_reg_image], starting
 *   at uI2CRegIndex. Latch active_reg_image on ADDR for READ; use that buffer for whole transaction.
 */
void I2C1_IRQHandler(void)
{
    /* Process all active flags in priority order */
    /* Multiple flags can be set simultaneously, so we check all of them */
    
    /* 1. ADDR: Address match - highest priority, must be handled first */
    if (LL_I2C_IsActiveFlag_ADDR(I2C1)) {
        uint32_t addr_match = LL_I2C_GetAddressMatchCode(I2C1);
        if (addr_match == (0x17 << 1)) {
            i2c_slave_txn_active = 1u;
            i2c_addr_matched_since_stop = 1u;
            i2c_last_addr_us = (uint32_t)LL_TIM_GetCounter(TIM3);
            if (LL_I2C_GetTransferDirection(I2C1) == LL_I2C_DIRECTION_READ) {
                /* Read: latch which reg_image buffer to use for whole transaction */
                latched_reg_image = active_reg_image;
                LL_I2C_ClearFlag_ADDR(I2C1);
                LL_I2C_ClearFlag_TXE(I2C1);
                LL_I2C_EnableIT_TX(I2C1);
            }
            else {
                /* Write: first byte = reg pointer. Overwrite protection: if pending != 0, ignore until STOP. */
                write_byte_index = 0;
                ignore_write = (i2c_pending_write.pending != 0) ? 1 : 0;
                if (!ignore_write) {
                    i2c_pending_write.length = 0;
                }
                LL_I2C_ClearFlag_ADDR(I2C1);
                LL_I2C_EnableIT_RX(I2C1);
            }
        }
        else {
            /* Address match but not our address - just clear */
            LL_I2C_ClearFlag_ADDR(I2C1);
        }
    }
    
    /* 2. Error flags - handle errors early to recover from bus issues */
    /* BERR: Bus error - detected when START or STOP condition is misplaced */
    if (LL_I2C_IsActiveFlag_BERR(I2C1)) {
        LL_I2C_ClearFlag_BERR(I2C1);
        /* Disable RX/TX interrupts to reset state */
        LL_I2C_DisableIT_RX(I2C1);
        LL_I2C_DisableIT_TX(I2C1);
    }
    
    /* ARLO: Arbitration lost - not applicable in slave mode, but clear if set */
    if (LL_I2C_IsActiveFlag_ARLO(I2C1)) {
        LL_I2C_ClearFlag_ARLO(I2C1);
        /* Disable RX/TX interrupts to reset state */
        LL_I2C_DisableIT_RX(I2C1);
        LL_I2C_DisableIT_TX(I2C1);
    }
    
    /* OVR: Overrun/Underrun - data register not read/written in time */
    if (LL_I2C_IsActiveFlag_OVR(I2C1)) {
        LL_I2C_ClearFlag_OVR(I2C1);
        /* Disable RX/TX interrupts to reset state */
        LL_I2C_DisableIT_RX(I2C1);
        LL_I2C_DisableIT_TX(I2C1);
    }
    
    /* 3. NACK: Not Acknowledge received */
    if (LL_I2C_IsActiveFlag_NACK(I2C1)) {
        LL_I2C_ClearFlag_NACK(I2C1);
    }
    
    /* 4. TXIS: Transmit interrupt - send from latched reg_image buffer */
    if (LL_I2C_IsActiveFlag_TXIS(I2C1)) {
        LL_I2C_TransmitData8(I2C1, reg_image[latched_reg_image][uI2CRegIndex++]);
    }

    /* 5. RXNE: Receive interrupt. First byte = register pointer; then data bytes → pending. */
    if (LL_I2C_IsActiveFlag_RXNE(I2C1)) {
        uint8_t byte = LL_I2C_ReceiveData8(I2C1);
        if (ignore_write) {
            (void)byte; /* Drop; read already done to clear RXNE */
        }
        else if (write_byte_index == 0) {
            /* First byte = register pointer. Set both so "write [reg] then repeated-start read" works. */
            uI2CRegIndex = byte;
            i2c_pending_write.reg_addr = byte;
            i2c_pending_write.length = 0;
            write_byte_index = 1;
        } else {
            if (i2c_pending_write.length < I2C_PENDING_WRITE_MAX_LEN) {
                i2c_pending_write.data[i2c_pending_write.length++] = byte;
            }
        }
    }

    /* 6. STOP: Pending only if length > 0 (data bytes). Pointer-only writes must not set pending. */
    if (LL_I2C_IsActiveFlag_STOP(I2C1)) {
        LL_I2C_ClearFlag_STOP(I2C1);
        LL_I2C_DisableIT_RX(I2C1);
        LL_I2C_DisableIT_TX(I2C1);
        if (ignore_write) {
            ignore_write = 0;
        } else if (i2c_pending_write.length > 0) {
            i2c_pending_write.pending = 1;
        }
        write_byte_index = 0;
        if (i2c_addr_matched_since_stop) {
            i2c_last_stop_us = (uint32_t)LL_TIM_GetCounter(TIM3);
        }
        i2c_addr_matched_since_stop = 0u;
        i2c_slave_txn_active = 0u;
    }
}

