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

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_DMA_Init(void);

void StorRegValue(void);
void GetRegValue(void);
uint32_t GetUptimeSeconds(void);
void UpdateBatteryPercentage(void);
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

__IO uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];
__IO uint32_t uADCTicks = 0;

__IO uint16_t uPIVCCVolt;
__IO uint16_t uVBATVolt;
__IO uint16_t uVBUSVolt;
__IO uint16_t uUSBINVolt;
__IO uint16_t uADCdegC;
__IO uint16_t uAVDDVolt = 3300;

__IO uint16_t uVBATMax = VBAT_MAX_VOLTAGE;
__IO uint16_t uVBATMin = VBAT_MIN_VOLTAGE; // Minimum voltage required to power the RPi
__IO uint16_t uVBATProtect = VBAT_PROTECT_VOLTAGE; // Absolute minimum voltage to prevent damage to the battery
__IO uint16_t uVBATPrecent;
__IO uint16_t uVBATPrecentReal;

__IO uint16_t uVBATLowPercent = VBAT_LOW_PERCENT;
__IO uint16_t sLoadOnDelayTime = LOAD_ON_DELAY_TIME; // Delay time in seconds to power on the load

__IO uint32_t LoadOnCountDown = 0;

__IO uint8_t sKeyFlag = 0;
__IO uint8_t sKeyClick = 0;        /* Normal press is 1, long press is 2. */
__IO uint8_t sLowPowerRequest = 0; /* Interrupted due to low battery */

__IO uint8_t sIPEnable = 0; /* Power is on (1) or off (0) to the RPi */
__IO uint16_t sIPTicks = 0;
__IO uint16_t sIPIdleTicks = 0;
__IO uint16_t sIPRefreshTime = IP_REFRESH_TIME;

__IO uint8_t sHoldTime = 0; /* Note: this variable is in units of 10 ms */
__IO uint8_t MustRefreshVDD = 1;

__IO uint8_t CountDownPowerOff = 0;
__IO uint8_t CountDownPowerOffTicks = 0;

__IO uint8_t CountDownReboot = 0;
__IO uint8_t CountDownRebootTicks = 0;

__IO uint64_t RuntimeOnetime = 0;
__IO uint64_t RuntimePowerOn = 0;
__IO uint64_t RuntimeAtAll = 0;

__IO uint16_t uUptimeMsCounter = 0; /* Counter for milliseconds (0-999) */

__IO uint8_t OTAShot = 0;

__IO uint16_t Version = 20;

__IO uint8_t FixedVbat = 0;

#define BL_START_ADDRESS 0x8000000

typedef void (*pFunction)(void);
__IO pFunction JumpToAplication;

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

    //In case of unexpected events where the MCU also loses power, we can still provide the last data.
    GetRegValue();
    sLowPowerRequest = 0;

    LL_mDelay(100);

    //__enable_irq(); /* Enable interrupts after initialization */

    while (1)
    {

        if (uVBATProtect != (aReceiveBuffer[17] | aReceiveBuffer[18] << 8))
        {
            uVBATProtect = (aReceiveBuffer[17] | aReceiveBuffer[18] << 8);
        }

        // If the battery protection voltage is not set, set it to 3.6V
        if (uVBATProtect == 0)
        {
            uVBATProtect = VBAT_PROTECT_VOLTAGE;
        }

        if ((sIPRefreshTime != (aReceiveBuffer[21] | aReceiveBuffer[22] << 8)) && ((aReceiveBuffer[21] | aReceiveBuffer[22] << 8) != 0))
        {
            sIPRefreshTime = (aReceiveBuffer[21] | aReceiveBuffer[22] << 8);
        }

        if (0 != aReceiveBuffer[24] && CountDownPowerOff == 0)
        {
            CountDownPowerOff = aReceiveBuffer[24];
        }

        if (0 == aReceiveBuffer[24] && CountDownPowerOff != 0)
        {
            CountDownPowerOff = 0;
        }

        if ((CountDownPowerOff + 1) < aReceiveBuffer[24])
        {
            CountDownPowerOff = aReceiveBuffer[24];
        }

        if (CountDownPowerOff == 1)
        {
            sIPEnable = 0;
            __disable_irq();
            LL_GPIO_ResetOutputPin(GPIOA, MT_EN);
            LL_mDelay(5000);

            if (aReceiveBuffer[25] /* AutoPowerOn */ == 1)
            {
                if (uUSBINVolt > 4000 || uVBUSVolt > 4000)
                {
                    LL_GPIO_SetOutputPin(GPIOA, MT_EN);
                    sIPEnable = 1;
                }
            }
            __enable_irq();
            CountDownPowerOff = 0;
        }

        if (0 != aReceiveBuffer[26] && CountDownReboot == 0)
        {
            CountDownReboot = aReceiveBuffer[24];
        }

        if (0 == aReceiveBuffer[26] && CountDownReboot != 0)
        {
            CountDownReboot = 0;
        }

        if ((CountDownReboot + 1) < aReceiveBuffer[26])
        {
            CountDownReboot = aReceiveBuffer[26];
        }

        if (CountDownReboot == 5)
        {
            __disable_irq();
            LL_GPIO_ResetOutputPin(GPIOA, MT_EN);
            LL_mDelay(5000);
            LL_GPIO_SetOutputPin(GPIOA, MT_EN);
            sIPEnable = 1;
            __enable_irq();
        }
        else if (CountDownReboot == 1)
        {
            sIPEnable = 1;
        }

        if (aReceiveBuffer[27] /* FactoryReset */ == 1)
        {
            FactoryReset();
            // Call StorRegValue() to update the factory reset values in flash once aReceiveBuffer variables are updated
        }

        if (sIPRefreshTime < IP_REFRESH_TIME)
        {
            sIPRefreshTime = IP_REFRESH_TIME;
        }

        if (FixedVbat != 0)
        {
            if (uVBATMax != (aReceiveBuffer[13] | aReceiveBuffer[14] << 8))
            {
                uVBATMax = (aReceiveBuffer[13] | aReceiveBuffer[14] << 8);
            }
            if (uVBATMin != (aReceiveBuffer[15] | aReceiveBuffer[16] << 8))
            {
                uVBATMin = (aReceiveBuffer[15] | aReceiveBuffer[16] << 8);
            }
        }

        aReceiveBuffer[1] = uAVDDVolt & 0xFF;
        aReceiveBuffer[2] = (uAVDDVolt >> 8) & 0xFF;
        aReceiveBuffer[3] = uPIVCCVolt & 0xFF;
        aReceiveBuffer[4] = (uPIVCCVolt >> 8) & 0xFF;
        aReceiveBuffer[5] = uVBATVolt & 0xFF;
        aReceiveBuffer[6] = (uVBATVolt >> 8) & 0xFF;
        aReceiveBuffer[7] = uVBUSVolt & 0xFF;
        aReceiveBuffer[8] = (uVBUSVolt >> 8) & 0xFF;
        aReceiveBuffer[9] = uUSBINVolt & 0xFF;
        aReceiveBuffer[10] = (uUSBINVolt >> 8) & 0xFF;
        aReceiveBuffer[11] = uADCdegC & 0xFF;
        aReceiveBuffer[12] = (uADCdegC >> 8) & 0xFF;

        if (FixedVbat == 0)
        {
            aReceiveBuffer[13] = uVBATMax & 0xFF;
            aReceiveBuffer[14] = (uVBATMax >> 8) & 0xFF;
            aReceiveBuffer[15] = uVBATMin & 0xFF;
            aReceiveBuffer[16] = (uVBATMin >> 8) & 0xFF;
        }

        aReceiveBuffer[17] = uVBATProtect & 0xFF;
        aReceiveBuffer[18] = (uVBATProtect >> 8) & 0xFF;

        UpdateBatteryPercentage();
        aReceiveBuffer[19] = uVBATPrecentReal & 0xFF;
        aReceiveBuffer[20] = (uVBATPrecentReal >> 8) & 0xFF;

        aReceiveBuffer[21] = sIPRefreshTime & 0xFF;
        aReceiveBuffer[22] = (sIPRefreshTime >> 8) & 0xFF;
        aReceiveBuffer[23] = sIPEnable;
        aReceiveBuffer[24] = CountDownPowerOff;
        aReceiveBuffer[26] = CountDownReboot;

        aReceiveBuffer[28] = RuntimePowerOn & 0xFF;
        aReceiveBuffer[29] = (RuntimePowerOn >> 8) & 0xFF;
        aReceiveBuffer[30] = (RuntimePowerOn >> 16) & 0xFF;
        aReceiveBuffer[31] = (RuntimePowerOn >> 24) & 0xFF;

        aReceiveBuffer[32] = RuntimeAtAll & 0xFF;
        aReceiveBuffer[33] = (RuntimeAtAll >> 8) & 0xFF;
        aReceiveBuffer[34] = (RuntimeAtAll >> 16) & 0xFF;
        aReceiveBuffer[35] = (RuntimeAtAll >> 24) & 0xFF;

        aReceiveBuffer[36] = RuntimeOnetime & 0xFF;
        aReceiveBuffer[37] = (RuntimeOnetime >> 8) & 0xFF;
        aReceiveBuffer[38] = (RuntimeOnetime >> 16) & 0xFF;
        aReceiveBuffer[39] = (RuntimeOnetime >> 24) & 0xFF;

        aReceiveBuffer[40] = Version & 0xFF;
        aReceiveBuffer[41] = (Version >> 8) & 0xFF;

        if (aReceiveBuffer[42] != FixedVbat)
        {
            FixedVbat = aReceiveBuffer[42];
        }

        aReceiveBuffer[43] = uVBATLowPercent;

        if (LoadOnCountDown != 0)
        {
            uint16_t remainingTime = LoadOnCountDown - GetUptimeSeconds()+5;
            aReceiveBuffer[44] = remainingTime & 0xFF;
            aReceiveBuffer[45] = (remainingTime >> 8) & 0xFF;
        }
        else
        {
            aReceiveBuffer[44] = sLoadOnDelayTime & 0xFF;
            aReceiveBuffer[45] = (sLoadOnDelayTime >> 8) & 0xFF;
        }

        aReceiveBuffer[240] = LL_GetUID_Word0() & 0xFF;
        aReceiveBuffer[241] = (LL_GetUID_Word0() >> 8) & 0xFF;
        aReceiveBuffer[242] = (LL_GetUID_Word0() >> 16) & 0xFF;
        aReceiveBuffer[243] = (LL_GetUID_Word0() >> 24) & 0xFF;

        aReceiveBuffer[244] = LL_GetUID_Word1() & 0xFF;
        aReceiveBuffer[245] = (LL_GetUID_Word1() >> 8) & 0xFF;
        aReceiveBuffer[246] = (LL_GetUID_Word1() >> 16) & 0xFF;
        aReceiveBuffer[247] = (LL_GetUID_Word1() >> 24) & 0xFF;

        aReceiveBuffer[248] = LL_GetUID_Word2() & 0xFF;
        aReceiveBuffer[249] = (LL_GetUID_Word2() >> 8) & 0xFF;
        aReceiveBuffer[250] = (LL_GetUID_Word2() >> 16) & 0xFF;
        aReceiveBuffer[251] = (LL_GetUID_Word2() >> 24) & 0xFF;

        if (aReceiveBuffer[27] /* FactoryReset */ == 1)
        {
            StorRegValue();
            aReceiveBuffer[27] = 0;
        }

        if (aReceiveBuffer[25] /* AutoPowerOn */ == 1)
        {
            CheckPowerOnConditions();
        }

        if (aReceiveBuffer[50] /* OTA */ == 127 && OTAShot == 0)
        {
            /* Attempt to enter OTA mode */
            StorRegValue();
            OTAShot = 1;
            while (1)
                ;
        }

        if (sKeyClick == 1)
        {
            sKeyClick = 0;
            sIPEnable = !sIPEnable;
            /* TODO: implement a feature here to record the previous state. */
        }
    }
}

void CheckPowerOnConditions(void)
{
    // Check if we are charging (USB power connected)
    uint8_t isCharging = (uUSBINVolt > 4000 || uVBUSVolt > 4000);

    // Check if battery percentage is above threshold
    uint8_t batteryOk = (uVBATPrecentReal > uVBATLowPercent);

    // If countdown is active, check if it has elapsed (even if conditions temporarily changed)
    if (LoadOnCountDown != 0)
    {
        if (GetUptimeSeconds() >= LoadOnCountDown)
        {
            // Countdown completed - power on if conditions are still met
            if (batteryOk)
            {
                sIPEnable = 1;
                LoadOnCountDown = 0;
                // Reset countdown timers
                CountDownPowerOff = 0;
                CountDownRebootTicks = 0;
            }
            else
            {
                // Conditions no longer met, reset countdown
                LoadOnCountDown = 0;
            }
        }
        else
        {
            // Countdown still active - reset only if charging stopped AND battery dropped below threshold
            if (!isCharging && !batteryOk)
            {
                LoadOnCountDown = 0;
            }
        }
    }
    // If we are charging the battery and the battery percentage is greater than the low percentage and the load is not already on
    else if (isCharging && !sIPEnable && batteryOk) 
    {
        // Start our power on countdown
        LoadOnCountDown = GetUptimeSeconds() + sLoadOnDelayTime;
    } 
}

void FactoryReset(void)
{
    uVBATMax = VBAT_MAX_VOLTAGE;
    uVBATMin = VBAT_MIN_VOLTAGE;
    uVBATProtect = VBAT_PROTECT_VOLTAGE;
    sIPRefreshTime = IP_REFRESH_TIME;
    uVBATLowPercent = VBAT_LOW_PERCENT;
    sLoadOnDelayTime = LOAD_ON_DELAY_TIME;
}

void StorRegValue(void)
{
    uint32_t Timeout = 0;
    uint8_t i = 0;
    uint32_t Address = 0x08003C00;
    if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET)
    {
        /* Authorize the FLASH Registers access */
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
        {
            /* Time-out occurred. Set LED2 to blinking mode */
            return;
        }
    }
    CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
    for (i = 0; i < 0xFF; i++)
    {

        /* Proceed to program the new data */
        SET_BIT(FLASH->CR, FLASH_CR_PG);
        /* Write data in the address */
        *(__IO uint16_t *)Address = aReceiveBuffer[i];

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

void GetRegValue(void)
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

        uVBATMax = aReceiveBuffer[13] | (aReceiveBuffer[14] << 8);
        uVBATMin = aReceiveBuffer[15] | (aReceiveBuffer[16] << 8);
        uVBATProtect = aReceiveBuffer[17] | (aReceiveBuffer[18] << 8);
        sIPRefreshTime = aReceiveBuffer[21] | (aReceiveBuffer[22] << 8);
        sIPEnable = aReceiveBuffer[23];
    }
    else
    {
        FactoryReset();
    }
}

/**
 * @brief Get system uptime in seconds
 * @retval Uptime in seconds since system start
 */
uint32_t GetUptimeSeconds(void)
{
    return RuntimeAtAll;
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
    // Clip the voltage to the min and max values if user has provided Max/Min values.
    if (FixedVbat == 1)
    {
        if (uVBATVolt > uVBATMax)
            uVBATVolt = uVBATMax;
        if (uVBATMin > uVBATVolt)
            uVBATVolt = uVBATMin;
    }
    else
    {
        if (uVBATVolt > uVBATMax)
            uVBATMax = uVBATVolt;
        else if (uVBATMin > uVBATVolt && uVBATVolt > uVBATProtect)
            uVBATMin = uVBATVolt;
    }
    if (uVBATMin < uVBATProtect)
    {
        uVBATMin = uVBATProtect;
    }
}

void UpdateBatteryPercentage(void) {

   uint16_t percentage = ((uVBATVolt - uVBATMin) * 100) / (uVBATMax - uVBATMin);
   if (percentage > 0 && percentage < 100)
   {
       // If we are charging the battery, only update if the percentage is higher
       if ((uUSBINVolt > 4000 || uVBUSVolt > 4000) && 
        percentage > uVBATPrecentReal)
           uVBATPrecentReal = percentage;
       // If we are discharging the battery, only update if the percentage is lower
       else if ((uUSBINVolt < 4000 && uVBUSVolt < 4000) && 
                percentage < uVBATPrecentReal)
           uVBATPrecentReal = percentage;
       else if (uVBATPrecentReal == 0)
           uVBATPrecentReal = percentage;
   }
}

void DMA1_CH1_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TC1(DMA1))
    {
        LL_DMA_ClearFlag_TC1(DMA1);
        if (MustRefreshVDD)
        {
            uAVDDVolt = (__LL_ADC_CALC_VREFANALOG_VOLTAGE(aADCxConvertedData[5],
                                                          LL_ADC_RESOLUTION_12B) +
                         uAVDDVolt) /
                        2;
            MustRefreshVDD = 0;
        }

        uPIVCCVolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(
            uAVDDVolt * 2, aADCxConvertedData[0], LL_ADC_RESOLUTION_12B);
        uVBATVolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(
            uAVDDVolt * 2, aADCxConvertedData[1], LL_ADC_RESOLUTION_12B);
        uVBUSVolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(
            uAVDDVolt * 4, aADCxConvertedData[2], LL_ADC_RESOLUTION_12B);
        uUSBINVolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(
            uAVDDVolt * 4, aADCxConvertedData[3], LL_ADC_RESOLUTION_12B);
        uADCdegC = __LL_ADC_CALC_TEMPERATURE(uAVDDVolt, aADCxConvertedData[4],
                                             LL_ADC_RESOLUTION_12B);

        // Clip battery numbers if needed
        UpdateBatteryMinMax();
    }

    /* Check whether DMA transfer error caused the DMA interruption */
    if(LL_DMA_IsActiveFlag_TE1(DMA1) == 1)
    {
        /* Clear flag DMA transfer error */
        LL_DMA_ClearFlag_TE1(DMA1);
    }
}

/**
  * @brief This function handles System tick timer.
  * Keep time tracking independent of other states and interrupts
  */
 void SysTick_Handler(void)
 {
   uUptimeMsCounter++;
   if (uUptimeMsCounter >= 1000)
   {
     uUptimeMsCounter = 0;
     RuntimeAtAll++;  /* Always increment total runtime */
     if (sIPEnable)
     {
        RuntimeOnetime++; // Current runtime of RPi
        // We can only be charging when there is power connected to the Raspberry Pi
        if (uUSBINVolt > 4000 || uVBUSVolt > 4000)
        {
            RuntimePowerOn++; // Charging time
        }
     }
     else
     {
        RuntimeOnetime = 0; //When not powering RPi, reset the runtime
     }
   }
 }

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM1) == 1)
    {
        LL_TIM_ClearFlag_UPDATE(TIM1);

        if (uVBATVolt > 1000 && (uUSBINVolt + uVBUSVolt) < 4000)
        {
            if (uVBATVolt < uVBATProtect)
            {
                // Store our state if we are shutting down
                if (sIPEnable)
                {
                    StorRegValue();
                }
                sIPEnable = 0;
                sLowPowerRequest = 1;
            }
        }

        if (CountDownPowerOff > 0)
        {
            if (CountDownPowerOffTicks > 100)
            {
                CountDownPowerOff--;
                CountDownPowerOffTicks = 0;
            }
            else
            {
                CountDownPowerOffTicks++;
            }
        }

        if (CountDownReboot > 0)
        {
            if (CountDownRebootTicks > 100)
            {
                CountDownReboot--;
                CountDownRebootTicks = 0;
            }
            else
            {
                CountDownRebootTicks++;
            }
        }

        /* Turn on/off power to RPi */
        if (sIPEnable)
        {
            LL_GPIO_SetOutputPin(GPIOA, MT_EN);
        }
        else
        {
            LL_GPIO_ResetOutputPin(GPIOA, MT_EN);
        }

        /* Refresh power interruption */
        if (uVBUSVolt > 3000 || uUSBINVolt > 3000 || sIPEnable) // If USB power is connected OR power to RPi is enabled.
        {
            sIPIdleTicks = 0;
            sIPTicks++;
            if (sIPTicks > 0 && sIPTicks < 12000) // If we just started up, check every 5.23 secs
            {
                /* 523 is a prime number; every time interval of this length, re-trigger the button. */
                if (((sIPTicks + 520) % 523) == 0)
                {
                    LL_GPIO_ResetOutputPin(GPIOA, IP_EN);
                }
                else if (((sIPTicks + 520) % 523) == 20)
                {
                    LL_GPIO_SetOutputPin(GPIOA, IP_EN);
                }
            }
            /*
            * Periodically perform a brief power interruption for 1.5seconds
            */
            else if (sIPTicks > (6000 * sIPRefreshTime) &&
                     sIPTicks < ((6000 * sIPRefreshTime) + 1500))
            {
                /* Periodically perform a brief power interruption; this helps to re-identify (re-detect) the battery level. */
                LL_GPIO_ResetOutputPin(GPIOA, IP_EN);
            }
            else if (sIPTicks > ((6000 * sIPRefreshTime) + 1500))
            {
                sIPTicks = 0;
                LL_GPIO_SetOutputPin(GPIOA, IP_EN);
                // At this time, also save the registers while we’re at it.
                StorRegValue();
                // At the same time, refresh/update the VDD voltage.
                MustRefreshVDD = 1;
            }
        }
        else
        {
            sIPTicks = 0;
            sIPIdleTicks++;
            if (sIPIdleTicks > 0x1000)
            {
                sIPIdleTicks = 0;
                // StorRegValue();
                // At the same time, refresh/update the VDD voltage.
                MustRefreshVDD = 1;
            }
        }

        /* Refresh ADC readings 
        * ADC sampling every 500 ms to monitor voltages and temperature.
        */
        if (uADCTicks > 50)
        {
            uADCTicks = 0;
            LL_ADC_REG_StartConversion(ADC1);
        }
        else
        {
            uADCTicks++;
        }

        /* Button Pressed */
        if (sKeyFlag)
        {
            if (!LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_1))
            {
                if (sHoldTime < 200)
                {
                    sHoldTime++;
                }
                else
                {
                    /* If 2 seconds is reached, treat it as a long press. */
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
    }
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
