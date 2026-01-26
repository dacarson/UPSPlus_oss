#include "I2C_Slave.h"

#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_i2c.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx.h"

/* I2C slave receive buffer - 256 bytes to support indices 0-255 */
volatile uint8_t aReceiveBuffer[256];

/* I2C Slave state variables */
static uint8_t uI2CRegIndex = 0;  /* Current register index being accessed */

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

    /* NVIC (enable before PE so we don't miss the first address match) */
    NVIC_SetPriority(I2C1_IRQn, 0);
    NVIC_EnableIRQ(I2C1_IRQn);

    // Disable I2C1 before configuring it
    LL_I2C_Disable(I2C1); 

    /* Program TIMINGR 100 kHz timing values */
    uint32_t timing_100k = __LL_I2C_CONVERT_TIMINGS(
        1,   /* PRESC  -> tPRESC = 250ns */
        4,   /* SCLDEL -> ~1.25us */
        1,   /* SDADEL -> ~0.25us */
        19,  /* SCLH   -> 5.0us */
        19   /* SCLL   -> 5.0us */
    );
    LL_I2C_SetTiming(I2C1, timing_100k);

    /*
     * Program Own Address 1 
     */
    LL_I2C_SetOwnAddress1(I2C1, 0x17 << 1, LL_I2C_OWNADDRESS1_7BIT);
    LL_I2C_EnableOwnAddress1(I2C1);

    /*
     * Enable interrupts:
     */
    LL_I2C_EnableIT_ADDR(I2C1);
    LL_I2C_EnableIT_NACK(I2C1);
    LL_I2C_EnableIT_STOP(I2C1);
    LL_I2C_EnableIT_ERR(I2C1);

    /* Enable peripheral */
    LL_I2C_Enable(I2C1); 
}

/**
 * @brief I2C1 Event Interrupt Handler
 * @retval None
 *
 * This handler is automatically registered by the linker, which overrides the weak
 * definition in the startup file (startup_stm32f030f4px.s). No explicit registration needed.
 *
 * This handler implements a register map where:
 * - Write: [SlaveAddr+W] [RegisterIndex] [Data1] [Data2] ... → writes to aReceiveBuffer[RegisterIndex++]
 * - Read: [SlaveAddr+R] [Data1] [Data2] ... → reads from aReceiveBuffer[currentIndex++]
 */
void I2C1_IRQHandler(void)
{
    /* Process all active flags in priority order */
    /* Multiple flags can be set simultaneously, so we check all of them */
    
    /* 1. ADDR: Address match - highest priority, must be handled first */
    if (LL_I2C_IsActiveFlag_ADDR(I2C1)) {
        uint32_t addr_match = LL_I2C_GetAddressMatchCode(I2C1);
        if (addr_match == (0x17 << 1)) {
            if (LL_I2C_GetTransferDirection(I2C1) == LL_I2C_DIRECTION_READ) {
                /* Read transfer: prepare to transmit */
                LL_I2C_ClearFlag_ADDR(I2C1);
                LL_I2C_ClearFlag_TXE(I2C1);
                LL_I2C_EnableIT_TX(I2C1);
            }
            else {
                /* Write transfer: prepare to receive */
                uI2CRegIndex = 0;
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
    
    /* 4. TXIS: Transmit interrupt - data register empty, ready to send */
    if (LL_I2C_IsActiveFlag_TXIS(I2C1)) {
        LL_I2C_TransmitData8(I2C1, aReceiveBuffer[uI2CRegIndex++]);
    }
    
    /* 5. RXNE: Receive interrupt - data available to read */
    if (LL_I2C_IsActiveFlag_RXNE(I2C1)) {
        if (uI2CRegIndex == 0) {
            /* First byte is the register index */
            uI2CRegIndex = LL_I2C_ReceiveData8(I2C1);
        }
        else {
            /* Subsequent bytes are data */
            aReceiveBuffer[uI2CRegIndex++] = LL_I2C_ReceiveData8(I2C1);
            aReceiveBuffer[0] |= 1U;
        }
    }
    
    /* 6. STOP: Stop condition detected */
    if (LL_I2C_IsActiveFlag_STOP(I2C1)) {
        LL_I2C_ClearFlag_STOP(I2C1);
        LL_I2C_DisableIT_RX(I2C1);
        LL_I2C_DisableIT_TX(I2C1);
    }
}

