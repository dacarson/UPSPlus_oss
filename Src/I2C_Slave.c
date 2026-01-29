#include "I2C_Slave.h"

#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_i2c.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx.h"

/* I2C slave receive buffer - 256 bytes; used for flash load (GetRegValue) only. I2C reads use reg_image[]. */
volatile uint8_t aReceiveBuffer[256];

/* Phase 2: Pending write - ISR stores here; main loop applies to authoritative state */
i2c_pending_write_t i2c_pending_write = {0};

/* I2C Slave state variables */
static uint8_t uI2CRegIndex = 0;             /* Register pointer: set by first byte of write; used for READ */
static uint8_t write_byte_index = 0;         /* 0 = expect reg ptr, 1+ = data bytes */
static uint8_t ignore_write = 0;             /* 1 = pending already set, drop this write until STOP */
static volatile uint8_t latched_reg_image = 0;  /* Latched at ADDR+READ; TX uses reg_image[latched_reg_image] for whole transaction */

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
    }
}

