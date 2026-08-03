/**
 * stm32f030_periph.h  –  Application-specific constants for the UPSPlus
 *                        STM32F030F4P6 bootloader.
 *
 * Peripheral register layouts (GPIO_TypeDef, I2C_TypeDef, FLASH_TypeDef,
 * RCC_TypeDef) and their bit-field macros are NOT redefined here anymore.
 * They come straight from the ST-supplied headers, which this file and
 * bootloader.c both depend on:
 *
 *   stm32f0xx.h                – CMSIS device header (pulls in stm32f030x6.h)
 *   stm32f0xx_ll_gpio.h        – LL_GPIO_* pin configuration helpers
 *   stm32f0xx_ll_i2c.h         – LL_I2C_* register-level slave I2C helpers
 *   stm32f0xx_hal_flash.h      – HAL_FLASH_Unlock/Lock/Program
 *   stm32f0xx_hal_flash_ex.h   – HAL_FLASHEx_Erase (page erase)
 *   stm32f0xx_hal_cortex.h     – HAL_NVIC_SetPriority/EnableIRQ
 *
 * The register addresses and bit positions in those official headers were
 * cross-checked against this reconstruction and matched exactly, with one
 * correction: I2C_ISR_DIR is bit 16, not bit 24 as this file previously
 * (incorrectly) assumed — see bootloader.c I2C1_IRQHandler for the fix.
 *
 * What remains here is only the bootloader/application-specific memory
 * map, RAM layout, and OTA wire-protocol constants, none of which are part
 * of the standard STM32F0 headers.
 */

#ifndef STM32F030_PERIPH_H
#define STM32F030_PERIPH_H

#include <stdint.h>
#include "stm32f0xx.h"
#include "stm32f0xx_ll_gpio.h"

/* ------------------------------------------------------------------ */
/*  STM32 Unique Device ID (96 bits)                                   */
/*  CMSIS UID_BASE (0x1FFFF7AC) already points at the first UID word,  */
/*  unlike this file's previous UID_BASE + 0x2C/0x30/0x34 offsets.     */
/* ------------------------------------------------------------------ */
#define UID_WORD0   (*(const volatile uint32_t *)(UID_BASE + 0x00UL))
#define UID_WORD1   (*(const volatile uint32_t *)(UID_BASE + 0x04UL))
#define UID_WORD2   (*(const volatile uint32_t *)(UID_BASE + 0x08UL))

/* ------------------------------------------------------------------ */
/*  Flash memory map (confirmed from binary constants)                 */
/* ------------------------------------------------------------------ */
#define FLASH_APP_START     0x08000800UL   /* application start        */
#define FLASH_APP_END       0x08003FFFUL   /* application end          */
#define FLASH_APP_NB_PAGES  14U            /* 14 x 1 KB pages          */
#define FLASH_SETTINGS_BASE 0x08003C00UL   /* settings/persistence page*/
#define FLASH_OTA_FLAG_ADDR 0x08003C64UL   /* OTA request flag byte    */
#define FLASH_OTA_FLAG_VAL  0x7FU          /* value requesting OTA     */
/*
 * The binary itself uses the aliased address space (0x00800800..0x00803FFF,
 * step 0x400) as the FLASH->AR page address during erase instead of the
 * canonical 0x08000800..0x08003FFF range (see README "Known Limitations").
 * HAL_FLASHEx_Erase() requires a canonical address (FLASH_BASE..
 * FLASH_BANK1_END), so this reconstruction erases via FLASH_APP_START;
 * the two ranges address the same physical pages on this part.
 */

/* ------------------------------------------------------------------ */
/*  Application vector table layout (at FLASH_APP_START)               */
/* ------------------------------------------------------------------ */
typedef struct {
    uint32_t initial_sp;        /* 0x08000800 */
    uint32_t reset_handler;     /* 0x08000804 */
} AppVectors_t;

/* MSP validation mask: MSP & 0x2FFE0000 must equal 0x20000000        */
#define SRAM_MASK           0x2FFE0000UL
#define SRAM_BASE_MASKED    0x20000000UL

/* ------------------------------------------------------------------ */
/*  I2C slave protocol                                                 */
/* ------------------------------------------------------------------ */
#define BOOTLOADER_I2C_ADDR 0x18U       /* 7-bit slave address        */

/* TIMINGR for 100 kHz standard mode at 48 MHz system clock.          */
/* Confirmed constant 0x00901850 in binary; pass directly to          */
/* LL_I2C_SetTiming().                                                 */
#define I2C_TIMINGR_100KHZ  0x00901850UL

/* ------------------------------------------------------------------ */
/*  OTA protocol                                                       */
/* ------------------------------------------------------------------ */
#define OTA_BLOCK_MARKER     0xFAU      /* start-of-block byte        */
#define OTA_BLOCK_DATA_BYTES 16U        /* data bytes per block       */
#define OTA_BLOCK_TOTAL      17U        /* marker + 16 data bytes     */
#define OTA_BLOCK_HALFWORDS  8U         /* 8 x 16-bit halfword writes */

/* ------------------------------------------------------------------ */
/*  GPIO signal mapping (LL_GPIO_PIN_x bit masks, from binary + app    */
/*  context)                                                            */
/* ------------------------------------------------------------------ */
#define GPIO_PIN_MT_EN      LL_GPIO_PIN_6    /* PA6:  RPi power control   */
#define GPIO_PIN_PWR_EN     LL_GPIO_PIN_7    /* PA7:  MCU self-power hold */
#define GPIO_PIN_BOOT_BTN   LL_GPIO_PIN_1    /* PB1:  force-boot button   */
#define GPIO_PIN_SCL        LL_GPIO_PIN_9    /* PA9:  I2C1 SCL (AF1)      */
#define GPIO_PIN_SDA        LL_GPIO_PIN_10   /* PA10: I2C1 SDA (AF1)      */

/* ------------------------------------------------------------------ */
/*  RAM addresses (confirmed from binary)                              */
/* ------------------------------------------------------------------ */
#define RAM_I2C_STATE_BASE  0x20000008UL    /* I2C state variables    */
#define RAM_SETTINGS_BUF    0x20000010UL    /* flash settings buffer  */
#define RAM_I2C_REGMAP      0x2000010FUL    /* I2C register map[256]  */
#define RAM_I2C_RX_STATE    0x2000012FUL    /* RX state pointer       */
#define RAM_SERIAL_OUT      0x200001EFUL    /* serial/UID base        */

#endif /* STM32F030_PERIPH_H */
