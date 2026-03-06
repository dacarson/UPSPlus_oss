/**
 * stm32f030_periph.h  –  Minimal peripheral definitions for the bootloader.
 *
 * Reconstructed from bootloader.bin disassembly.
 * Only registers actually accessed by the bootloader are listed.
 */

#ifndef STM32F030_PERIPH_H
#define STM32F030_PERIPH_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  Base addresses (confirmed from binary constants)                   */
/* ------------------------------------------------------------------ */
#define FLASH_BASE          0x40022000UL
#define RCC_BASE            0x40021000UL
#define GPIOA_BASE          0x48000000UL
#define GPIOB_BASE          0x48000400UL
#define I2C1_BASE           0x40005400UL
#define NVIC_ISER0          ((volatile uint32_t *)0xE000E100UL)
#define NVIC_IPR5           ((volatile uint32_t *)0xE000E414UL)

/* ------------------------------------------------------------------ */
/*  STM32 Unique Device ID (96 bits, at 0x1FFFF7AC)                    */
/* ------------------------------------------------------------------ */
#define UID_BASE            0x1FFFF780UL   /* base used by bootloader  */
#define UID_WORD0           (*(volatile uint32_t *)(UID_BASE + 0x2C))  /* 0x1FFFF7AC */
#define UID_WORD1           (*(volatile uint32_t *)(UID_BASE + 0x30))  /* 0x1FFFF7B0 */
#define UID_WORD2           (*(volatile uint32_t *)(UID_BASE + 0x34))  /* 0x1FFFF7B4 */

/* ------------------------------------------------------------------ */
/*  Flash memory map                                                   */
/* ------------------------------------------------------------------ */
#define FLASH_APP_START     0x08000800UL   /* application start        */
#define FLASH_APP_END       0x08003FFFUL   /* application end          */
#define FLASH_SETTINGS_BASE 0x08003C00UL   /* settings/persistence page*/
#define FLASH_OTA_FLAG_ADDR 0x08003C64UL   /* OTA request flag byte    */
#define FLASH_OTA_FLAG_VAL  0x7F           /* value requesting OTA     */
#define FLASH_PAGE_SIZE     0x400U         /* 1 KB per page            */
/* Erase uses aliased address space (0x00xxxxxx) matching observed bin */
#define FLASH_ERASE_START   0x00800800UL   /* aliased start for erase  */
#define FLASH_ERASE_END     0x00804000UL   /* aliased end (exclusive)  */

/* Flash KEY1/KEY2 unlock values */
#define FLASH_KEY1          0x45670123UL
#define FLASH_KEY2          0xCDEF89ABUL

/* ------------------------------------------------------------------ */
/*  FLASH register struct (offset from FLASH_BASE = 0x40022000)        */
/* ------------------------------------------------------------------ */
typedef struct {
    volatile uint32_t ACR;      /* 0x00 Access control             */
    volatile uint32_t KEYR;     /* 0x04 Key register               */
    volatile uint32_t OPTKEYR;  /* 0x08 Option key register        */
    volatile uint32_t SR;       /* 0x0C Status register            */
    volatile uint32_t CR;       /* 0x10 Control register           */
    volatile uint32_t AR;       /* 0x14 Address register           */
    volatile uint32_t reserved; /* 0x18                            */
    volatile uint32_t OBR;      /* 0x1C Option byte register       */
    volatile uint32_t WRPR;     /* 0x20 Write protection register  */
} FLASH_TypeDef;

#define FLASH   ((FLASH_TypeDef *)FLASH_BASE)

/* FLASH->CR bits */
#define FLASH_CR_PG         (1U << 0)   /* programming                */
#define FLASH_CR_PER        (1U << 1)   /* page erase                 */
#define FLASH_CR_MER        (1U << 2)   /* mass erase                 */
#define FLASH_CR_STRT       (1U << 6)   /* start erase                */
#define FLASH_CR_LOCK       (1U << 7)   /* lock                       */

/* FLASH->SR bits */
#define FLASH_SR_BSY        (1U << 0)   /* busy                       */

/* ------------------------------------------------------------------ */
/*  RCC register struct (offset from RCC_BASE = 0x40021000)            */
/* ------------------------------------------------------------------ */
typedef struct {
    volatile uint32_t CR;       /* 0x00 Clock control              */
    volatile uint32_t CFGR;     /* 0x04 Clock configuration        */
    volatile uint32_t CIR;      /* 0x08 Clock interrupt            */
    volatile uint32_t APB2RSTR; /* 0x0C APB2 reset                 */
    volatile uint32_t APB1RSTR; /* 0x10 APB1 reset                 */
    volatile uint32_t AHBENR;   /* 0x14 AHB enable                 */
    volatile uint32_t APB2ENR;  /* 0x18 APB2 enable                */
    volatile uint32_t APB1ENR;  /* 0x1C APB1 enable                */
    volatile uint32_t BDCR;     /* 0x20 Backup domain control      */
    volatile uint32_t CSR;      /* 0x24 Control/status             */
    volatile uint32_t AHBRSTR;  /* 0x28 AHB reset                  */
    volatile uint32_t CFGR2;    /* 0x2C Clock configuration 2      */
    volatile uint32_t CFGR3;    /* 0x30 Clock configuration 3      */
    volatile uint32_t CR2;      /* 0x34 Clock control 2            */
} RCC_TypeDef;

#define RCC     ((RCC_TypeDef *)RCC_BASE)

/* RCC->AHBENR bits */
#define RCC_AHBENR_IOPAEN   (1U << 17)  /* GPIOA clock               */
#define RCC_AHBENR_IOPBEN   (1U << 18)  /* GPIOB clock               */

/* RCC->APB2ENR bits */
#define RCC_APB2ENR_SYSCFGEN (1U << 0)  /* SYSCFG clock              */

/* RCC->APB1ENR bits */
#define RCC_APB1ENR_I2C1EN  (1U << 21)  /* I2C1 clock               */
#define RCC_APB1ENR_PWREN   (1U << 28)  /* power interface clock     */

/* RCC->CFGR3 bits */
#define RCC_CFGR3_I2C1SW    (1U << 4)   /* I2C1 clock source        */

/* ------------------------------------------------------------------ */
/*  GPIO register struct                                               */
/* ------------------------------------------------------------------ */
typedef struct {
    volatile uint32_t MODER;    /* 0x00 Mode                       */
    volatile uint32_t OTYPER;   /* 0x04 Output type                */
    volatile uint32_t OSPEEDR;  /* 0x08 Output speed               */
    volatile uint32_t PUPDR;    /* 0x0C Pull-up/pull-down          */
    volatile uint32_t IDR;      /* 0x10 Input data                 */
    volatile uint32_t ODR;      /* 0x14 Output data                */
    volatile uint32_t BSRR;     /* 0x18 Bit set/reset              */
    volatile uint32_t LCKR;     /* 0x1C Lock                       */
    volatile uint32_t AFR[2];   /* 0x20,0x24 Alternate function    */
    volatile uint32_t BRR;      /* 0x28 Bit reset (F0-specific)    */
} GPIO_TypeDef;

#define GPIOA   ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB   ((GPIO_TypeDef *)GPIOB_BASE)

/* GPIO MODER values (2 bits per pin) */
#define GPIO_MODE_INPUT     0U
#define GPIO_MODE_OUTPUT    1U
#define GPIO_MODE_AF        2U
#define GPIO_MODE_ANALOG    3U

/* GPIO OTYPER values */
#define GPIO_OTYPE_PP       0U  /* push-pull                          */
#define GPIO_OTYPE_OD       1U  /* open-drain                         */

/* GPIO OSPEEDR values */
#define GPIO_SPEED_LOW      0U
#define GPIO_SPEED_MEDIUM   1U
#define GPIO_SPEED_HIGH     3U

/* GPIO PUPDR values */
#define GPIO_PUPD_NONE      0U
#define GPIO_PUPD_UP        1U
#define GPIO_PUPD_DOWN      2U

/* Alternate function for I2C1 on PA9/PA10 */
#define GPIO_AF_I2C1        1U  /* AF1 = I2C1 on PA9/PA10 (STM32F030) */

/* ------------------------------------------------------------------ */
/*  I2C register struct (new I2C on STM32F0)                           */
/* ------------------------------------------------------------------ */
typedef struct {
    volatile uint32_t CR1;      /* 0x00 Control 1                  */
    volatile uint32_t CR2;      /* 0x04 Control 2                  */
    volatile uint32_t OAR1;     /* 0x08 Own address 1              */
    volatile uint32_t OAR2;     /* 0x0C Own address 2              */
    volatile uint32_t TIMINGR;  /* 0x10 Timing                     */
    volatile uint32_t TIMEOUTR; /* 0x14 Timeout                    */
    volatile uint32_t ISR;      /* 0x18 Interrupt and status       */
    volatile uint32_t ICR;      /* 0x1C Interrupt clear            */
    volatile uint32_t PECR;     /* 0x20 PEC                        */
    volatile uint32_t RXDR;     /* 0x24 Receive data               */
    volatile uint32_t TXDR;     /* 0x28 Transmit data              */
} I2C_TypeDef;

#define I2C1    ((I2C_TypeDef *)I2C1_BASE)

/* I2C->CR1 bits */
#define I2C_CR1_PE          (1U << 0)   /* peripheral enable          */
#define I2C_CR1_TXIE        (1U << 1)   /* TX interrupt enable        */
#define I2C_CR1_RXIE        (1U << 2)   /* RX interrupt enable        */
#define I2C_CR1_ADDRIE      (1U << 3)   /* address match int enable   */
#define I2C_CR1_STOPIE      (1U << 5)   /* STOP detection int enable  */
#define I2C_CR1_ERRIE       (1U << 7)   /* error interrupt enable     */
#define I2C_CR1_ANFOFF      (1U << 12)  /* analog filter off          */

/* I2C->ISR bits */
#define I2C_ISR_TXE         (1U << 0)   /* TX empty                   */
#define I2C_ISR_TXIS        (1U << 1)   /* TX interrupt status        */
#define I2C_ISR_RXNE        (1U << 2)   /* RX not empty               */
#define I2C_ISR_ADDR        (1U << 3)   /* address match              */
#define I2C_ISR_NACKF       (1U << 4)   /* NACK received              */
#define I2C_ISR_STOPF       (1U << 5)   /* STOP detected              */
#define I2C_ISR_DIR         (1U << 24)  /* transfer direction         */
/* ADDCODE in ISR[23:17] – shifts right by 16 give bits [7:1] */

/* I2C->ICR bits */
#define I2C_ICR_ADDRCF      (1U << 3)   /* clear ADDR flag            */
#define I2C_ICR_NACKCF      (1U << 4)   /* clear NACK flag            */
#define I2C_ICR_STOPCF      (1U << 5)   /* clear STOP flag            */

/* I2C->OAR1 bits */
#define I2C_OAR1_OA1EN      (1U << 15)  /* own address 1 enable       */

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
/*  I2C slave address                                                  */
/* ------------------------------------------------------------------ */
#define BOOTLOADER_I2C_ADDR 0x18U       /* 7-bit slave address        */

/* ------------------------------------------------------------------ */
/*  I2C TIMINGR for 100 kHz standard mode at 48 MHz system clock       */
/*  Confirmed constant 0x00901850 in binary.                           */
/* ------------------------------------------------------------------ */
#define I2C_TIMINGR_100KHZ  0x00901850UL

/* ------------------------------------------------------------------ */
/*  NVIC                                                               */
/* ------------------------------------------------------------------ */
#define NVIC_I2C1_IRQn      23U         /* I2C1 global interrupt      */
/* Bit 23 in NVIC_ISER0 enables I2C1 */
#define NVIC_ISER_I2C1      (1U << 23)
/* NVIC_IPR5 bits[15:8] = priority for IRQ23 (I2C1) */

/* ------------------------------------------------------------------ */
/*  OTA protocol                                                       */
/* ------------------------------------------------------------------ */
#define OTA_BLOCK_MARKER    0xFA        /* start-of-block byte        */
#define OTA_BLOCK_DATA_BYTES 16         /* data bytes per block       */
#define OTA_BLOCK_TOTAL     17          /* marker + 16 data bytes     */
#define OTA_BLOCK_HALFWORDS  8          /* 8 x 16-bit halfword writes */

/* ------------------------------------------------------------------ */
/*  GPIO signal mapping (from binary + app context)                    */
/* ------------------------------------------------------------------ */
#define GPIO_PIN_MT_EN      6U          /* PA6: RPi power control     */
#define GPIO_PIN_PWR_EN     7U          /* PA7: MCU self-power hold   */
#define GPIO_PIN_BOOT_BTN   1U          /* PB1: force-boot button     */
#define GPIO_PIN_SCL        9U          /* PA9: I2C1 SCL (AF1)        */
#define GPIO_PIN_SDA        10U         /* PA10: I2C1 SDA (AF1)       */

/* ------------------------------------------------------------------ */
/*  RAM addresses (confirmed from binary)                              */
/* ------------------------------------------------------------------ */
#define RAM_I2C_STATE_BASE  0x20000008UL    /* I2C state variables    */
#define RAM_SETTINGS_BUF    0x20000010UL    /* flash settings buffer  */
#define RAM_I2C_REGMAP      0x2000010FUL    /* I2C register map[256]  */
#define RAM_I2C_RX_STATE    0x2000012FUL    /* RX state pointer       */
#define RAM_SERIAL_OUT      0x200001EFUL    /* serial/UID base        */

/* System clock (48 MHz – confirmed by TIMINGR constant) */
#define SYSCLK_HZ           48000000UL

#endif /* STM32F030_PERIPH_H */
