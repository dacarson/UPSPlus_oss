/**
 * bootloader.c  –  Reconstructed from bootloader.bin
 *
 * Approximate C source for the UPSPlus STM32F030F4P6 bootloader.
 * Reconstructed by disassembly of bootloader.bin (2048 bytes,
 * loaded at 0x08000000–0x080007FF).
 *
 * Confidence levels:
 *   HIGH  – logic verified directly from disassembly trace
 *   MED   – structure inferred; details may differ
 *   LOW   – plausible but not directly confirmed
 *
 * Key behaviour (matches UPSPlus_Behavior_Spec.md §14):
 *   1. Always asserts PWR_EN (PA7) HIGH to keep power.
 *   2. Checks OTA flag at 0x08003C64 == 0x7F  → OTA mode.
 *   3. Checks force-boot button (PB1 == 0)   → OTA mode.
 *   4. Validates app MSP (must be 0x2000xxxx) → OTA or boot.
 *   5. In OTA mode:
 *        a. Assert MT_EN (PA6) HIGH (keep RPi powered during OTA).
 *        b. Load/preserve settings from 0x08003C00 into RAM.
 *        c. Unlock flash.
 *        d. Erase app flash: 0x00800800..0x00803FFF (14 × 1 KB pages).
 *        e. Save settings back to 0x08003C00 (preserves UID copy).
 *        f. Receive I2C blocks (0xFA + 16 bytes) and program flash.
 *        g. Long timeout waiting for I2C data; return on timeout.
 *   6. Boot app: load MSP, set MSP, BLX to app reset handler.
 */

#include <stdint.h>
#include "stm32f030_periph.h"

/* ------------------------------------------------------------------ */
/*  RAM-resident I2C state (confirmed layout from disassembly)         */
/* ------------------------------------------------------------------ */

/*
 * State block at 0x20000008 (6 bytes, confirmed from ISR r2 = 0x20000008):
 *   [0] uint8_t rx_index    – current receive buffer index (ISR: [r2, #0])
 *
 * Register map at 0x2000010F (256 bytes, ISR r3 = 0x2000010F):
 *   reg_map[0x00]         – bootloader mode/state (returned on read)
 *   reg_map[0x20]         – unknown (byte set to 0 in load_settings)
 *   reg_map[0x32] (alias 0x2000010F+0x20+0x12 = [r0,#18]) – receive status
 *                           0xC8 = waiting, 0–127 = bytes received
 *   reg_map[0xF0..0xFB]   – MCU unique ID (12 bytes, written by main)
 *
 * The I2C register map doubles as the OTA receive buffer:
 *   reg_map[0x20]         = 0xFA block start byte (when receiving)
 *   reg_map[0x21..0x30]   = 16 data bytes (8 halfwords to program)
 */

/* Alias pointers to confirmed RAM addresses (HIGH confidence) */
static volatile uint8_t  *const rx_index_ptr =
    (volatile uint8_t *)0x20000008UL;

static volatile uint8_t  *const i2c_reg_map =
    (volatile uint8_t *)0x2000010FUL;

/* Offset within reg_map where receive status byte lives:            */
/*   0x2000012F - 0x2000010F = 0x20, then [r4+18] = 0x20+18 = 0x32  */
/*   This byte is set to 0xC8 (waiting) and decrements as data arrives */
#define REGMAP_RXSTATUS_OFF  0x32U   /* offset from i2c_reg_map base  */

/* OTA receive buffer starts at reg_map[0x20] (= 0x2000012F - 0x20)  */
/* First byte [0x20] = 0xFA marker; [0x21]..[0x30] = 16 data bytes   */
#define REGMAP_OTA_BUF_OFF   0x20U

/* ------------------------------------------------------------------ */
/*  Forward declarations                                               */
/* ------------------------------------------------------------------ */
static void     gpio_set_mode(GPIO_TypeDef *gpio, uint32_t pin_mask, uint32_t mode);
static void     gpio_set_ospeedr(GPIO_TypeDef *gpio, uint32_t pin_mask, uint32_t speed);
static void     gpio_set_pupdr(GPIO_TypeDef *gpio, uint32_t pin_mask, uint32_t pupd);
static void     gpio_set_afr(GPIO_TypeDef *gpio, uint32_t pin_mask, uint32_t af);
static void     rcc_enable_clock(uint32_t ahbenr_bit);
static void     i2c1_gpio_init(void);
static void     i2c1_peripheral_init(void);
static void     i2c1_full_init(void);
static void     load_settings_from_flash(void);
static void     save_settings_to_flash(void);
static void     flash_unlock(void);
static void     flash_erase_app(void);
static void     bootloader_main(void);
void            I2C1_IRQHandler(void);
int             main(void);

/* ------------------------------------------------------------------ */
/*  GPIO helpers  (0x0800031C / 0x08000300 / 0x08000330 / 0x08000344) */
/*                                                                     */
/*  All four helpers share the same "squared pin mask" trick:          */
/*    pin_mask (single-bit) squared gives bit-0 of the 2-bit field.   */
/*  Example: pin 6 → mask=0x40, mask²=0x1000 (bit 12 of MODER).      */
/*                                                                     */
/*  Confidence: HIGH – directly traced from disassembly.              */
/* ------------------------------------------------------------------ */

/** Set 2-bit MODER field for the pin described by pin_mask.         */
static void gpio_set_mode(GPIO_TypeDef *gpio, uint32_t pin_mask, uint32_t mode)
{
    /* mask²   = bit-0 of 2-bit MODER field for this pin             */
    /* 3×mask² = both bits of the 2-bit field (clear mask)           */
    uint32_t sq   = pin_mask * pin_mask;
    uint32_t bits = sq + 2U * sq;       /* = 3 * sq                  */
    uint32_t val  = gpio->MODER;
    val &= ~bits;
    val |= sq * mode;
    gpio->MODER = val;
}

/** Set 2-bit OSPEEDR field for the pin described by pin_mask.       */
static void gpio_set_ospeedr(GPIO_TypeDef *gpio, uint32_t pin_mask, uint32_t speed)
{
    /* Note: disassembly at 0x8000300 first right-shifts pin_mask by 8
     * before squaring, suggesting pin_mask here is the raw IDR bit
     * shifted so its square falls in the correct OSPEEDR field.
     * For pins 0–7 this matches; the GPIO_configure_pin wrapper
     * accounts for the field width. (MED confidence on exact encoding) */
    uint32_t p    = pin_mask >> 8U;
    uint32_t sq   = p * p * p * p;      /* p^4 lands in OSPEEDR field */
    uint32_t bits = sq + 2U * sq;
    uint32_t val  = gpio->OSPEEDR;
    val &= ~bits;
    val |= sq * speed;
    gpio->OSPEEDR = val;
}

/** Set 2-bit PUPDR field for the pin described by pin_mask.         */
static void gpio_set_pupdr(GPIO_TypeDef *gpio, uint32_t pin_mask, uint32_t pupd)
{
    uint32_t sq   = pin_mask * pin_mask;
    uint32_t bits = sq + 2U * sq;
    uint32_t val  = gpio->PUPDR;
    val &= ~bits;
    val |= sq * pupd;
    gpio->PUPDR = val;
}

/** Set 4-bit AFR field for the pin described by pin_mask.           */
static void gpio_set_afr(GPIO_TypeDef *gpio, uint32_t pin_mask, uint32_t af)
{
    /* 4-bit AFR field: mask = 15 × sq, val = sq × af (MED) */
    uint32_t sq   = pin_mask * pin_mask;
    uint32_t bits = sq + 2U * sq;
    /* Select AFRL vs AFRH based on whether pin is 0-7 or 8-15 */
    volatile uint32_t *afr;
    if (pin_mask <= (1U << 7)) {
        afr = &gpio->AFR[0];
    } else {
        afr = &gpio->AFR[1];
        sq >>= 16;              /* shift into AFRH field position    */
        bits >>= 16;
    }
    uint32_t val = *afr;
    val &= ~bits;
    val |= sq * af;
    *afr = val;
}

/* ------------------------------------------------------------------ */
/*  RCC clock enable  (0x0800023C)                                    */
/*                                                                     */
/*  Writes bit to RCC->AHBENR and reads back.                          */
/*  (Confidence: HIGH)                                                 */
/* ------------------------------------------------------------------ */
static void rcc_enable_clock(uint32_t ahbenr_bit)
{
    RCC->AHBENR |= ahbenr_bit;
    (void)RCC->AHBENR;          /* read-back to ensure write completes */
}

/* ------------------------------------------------------------------ */
/*  I2C1 GPIO init  (0x080003EC)                                       */
/*                                                                     */
/*  Configures PA9 (SCL) and PA10 (SDA) as AF open-drain.             */
/*  Alternate function 1 (I2C1) on STM32F030F4P6.                     */
/*  (Confidence: HIGH for structure, MED for exact AF number)          */
/* ------------------------------------------------------------------ */
static void i2c1_gpio_init(void)
{
    const uint32_t scl_mask = (1U << GPIO_PIN_SCL);   /* PA9  = 0x200 */
    const uint32_t sda_mask = (1U << GPIO_PIN_SDA);   /* PA10 = 0x400 */

    /* SCL: AF mode, high speed, no pull, open-drain, AF1 */
    gpio_set_mode(GPIOA, scl_mask, GPIO_MODE_AF);
    gpio_set_ospeedr(GPIOA, scl_mask, GPIO_SPEED_HIGH);
    gpio_set_afr(GPIOA, scl_mask, GPIO_AF_I2C1);
    GPIOA->OTYPER |=  scl_mask;        /* open-drain                  */
    gpio_set_pupdr(GPIOA, scl_mask, GPIO_PUPD_NONE);

    /* SDA: AF mode, high speed, no pull, open-drain, AF1 */
    gpio_set_mode(GPIOA, sda_mask, GPIO_MODE_AF);
    gpio_set_ospeedr(GPIOA, sda_mask, GPIO_SPEED_HIGH);
    gpio_set_afr(GPIOA, sda_mask, GPIO_AF_I2C1);
    GPIOA->OTYPER |=  sda_mask;
    gpio_set_pupdr(GPIOA, sda_mask, GPIO_PUPD_NONE);
}

/* ------------------------------------------------------------------ */
/*  I2C1 peripheral init  (0x080003EC continuation + 0x8000384 body)  */
/*                                                                     */
/*  Sets up the I2C1 peripheral as a slave at address 0x18.           */
/*  Timing: 100 kHz standard mode (TIMINGR = 0x00901850 at 48 MHz).   */
/*  Enables NVIC IRQ 23 (I2C1) with priority from NVIC_IPR5.          */
/*  (Confidence: HIGH for register values; MED for exact init order)   */
/* ------------------------------------------------------------------ */
static void i2c1_peripheral_init(void)
{
    /* Enable I2C1 clock on APB1 */
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    (void)RCC->APB1ENR;

    /* Select HSI as I2C1 clock source (clear I2C1SW in CFGR3) */
    RCC->CFGR3 &= ~RCC_CFGR3_I2C1SW;

    /* Disable I2C1 to configure it */
    I2C1->CR1 &= ~I2C_CR1_PE;

    /* Wait for PE to clear */
    while (I2C1->CR1 & I2C_CR1_PE) {}

    /* Apply timing: 100 kHz at 48 MHz */
    I2C1->TIMINGR = I2C_TIMINGR_100KHZ;

    /* Configure own address 1: 7-bit, address = 0x18, enable */
    I2C1->OAR1 = (BOOTLOADER_I2C_ADDR << 1) | I2C_OAR1_OA1EN;

    /* Enable interrupts: ADDRIE, RXIE, TXIE, STOPIE */
    I2C1->CR1 |= I2C_CR1_ADDRIE | I2C_CR1_RXIE | I2C_CR1_TXIE | I2C_CR1_STOPIE;

    /* Enable I2C1 */
    I2C1->CR1 |= I2C_CR1_PE;

    /* Configure NVIC: enable IRQ23 (I2C1) */
    *NVIC_ISER0 = NVIC_ISER_I2C1;

    /* Set I2C1 interrupt priority in NVIC_IPR5
     * (IPR5 covers IRQs 20-23; I2C1=IRQ23 is in bits[31:24]) */
    *NVIC_IPR5 |= (0x80U << 24);   /* medium priority (MED confidence) */
}

/* ------------------------------------------------------------------ */
/*  Full GPIO + I2C + NVIC init  (0x08000384)                          */
/*                                                                     */
/*  Also sets up PA6 (MT_EN) LOW, PA7 (PWR_EN) HIGH, PB1 (button)     */
/*  as input. This is the first-called init from main().               */
/*  (Confidence: HIGH)                                                 */
/* ------------------------------------------------------------------ */
static void i2c1_full_init(void)
{
    /* Enable GPIO clocks */
    rcc_enable_clock(RCC_AHBENR_IOPAEN);
    rcc_enable_clock(RCC_AHBENR_IOPBEN);

    /* PA6 (MT_EN): drive LOW initially (RPi power off during boot)   */
    GPIOA->BRR = (1U << GPIO_PIN_MT_EN);

    /* PA7 (PWR_EN): drive HIGH immediately (keep MCU powered)        */
    GPIOA->BSRR = (1U << GPIO_PIN_PWR_EN);

    /* PB1 (force-boot button): input, no pull                        */
    gpio_set_mode(GPIOB, (1U << GPIO_PIN_BOOT_BTN), GPIO_MODE_INPUT);
    gpio_set_pupdr(GPIOB, (1U << GPIO_PIN_BOOT_BTN), GPIO_PUPD_NONE);

    /* PA6 (MT_EN): output push-pull                                  */
    gpio_set_mode(GPIOA, (1U << GPIO_PIN_MT_EN), GPIO_MODE_OUTPUT);

    /* PA7 (PWR_EN): output push-pull                                 */
    gpio_set_mode(GPIOA, (1U << GPIO_PIN_PWR_EN), GPIO_MODE_OUTPUT);

    /* Configure I2C GPIO pins */
    i2c1_gpio_init();

    /* Initialise I2C1 peripheral */
    i2c1_peripheral_init();
}

/* ------------------------------------------------------------------ */
/*  load_settings_from_flash  (0x08000134)                             */
/*                                                                     */
/*  Reads the settings page (0x08003C00) into the I2C register map    */
/*  RAM buffer (0x20000010).  Flash is stored as halfwords (one        */
/*  logical byte per 16-bit word, LSB only), so the read advances      */
/*  the flash pointer by 2 for each RAM byte.                          */
/*                                                                     */
/*  Stops early if the first byte reads 0xFF (erased flash).           */
/*  After copying, clears byte at reg_map[0x32] (rx status) to 0.      */
/*                                                                     */
/*  Confidence: HIGH                                                   */
/* ------------------------------------------------------------------ */
static void load_settings_from_flash(void)
{
    const volatile uint16_t *flash_ptr =
        (const volatile uint16_t *)FLASH_SETTINGS_BASE;
    volatile uint8_t *ram_ptr = (volatile uint8_t *)RAM_SETTINGS_BUF;
    uint8_t  i = 0;

    /* Check if flash is erased (first byte == 0xFF) */
    if ((uint8_t)flash_ptr[0] == 0xFFU) {
        goto done;
    }

    /* Copy up to 255 bytes: read 16-bit flash word, store LSB byte   */
    do {
        ram_ptr[i] = (uint8_t)flash_ptr[i];   /* only LSB of halfword */
        i++;
    } while (i < 0xFFU);

done:
    /* Clear receive-status byte in register map */
    i2c_reg_map[REGMAP_RXSTATUS_OFF] = 0U;
}

/* ------------------------------------------------------------------ */
/*  flash_unlock  (helper, inlined from call at 0x08000614)            */
/*                                                                     */
/*  Unlocks flash programming if LOCK bit is set.                      */
/*  Confidence: HIGH                                                   */
/* ------------------------------------------------------------------ */
static void flash_unlock(void)
{
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
}

/* ------------------------------------------------------------------ */
/*  save_settings_to_flash  (0x080004EC)                               */
/*                                                                     */
/*  Erases the settings page (0x08003C00) and reprograms it from the   */
/*  RAM buffer (0x20000010), writing each byte as a 16-bit halfword.   */
/*  This preserves the MCU UID copy (serial number) across OTA.        */
/*                                                                     */
/*  The binary uses 0x08003C00 as the page address for FLASH->AR.      */
/*  Confidence: HIGH                                                   */
/* ------------------------------------------------------------------ */
static void save_settings_to_flash(void)
{
    volatile uint16_t *flash_dst = (volatile uint16_t *)FLASH_SETTINGS_BASE;
    const volatile uint8_t *ram_src = (const volatile uint8_t *)RAM_SETTINGS_BUF;
    uint8_t  i = 0;

    /* Erase the settings page */
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR  = FLASH_SETTINGS_BASE;
    FLASH->CR |= FLASH_CR_STRT;
    while (FLASH->SR & FLASH_SR_BSY) {}
    FLASH->CR &= ~FLASH_CR_PER;

    /* Program 255 bytes from RAM as 16-bit halfwords */
    do {
        FLASH->CR |= FLASH_CR_PG;
        flash_dst[i] = (uint16_t)ram_src[i];  /* each byte → halfword */
        while (FLASH->SR & FLASH_SR_BSY) {}
        FLASH->CR &= ~FLASH_CR_PG;
        i++;
    } while (i < 0xFFU);

    /* Lock flash */
    FLASH->CR |= FLASH_CR_LOCK;
}

/* ------------------------------------------------------------------ */
/*  flash_erase_app  (inlined in bootloader_main at 0x08000624)        */
/*                                                                     */
/*  Erases all 14 × 1 KB pages from 0x08000800 to 0x08003FFF.         */
/*  The binary uses the aliased address space 0x00800800..0x00804000   */
/*  for FLASH->AR; the step is 0x400 (page size).                      */
/*                                                                     */
/*  Confidence: HIGH (addresses confirmed from binary)                 */
/* ------------------------------------------------------------------ */
static void flash_erase_app(void)
{
    uint32_t page_addr = FLASH_ERASE_START;  /* 0x00800800 (aliased)  */

    while (page_addr < FLASH_ERASE_END) {    /* 0x00804000 (aliased)  */
        FLASH->CR |= FLASH_CR_PER;
        FLASH->AR  = page_addr;
        FLASH->CR |= FLASH_CR_STRT;
        while (FLASH->SR & FLASH_SR_BSY) {}
        FLASH->CR &= ~FLASH_CR_PER;
        page_addr += FLASH_PAGE_SIZE;        /* advance by 1 KB       */
    }
}

/* ------------------------------------------------------------------ */
/*  bootloader_main  (0x080005FC)                                       */
/*                                                                     */
/*  Main OTA state machine:                                            */
/*    1. Keep MT_EN HIGH (RPi stays powered during OTA).               */
/*    2. Initialize: set rx-status = 0xC8 (waiting).                  */
/*    3. Load settings from flash (preserves UID in reg map).          */
/*    4. Unlock flash.                                                 */
/*    5. Erase app flash (0x08000800–0x08003FFF).                      */
/*    6. Save settings back (protect UID from erase).                  */
/*    7. Set flash write pointer to 0x08000800.                        */
/*    8. Receive OTA blocks via I2C ISR; program each to flash.        */
/*    9. Long timeout (~minutes) waiting for data; return on timeout.  */
/*                                                                     */
/*  The I2C ISR deposits received bytes into the register map buffer.  */
/*  This function reads the rx-status byte to detect complete blocks   */
/*  (status drops from 0xC8 down to 0 as bytes arrive), then programs  */
/*  the 16-byte block to flash as 8 halfwords.                         */
/*                                                                     */
/*  Confidence: HIGH for structure; MED for exact timeout loop         */
/* ------------------------------------------------------------------ */
static void bootloader_main(void)
{
    /* Keep RPi power on during OTA (PA6 = MT_EN HIGH) */
    GPIOA->BSRR = (1U << GPIO_PIN_MT_EN);

    /* Initialise: set rx-status byte to 0xC8 (waiting for data) */
    i2c_reg_map[REGMAP_RXSTATUS_OFF] = 0xC8U;

    /* Copy settings page into RAM (preserves UID across erase) */
    load_settings_from_flash();

    /* Unlock flash for programming */
    flash_unlock();

    /* Erase all application flash pages */
    flash_erase_app();

    /* Save settings (UID etc.) back to settings page */
    save_settings_to_flash();

    /* Flash write pointer in RAM at 0x20000000                       */
    volatile uint32_t *flash_ptr_ram = (volatile uint32_t *)0x20000000UL;
    *flash_ptr_ram = FLASH_APP_START;   /* start writing at app base  */

    /* ---- Main receive / program loop ---- */
    /* The I2C ISR writes incoming bytes into i2c_reg_map[REGMAP_OTA_BUF_OFF].
     * The rx-status byte at [REGMAP_RXSTATUS_OFF] starts at 0xC8.
     * When a complete 17-byte block arrives (marker 0xFA + 16 data bytes),
     * the ISR triggers and the status transitions to indicate a block ready.
     */
    for (;;) {
        uint8_t status = i2c_reg_map[REGMAP_RXSTATUS_OFF];

        if (status == OTA_BLOCK_MARKER) {
            /* Complete block received: program 8 halfwords to flash  */
            FLASH->CR |= FLASH_CR_PG;

            uint32_t flash_write_addr = *flash_ptr_ram;
            volatile uint16_t *flash_hw = (volatile uint16_t *)flash_write_addr;

            /* OTA buffer: [0x20] = 0xFA, [0x21]=lo0,[0x22]=hi0, ... */
            const uint8_t *buf = (const uint8_t *)
                (RAM_I2C_REGMAP + REGMAP_OTA_BUF_OFF);

            for (uint8_t hw = 0; hw < OTA_BLOCK_DATA_BYTES; hw += 2) {
                uint16_t halfword = (uint16_t)buf[hw + 1] |
                                    ((uint16_t)buf[hw + 2] << 8);
                *flash_hw++ = halfword;
                while (FLASH->SR & FLASH_SR_BSY) {}
                FLASH->CR &= ~FLASH_CR_PG;
                FLASH->CR |= FLASH_CR_PG;
            }

            FLASH->CR &= ~FLASH_CR_PG;

            /* Advance flash write pointer by 16 bytes */
            *flash_ptr_ram = flash_write_addr + OTA_BLOCK_DATA_BYTES;

            /* Reset rx-status for next block */
            i2c_reg_map[REGMAP_RXSTATUS_OFF] = 0xC8U;

        } else if (status == 0U || status > 0x7FU) {
            /* 0 = no data / command; >0x7F = still waiting (0xC8).
             * If 0xC8 (waiting), stay in loop.
             * If 0, fall through to timeout wait.               */
            if (status == 0U) {
                /* No OTA data arrived – do a long delay then return  */
                save_settings_to_flash();

                /* Long timeout loop (~minutes at Cortex-M0 at 8 MHz) */
                for (uint8_t outer = 0; outer < 255U; outer++) {
                    volatile uint32_t *ctr = (volatile uint32_t *)0x20000000UL;
                    *ctr = 0;
                    do {
                        (*ctr)++;
                        (*ctr)++;
                    } while (*ctr != 0U);   /* overflows → exits      */
                }
                return;  /* return to main() which will re-evaluate   */
            }
            /* else: 0xC8 or other waiting value; keep looping        */
        }
    }
}

/* ------------------------------------------------------------------ */
/*  I2C1_IRQHandler  (0x08000164)                                      */
/*                                                                     */
/*  I2C slave interrupt handler.  Services ADDR match, TXE/TXIS        */
/*  (master reading a register), RXNE (master writing), and STOPF.     */
/*                                                                     */
/*  Register read (master sends register address, then reads):         */
/*    – On ADDR match (DIR=1, master reading):                         */
/*        Load byte from reg_map[rx_index] into TXDR.                 */
/*    – On TXIS (continue TX):                                         */
/*        Load next byte from reg_map[rx_index++] into TXDR.          */
/*                                                                     */
/*  Register write (master writes register address then data):         */
/*    – On ADDR match (DIR=0, master writing): reset rx_index = 0.    */
/*    – On RXNE: store received byte; first byte sets register pointer,*/
/*        subsequent bytes go into reg_map[reg_ptr++].                 */
/*    – On STOPF: clear flag; optionally act on received data.         */
/*                                                                     */
/*  OTA receive:                                                       */
/*    – Master writes block: 0xFA + 16 data bytes to any register.    */
/*    – ISR stores in reg_map[REGMAP_OTA_BUF_OFF..].                  */
/*    – When 0xFA is detected, rx-status is set to trigger programming.*/
/*                                                                     */
/*  Confidence: HIGH for the overall structure and conditions;         */
/*              MED for exact TXDR/RXDR access patterns and rx_index.  */
/* ------------------------------------------------------------------ */
void I2C1_IRQHandler(void)
{
    uint32_t isr = I2C1->ISR;

    /* ---- Address match (ADDR, bit 3) ---- */
    if (isr & I2C_ISR_ADDR) {
        uint32_t isr2 = I2C1->ISR;     /* fresh read                  */

        /* ADDCODE[6:0] in ISR[23:17]: after >>16, occupies bits[7:1].
         * Address 0x18 → bits[7:1] = 0x18 → byte value = 0x30.
         * Masked with 0xFE to ignore the LSB.                        */
        uint32_t addcode = (isr2 >> 16) & 0xFEU;

        if (addcode == 0x30U) {
            /* Our address 0x18 matched */
            uint32_t dir_bit = isr2 & I2C_ISR_DIR;  /* bit 24 = DIR  */

            if (dir_bit) {
                /* Master wants to READ → slave must TRANSMIT          */
                /* Load first byte from register map into TXDR         */
                uint8_t reg_val = i2c_reg_map[*rx_index_ptr];
                I2C1->TXDR = reg_val;

                /* Clear ADDR flag and enable TXIS */
                I2C1->ICR = I2C_ICR_ADDRCF;
                I2C1->CR1 |= I2C_CR1_TXIE;
                I2C1->CR1 &= ~I2C_CR1_RXIE;
            } else {
                /* Master wants to WRITE → slave receives              */
                /* Reset rx index */
                *rx_index_ptr = 0U;

                /* Clear ADDR flag */
                I2C1->ICR = I2C_ICR_ADDRCF;
            }
        } else {
            /* Address does not match 0x18 – NACK it                  */
            I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_ADDRCF;
        }
        return;
    }

    /* ---- Transmit interrupt (TXIS, bit 1) ---- */
    /* Master is reading; provide next byte from register map          */
    if (isr & I2C_ISR_TXIS) {
        uint8_t idx = *rx_index_ptr;
        *rx_index_ptr = idx + 1U;
        I2C1->TXDR = i2c_reg_map[idx];
        return;
    }

    /* ---- Receive not empty (RXNE, bit 2) ---- */
    if (isr & I2C_ISR_RXNE) {
        uint8_t idx   = *rx_index_ptr;
        uint8_t byte  = (uint8_t)I2C1->RXDR;

        if (idx == 0U) {
            /* First byte = register pointer                           */
            *rx_index_ptr = byte;           /* set register address   */
        } else if (idx <= 0xC8U) {
            /* Subsequent bytes: store at reg_map[idx]                 */
            i2c_reg_map[idx] = byte;
            *rx_index_ptr = idx + 1U;

            /* Detect OTA block start marker                           */
            if (byte == OTA_BLOCK_MARKER && idx == (uint8_t)REGMAP_OTA_BUF_OFF) {
                /* Store marker in status byte for bootloader_main     */
                i2c_reg_map[REGMAP_RXSTATUS_OFF] = OTA_BLOCK_MARKER;
            }
        } else {
            /* Buffer full – load current reg value for TX if needed   */
            I2C1->TXDR = i2c_reg_map[idx];
        }
        return;
    }

    /* ---- Stop condition (STOPF, bit 5) ---- */
    if (isr & I2C_ISR_STOPF) {
        /* Clear STOP flag */
        I2C1->ICR = I2C_ICR_STOPCF;

        /* Disable TX, re-enable RX for next transaction               */
        I2C1->CR1 &= ~(I2C_CR1_TXIE);
        I2C1->CR1 |=   I2C_CR1_RXIE;
    }
}

/* ------------------------------------------------------------------ */
/*  main  (0x0800070C)                                                  */
/*                                                                     */
/*  Entry point called from __init_data after RAM sections are ready.  */
/*                                                                     */
/*  Sequence:                                                          */
/*    1. Enable SYSCFG and power clocks (APB2/APB1).                  */
/*    2. Initialise I2C flash-unlock helper.                           */
/*    3. Initialise I2C1 GPIO, peripheral, and NVIC.                   */
/*    4. Copy MCU UID (96 bits, 3×32-bit words) into I2C reg map       */
/*       at registers 0xF0–0xFB.                                       */
/*    5. Read OTA flag byte at 0x08003C64.                             */
/*    6. Check PB1 (force-boot button, active-low).                   */
/*    7. If OTA flag == 0x7F OR button pressed → enter OTA mode.       */
/*    8. Validate app MSP: must satisfy (MSP & 0x2FFE0000)==0x20000000 */
/*    9. If valid: set MSP from app vector table, BLX to app reset.    */
/*   10. If invalid: enter OTA mode.                                   */
/*                                                                     */
/*  Confidence: HIGH for all decisions; MED for exact clock enables    */
/* ------------------------------------------------------------------ */
int main(void)
{
    /* ---- Clock enables ---- */
    /* Enable SYSCFG (APB2 bit 0) */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;

    /* Enable power interface (APB1 bit 28) */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;

    /* ---- Peripheral init ---- */
    /* (The binary calls two init functions here; the first is likely  */
    /*  the flash access timing setup; the second is i2c1_full_init.)  */
    i2c1_full_init();

    /* ---- Copy MCU UID to I2C register map at 0xF0–0xFB ---- */
    /* UID base 0x1FFFF780; UID words at offsets +0x2C, +0x30, +0x34  */
    {
        /* reg_map[0xF0..0xFB] = UID bytes 0..11 (3 × little-endian words) */
        volatile uint8_t *serial = (volatile uint8_t *)(RAM_SERIAL_OUT + 16U);
        /* = 0x200001EF + 16 = 0x200001FF = i2c_reg_map + 0xF0 ✓     */

        uint32_t uid0 = UID_WORD0;
        serial[0]  = (uint8_t)(uid0);
        serial[1]  = (uint8_t)(uid0 >> 8);
        serial[2]  = (uint8_t)(uid0 >> 16);
        serial[3]  = (uint8_t)(uid0 >> 24);

        uint32_t uid1 = UID_WORD1;
        serial[4]  = (uint8_t)(uid1);
        serial[5]  = (uint8_t)(uid1 >> 8);
        serial[6]  = (uint8_t)(uid1 >> 16);
        serial[7]  = (uint8_t)(uid1 >> 24);

        uint32_t uid2 = UID_WORD2;
        serial[8]  = (uint8_t)(uid2);
        serial[9]  = (uint8_t)(uid2 >> 8);
        serial[10] = (uint8_t)(uid2 >> 16);
        serial[11] = (uint8_t)(uid2 >> 24);
    }

    /* ---- Decide: OTA or boot ---- */
    const volatile uint8_t *settings = (const volatile uint8_t *)FLASH_SETTINGS_BASE;
    uint8_t ota_flag = settings[0x64];  /* byte at 0x08003C64         */

    /* Case 1: OTA flag == 0x7F (set by application register 0xFC)    */
    if (ota_flag == FLASH_OTA_FLAG_VAL) {
        bootloader_main();
        /* Falls through after timeout to re-evaluate below           */
    }

    /* Case 2: Force-boot button PB1 pressed (active-low)             */
    {
        uint32_t idr = GPIOB->IDR;
        if (!(idr & (1U << GPIO_PIN_BOOT_BTN))) {
            /* Button pressed (IDR bit 1 = 0) */
            bootloader_main();
        }
    }

    /* ---- Validate app vector table ---- */
    /* App must have a valid MSP in the SRAM range 0x2000xxxx          */
    const volatile uint32_t *app_vectors =
        (const volatile uint32_t *)FLASH_APP_START;

    uint32_t app_msp  = app_vectors[0];   /* initial SP at 0x08000800 */
    uint32_t app_rst  = app_vectors[1];   /* reset handler at 0x08000804 */

    if ((app_msp & SRAM_MASK) == SRAM_BASE_MASKED) {
        /* Valid SRAM address – boot the application                   */
        /* Store reset vector in RAM scratch area for the BLX          */
        volatile uint32_t *scratch = (volatile uint32_t *)0x20000000UL;
        scratch[1] = app_rst;

        /* Set MSP to app's initial stack pointer */
        __asm volatile (
            "MSR MSP, %0 \n"
            : : "r" (app_msp) : "memory"
        );

        /* Jump to app reset handler (no return) */
        typedef void (*app_entry_t)(void);
        app_entry_t app_start = (app_entry_t)app_rst;
        app_start();
    }

    /* App vector table invalid – stay in OTA mode indefinitely        */
    while (1) {
        bootloader_main();
    }
}
