/**
 * bootloader.c  –  Reconstructed from bootloader.bin
 *
 * Approximate C source for the UPSPlus STM32F030F4P6 bootloader.
 * Reconstructed by disassembly of bootloader.bin (2048 bytes,
 * loaded at 0x08000000–0x080007FF).
 *
 * This revision expresses the reconstruction in terms of the official
 * ST CMSIS device header and HAL/LL drivers instead of hand-rolled
 * peripheral register structs:
 *   stm32f0xx.h                 – CMSIS GPIO/I2C/FLASH/RCC/NVIC definitions
 *   stm32f0xx_ll_gpio.h          – pin mode/speed/pull/AF configuration
 *   stm32f0xx_ll_i2c.h           – register-level slave I2C access
 *   stm32f0xx_hal_flash.h/_ex.h  – unlock/lock/program/erase
 *   stm32f0xx_hal_cortex.h       – NVIC priority/enable
 *   stm32f0xx_hal_rcc.h          – __HAL_RCC_*_CLK_ENABLE() peripheral clocks
 *
 * Doing this against the real headers turned up one confirmed bug in the
 * earlier hand-rolled reconstruction: I2C_ISR_DIR is bit 16 in the real
 * ISR register, not bit 24. See the note in I2C1_IRQHandler() below.
 *
 * This file is a reference reconstruction, not a build target: it lives
 * under documents/ and is not compiled as part of the CubeIDE project.
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
#include "stm32f0xx.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_i2c.h"
#include "stm32f0xx_hal_flash.h"
#include "stm32f0xx_hal_flash_ex.h"
#include "stm32f0xx_hal_cortex.h"
#include "stm32f0xx_hal_rcc.h"
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
static void     i2c1_gpio_init(void);
static void     i2c1_peripheral_init(void);
static void     i2c1_full_init(void);
static void     load_settings_from_flash(void);
static void     save_settings_to_flash(void);
static void     flash_erase_app(void);
static void     bootloader_main(void);
void            I2C1_IRQHandler(void);
int             main(void);

/* ------------------------------------------------------------------ */
/*  I2C1 GPIO init  (0x080003EC)                                       */
/*                                                                     */
/*  Configures PA9 (SCL) and PA10 (SDA) as AF open-drain, AF1 (I2C1).  */
/*  The hand-rolled bit-twiddling helpers this reconstruction used to  */
/*  have (gpio_set_mode/ospeedr/pupdr/afr) computed exactly the same   */
/*  "pin-squared" MODER/OSPEEDR/PUPDR field math as LL_GPIO_SetPinMode /*
/*  SetPinSpeed / SetPinPull, and the same pin^4 AFR math as           */
/*  LL_GPIO_SetAFPin_8_15 — strong evidence the original firmware was  */
/*  built against these LL_GPIO_* calls rather than raw MODIFY_REG.    */
/*  (Confidence: HIGH for structure, MED for exact AF number)          */
/* ------------------------------------------------------------------ */
static void i2c1_gpio_init(void)
{
    /* SCL: AF mode, high speed, open-drain, no pull, AF1 (I2C1) */
    LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_SCL, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(GPIOA, GPIO_PIN_SCL, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetAFPin_8_15(GPIOA, GPIO_PIN_SCL, LL_GPIO_AF_1);
    LL_GPIO_SetPinOutputType(GPIOA, GPIO_PIN_SCL, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinPull(GPIOA, GPIO_PIN_SCL, LL_GPIO_PULL_NO);

    /* SDA: AF mode, high speed, open-drain, no pull, AF1 (I2C1) */
    LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_SDA, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(GPIOA, GPIO_PIN_SDA, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetAFPin_8_15(GPIOA, GPIO_PIN_SDA, LL_GPIO_AF_1);
    LL_GPIO_SetPinOutputType(GPIOA, GPIO_PIN_SDA, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinPull(GPIOA, GPIO_PIN_SDA, LL_GPIO_PULL_NO);
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
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* Select HSI as I2C1 clock source (clear I2C1SW in CFGR3) */
    __HAL_RCC_I2C1_CONFIG(RCC_I2C1CLKSOURCE_HSI);

    /* Disable I2C1 to configure it */
    LL_I2C_Disable(I2C1);
    while (LL_I2C_IsEnabled(I2C1)) {}

    /* Apply timing: 100 kHz at 48 MHz */
    LL_I2C_SetTiming(I2C1, I2C_TIMINGR_100KHZ);

    /* Configure own address 1: 7-bit, address = 0x18, enable */
    LL_I2C_SetOwnAddress1(I2C1, (BOOTLOADER_I2C_ADDR << 1), LL_I2C_OWNADDRESS1_7BIT);
    LL_I2C_EnableOwnAddress1(I2C1);

    /* Enable interrupts: ADDRIE, RXIE, TXIE, STOPIE */
    LL_I2C_EnableIT_ADDR(I2C1);
    LL_I2C_EnableIT_RX(I2C1);
    LL_I2C_EnableIT_TX(I2C1);
    LL_I2C_EnableIT_STOP(I2C1);

    /* Enable I2C1 */
    LL_I2C_Enable(I2C1);

    /* NVIC: enable IRQ23 (I2C1) at medium priority (0x80 in the original
     * 8-bit NVIC_IPR5 field == priority level 2 of 0-3 on Cortex-M0). */
    HAL_NVIC_SetPriority(I2C1_IRQn, 2U, 0U);
    HAL_NVIC_EnableIRQ(I2C1_IRQn);
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
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PA6 (MT_EN): drive LOW initially (RPi power off during boot)   */
    LL_GPIO_ResetOutputPin(GPIOA, GPIO_PIN_MT_EN);

    /* PA7 (PWR_EN): drive HIGH immediately (keep MCU powered)        */
    LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_PWR_EN);

    /* PB1 (force-boot button): input, no pull                        */
    LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_BOOT_BTN, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOB, GPIO_PIN_BOOT_BTN, LL_GPIO_PULL_NO);

    /* PA6 (MT_EN): output push-pull                                  */
    LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_MT_EN, LL_GPIO_MODE_OUTPUT);

    /* PA7 (PWR_EN): output push-pull                                 */
    LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_PWR_EN, LL_GPIO_MODE_OUTPUT);

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
/*  This is a plain memory read, not a FLASH-controller operation, so  */
/*  it has no HAL/LL equivalent — direct pointer access is idiomatic.  */
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
/*  save_settings_to_flash  (0x080004EC)                               */
/*                                                                     */
/*  Erases the settings page (0x08003C00) and reprograms it from the   */
/*  RAM buffer (0x20000010), writing each byte as a 16-bit halfword.   */
/*  This preserves the MCU UID copy (serial number) across OTA.        */
/*                                                                     */
/*  Confidence: HIGH                                                   */
/* ------------------------------------------------------------------ */
static void save_settings_to_flash(void)
{
    volatile uint16_t *flash_dst = (volatile uint16_t *)FLASH_SETTINGS_BASE;
    const volatile uint8_t *ram_src = (const volatile uint8_t *)RAM_SETTINGS_BUF;
    uint8_t  i = 0;

    /* Erase the settings page */
    FLASH_EraseInitTypeDef erase_init = {
        .TypeErase   = FLASH_TYPEERASE_PAGES,
        .PageAddress = FLASH_SETTINGS_BASE,
        .NbPages     = 1U,
    };
    uint32_t page_error = 0U;
    (void)HAL_FLASHEx_Erase(&erase_init, &page_error);

    /* Program 255 bytes from RAM as 16-bit halfwords */
    do {
        (void)HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                                 (uint32_t)&flash_dst[i],
                                 (uint16_t)ram_src[i]);
        i++;
    } while (i < 0xFFU);

    /* Lock flash */
    (void)HAL_FLASH_Lock();
}

/* ------------------------------------------------------------------ */
/*  flash_erase_app  (inlined in bootloader_main at 0x08000624)        */
/*                                                                     */
/*  Erases all 14 x 1 KB pages covering 0x08000800..0x08003FFF.       */
/*  The binary itself uses the aliased address space 0x00800800..     */
/*  0x00804000 for FLASH->AR; HAL_FLASHEx_Erase requires the canonical */
/*  0x08xxxxxx range, which addresses the same physical pages.         */
/*                                                                     */
/*  Confidence: HIGH (addresses confirmed from binary)                 */
/* ------------------------------------------------------------------ */
static void flash_erase_app(void)
{
    FLASH_EraseInitTypeDef erase_init = {
        .TypeErase   = FLASH_TYPEERASE_PAGES,
        .PageAddress = FLASH_APP_START,
        .NbPages     = FLASH_APP_NB_PAGES,
    };
    uint32_t page_error = 0U;

    (void)HAL_FLASHEx_Erase(&erase_init, &page_error);
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
/*  NOTE (HIGH confidence, preserved from disassembly): save_settings_ */
/*  to_flash() re-locks FLASH->CR at its end, and nothing in the OTA   */
/*  receive loop below calls HAL_FLASH_Unlock() again before           */
/*  programming blocks. On real silicon a locked FLASH_CR ignores      */
/*  writes to PG, so as reconstructed here OTA data would not actually */
/*  land in flash. This is called out as a MED-confidence detail in    */
/*  the original disassembly notes; it is reproduced as observed       */
/*  rather than "fixed", since this file documents the shipped binary. */
/*                                                                     */
/*  Confidence: HIGH for structure; MED for exact timeout loop         */
/* ------------------------------------------------------------------ */
static void bootloader_main(void)
{
    /* Keep RPi power on during OTA (PA6 = MT_EN HIGH) */
    LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_MT_EN);

    /* Initialise: set rx-status byte to 0xC8 (waiting for data) */
    i2c_reg_map[REGMAP_RXSTATUS_OFF] = 0xC8U;

    /* Copy settings page into RAM (preserves UID across erase) */
    load_settings_from_flash();

    /* Unlock flash for programming */
    (void)HAL_FLASH_Unlock();

    /* Erase all application flash pages */
    flash_erase_app();

    /* Save settings (UID etc.) back to settings page; re-locks flash */
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
            uint32_t flash_write_addr = *flash_ptr_ram;

            /* OTA buffer: [0x20] = 0xFA, [0x21]=lo0,[0x22]=hi0, ... */
            const uint8_t *buf = (const uint8_t *)
                (RAM_I2C_REGMAP + REGMAP_OTA_BUF_OFF);

            for (uint8_t hw = 0; hw < OTA_BLOCK_DATA_BYTES; hw += 2) {
                uint16_t halfword = (uint16_t)buf[hw + 1] |
                                    ((uint16_t)buf[hw + 2] << 8);
                (void)HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                                         flash_write_addr + hw,
                                         halfword);
            }

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
/*  BUG FIX vs. the earlier hand-rolled reconstruction: I2C_ISR_DIR is */
/*  bit 16 of I2C->ISR on the real STM32F0 (confirmed against          */
/*  stm32f030x6.h / LL_I2C_GetTransferDirection()), not bit 24 as the  */
/*  previous stm32f030_periph.h defined it. The ADDCODE extraction     */
/*  below was already correct (it never used the DIR macro, just a    */
/*  shift+mask), so this only affects the addcode==0x30 dir_bit check, */
/*  now expressed via LL_I2C_GetTransferDirection().                   */
/*                                                                     */
/*  A raw-register slave ISR like this doesn't map cleanly onto the    */
/*  callback-based stm32f0xx_hal_i2c.h slave API (HAL_I2C_EnableListen_*/
/*  IT / AddrCallback / Slave_Seq_*), which is built around HAL's      */
/*  I2C_HandleTypeDef state machine rather than a custom byte-oriented */
/*  register map; stm32f0xx_ll_i2c.h's thin register wrappers are the  */
/*  correct fit here and were used throughout.                         */
/*                                                                     */
/*  Confidence: HIGH for the overall structure and conditions;         */
/*              MED for exact TXDR/RXDR access patterns and rx_index.  */
/* ------------------------------------------------------------------ */
void I2C1_IRQHandler(void)
{
    /* ---- Address match ---- */
    if (LL_I2C_IsActiveFlag_ADDR(I2C1)) {
        /* ADDCODE[6:0] in ISR[23:17]: LL_I2C_GetAddressMatchCode() shifts
         * and left-shifts by 1, giving the byte-form address (0x18 -> 0x30),
         * matching how the disassembly compared against 0x30 directly.     */
        uint32_t addcode = LL_I2C_GetAddressMatchCode(I2C1);

        if (addcode == 0x30U) {
            /* Our address 0x18 matched */
            if (LL_I2C_GetTransferDirection(I2C1) == LL_I2C_DIRECTION_READ) {
                /* Master wants to READ → slave must TRANSMIT          */
                /* Load first byte from register map into TXDR         */
                LL_I2C_TransmitData8(I2C1, i2c_reg_map[*rx_index_ptr]);

                /* Clear ADDR flag and enable TXIS */
                LL_I2C_ClearFlag_ADDR(I2C1);
                LL_I2C_EnableIT_TX(I2C1);
                LL_I2C_DisableIT_RX(I2C1);
            } else {
                /* Master wants to WRITE → slave receives              */
                /* Reset rx index */
                *rx_index_ptr = 0U;

                /* Clear ADDR flag */
                LL_I2C_ClearFlag_ADDR(I2C1);
            }
        } else {
            /* Address does not match 0x18 – NACK it                  */
            LL_I2C_ClearFlag_NACK(I2C1);
            LL_I2C_ClearFlag_ADDR(I2C1);
        }
        return;
    }

    /* ---- Transmit interrupt (TXIS) ---- */
    /* Master is reading; provide next byte from register map          */
    if (LL_I2C_IsActiveFlag_TXIS(I2C1)) {
        uint8_t idx = *rx_index_ptr;
        *rx_index_ptr = idx + 1U;
        LL_I2C_TransmitData8(I2C1, i2c_reg_map[idx]);
        return;
    }

    /* ---- Receive not empty (RXNE) ---- */
    if (LL_I2C_IsActiveFlag_RXNE(I2C1)) {
        uint8_t idx   = *rx_index_ptr;
        uint8_t byte  = LL_I2C_ReceiveData8(I2C1);

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
            LL_I2C_TransmitData8(I2C1, i2c_reg_map[idx]);
        }
        return;
    }

    /* ---- Stop condition (STOPF) ---- */
    if (LL_I2C_IsActiveFlag_STOP(I2C1)) {
        /* Clear STOP flag */
        LL_I2C_ClearFlag_STOP(I2C1);

        /* Disable TX, re-enable RX for next transaction               */
        LL_I2C_DisableIT_TX(I2C1);
        LL_I2C_EnableIT_RX(I2C1);
    }
}

/* ------------------------------------------------------------------ */
/*  main  (0x0800070C)                                                  */
/*                                                                     */
/*  Entry point called from __init_data after RAM sections are ready.  */
/*                                                                     */
/*  Sequence:                                                          */
/*    1. Enable SYSCFG and power clocks (APB2/APB1).                  */
/*    2. Initialise I2C1 GPIO, peripheral, and NVIC.                   */
/*    3. Copy MCU UID (96 bits, 3×32-bit words) into I2C reg map       */
/*       at registers 0xF0–0xFB.                                       */
/*    4. Read OTA flag byte at 0x08003C64.                             */
/*    5. Check PB1 (force-boot button, active-low).                   */
/*    6. If OTA flag == 0x7F OR button pressed → enter OTA mode.       */
/*    7. Validate app MSP: must satisfy (MSP & 0x2FFE0000)==0x20000000 */
/*    8. If valid: set MSP from app vector table, jump to app reset.   */
/*    9. If invalid: enter OTA mode.                                   */
/*                                                                     */
/*  Confidence: HIGH for all decisions; MED for exact clock enables    */
/* ------------------------------------------------------------------ */
int main(void)
{
    /* ---- Clock enables ---- */
    /* Enable SYSCFG (APB2 bit 0) */
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    /* Enable power interface (APB1 bit 28) */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* ---- Peripheral init ---- */
    i2c1_full_init();

    /* ---- Copy MCU UID to I2C register map at 0xF0–0xFB ---- */
    /* UID base 0x1FFFF7AC (CMSIS UID_BASE); UID words at +0x00/+0x04/+0x08 */
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
    if (!LL_GPIO_IsInputPinSet(GPIOB, GPIO_PIN_BOOT_BTN)) {
        bootloader_main();
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

        /* Set MSP to app's initial stack pointer (CMSIS intrinsic) */
        __set_MSP(app_msp);

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
