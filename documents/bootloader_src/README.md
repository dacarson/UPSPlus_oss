# Bootloader Reconstructed Source

Approximate C/assembly source reconstructed from `bootloader.bin`
(2048 bytes, STM32F030F4P6, loaded at 0x08000000).

---

## Files

| File | Description |
|------|-------------|
| `bootloader.c` | Main bootloader logic: gpio init, flash, OTA loop, `main()` |
| `stm32f030_periph.h` | Bootloader/application-specific constants (memory map, RAM layout, OTA protocol, pin mapping) |
| `startup_stm32f030f4.s` | Vector table, `Reset_Handler`, `__init_data`, helpers |
| `STM32F030F4_BOOT.ld` | Linker script matching observed binary layout |

`bootloader.c` and `stm32f030_periph.h` are expressed against the official
ST CMSIS device header and HAL/LL drivers rather than hand-rolled
peripheral register structs:

- `stm32f0xx.h` (CMSIS device header, pulls in `stm32f030x6.h`)
- `stm32f0xx_ll_gpio.h` — pin mode/speed/pull/AF configuration
- `stm32f0xx_ll_i2c.h` — register-level slave I2C access in the ISR
- `stm32f0xx_hal_flash.h` / `stm32f0xx_hal_flash_ex.h` — unlock/lock/program/erase
- `stm32f0xx_hal_cortex.h` — NVIC priority/enable
- `stm32f0xx_hal_rcc.h` — `__HAL_RCC_*_CLK_ENABLE()` peripheral clock gating

These headers live under `Drivers/` in this repo. `documents/bootloader_src/`
is a standalone reconstruction/reference and is not itself part of the
CubeIDE build target, so IDE include-path warnings on these files are
expected.

Cross-checking against the real headers confirmed the original hand-rolled
`gpio_set_mode/ospeedr/pupdr/afr()` helpers computed exactly the same
"pin-squared" bitfield math as `LL_GPIO_SetPinMode/SetPinSpeed/SetPinPull/
SetAFPin_8_15()` — strong evidence the shipped firmware was itself built
against LL_GPIO calls. It also turned up one confirmed bug in the earlier
reconstruction: `I2C_ISR_DIR` is bit 16 of `I2C->ISR`, not bit 24 as
`stm32f030_periph.h` previously (incorrectly) defined it. This is now fixed
via `LL_I2C_GetTransferDirection()`.

---

## Reconstruction Methodology

The binary was disassembled using `arm-none-eabi-objdump -D -b binary -m arm -M force-thumb --adjust-vma=0x08000000`. All peripheral base addresses and literal-pool constants were extracted programmatically with Python to avoid endian-reading errors.

**Confidence levels** used in comments:
- **HIGH** – logic traced directly from disassembly (register address, branch condition, loop structure all verified)
- **MED** – structure correct; exact encoding detail may differ
- **LOW** – plausible but not directly confirmed

---

## Key Facts (all HIGH confidence)

### Memory Map
| Region | Address | Notes |
|--------|---------|-------|
| Bootloader | `0x08000000–0x080007FF` | 2 KB, this binary |
| Application | `0x08000800–0x08003BFF` | 14 KB, erased during OTA |
| Settings | `0x08003C00–0x08003FFF` | 1 KB, preserved across OTA |
| OTA flag | `0x08003C64` | byte; `0x7F` = request OTA |

### RAM Layout
| Address | Size | Purpose |
|---------|------|---------|
| `0x20000000` | 16 B | Initialised data (init_value word + erase-ptr temp) |
| `0x20000008` | 1 B | `rx_index` – I2C receive index |
| `0x2000000C` | 4 B | `0x007A1200` – boot timeout counter |
| `0x20000010` | 255 B | Flash settings buffer (read from `0x08003C00`) |
| `0x2000010F` | 256 B | I2C register map (`reg_map[0x00..0xFF]`) |
| `0x200001FF` | 12 B | I2C registers `0xF0–0xFB` = MCU UID (overlaps map) |
| `0x20000610` | — | Initial SP (top of 1536-byte stack) |

### Peripherals
| Peripheral | Base | Details |
|------------|------|---------|
| I2C1 | `0x40005400` | Slave, address `0x18`, 100 kHz |
| FLASH | `0x40022000` | Standard F0 controller |
| RCC | `0x40021000` | HSI clock assumed |
| GPIOA | `0x48000000` | PA6=MT_EN, PA7=PWR_EN, PA9=SCL, PA10=SDA |
| GPIOB | `0x48000400` | PB1=force-boot button |
| STM32 UID | `0x1FFFF7AC` | 96-bit (accessed via base `0x1FFFF780 + 0x2C`) |

### I2C Timing
`TIMINGR = 0x00901850` → 100 kHz standard mode at 48 MHz (matches STM32CubeMX output for these parameters). The bootloader runs on HSI (8 MHz); the timing register value suggests the application has already switched to a 48 MHz PLL before invoking OTA, or the bootloader is tolerant of slower speeds.

### I2C Protocol (OTA)
- Slave address: `0x18`
- Each programming block: `0xFA` marker byte + 16 data bytes = 17 bytes total
- Data is 8 × 16-bit halfwords (LSB first per halfword)
- Flash is programmed sequentially starting at `0x08000800`
- No explicit end-of-image command; bootloader returns after a long timeout

### Flash Erase
- Uses **aliased** address `0x00800800–0x00803FFF` (14 pages of 1 KB)
- Corresponding real addresses: `0x08000800–0x08003FFF`
- Settings page `0x08003C00` is saved to RAM before erase and restored after

### Boot Decision (in order)
1. OTA flag at `0x08003C64 == 0x7F` → enter OTA
2. PB1 button = 0 (active-low) → enter OTA
3. App MSP `& 0x2FFE0000 != 0x20000000` (invalid SRAM) → enter OTA
4. Otherwise: set MSP from `0x08000800`, BLX to `0x08000804`

### RPi Power Sequencing (MT_EN / PA6)
The RPi power reset happens at the **start** of bootloader execution, not the
end:
1. `system_init()` — the first thing called from `main()` — drives MT_EN
   **LOW** (`bootloader.c:189-190`), cutting RPi power, before the OTA/boot
   decision is made.
2. If OTA mode is entered, `bootloader_main()` immediately drives MT_EN
   **HIGH** again (`bootloader.c:342`) as its first action, restoring RPi
   power so it stays powered through the OTA transfer.
3. If the app's vector table is valid instead, `main()` jumps straight to the
   app (`bootloader.c:629-632`) without touching MT_EN again — the LOW pulse
   from step 1 is what causes the RPi to reboot; power is left to the
   application to manage from there.

There is no explicit reset/toggle on the way *out* of the bootloader — the
reset-like effect comes entirely from the initial LOW pulse in
`system_init()`, before OTA/boot is decided.

---

## Known Limitations / Uncertainties

1. **I2C ISR detail (MED)**: The exact interplay between `rx_index`, the OTA
   buffer offset, and the status byte transitions was inferred from the ISR
   disassembly. The reconstructed C ISR captures the essential logic but the
   byte-by-byte state machine may differ in edge cases.

2. **GPIO alternate function number (MED, now HIGH)**: PA9/PA10 are configured
   as AF with function 1 (`LL_GPIO_AF_1`). This matches the STM32F030
   datasheet (AF1 = I2C1 on PA9/PA10). The AF register field encoding is now
   expressed via `LL_GPIO_SetAFPin_8_15()`, whose `AFRH` bitfield math is
   identical to what the disassembly showed, resolving the earlier
   uncertainty in the hand-rolled `gpio_set_afr()` helper.

3. **Flash erase aliased addresses (HIGH – observed)**: The binary stores
   `0x00800800` (not `0x08000800`) as the starting address for `FLASH->AR`.
   This is unusual; most bootloaders use the non-aliased address. The
   STM32F030 appears to accept either, and this is what the binary actually
   does.

4. **I2C TIMINGR vs system clock**: `TIMINGR = 0x00901850` is correct for
   48 MHz. The bootloader binary itself does not configure the PLL, suggesting
   either: (a) it runs at a lower effective speed, or (b) the MCU was already
   at 48 MHz when the bootloader was burned and timing tolerances are wide
   enough for I2C slave operation.

5. **`gpio_set_ospeedr` encoding (MED, now HIGH)**: The disassembly at
   `0x08000300` showed an inconsistent-looking field computation versus the
   other GPIO helpers. Replacing it with `LL_GPIO_SetPinSpeed()` — ST's own
   `(Pin*Pin)`-based OSPEEDR field math — removes this uncertainty entirely;
   the earlier hand-rolled `p^4` formula was an artifact of a MED-confidence
   disassembly read, not a real difference in the OSPEEDR encoding.

6. **`I2C_ISR_DIR` bit position (bug, now fixed)**: The original
   `stm32f030_periph.h` defined `I2C_ISR_DIR` as bit 24. Cross-checking
   against `stm32f030x6.h` shows DIR is actually bit 16 of `I2C->ISR`. The
   ADDCODE extraction in `I2C1_IRQHandler()` was unaffected (it never used
   this macro, just a shift+mask), but the `dir_bit` check that gated
   TX vs. RX setup on ADDR match was reading the wrong bit. This is now
   expressed via `LL_I2C_GetTransferDirection()`.

7. **FLASH lock/unlock ordering (HIGH – observed, unresolved)**:
   `save_settings_to_flash()` re-locks `FLASH->CR` (`HAL_FLASH_Lock()`) at
   its end, and the OTA receive loop in `bootloader_main()` that follows it
   never calls `HAL_FLASH_Unlock()` again before programming incoming
   blocks. On real silicon a locked `FLASH->CR` ignores writes to `PG`. This
   is reproduced as observed in the disassembly rather than corrected, since
   this file documents the shipped binary's actual behaviour; see the note
   above `bootloader_main()` in `bootloader.c`.
