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
- Uses `FLASH->AR = 0x00800800–0x00803FFF` (14 pages of 1 KB) instead of the
  expected `0x08000800–0x08003FFF` — see limitation #2 below; likely a typo
  in the original firmware that's harmless because the erase controller only
  decodes the low-order page-offset bits
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

These are the items where cross-checking against the real CMSIS/HAL/LL
headers did **not** resolve the uncertainty — either because the question is
about application-level protocol logic rather than register definitions, or
because the observed behavior is confirmed but its cause/intent is not.

1. **I2C ISR detail (MED)**: The exact interplay between `rx_index`, the OTA
   buffer offset, and the status byte transitions was inferred from the ISR
   disassembly. Switching to `stm32f0xx_ll_i2c.h` firmed up the register-level
   parts (flag checks, ADDCODE, DIR — see resolved items below), but the
   byte-by-byte state machine around `rx_index`/OTA buffer offsets is
   protocol logic specific to this firmware, not something a header
   cross-check can verify. It may still differ in edge cases.

2. **Flash erase address is likely a typo, not intentional aliasing (HIGH –
   observed)**: The binary stores `0x00800800` (not `0x08000800`) as the
   starting address for `FLASH->AR`. The two differ only in bit 27 vs. bit 23
   (`0x08000000` vs `0x00800000`); both mask to the same low-order page offset
   (`& 0x3FFF == 0x0800`), which is all the erase controller needs to select
   the correct 1 KB page on this 16 KB part. That makes this look like an
   off-by-a-region-bit mistake in the original firmware (e.g. computed from
   the wrong base address) that happened to be harmless — not a deliberate
   choice to use the alias region. Independent of the header cross-check;
   this is about the firmware's own arithmetic, not a register definition.

3. **I2C `TIMINGR` vs. system clock**: `TIMINGR = 0x00901850` is correct for
   100 kHz standard mode at 48 MHz. The bootloader binary itself does not
   configure the PLL, suggesting either: (a) it runs at a lower effective
   speed, or (b) the MCU was already at 48 MHz when the bootloader was burned
   and timing tolerances are wide enough for I2C slave operation. This is a
   system-clock question, not a register-definition one, so the header
   cross-check doesn't bear on it.

4. **FLASH lock/unlock ordering (HIGH – observed, unresolved)**:
   `save_settings_to_flash()` re-locks `FLASH->CR` (`HAL_FLASH_Lock()`) at
   its end, and the OTA receive loop in `bootloader_main()` that follows it
   never calls `HAL_FLASH_Unlock()` again before programming incoming
   blocks. On real silicon a locked `FLASH->CR` ignores writes to `PG`. This
   is reproduced as observed in the disassembly rather than corrected, since
   this file documents the shipped binary's actual behaviour; see the note
   above `bootloader_main()` in `bootloader.c`. The HAL flash header
   cross-check confirmed the lock/unlock/program call shapes are right — it's
   the *ordering* between two otherwise-correct calls that's in question, so
   this stays open.

(GPIO alternate-function number, `gpio_set_ospeedr` encoding, and the
`I2C_ISR_DIR` bit position were previously listed here as uncertainties; the
CMSIS/HAL/LL cross-check resolved all three — see the note near the top of
this file.)
