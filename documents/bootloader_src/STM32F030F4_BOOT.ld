/**
 * STM32F030F4_BOOT.ld  –  Linker script for the UPSPlus bootloader.
 *
 * Reconstructed from flash memory map observed in bootloader.bin:
 *
 *   Flash:
 *     0x08000000–0x080007FF  Bootloader (2 KB, this firmware)
 *     0x08000800–0x08003BFF  Application (14 KB)
 *     0x08003C00–0x08003FFF  Settings / persistence page (1 KB)
 *
 *   RAM:
 *     0x20000000–0x2000000F  Initialised data (copied from flash, 16 bytes)
 *     0x20000010–0x2000060F  BSS / zero-initialised (1536 bytes)
 *     0x20000610             Initial stack pointer (SP grows downward)
 *
 * Notes:
 *   • The settings page (0x08003C00) is NOT included in the bootloader
 *     binary and is erased during OTA but rewritten by the bootloader
 *     to preserve the MCU UID copy.
 *   • OTA flag at 0x08003C64 (settings page + 0x64).
 */

ENTRY(Reset_Handler)

MEMORY
{
    FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 2K    /* bootloader   */
    RAM    (rwx) : ORIGIN = 0x20000000, LENGTH = 1536  /* 0x600 bytes  */
}

SECTIONS
{
    /* ---------------------------------------------------------------- */
    /*  Interrupt vector table (must be at 0x08000000)                  */
    /* ---------------------------------------------------------------- */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH

    /* ---------------------------------------------------------------- */
    /*  Code and read-only data                                          */
    /* ---------------------------------------------------------------- */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)

        /* Copy table at 0x080007C4 */
        *(.rodata.copy_table)

        /* Init data at 0x080007E4 (copied to RAM by __init_data) */
        *(.rodata.init_data)

        . = ALIGN(4);
        _etext = .;
    } > FLASH

    /* ---------------------------------------------------------------- */
    /*  Symbols for __init_data copy loop                                */
    /* ---------------------------------------------------------------- */
    __copy_table_start = 0x080007C4;
    __copy_table_end   = 0x080007E4;

    /* ---------------------------------------------------------------- */
    /*  Initialised data (RAM 0x20000000–0x2000000F)                    */
    /*  Source in flash, copied to RAM by __init_data.                  */
    /* ---------------------------------------------------------------- */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH

    /* ---------------------------------------------------------------- */
    /*  Zero-initialised (BSS) section (RAM 0x20000010–0x2000060F)      */
    /* ---------------------------------------------------------------- */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    /* ---------------------------------------------------------------- */
    /*  Stack (grows downward from 0x20000610)                           */
    /* ---------------------------------------------------------------- */
    .stack (NOLOAD) :
    {
        . = ALIGN(8);
        _sstack = .;
        . += 0x600;             /* 1536 bytes                          */
        _estack = .;            /* = 0x20000610 = initial SP           */
    } > RAM

    /* ---------------------------------------------------------------- */
    /*  Assert that everything fits in the 2 KB bootloader region        */
    /* ---------------------------------------------------------------- */
    ASSERT(. <= 0x08000800, "Bootloader exceeds 2 KB limit")
}
