/**
 * startup_stm32f030f4.s  –  Reconstructed from bootloader.bin
 *
 * Approximated from disassembly of bootloader.bin loaded at 0x08000000.
 * Target: STM32F030F4P6 (Cortex-M0, Thumb-only)
 *
 * Vector table (0x08000000):
 *   [0x000] Initial SP   = 0x20000610
 *   [0x004] Reset_Handler
 *   [0x008] NMI_Handler
 *   [0x00C] HardFault_Handler
 *   [0x02C] SVCall_Handler
 *   [0x038] PendSV_Handler
 *   [0x03C] SysTick_Handler
 *   [0x040..0x0AB] IRQ0..27 = Default_Handler (except I2C1)
 *   [0x09C] I2C1_IRQHandler   (IRQ 23)
 *
 * RAM layout (deduced):
 *   0x20000000  temp word (erase addr / init data destination)
 *   0x20000008  I2C state variables (rx_index, etc.)
 *   0x2000000C  boot timeout counter initial value (0x007A1200)
 *   0x20000010  flash settings buffer (255 bytes, from 0x08003C00)
 *   0x2000010F  I2C register map (256 bytes, reg 0x00..0xFF)
 *   0x200001FF  I2C reg 0xF0 = UID byte 0  (overlaps map above)
 *   0x2000012F  I2C RX state pointer
 *   0x200001EF  serial/UID output base (12 bytes at reg_map[0xF0])
 *   0x20000610  initial SP (top of stack, = stack base + 0x600)
 */

        .syntax unified
        .cpu cortex-m0
        .thumb

/* ------------------------------------------------------------------ */
/*  External symbols                                                    */
/* ------------------------------------------------------------------ */
        .extern main
        .extern I2C1_IRQHandler

/* ------------------------------------------------------------------ */
/*  Weak definitions for all handlers                                  */
/* ------------------------------------------------------------------ */
        .weak NMI_Handler
        .weak HardFault_Handler
        .weak SVCall_Handler
        .weak PendSV_Handler
        .weak SysTick_Handler
        .weak Default_Handler

/* ------------------------------------------------------------------ */
/*  .isr_vector section (maps to 0x08000000)                           */
/* ------------------------------------------------------------------ */
        .section .isr_vector, "a", %progbits
        .type g_pfnVectors, %object
g_pfnVectors:
        .word   0x20000610              /* Initial SP                  */
        .word   Reset_Handler + 1       /* Reset                       */
        .word   NMI_Handler + 1         /* NMI       (0x080004E4)      */
        .word   HardFault_Handler + 1   /* HardFault (0x08000160)      */
        .word   0                       /* reserved                    */
        .word   0                       /* reserved                    */
        .word   0                       /* reserved                    */
        .word   0                       /* reserved                    */
        .word   0                       /* reserved                    */
        .word   0                       /* reserved                    */
        .word   0                       /* reserved                    */
        .word   SVCall_Handler + 1      /* SVCall    (0x080004E8)      */
        .word   0                       /* reserved                    */
        .word   0                       /* reserved                    */
        .word   PendSV_Handler + 1      /* PendSV    (0x080004E6)      */
        .word   SysTick_Handler + 1     /* SysTick   (0x08000550)      */
        /* Peripheral IRQs (IRQ0..27) */
        .word   Default_Handler + 1     /* IRQ0  WWDG                  */
        .word   0                       /* IRQ1  PVD (reserved F030)   */
        .word   Default_Handler + 1     /* IRQ2  RTC                   */
        .word   Default_Handler + 1     /* IRQ3  FLASH                 */
        .word   Default_Handler + 1     /* IRQ4  RCC                   */
        .word   Default_Handler + 1     /* IRQ5  EXTI0_1               */
        .word   Default_Handler + 1     /* IRQ6  EXTI2_3               */
        .word   Default_Handler + 1     /* IRQ7  EXTI4_15              */
        .word   0                       /* IRQ8  TSC (reserved F030)   */
        .word   Default_Handler + 1     /* IRQ9  DMA1_CH1              */
        .word   Default_Handler + 1     /* IRQ10 DMA1_CH2_3            */
        .word   Default_Handler + 1     /* IRQ11 DMA1_CH4_5            */
        .word   Default_Handler + 1     /* IRQ12 ADC1                  */
        .word   Default_Handler + 1     /* IRQ13 TIM1_BRK_UP_TRG_COM  */
        .word   Default_Handler + 1     /* IRQ14 TIM1_CC               */
        .word   0                       /* IRQ15 TIM2 (reserved F030)  */
        .word   Default_Handler + 1     /* IRQ16 TIM3                  */
        .word   0                       /* IRQ17 TIM6 (reserved F030)  */
        .word   0                       /* IRQ18 TIM7 (reserved)       */
        .word   Default_Handler + 1     /* IRQ19 TIM14                 */
        .word   0                       /* IRQ20 TIM15 (reserved F030) */
        .word   Default_Handler + 1     /* IRQ21 TIM16                 */
        .word   Default_Handler + 1     /* IRQ22 TIM17                 */
        .word   I2C1_IRQHandler + 1     /* IRQ23 I2C1  (0x08000164)    */
        .word   0                       /* IRQ24 I2C2 (reserved F030)  */
        .word   Default_Handler + 1     /* IRQ25 SPI1                  */
        .word   0                       /* IRQ26 SPI2 (reserved)       */
        .word   Default_Handler + 1     /* IRQ27 USART1                */

/* ------------------------------------------------------------------ */
/*  Reset_Handler  (0x080000C8)                                        */
/*                                                                     */
/*  Calls a no-op SystemInit, then falls through to _startup.          */
/* ------------------------------------------------------------------ */
        .section .text.Reset_Handler, "ax", %progbits
        .type Reset_Handler, %function
        .global Reset_Handler
Reset_Handler:
        ldr     r0, =_SystemInit_ptr    /* load pointer to SystemInit  */
        blx     r0                      /* call SystemInit (no-op)     */
        ldr     r0, =_startup_ptr       /* load pointer to _startup    */
        bx      r0                      /* jump to _startup            */

        .align  2
_SystemInit_ptr:
        .word   SystemInit              /* 0x080005F8: just bx lr      */
_startup_ptr:
        .word   _startup                /* 0x080000B0                  */

/* ------------------------------------------------------------------ */
/*  _startup  (0x080000B0)                                             */
/*                                                                     */
/*  Sets the initial SP, calls __init_data (which copies/zeroes RAM    */
/*  sections), then jumps to main.                                     */
/* ------------------------------------------------------------------ */
        .section .text._startup, "ax", %progbits
        .type _startup, %function
_startup:
        ldr     r0, =0x20000610         /* initial SP value            */
        mov     sp, r0
        bl      __init_data             /* copy/zero data sections     */
        ldr     r0, =main               /* load address of main()      */
        bx      r0                      /* jump to main (no return)    */

/* ------------------------------------------------------------------ */
/*  SystemInit  (0x080005F8) – no-op                                   */
/*                                                                     */
/*  The bootloader does not reconfigure the system clock; HSI (8 MHz   */
/*  default) is used, and the I2C timing register is configured for    */
/*  100 kHz at 48 MHz (if the application has already set PLL).  The  */
/*  bootloader itself appears to run on HSI.                           */
/* ------------------------------------------------------------------ */
        .section .text.SystemInit, "ax", %progbits
        .type SystemInit, %function
        .global SystemInit
SystemInit:
        bx      lr

/* ------------------------------------------------------------------ */
/*  Default_Handler / HardFault_Handler (infinite loops)               */
/*  0x080000DA / 0x08000160                                            */
/* ------------------------------------------------------------------ */
        .section .text.Default_Handler, "ax", %progbits
        .type Default_Handler, %function
Default_Handler:
        b       .                       /* spin forever                */

        .type HardFault_Handler, %function
HardFault_Handler:
        b       .

/* ------------------------------------------------------------------ */
/*  Trivial handlers (just return)                                     */
/*  NMI 0x080004E4 / SVCall 0x080004E8 / PendSV 0x080004E6            */
/*  SysTick 0x08000550                                                 */
/* ------------------------------------------------------------------ */
        .section .text.NMI_Handler, "ax", %progbits
NMI_Handler:
SVCall_Handler:
PendSV_Handler:
SysTick_Handler:
        bx      lr

/* ------------------------------------------------------------------ */
/*  __init_data  (0x08000110)                                          */
/*                                                                     */
/*  Processes a table of copy/zero descriptors.  Each entry is 16      */
/*  bytes: { src, dst, len, handler }.  The handler is called as       */
/*  handler(src, dst, len) with the Thumb bit set.                     */
/*                                                                     */
/*  Table at 0x080007C4..0x080007E3 (two entries):                    */
/*    Entry 0: memcpy_words(0x080007E4, 0x20000000, 0x10)             */
/*             – copies 16 bytes of init data to RAM                  */
/*    Entry 1: memset_zero(_, 0x20000010, 0x600)                      */
/*             – zeros 1536 bytes of BSS (0x20000010..0x2000060F)     */
/*                                                                     */
/*  After all entries, jumps to main via the trampoline below.         */
/* ------------------------------------------------------------------ */
        .section .text.__init_data, "ax", %progbits
        .type __init_data, %function
__init_data:
        ldr     r4, =__copy_table_start
        movs    r5, #1                  /* Thumb flag                  */
        ldr     r6, =__copy_table_end
        b       __init_data_check

__init_data_loop:
        ldr     r3, [r4, #12]           /* handler function pointer    */
        ldmia   r4!, {r0, r1, r2}       /* src, dst, len (advances r4) */
        orrs    r3, r5                  /* set Thumb bit               */
        subs    r4, #12                 /* restore r4 to entry start   */
        blx     r3                      /* call handler(src, dst, len) */
        adds    r4, #16                 /* advance to next entry       */

__init_data_check:
        cmp     r4, r6
        bcc     __init_data_loop

        /* Jump to main via literal pool trampoline */
        ldr     r0, =main
        bx      r0

/* ------------------------------------------------------------------ */
/*  memcpy_words  (0x080006EC)                                         */
/*  Copies r2 bytes (in 4-byte words) from r0 to r1.                  */
/* ------------------------------------------------------------------ */
        .section .text.memcpy_words, "ax", %progbits
        .type memcpy_words, %function
memcpy_words:
        b       memcpy_words_check
memcpy_words_loop:
        ldmia   r0!, {r3}               /* r3 = *src++                 */
        subs    r2, r2, #4
        stmia   r1!, {r3}               /* *dst++ = r3                 */
memcpy_words_check:
        cmp     r2, #0
        bne     memcpy_words_loop
        bx      lr

/* ------------------------------------------------------------------ */
/*  memset_zero  (0x080006FC)                                          */
/*  Zeros r2 bytes at r1 (called with any r0; r0 is overwritten).     */
/* ------------------------------------------------------------------ */
        .section .text.memset_zero, "ax", %progbits
        .type memset_zero, %function
memset_zero:
        movs    r0, #0                  /* zero value (overwrites r0)  */
        b       memset_zero_check
memset_zero_loop:
        stmia   r1!, {r0}               /* *dst++ = 0                  */
        subs    r2, r2, #4
memset_zero_check:
        cmp     r2, #0
        bne     memset_zero_loop
        bx      lr

/* ------------------------------------------------------------------ */
/*  __aeabi_uidiv  (0x080000E4)                                        */
/*  Unsigned integer division: r0 = r1 / r3 (quotient in r0)          */
/*  Standard non-restoring bit-by-bit algorithm.                       */
/* ------------------------------------------------------------------ */
        .section .text.__aeabi_uidiv, "ax", %progbits
        .type __aeabi_uidiv, %function
__aeabi_uidiv:
        push    {r4, r5, lr}
        mov     r3, r1                  /* divisor                     */
        mov     r1, r0                  /* dividend                    */
        movs    r0, #0                  /* quotient = 0                */
        movs    r2, #32                 /* 32 bit positions            */
        movs    r4, #1
        b       __uidiv_check
__uidiv_loop:
        mov     r5, r1
        lsrs    r5, r2                  /* partial remainder           */
        cmp     r5, r3
        bcc     __uidiv_check           /* if remainder < divisor, skip */
        mov     r5, r3
        lsls    r5, r2
        subs    r1, r1, r5             /* subtract shifted divisor    */
        mov     r5, r4
        lsls    r5, r2
        adds    r0, r0, r5             /* add bit to quotient         */
__uidiv_check:
        mov     r5, r2
        subs    r2, r2, #1
        cmp     r5, #0
        bgt     __uidiv_loop
        pop     {r4, r5, pc}

/* ------------------------------------------------------------------ */
/*  Copy table (rodata, lives at 0x080007C4)                           */
/* ------------------------------------------------------------------ */
        .section .rodata.copy_table, "a", %progbits
        .align  2
__copy_table_start:
        /* Entry 0: memcpy_words(src=0x080007E4, dst=0x20000000, len=0x10) */
        .word   0x080007E4              /* src  (init data in flash)   */
        .word   0x20000000              /* dst  (RAM)                  */
        .word   0x00000010              /* len  (16 bytes, 4 words)    */
        .word   memcpy_words            /* handler                     */
        /* Entry 1: memset_zero(any, dst=0x20000010, len=0x600) */
        .word   0x080007F4              /* src  (unused by memset)     */
        .word   0x20000010              /* dst  (RAM BSS start)        */
        .word   0x00000600              /* len  (1536 bytes)           */
        .word   memset_zero             /* handler                     */
__copy_table_end:

/* ------------------------------------------------------------------ */
/*  Init data section (rodata, lives at 0x080007E4)                    */
/*  Copied to RAM 0x20000000..0x2000000F                               */
/* ------------------------------------------------------------------ */
        .section .rodata.init_data, "a", %progbits
        .align  2
        .word   0x00000000              /* RAM[0x20000000] = 0         */
        .word   0x00000000              /* RAM[0x20000004] = 0         */
        .word   0x00000000              /* RAM[0x20000008] = 0 (I2C state init) */
        .word   0x007A1200              /* RAM[0x2000000C] = 8000000 (boot timeout) */
        /* padding / flash constant area starts here (0x080007F4+)    */
        .word   0xFFFFFFFF
        .word   0xFFFFFFFF
        .word   0xFFFFFFFF

        .end
