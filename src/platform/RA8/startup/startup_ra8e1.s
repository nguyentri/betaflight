/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

    .syntax unified
    .cpu cortex-m85
    .fpu fpv5-d16
    .thumb

.global g_pfnVectors
.global Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word _sidata
/* start address for the .data section. defined in linker script */
.word _sdata
/* end address for the .data section. defined in linker script */
.word _edata
/* start address for the .bss section. defined in linker script */
.word _sbss
/* end address for the .bss section. defined in linker script */
.word _ebss

/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called.
 * @param  None
 * @retval : None
*/

    .section .text.Reset_Handler
    .weak Reset_Handler
    .type Reset_Handler, %function
Reset_Handler:
    ldr r0, =_estack
    mov sp, r0          /* set stack pointer */

/* Copy the data segment initializers from flash to SRAM */
    ldr r0, =_sdata
    ldr r1, =_edata
    ldr r2, =_sidata
    movs r3, #0
    b LoopCopyDataInit

CopyDataInit:
    ldr r4, [r2, r3]
    str r4, [r0, r3]
    adds r3, r3, #4

LoopCopyDataInit:
    adds r4, r0, r3
    cmp r4, r1
    bcc CopyDataInit

/* Zero fill the bss segment. */
    ldr r2, =_sbss
    ldr r4, =_ebss
    movs r3, #0
    b LoopFillZerobss

FillZerobss:
    str r3, [r2]
    adds r2, r2, #4

LoopFillZerobss:
    cmp r2, r4
    bcc FillZerobss

/* Call the clock system initialization function.*/
    bl SystemInit

/* Call static constructors */
    bl __libc_init_array

/* Call the application's entry point.*/
    bl main

LoopForever:
    b LoopForever

.size Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
    .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
    b Infinite_Loop
    .size Default_Handler, .-Default_Handler

/******************************************************************************
*
* The RA8E1 vector table.
*
******************************************************************************/
    .section .isr_vector,"a",%progbits
    .type g_pfnVectors, %object
    .size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
    .word _estack
    .word Reset_Handler
    .word NMI_Handler
    .word HardFault_Handler
    .word MemManage_Handler
    .word BusFault_Handler
    .word UsageFault_Handler
    .word SecureFault_Handler
    .word 0
    .word 0
    .word 0
    .word SVC_Handler
    .word DebugMon_Handler
    .word 0
    .word PendSV_Handler
    .word SysTick_Handler

    /* External Interrupts */
    .word ICU_IRQ0_IRQHandler           /* 0: ICU IRQ0 */
    .word ICU_IRQ1_IRQHandler           /* 1: ICU IRQ1 */
    .word ICU_IRQ2_IRQHandler           /* 2: ICU IRQ2 */
    .word ICU_IRQ3_IRQHandler           /* 3: ICU IRQ3 */
    .word ICU_IRQ4_IRQHandler           /* 4: ICU IRQ4 */
    .word ICU_IRQ5_IRQHandler           /* 5: ICU IRQ5 */
    .word ICU_IRQ6_IRQHandler           /* 6: ICU IRQ6 */
    .word ICU_IRQ7_IRQHandler           /* 7: ICU IRQ7 */
    .word ICU_IRQ8_IRQHandler           /* 8: ICU IRQ8 */
    .word ICU_IRQ9_IRQHandler           /* 9: ICU IRQ9 */
    .word ICU_IRQ10_IRQHandler          /* 10: ICU IRQ10 */
    .word ICU_IRQ11_IRQHandler          /* 11: ICU IRQ11 */
    .word ICU_IRQ12_IRQHandler          /* 12: ICU IRQ12 */
    .word ICU_IRQ13_IRQHandler          /* 13: ICU IRQ13 */
    .word ICU_IRQ14_IRQHandler          /* 14: ICU IRQ14 */
    .word ICU_IRQ15_IRQHandler          /* 15: ICU IRQ15 */
    .word DMAC0_INT_IRQHandler          /* 16: DMAC0 INT */
    .word DMAC1_INT_IRQHandler          /* 17: DMAC1 INT */
    .word DMAC2_INT_IRQHandler          /* 18: DMAC2 INT */
    .word DMAC3_INT_IRQHandler          /* 19: DMAC3 INT */
    .word DTC_COMPLETE_IRQHandler       /* 20: DTC COMPLETE */
    .word DTC_END_IRQHandler            /* 21: DTC END */
    .word ICU_SNOOZE_CANCEL_IRQHandler  /* 22: ICU SNOOZE CANCEL */
    .word FCU_FIFERR_IRQHandler         /* 23: FCU FIFERR */
    .word FCU_FRDYI_IRQHandler          /* 24: FCU FRDYI */
    .word LVD_LVD1_IRQHandler           /* 25: LVD LVD1 */
    .word LVD_LVD2_IRQHandler           /* 26: LVD LVD2 */
    .word CGC_MOSC_STOP_IRQHandler      /* 27: CGC MOSC STOP */
    .word LPM_SNOOZE_REQUEST_IRQHandler /* 28: LPM SNOOZE REQUEST */
    .word AGT0_INT_IRQHandler           /* 29: AGT0 INT */
    .word AGT0_COMPARE_A_IRQHandler     /* 30: AGT0 COMPARE A */
    .word AGT0_COMPARE_B_IRQHandler     /* 31: AGT0 COMPARE B */
    .word AGT1_INT_IRQHandler           /* 32: AGT1 INT */
    .word AGT1_COMPARE_A_IRQHandler     /* 33: AGT1 COMPARE A */
    .word AGT1_COMPARE_B_IRQHandler     /* 34: AGT1 COMPARE B */
    .word IWDT_UNDERFLOW_IRQHandler     /* 35: IWDT UNDERFLOW */
    .word WDT_UNDERFLOW_IRQHandler      /* 36: WDT UNDERFLOW */
    .word RTC_ALARM_IRQHandler          /* 37: RTC ALARM */
    .word RTC_PERIOD_IRQHandler         /* 38: RTC PERIOD */
    .word RTC_CARRY_IRQHandler          /* 39: RTC CARRY */
    .word ADC0_SCAN_END_IRQHandler      /* 40: ADC0 SCAN END */
    .word ADC0_SCAN_END_B_IRQHandler    /* 41: ADC0 SCAN END B */
    .word ADC0_WINDOW_A_IRQHandler      /* 42: ADC0 WINDOW A */
    .word ADC0_WINDOW_B_IRQHandler      /* 43: ADC0 WINDOW B */
    .word ADC0_COMPARE_MATCH_IRQHandler /* 44: ADC0 COMPARE MATCH */
    .word ADC0_COMPARE_MISMATCH_IRQHandler /* 45: ADC0 COMPARE MISMATCH */
    .word COMP_HS_0_IRQHandler          /* 46: COMP HS 0 */
    .word COMP_HS_1_IRQHandler          /* 47: COMP HS 1 */
    .word USBFS_INT_IRQHandler          /* 48: USBFS INT */
    .word USBFS_RESUME_IRQHandler       /* 49: USBFS RESUME */
    .word IIC0_RXI_IRQHandler           /* 50: IIC0 RXI */
    .word IIC0_TXI_IRQHandler           /* 51: IIC0 TXI */
    .word IIC0_TEI_IRQHandler           /* 52: IIC0 TEI */
    .word IIC0_ERI_IRQHandler           /* 53: IIC0 ERI */
    .word IIC1_RXI_IRQHandler           /* 54: IIC1 RXI */
    .word IIC1_TXI_IRQHandler           /* 55: IIC1 TXI */
    .word IIC1_TEI_IRQHandler           /* 56: IIC1 TEI */
    .word IIC1_ERI_IRQHandler           /* 57: IIC1 ERI */
    .word IIC2_RXI_IRQHandler           /* 58: IIC2 RXI */
    .word IIC2_TXI_IRQHandler           /* 59: IIC2 TXI */
    .word IIC2_TEI_IRQHandler           /* 60: IIC2 TEI */
    .word IIC2_ERI_IRQHandler           /* 61: IIC2 ERI */
    .word SPI0_RXI_IRQHandler           /* 62: SPI0 RXI */
    .word SPI0_TXI_IRQHandler           /* 63: SPI0 TXI */
    .word SPI0_TEI_IRQHandler           /* 64: SPI0 TEI */
    .word SPI0_ERI_IRQHandler           /* 65: SPI0 ERI */
    .word SPI1_RXI_IRQHandler           /* 66: SPI1 RXI */
    .word SPI1_TXI_IRQHandler           /* 67: SPI1 TXI */
    .word SPI1_TEI_IRQHandler           /* 68: SPI1 TEI */
    .word SPI1_ERI_IRQHandler           /* 69: SPI1 ERI */
    .word SCI0_RXI_IRQHandler           /* 70: SCI0 RXI */
    .word SCI0_TXI_IRQHandler           /* 71: SCI0 TXI */
    .word SCI0_TEI_IRQHandler           /* 72: SCI0 TEI */
    .word SCI0_ERI_IRQHandler           /* 73: SCI0 ERI */
    .word SCI0_AM_IRQHandler            /* 74: SCI0 AM */
    .word SCI0_RXI_OR_ERI_IRQHandler    /* 75: SCI0 RXI OR ERI */
    .word SCI1_RXI_IRQHandler           /* 76: SCI1 RXI */
    .word SCI1_TXI_IRQHandler           /* 77: SCI1 TXI */
    .word SCI1_TEI_IRQHandler           /* 78: SCI1 TEI */
    .word SCI1_ERI_IRQHandler           /* 79: SCI1 ERI */
    .word SCI1_AM_IRQHandler            /* 80: SCI1 AM */
    .word SCI2_RXI_IRQHandler           /* 81: SCI2 RXI */
    .word SCI2_TXI_IRQHandler           /* 82: SCI2 TXI */
    .word SCI2_TEI_IRQHandler           /* 83: SCI2 TEI */
    .word SCI2_ERI_IRQHandler           /* 84: SCI2 ERI */
    .word SCI2_AM_IRQHandler            /* 85: SCI2 AM */
    .word SCI3_RXI_IRQHandler           /* 86: SCI3 RXI */
    .word SCI3_TXI_IRQHandler           /* 87: SCI3 TXI */
    .word SCI3_TEI_IRQHandler           /* 88: SCI3 TEI */
    .word SCI3_ERI_IRQHandler           /* 89: SCI3 ERI */
    .word SCI3_AM_IRQHandler            /* 90: SCI3 AM */
    .word SCI4_RXI_IRQHandler           /* 91: SCI4 RXI */
    .word SCI4_TXI_IRQHandler           /* 92: SCI4 TXI */
    .word SCI4_TEI_IRQHandler           /* 93: SCI4 TEI */
    .word SCI4_ERI_IRQHandler           /* 94: SCI4 ERI */
    .word SCI4_AM_IRQHandler            /* 95: SCI4 AM */
    .word SCI9_RXI_IRQHandler           /* 96: SCI9 RXI */
    .word SCI9_TXI_IRQHandler           /* 97: SCI9 TXI */
    .word SCI9_TEI_IRQHandler           /* 98: SCI9 TEI */
    .word SCI9_ERI_IRQHandler           /* 99: SCI9 ERI */
    .word SCI9_AM_IRQHandler            /* 100: SCI9 AM */
    .word GPT0_CAPTURE_COMPARE_A_IRQHandler /* 101: GPT0 CAPTURE COMPARE A */
    .word GPT0_CAPTURE_COMPARE_B_IRQHandler /* 102: GPT0 CAPTURE COMPARE B */
    .word GPT0_COMPARE_C_IRQHandler     /* 103: GPT0 COMPARE C */
    .word GPT0_COMPARE_D_IRQHandler     /* 104: GPT0 COMPARE D */
    .word GPT0_COMPARE_E_IRQHandler     /* 105: GPT0 COMPARE E */
    .word GPT0_COMPARE_F_IRQHandler     /* 106: GPT0 COMPARE F */
    .word GPT0_COUNTER_OVERFLOW_IRQHandler /* 107: GPT0 COUNTER OVERFLOW */
    .word GPT0_COUNTER_UNDERFLOW_IRQHandler /* 108: GPT0 COUNTER UNDERFLOW */
    .word GPT1_CAPTURE_COMPARE_A_IRQHandler /* 109: GPT1 CAPTURE COMPARE A */
    .word GPT1_CAPTURE_COMPARE_B_IRQHandler /* 110: GPT1 CAPTURE COMPARE B */
    .word GPT1_COMPARE_C_IRQHandler     /* 111: GPT1 COMPARE C */
    .word GPT1_COMPARE_D_IRQHandler     /* 112: GPT1 COMPARE D */
    .word GPT1_COMPARE_E_IRQHandler     /* 113: GPT1 COMPARE E */
    .word GPT1_COMPARE_F_IRQHandler     /* 114: GPT1 COMPARE F */
    .word GPT1_COUNTER_OVERFLOW_IRQHandler /* 115: GPT1 COUNTER OVERFLOW */
    .word GPT1_COUNTER_UNDERFLOW_IRQHandler /* 116: GPT1 COUNTER UNDERFLOW */
    .word GPT2_CAPTURE_COMPARE_A_IRQHandler /* 117: GPT2 CAPTURE COMPARE A */
    .word GPT2_CAPTURE_COMPARE_B_IRQHandler /* 118: GPT2 CAPTURE COMPARE B */
    .word GPT2_COMPARE_C_IRQHandler     /* 119: GPT2 COMPARE C */
    .word GPT2_COMPARE_D_IRQHandler     /* 120: GPT2 COMPARE D */
    .word GPT2_COMPARE_E_IRQHandler     /* 121: GPT2 COMPARE E */
    .word GPT2_COMPARE_F_IRQHandler     /* 122: GPT2 COMPARE F */
    .word GPT2_COUNTER_OVERFLOW_IRQHandler /* 123: GPT2 COUNTER OVERFLOW */
    .word GPT2_COUNTER_UNDERFLOW_IRQHandler /* 124: GPT2 COUNTER UNDERFLOW */
    .word GPT3_CAPTURE_COMPARE_A_IRQHandler /* 125: GPT3 CAPTURE COMPARE A */
    .word GPT3_CAPTURE_COMPARE_B_IRQHandler /* 126: GPT3 CAPTURE COMPARE B */
    .word GPT3_COMPARE_C_IRQHandler     /* 127: GPT3 COMPARE C */
    .word GPT3_COMPARE_D_IRQHandler     /* 128: GPT3 COMPARE D */
    .word GPT3_COMPARE_E_IRQHandler     /* 129: GPT3 COMPARE E */
    .word GPT3_COMPARE_F_IRQHandler     /* 130: GPT3 COMPARE F */
    .word GPT3_COUNTER_OVERFLOW_IRQHandler /* 131: GPT3 COUNTER OVERFLOW */
    .word GPT3_COUNTER_UNDERFLOW_IRQHandler /* 132: GPT3 COUNTER UNDERFLOW */

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

    .weak NMI_Handler
    .thumb_set NMI_Handler,Default_Handler

    .weak HardFault_Handler
    .thumb_set HardFault_Handler,Default_Handler

    .weak MemManage_Handler
    .thumb_set MemManage_Handler,Default_Handler

    .weak BusFault_Handler
    .thumb_set BusFault_Handler,Default_Handler

    .weak UsageFault_Handler
    .thumb_set UsageFault_Handler,Default_Handler

    .weak SecureFault_Handler
    .thumb_set SecureFault_Handler,Default_Handler

    .weak SVC_Handler
    .thumb_set SVC_Handler,Default_Handler

    .weak DebugMon_Handler
    .thumb_set DebugMon_Handler,Default_Handler

    .weak PendSV_Handler
    .thumb_set PendSV_Handler,Default_Handler

    .weak SysTick_Handler
    .thumb_set SysTick_Handler,Default_Handler

    /* External Interrupts */
    .weak ICU_IRQ0_IRQHandler
    .thumb_set ICU_IRQ0_IRQHandler,Default_Handler

    .weak ICU_IRQ1_IRQHandler
    .thumb_set ICU_IRQ1_IRQHandler,Default_Handler

    .weak ICU_IRQ2_IRQHandler
    .thumb_set ICU_IRQ2_IRQHandler,Default_Handler

    .weak ICU_IRQ3_IRQHandler
    .thumb_set ICU_IRQ3_IRQHandler,Default_Handler

    .weak ICU_IRQ4_IRQHandler
    .thumb_set ICU_IRQ4_IRQHandler,Default_Handler

    .weak ICU_IRQ5_IRQHandler
    .thumb_set ICU_IRQ5_IRQHandler,Default_Handler

    .weak ICU_IRQ6_IRQHandler
    .thumb_set ICU_IRQ6_IRQHandler,Default_Handler

    .weak ICU_IRQ7_IRQHandler
    .thumb_set ICU_IRQ7_IRQHandler,Default_Handler

    .weak ICU_IRQ8_IRQHandler
    .thumb_set ICU_IRQ8_IRQHandler,Default_Handler

    .weak ICU_IRQ9_IRQHandler
    .thumb_set ICU_IRQ9_IRQHandler,Default_Handler

    .weak ICU_IRQ10_IRQHandler
    .thumb_set ICU_IRQ10_IRQHandler,Default_Handler

    .weak ICU_IRQ11_IRQHandler
    .thumb_set ICU_IRQ11_IRQHandler,Default_Handler

    .weak ICU_IRQ12_IRQHandler
    .thumb_set ICU_IRQ12_IRQHandler,Default_Handler

    .weak ICU_IRQ13_IRQHandler
    .thumb_set ICU_IRQ13_IRQHandler,Default_Handler

    .weak ICU_IRQ14_IRQHandler
    .thumb_set ICU_IRQ14_IRQHandler,Default_Handler

    .weak ICU_IRQ15_IRQHandler
    .thumb_set ICU_IRQ15_IRQHandler,Default_Handler

    .weak DMAC0_INT_IRQHandler
    .thumb_set DMAC0_INT_IRQHandler,Default_Handler

    .weak DMAC1_INT_IRQHandler
    .thumb_set DMAC1_INT_IRQHandler,Default_Handler

    .weak DMAC2_INT_IRQHandler
    .thumb_set DMAC2_INT_IRQHandler,Default_Handler

    .weak DMAC3_INT_IRQHandler
    .thumb_set DMAC3_INT_IRQHandler,Default_Handler

    .weak DTC_COMPLETE_IRQHandler
    .thumb_set DTC_COMPLETE_IRQHandler,Default_Handler

    .weak DTC_END_IRQHandler
    .thumb_set DTC_END_IRQHandler,Default_Handler

    .weak ICU_SNOOZE_CANCEL_IRQHandler
    .thumb_set ICU_SNOOZE_CANCEL_IRQHandler,Default_Handler

    .weak FCU_FIFERR_IRQHandler
    .thumb_set FCU_FIFERR_IRQHandler,Default_Handler

    .weak FCU_FRDYI_IRQHandler
    .thumb_set FCU_FRDYI_IRQHandler,Default_Handler

    .weak LVD_LVD1_IRQHandler
    .thumb_set LVD_LVD1_IRQHandler,Default_Handler

    .weak LVD_LVD2_IRQHandler
    .thumb_set LVD_LVD2_IRQHandler,Default_Handler

    .weak CGC_MOSC_STOP_IRQHandler
    .thumb_set CGC_MOSC_STOP_IRQHandler,Default_Handler

    .weak LPM_SNOOZE_REQUEST_IRQHandler
    .thumb_set LPM_SNOOZE_REQUEST_IRQHandler,Default_Handler

    .weak AGT0_INT_IRQHandler
    .thumb_set AGT0_INT_IRQHandler,Default_Handler

    .weak AGT0_COMPARE_A_IRQHandler
    .thumb_set AGT0_COMPARE_A_IRQHandler,Default_Handler

    .weak AGT0_COMPARE_B_IRQHandler
    .thumb_set AGT0_COMPARE_B_IRQHandler,Default_Handler

    .weak AGT1_INT_IRQHandler
    .thumb_set AGT1_INT_IRQHandler,Default_Handler

    .weak AGT1_COMPARE_A_IRQHandler
    .thumb_set AGT1_COMPARE_A_IRQHandler,Default_Handler

    .weak AGT1_COMPARE_B_IRQHandler
    .thumb_set AGT1_COMPARE_B_IRQHandler,Default_Handler

    .weak IWDT_UNDERFLOW_IRQHandler
    .thumb_set IWDT_UNDERFLOW_IRQHandler,Default_Handler

    .weak WDT_UNDERFLOW_IRQHandler
    .thumb_set WDT_UNDERFLOW_IRQHandler,Default_Handler

    .weak RTC_ALARM_IRQHandler
    .thumb_set RTC_ALARM_IRQHandler,Default_Handler

    .weak RTC_PERIOD_IRQHandler
    .thumb_set RTC_PERIOD_IRQHandler,Default_Handler

    .weak RTC_CARRY_IRQHandler
    .thumb_set RTC_CARRY_IRQHandler,Default_Handler

    .weak ADC0_SCAN_END_IRQHandler
    .thumb_set ADC0_SCAN_END_IRQHandler,Default_Handler

    .weak ADC0_SCAN_END_B_IRQHandler
    .thumb_set ADC0_SCAN_END_B_IRQHandler,Default_Handler

    .weak ADC0_WINDOW_A_IRQHandler
    .thumb_set ADC0_WINDOW_A_IRQHandler,Default_Handler

    .weak ADC0_WINDOW_B_IRQHandler
    .thumb_set ADC0_WINDOW_B_IRQHandler,Default_Handler

    .weak ADC0_COMPARE_MATCH_IRQHandler
    .thumb_set ADC0_COMPARE_MATCH_IRQHandler,Default_Handler

    .weak ADC0_COMPARE_MISMATCH_IRQHandler
    .thumb_set ADC0_COMPARE_MISMATCH_IRQHandler,Default_Handler

    .weak COMP_HS_0_IRQHandler
    .thumb_set COMP_HS_0_IRQHandler,Default_Handler

    .weak COMP_HS_1_IRQHandler
    .thumb_set COMP_HS_1_IRQHandler,Default_Handler

    .weak USBFS_INT_IRQHandler
    .thumb_set USBFS_INT_IRQHandler,Default_Handler

    .weak USBFS_RESUME_IRQHandler
    .thumb_set USBFS_RESUME_IRQHandler,Default_Handler

    .weak IIC0_RXI_IRQHandler
    .thumb_set IIC0_RXI_IRQHandler,Default_Handler

    .weak IIC0_TXI_IRQHandler
    .thumb_set IIC0_TXI_IRQHandler,Default_Handler

    .weak IIC0_TEI_IRQHandler
    .thumb_set IIC0_TEI_IRQHandler,Default_Handler

    .weak IIC0_ERI_IRQHandler
    .thumb_set IIC0_ERI_IRQHandler,Default_Handler

    .weak IIC1_RXI_IRQHandler
    .thumb_set IIC1_RXI_IRQHandler,Default_Handler

    .weak IIC1_TXI_IRQHandler
    .thumb_set IIC1_TXI_IRQHandler,Default_Handler

    .weak IIC1_TEI_IRQHandler
    .thumb_set IIC1_TEI_IRQHandler,Default_Handler

    .weak IIC1_ERI_IRQHandler
    .thumb_set IIC1_ERI_IRQHandler,Default_Handler

    .weak IIC2_RXI_IRQHandler
    .thumb_set IIC2_RXI_IRQHandler,Default_Handler

    .weak IIC2_TXI_IRQHandler
    .thumb_set IIC2_TXI_IRQHandler,Default_Handler

    .weak IIC2_TEI_IRQHandler
    .thumb_set IIC2_TEI_IRQHandler,Default_Handler

    .weak IIC2_ERI_IRQHandler
    .thumb_set IIC2_ERI_IRQHandler,Default_Handler

    .weak SPI0_RXI_IRQHandler
    .thumb_set SPI0_RXI_IRQHandler,Default_Handler

    .weak SPI0_TXI_IRQHandler
    .thumb_set SPI0_TXI_IRQHandler,Default_Handler

    .weak SPI0_TEI_IRQHandler
    .thumb_set SPI0_TEI_IRQHandler,Default_Handler

    .weak SPI0_ERI_IRQHandler
    .thumb_set SPI0_ERI_IRQHandler,Default_Handler

    .weak SPI1_RXI_IRQHandler
    .thumb_set SPI1_RXI_IRQHandler,Default_Handler

    .weak SPI1_TXI_IRQHandler
    .thumb_set SPI1_TXI_IRQHandler,Default_Handler

    .weak SPI1_TEI_IRQHandler
    .thumb_set SPI1_TEI_IRQHandler,Default_Handler

    .weak SPI1_ERI_IRQHandler
    .thumb_set SPI1_ERI_IRQHandler,Default_Handler

    .weak SCI0_RXI_IRQHandler
    .thumb_set SCI0_RXI_IRQHandler,Default_Handler

    .weak SCI0_TXI_IRQHandler
    .thumb_set SCI0_TXI_IRQHandler,Default_Handler

    .weak SCI0_TEI_IRQHandler
    .thumb_set SCI0_TEI_IRQHandler,Default_Handler

    .weak SCI0_ERI_IRQHandler
    .thumb_set SCI0_ERI_IRQHandler,Default_Handler

    .weak SCI0_AM_IRQHandler
    .thumb_set SCI0_AM_IRQHandler,Default_Handler

    .weak SCI0_RXI_OR_ERI_IRQHandler
    .thumb_set SCI0_RXI_OR_ERI_IRQHandler,Default_Handler

    .weak SCI1_RXI_IRQHandler
    .thumb_set SCI1_RXI_IRQHandler,Default_Handler

    .weak SCI1_TXI_IRQHandler
    .thumb_set SCI1_TXI_IRQHandler,Default_Handler

    .weak SCI1_TEI_IRQHandler
    .thumb_set SCI1_TEI_IRQHandler,Default_Handler

    .weak SCI1_ERI_IRQHandler
    .thumb_set SCI1_ERI_IRQHandler,Default_Handler

    .weak SCI1_AM_IRQHandler
    .thumb_set SCI1_AM_IRQHandler,Default_Handler

    .weak SCI2_RXI_IRQHandler
    .thumb_set SCI2_RXI_IRQHandler,Default_Handler

    .weak SCI2_TXI_IRQHandler
    .thumb_set SCI2_TXI_IRQHandler,Default_Handler

    .weak SCI2_TEI_IRQHandler
    .thumb_set SCI2_TEI_IRQHandler,Default_Handler

    .weak SCI2_ERI_IRQHandler
    .thumb_set SCI2_ERI_IRQHandler,Default_Handler

    .weak SCI2_AM_IRQHandler
    .thumb_set SCI2_AM_IRQHandler,Default_Handler

    .weak SCI3_RXI_IRQHandler
    .thumb_set SCI3_RXI_IRQHandler,Default_Handler

    .weak SCI3_TXI_IRQHandler
    .thumb_set SCI3_TXI_IRQHandler,Default_Handler

    .weak SCI3_TEI_IRQHandler
    .thumb_set SCI3_TEI_IRQHandler,Default_Handler

    .weak SCI3_ERI_IRQHandler
    .thumb_set SCI3_ERI_IRQHandler,Default_Handler

    .weak SCI3_AM_IRQHandler
    .thumb_set SCI3_AM_IRQHandler,Default_Handler

    .weak SCI4_RXI_IRQHandler
    .thumb_set SCI4_RXI_IRQHandler,Default_Handler

    .weak SCI4_TXI_IRQHandler
    .thumb_set SCI4_TXI_IRQHandler,Default_Handler

    .weak SCI4_TEI_IRQHandler
    .thumb_set SCI4_TEI_IRQHandler,Default_Handler

    .weak SCI4_ERI_IRQHandler
    .thumb_set SCI4_ERI_IRQHandler,Default_Handler

    .weak SCI4_AM_IRQHandler
    .thumb_set SCI4_AM_IRQHandler,Default_Handler

    .weak SCI9_RXI_IRQHandler
    .thumb_set SCI9_RXI_IRQHandler,Default_Handler

    .weak SCI9_TXI_IRQHandler
    .thumb_set SCI9_TXI_IRQHandler,Default_Handler

    .weak SCI9_TEI_IRQHandler
    .thumb_set SCI9_TEI_IRQHandler,Default_Handler

    .weak SCI9_ERI_IRQHandler
    .thumb_set SCI9_ERI_IRQHandler,Default_Handler

    .weak SCI9_AM_IRQHandler
    .thumb_set SCI9_AM_IRQHandler,Default_Handler

    .weak GPT0_CAPTURE_COMPARE_A_IRQHandler
    .thumb_set GPT0_CAPTURE_COMPARE_A_IRQHandler,Default_Handler

    .weak GPT0_CAPTURE_COMPARE_B_IRQHandler
    .thumb_set GPT0_CAPTURE_COMPARE_B_IRQHandler,Default_Handler

    .weak GPT0_COMPARE_C_IRQHandler
    .thumb_set GPT0_COMPARE_C_IRQHandler,Default_Handler

    .weak GPT0_COMPARE_D_IRQHandler
    .thumb_set GPT0_COMPARE_D_IRQHandler,Default_Handler

    .weak GPT0_COMPARE_E_IRQHandler
    .thumb_set GPT0_COMPARE_E_IRQHandler,Default_Handler

    .weak GPT0_COMPARE_F_IRQHandler
    .thumb_set GPT0_COMPARE_F_IRQHandler,Default_Handler

    .weak GPT0_COUNTER_OVERFLOW_IRQHandler
    .thumb_set GPT0_COUNTER_OVERFLOW_IRQHandler,Default_Handler

    .weak GPT0_COUNTER_UNDERFLOW_IRQHandler
    .thumb_set GPT0_COUNTER_UNDERFLOW_IRQHandler,Default_Handler

    .weak GPT1_CAPTURE_COMPARE_A_IRQHandler
    .thumb_set GPT1_CAPTURE_COMPARE_A_IRQHandler,Default_Handler

    .weak GPT1_CAPTURE_COMPARE_B_IRQHandler
    .thumb_set GPT1_CAPTURE_COMPARE_B_IRQHandler,Default_Handler

    .weak GPT1_COMPARE_C_IRQHandler
    .thumb_set GPT1_COMPARE_C_IRQHandler,Default_Handler

    .weak GPT1_COMPARE_D_IRQHandler
    .thumb_set GPT1_COMPARE_D_IRQHandler,Default_Handler

    .weak GPT1_COMPARE_E_IRQHandler
    .thumb_set GPT1_COMPARE_E_IRQHandler,Default_Handler

    .weak GPT1_COMPARE_F_IRQHandler
    .thumb_set GPT1_COMPARE_F_IRQHandler,Default_Handler

    .weak GPT1_COUNTER_OVERFLOW_IRQHandler
    .thumb_set GPT1_COUNTER_OVERFLOW_IRQHandler,Default_Handler

    .weak GPT1_COUNTER_UNDERFLOW_IRQHandler
    .thumb_set GPT1_COUNTER_UNDERFLOW_IRQHandler,Default_Handler

    .weak GPT2_CAPTURE_COMPARE_A_IRQHandler
    .thumb_set GPT2_CAPTURE_COMPARE_A_IRQHandler,Default_Handler

    .weak GPT2_CAPTURE_COMPARE_B_IRQHandler
    .thumb_set GPT2_CAPTURE_COMPARE_B_IRQHandler,Default_Handler

    .weak GPT2_COMPARE_C_IRQHandler
    .thumb_set GPT2_COMPARE_C_IRQHandler,Default_Handler

    .weak GPT2_COMPARE_D_IRQHandler
    .thumb_set GPT2_COMPARE_D_IRQHandler,Default_Handler

    .weak GPT2_COMPARE_E_IRQHandler
    .thumb_set GPT2_COMPARE_E_IRQHandler,Default_Handler

    .weak GPT2_COMPARE_F_IRQHandler
    .thumb_set GPT2_COMPARE_F_IRQHandler,Default_Handler

    .weak GPT2_COUNTER_OVERFLOW_IRQHandler
    .thumb_set GPT2_COUNTER_OVERFLOW_IRQHandler,Default_Handler

    .weak GPT2_COUNTER_UNDERFLOW_IRQHandler
    .thumb_set GPT2_COUNTER_UNDERFLOW_IRQHandler,Default_Handler

    .weak GPT3_CAPTURE_COMPARE_A_IRQHandler
    .thumb_set GPT3_CAPTURE_COMPARE_A_IRQHandler,Default_Handler

    .weak GPT3_CAPTURE_COMPARE_B_IRQHandler
    .thumb_set GPT3_CAPTURE_COMPARE_B_IRQHandler,Default_Handler

    .weak GPT3_COMPARE_C_IRQHandler
    .thumb_set GPT3_COMPARE_C_IRQHandler,Default_Handler

    .weak GPT3_COMPARE_D_IRQHandler
    .thumb_set GPT3_COMPARE_D_IRQHandler,Default_Handler

    .weak GPT3_COMPARE_E_IRQHandler
    .thumb_set GPT3_COMPARE_E_IRQHandler,Default_Handler

    .weak GPT3_COMPARE_F_IRQHandler
    .thumb_set GPT3_COMPARE_F_IRQHandler,Default_Handler

    .weak GPT3_COUNTER_OVERFLOW_IRQHandler
    .thumb_set GPT3_COUNTER_OVERFLOW_IRQHandler,Default_Handler

    .weak GPT3_COUNTER_UNDERFLOW_IRQHandler
    .thumb_set GPT3_COUNTER_UNDERFLOW_IRQHandler,Default_Handler