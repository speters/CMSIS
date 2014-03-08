;/**************************************************************************//**
; * @file     startup_LM3S.s
; * @brief    CMSIS Cortex-M3 Core Device Startup File for
; *           TI Stellaris
; * @version  V3.00
; * @date     19. December 2011
; *
; * @note
; * Copyright (C) 2011 ARM Limited. All rights reserved.
; *
; * @par
; * ARM Limited (ARM) is supplying this software for use with Cortex-M
; * processor based microcontrollers.  This file can be freely distributed
; * within development tools that are supporting such ARM based processors.
; *
; * @par
; * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
; * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
; * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
; * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
; * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
; *
; ******************************************************************************/


;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        EXTERN  SystemInit
        PUBLIC  __vector_table
        PUBLIC  __vector_table_0x1c
        PUBLIC  __Vectors
        PUBLIC  __Vectors_End
        PUBLIC  __Vectors_Size

        DATA

__vector_table
        DCD     sfe(CSTACK)
        DCD     Reset_Handler

        DCD     NMI_Handler
        DCD     HardFault_Handler
        DCD     MemManage_Handler
        DCD     BusFault_Handler
        DCD     UsageFault_Handler
__vector_table_0x1c
        DCD     0
        DCD     0
        DCD     0
        DCD     0
        DCD     SVC_Handler
        DCD     DebugMon_Handler
        DCD     0
        DCD     PendSV_Handler
        DCD     SysTick_Handler

        ; External Interrupts

        DCD     GPIOA_Handler             ; GPIO Port A
        DCD     GPIOB_Handler             ; GPIO Port B
        DCD     GPIOC_Handler             ; GPIO Port C
        DCD     GPIOD_Handler             ; GPIO Port D
        DCD     GPIOE_Handler             ; GPIO Port E
        DCD     UART0_Handler             ; UART0 Rx and Tx
        DCD     UART1_Handler             ; UART1 Rx and Tx
        DCD     SSI0_Handler              ; SSI0 Rx and Tx
        DCD     I2C0_Handler              ; I2C0 Master and Slave
        DCD     PMW0_FAULT_Handler        ; PWM Fault
        DCD     PWM0_0_Handler            ; PWM Generator 0
        DCD     PWM0_1_Handler            ; PWM Generator 1
        DCD     PWM0_2_Handler            ; PWM Generator 2
        DCD     QEI0_Handler              ; Quadrature Encoder 0
        DCD     ADC0SS0_Handler           ; ADC Sequence 0
        DCD     ADC0SS1_Handler           ; ADC Sequence 1
        DCD     ADC0SS2_Handler           ; ADC Sequence 2
        DCD     ADC0SS3_Handler           ; ADC Sequence 3
        DCD     WDT0_Handler              ; Watchdog timer
        DCD     TIMER0A_Handler           ; Timer 0 subtimer A
        DCD     TIMER0B_Handler           ; Timer 0 subtimer B
        DCD     TIMER1A_Handler           ; Timer 1 subtimer A
        DCD     TIMER1B_Handler           ; Timer 1 subtimer B
        DCD     TIMER2A_Handler           ; Timer 2 subtimer A
        DCD     TIMER2B_Handler           ; Timer 2 subtimer B
        DCD     COMP0_Handler             ; Analog Comparator 0
        DCD     COMP1_Handler             ; Analog Comparator 1
        DCD     COMP2_Handler             ; Analog Comparator 2
        DCD     SYSCTL_Handler            ; System Control (PLL, OSC, BO)
        DCD     FLASH_Handler             ; FLASH Control
        DCD     GPIOF_Handler             ; GPIO Port F
        DCD     GPIOG_Handler             ; GPIO Port G
        DCD     GPIOH_Handler             ; GPIO Port H
        DCD     UART2_Handler             ; UART2 Rx and Tx
        DCD     SSI1_Handler              ; SSI1 Rx and Tx
        DCD     TIMER3A_Handler           ; Timer 3 subtimer A
        DCD     TIMER3B_Handler           ; Timer 3 subtimer B
        DCD     I2C1_Handler              ; I2C1 Master and Slave
        DCD     QEI1_Handler              ; Quadrature Encoder 1
        DCD     CAN0_Handler              ; CAN0
        DCD     CAN1_Handler              ; CAN1
        DCD     CAN2_Handler              ; CAN2
        DCD     ETH0_Handler              ; Ethernet
        DCD     HIB_Handler               ; Hibernate
        DCD     USB0_Handler              ; USB0
        DCD     PWM0_3_Handler            ; PWM Generator 3
        DCD     UDMA_Handler              ; uDMA Software Transfer
        DCD     UDMAERR_Handler           ; uDMA Error
        DCD     ADC1SS0_Handler           ; ADC1 Sequence 0
        DCD     ADC1SS1_Handler           ; ADC1 Sequence 1
        DCD     ADC1SS2_Handler           ; ADC1 Sequence 2
        DCD     ADC1SS3_Handler           ; ADC1 Sequence 3
        DCD     I2S0_Handler              ; I2S0
        DCD     EPI0_Handler              ; External Bus Interface 0
        DCD     GPIOJ_Handler             ; GPIO Port J

__Vectors_End

__Vectors       EQU   __vector_table
__Vectors_Size  EQU   __Vectors_End - __Vectors


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
        THUMB

        PUBWEAK Reset_Handler
        SECTION .text:CODE:REORDER(2)
Reset_Handler
        LDR     R0, =SystemInit
        BLX     R0
        LDR     R0, =__iar_program_start
        BX      R0

        PUBWEAK NMI_Handler
        SECTION .text:CODE:REORDER(1)
NMI_Handler
        B NMI_Handler

        PUBWEAK HardFault_Handler
        SECTION .text:CODE:REORDER(1)
HardFault_Handler
        B HardFault_Handler

        PUBWEAK MemManage_Handler
        SECTION .text:CODE:REORDER(1)
MemManage_Handler
        B MemManage_Handler

        PUBWEAK BusFault_Handler
        SECTION .text:CODE:REORDER(1)
BusFault_Handler
        B BusFault_Handler

        PUBWEAK UsageFault_Handler
        SECTION .text:CODE:REORDER(1)
UsageFault_Handler
        B UsageFault_Handler

        PUBWEAK SVC_Handler
        SECTION .text:CODE:REORDER(1)
SVC_Handler
        B SVC_Handler

        PUBWEAK DebugMon_Handler
        SECTION .text:CODE:REORDER(1)
DebugMon_Handler
        B DebugMon_Handler

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:REORDER(1)
PendSV_Handler
        B PendSV_Handler

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:REORDER(1)
SysTick_Handler
        B SysTick_Handler


        PUBWEAK GPIOA_Handler
        SECTION .text:CODE:REORDER(1)
GPIOA_Handler
        B GPIOA_Handler

        PUBWEAK GPIOB_Handler
        SECTION .text:CODE:REORDER(1)
GPIOB_Handler
        B GPIOB_Handler

        PUBWEAK GPIOC_Handler
        SECTION .text:CODE:REORDER(1)
GPIOC_Handler
        B GPIOC_Handler

        PUBWEAK GPIOD_Handler
        SECTION .text:CODE:REORDER(1)
GPIOD_Handler
        B GPIOD_Handler

        PUBWEAK GPIOE_Handler
        SECTION .text:CODE:REORDER(1)
GPIOE_Handler
        B GPIOE_Handler

        PUBWEAK UART0_Handler
        SECTION .text:CODE:REORDER(1)
UART0_Handler
        B UART0_Handler

        PUBWEAK UART1_Handler
        SECTION .text:CODE:REORDER(1)
UART1_Handler
        B UART1_Handler

        PUBWEAK SSI0_Handler
        SECTION .text:CODE:REORDER(1)
SSI0_Handler
        B SSI0_Handler

        PUBWEAK I2C0_Handler
        SECTION .text:CODE:REORDER(1)
I2C0_Handler
        B I2C0_Handler

        PUBWEAK PMW0_FAULT_Handler
        SECTION .text:CODE:REORDER(1)
PMW0_FAULT_Handler
        B PMW0_FAULT_Handler

        PUBWEAK PWM0_0_Handler
        SECTION .text:CODE:REORDER(1)
PWM0_0_Handler
        B PWM0_0_Handler

        PUBWEAK PWM0_1_Handler
        SECTION .text:CODE:REORDER(1)
PWM0_1_Handler
        B PWM0_1_Handler

        PUBWEAK PWM0_2_Handler
        SECTION .text:CODE:REORDER(1)
PWM0_2_Handler
        B PWM0_2_Handler

        PUBWEAK QEI0_Handler
        SECTION .text:CODE:REORDER(1)
QEI0_Handler
        B QEI0_Handler

        PUBWEAK ADC0SS0_Handler
        SECTION .text:CODE:REORDER(1)
ADC0SS0_Handler
        B ADC0SS0_Handler

        PUBWEAK ADC0SS1_Handler
        SECTION .text:CODE:REORDER(1)
ADC0SS1_Handler
        B ADC0SS1_Handler

        PUBWEAK ADC0SS2_Handler
        SECTION .text:CODE:REORDER(1)
ADC0SS2_Handler
        B ADC0SS2_Handler

        PUBWEAK ADC0SS3_Handler
        SECTION .text:CODE:REORDER(1)
ADC0SS3_Handler
        B ADC0SS3_Handler

        PUBWEAK WDT0_Handler
        SECTION .text:CODE:REORDER(1)
WDT0_Handler
        B WDT0_Handler

        PUBWEAK TIMER0A_Handler
        SECTION .text:CODE:REORDER(1)
TIMER0A_Handler
        B TIMER0A_Handler

        PUBWEAK TIMER0B_Handler
        SECTION .text:CODE:REORDER(1)
TIMER0B_Handler
        B TIMER0B_Handler

        PUBWEAK TIMER1A_Handler
        SECTION .text:CODE:REORDER(1)
TIMER1A_Handler
        B TIMER1A_Handler

        PUBWEAK TIMER1B_Handler
        SECTION .text:CODE:REORDER(1)
TIMER1B_Handler
        B TIMER1B_Handler

        PUBWEAK TIMER2A_Handler
        SECTION .text:CODE:REORDER(1)
TIMER2A_Handler
        B TIMER2A_Handler

        PUBWEAK TIMER2B_Handler
        SECTION .text:CODE:REORDER(1)
TIMER2B_Handler
        B TIMER2B_Handler

        PUBWEAK COMP0_Handler
        SECTION .text:CODE:REORDER(1)
COMP0_Handler
        B COMP0_Handler

        PUBWEAK COMP1_Handler
        SECTION .text:CODE:REORDER(1)
COMP1_Handler
        B COMP1_Handler

        PUBWEAK COMP2_Handler
        SECTION .text:CODE:REORDER(1)
COMP2_Handler
        B COMP2_Handler

        PUBWEAK SYSCTL_Handler
        SECTION .text:CODE:REORDER(1)
SYSCTL_Handler
        B SYSCTL_Handler

        PUBWEAK FLASH_Handler
        SECTION .text:CODE:REORDER(1)
FLASH_Handler
        B FLASH_Handler

        PUBWEAK GPIOF_Handler
        SECTION .text:CODE:REORDER(1)
GPIOF_Handler
        B GPIOF_Handler

        PUBWEAK GPIOG_Handler
        SECTION .text:CODE:REORDER(1)
GPIOG_Handler
        B GPIOG_Handler

        PUBWEAK GPIOH_Handler
        SECTION .text:CODE:REORDER(1)
GPIOH_Handler
        B GPIOH_Handler

        PUBWEAK UART2_Handler
        SECTION .text:CODE:REORDER(1)
UART2_Handler
        B UART2_Handler

        PUBWEAK SSI1_Handler
        SECTION .text:CODE:REORDER(1)
SSI1_Handler
        B SSI1_Handler

        PUBWEAK TIMER3A_Handler
        SECTION .text:CODE:REORDER(1)
TIMER3A_Handler
        B TIMER3A_Handler

        PUBWEAK TIMER3B_Handler
        SECTION .text:CODE:REORDER(1)
TIMER3B_Handler
        B TIMER3B_Handler

        PUBWEAK I2C1_Handler
        SECTION .text:CODE:REORDER(1)
I2C1_Handler
        B I2C1_Handler

        PUBWEAK QEI1_Handler
        SECTION .text:CODE:REORDER(1)
QEI1_Handler
        B QEI1_Handler

        PUBWEAK CAN0_Handler
        SECTION .text:CODE:REORDER(1)
CAN0_Handler
        B CAN0_Handler

        PUBWEAK CAN1_Handler
        SECTION .text:CODE:REORDER(1)
CAN1_Handler
        B CAN1_Handler

        PUBWEAK CAN2_Handler
        SECTION .text:CODE:REORDER(1)
CAN2_Handler
        B CAN2_Handler

        PUBWEAK ETH0_Handler
        SECTION .text:CODE:REORDER(1)
ETH0_Handler
        B ETH0_Handler

        PUBWEAK HIB_Handler
        SECTION .text:CODE:REORDER(1)
HIB_Handler
        B HIB_Handler

        PUBWEAK USB0_Handler
        SECTION .text:CODE:REORDER(1)
USB0_Handler
        B USB0_Handler

        PUBWEAK PWM0_3_Handler
        SECTION .text:CODE:REORDER(1)
PWM0_3_Handler
        B PWM0_3_Handler

        PUBWEAK UDMA_Handler
        SECTION .text:CODE:REORDER(1)
UDMA_Handler
        B UDMA_Handler

        PUBWEAK UDMAERR_Handler
        SECTION .text:CODE:REORDER(1)
UDMAERR_Handler
        B UDMAERR_Handler

        PUBWEAK ADC1SS0_Handler
        SECTION .text:CODE:REORDER(1)
ADC1SS0_Handler
        B ADC1SS0_Handler

        PUBWEAK ADC1SS1_Handler
        SECTION .text:CODE:REORDER(1)
ADC1SS1_Handler
        B ADC1SS1_Handler

        PUBWEAK ADC1SS2_Handler
        SECTION .text:CODE:REORDER(1)
ADC1SS2_Handler
        B ADC1SS2_Handler

        PUBWEAK ADC1SS3_Handler
        SECTION .text:CODE:REORDER(1)
ADC1SS3_Handler
        B ADC1SS3_Handler

        PUBWEAK I2S0_Handler
        SECTION .text:CODE:REORDER(1)
I2S0_Handler
        B I2S0_Handler

        PUBWEAK EPI0_Handler
        SECTION .text:CODE:REORDER(1)
EPI0_Handler
        B EPI0_Handler

        PUBWEAK GPIOJ_Handler
        SECTION .text:CODE:REORDER(1)
GPIOJ_Handler
        B GPIOJ_Handler


        END
