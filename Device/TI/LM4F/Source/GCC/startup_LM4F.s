/**************************************************************************//**
 * @file     startup_LM4F.s
 * @brief    CMSIS Cortex-M4 Core Device Startup File for
 *           TI Stellaris
 * @version  V3.00
 * @date     19. December 2011
 *
 * @note     Version CodeSourcery Sourcery G++ Lite (with CS3)
 * Copyright (C) 2011 ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M
 * processor based microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such ARM based processors.
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/
/*
//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
*/


/*
// <h> Stack Configuration
//   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
// </h>
*/

    .equ    Stack_Size, 0x00000400
    .section ".stack", "w"
    .align  3
    .globl  __cs3_stack_mem
    .globl  __cs3_stack_size
__cs3_stack_mem:
    .if     Stack_Size
    .space  Stack_Size
    .endif
    .size   __cs3_stack_mem,  . - __cs3_stack_mem
    .set    __cs3_stack_size, . - __cs3_stack_mem


/*
// <h> Heap Configuration
//   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
// </h>
*/

    .equ    Heap_Size,  0x00000100

    .section ".heap", "w"
    .align  3
    .globl  __cs3_heap_start
    .globl  __cs3_heap_end
__cs3_heap_start:
    .if     Heap_Size
    .space  Heap_Size
    .endif
__cs3_heap_end:


/* Vector Table */

    .section ".cs3.interrupt_vector"
    .globl  __cs3_interrupt_vector_cortex_m
    .type   __cs3_interrupt_vector_cortex_m, %object

__cs3_interrupt_vector_cortex_m:
    .long   __cs3_stack                 /* Top of Stack                 */
    .long   __cs3_reset                 /* Reset Handler                */
    .long   NMI_Handler                 /* NMI Handler                  */
    .long   HardFault_Handler           /* Hard Fault Handler           */
    .long   MemManage_Handler           /* MPU Fault Handler            */
    .long   BusFault_Handler            /* Bus Fault Handler            */
    .long   UsageFault_Handler          /* Usage Fault Handler          */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   SVC_Handler                 /* SVCall Handler               */
    .long   DebugMon_Handler            /* Debug Monitor Handler        */
    .long   0                           /* Reserved                     */
    .long   PendSV_Handler              /* PendSV Handler               */
    .long   SysTick_Handler             /* SysTick Handler              */

    /* External Interrupts */

    .long   GPIOA_Handler               /* GPIO Port A */
    .long   GPIOB_Handler               /* GPIO Port B */
    .long   GPIOC_Handler               /* GPIO Port C */
    .long   GPIOD_Handler               /* GPIO Port D */
    .long   GPIOE_Handler               /* GPIO Port E */
    .long   UART0_Handler               /* UART0 Rx and Tx */
    .long   UART1_Handler               /* UART1 Rx and Tx */
    .long   SSI0_Handler                /* SSI0 Rx and Tx */
    .long   I2C0_Handler                /* I2C0 Master and Slave */
    .long   PMW0_FAULT_Handler          /* PWM Fault */
    .long   PWM0_0_Handler              /* PWM Generator 0 */
    .long   PWM0_1_Handler              /* PWM Generator 1 */
    .long   PWM0_2_Handler              /* PWM Generator 2 */
    .long   QEI0_Handler                /* Quadrature Encoder 0 */
    .long   ADC0SS0_Handler             /* ADC Sequence 0 */
    .long   ADC0SS1_Handler             /* ADC Sequence 1 */
    .long   ADC0SS2_Handler             /* ADC Sequence 2 */
    .long   ADC0SS3_Handler             /* ADC Sequence 3 */
    .long   WDT0_Handler                /* Watchdog timer */
    .long   TIMER0A_Handler             /* Timer 0 subtimer A */
    .long   TIMER0B_Handler             /* Timer 0 subtimer B */
    .long   TIMER1A_Handler             /* Timer 1 subtimer A */
    .long   TIMER1B_Handler             /* Timer 1 subtimer B */
    .long   TIMER2A_Handler             /* Timer 2 subtimer A */
    .long   TIMER2B_Handler             /* Timer 2 subtimer B */
    .long   COMP0_Handler               /* Analog Comparator 0 */
    .long   COMP1_Handler               /* Analog Comparator 1 */
    .long   COMP2_Handler               /* Analog Comparator 2 */
    .long   SYSCTL_Handler              /* System Control (PLL, OSC, BO) */
    .long   FLASH_Handler               /* FLASH Control */
    .long   GPIOF_Handler               /* GPIO Port F */
    .long   GPIOG_Handler               /* GPIO Port G */
    .long   GPIOH_Handler               /* GPIO Port H */
    .long   UART2_Handler               /* UART2 Rx and Tx */
    .long   SSI1_Handler                /* SSI1 Rx and Tx */
    .long   TIMER3A_Handler             /* Timer 3 subtimer A */
    .long   TIMER3B_Handler             /* Timer 3 subtimer B */
    .long   I2C1_Handler                /* I2C1 Master and Slave */
    .long   QEI1_Handler                /* Quadrature Encoder 1 */
    .long   CAN0_Handler                /* CAN0 */
    .long   CAN1_Handler                /* CAN1 */
    .long   CAN2_Handler                /* CAN2 */
    .long   ETH0_Handler                /* Ethernet */
    .long   HIB_Handler                 /* Hibernate */
    .long   USB0_Handler                /* USB0 */
    .long   PWM0_3_Handler              /* PWM Generator 3 */
    .long   UDMA_Handler                /* uDMA Software Transfer */
    .long   UDMAERR_Handler             /* uDMA Error */
    .long   ADC1SS0_Handler             /* ADC1 Sequence 0 */
    .long   ADC1SS1_Handler             /* ADC1 Sequence 1 */
    .long   ADC1SS2_Handler             /* ADC1 Sequence 2 */
    .long   ADC1SS3_Handler             /* ADC1 Sequence 3 */
    .long   I2S0_Handler                /* I2S0 */
    .long   EPI0_Handler                /* External Bus Interface 0 */
    .long   GPIOJ_Handler               /* GPIO Port J */

    .long   GPIOK_Handler               /* GPIO Port K */
    .long   GPIOL_Handler               /* GPIO Port L */
    .long   SSI2_Handler                /* SSI2 Rx and Tx */
    .long   SSI3_Handler                /* SSI3 Rx and Tx */
    .long   UART3_Handler               /* UART3 Rx and Tx */
    .long   UART4_Handler               /* UART4 Rx and Tx */
    .long   UART5_Handler               /* UART5 Rx and Tx */
    .long   UART6_Handler               /* UART6 Rx and Tx */
    .long   UART7_Handler               /* UART7 Rx and Tx */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   I2C2_Handler                /* I2C2 Master and Slave */
    .long   I2C3_Handler                /* I2C3 Master and Slave */
    .long   TIMER4A_Handler             /* Timer 4 subtimer A */
    .long   TIMER4B_Handler             /* Timer 4 subtimer B */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   TIMER5A_Handler             /* Timer 5 subtimer A */
    .long   TIMER5B_Handler             /* Timer 5 subtimer B */
    .long   WTIMER0A_Handler            /* Wide Timer 0 subtimer A */
    .long   WTIMER0B_Handler            /* Wide Timer 0 subtimer B */
    .long   WTIMER1A_Handler            /* Wide Timer 1 subtimer A */
    .long   WTIMER1B_Handler            /* Wide Timer 1 subtimer B */
    .long   WTIMER2A_Handler            /* Wide Timer 2 subtimer A */
    .long   WTIMER2B_Handler            /* Wide Timer 2 subtimer B */
    .long   WTIMER3A_Handler            /* Wide Timer 3 subtimer A */
    .long   WTIMER3B_Handler            /* Wide Timer 3 subtimer B */
    .long   WTIMER4A_Handler            /* Wide Timer 4 subtimer A */
    .long   WTIMER4B_Handler            /* Wide Timer 4 subtimer B */
    .long   WTIMER5A_Handler            /* Wide Timer 5 subtimer A */
    .long   WTIMER5B_Handler            /* Wide Timer 5 subtimer B */
    .long   FPU_Handler                 /* FPU */
    .long   PECI0_Handler               /* PECI 0 */
    .long   LPC0_Handler                /* LPC 0 */
    .long   I2C4_Handler                /* I2C4 Master and Slave */
    .long   I2C5_Handler                /* I2C5 Master and Slave */
    .long   GPIOM_Handler               /* GPIO Port M */
    .long   GPION_Handler               /* GPIO Port N */
    .long   QEI2_Handler                /* Quadrature Encoder 2 */
    .long   FAN0_Handler                /* Fan 0 */
    .long   0                           /* Reserved */
    .long   GPIOP0_Handler              /* GPIO Port P (Summary or P0) */
    .long   GPIOP1_Handler              /* GPIO Port P1 */
    .long   GPIOP2_Handler              /* GPIO Port P2 */
    .long   GPIOP3_Handler              /* GPIO Port P3 */
    .long   GPIOP4_Handler              /* GPIO Port P4 */
    .long   GPIOP5_Handler              /* GPIO Port P5 */
    .long   GPIOP6_Handler              /* GPIO Port P6 */
    .long   GPIOP7_Handler              /* GPIO Port P7 */
    .long   GPIOQ0_Handler              /* GPIO Port Q (Summary or Q0) */
    .long   GPIOQ1_Handler              /* GPIO Port Q1 */
    .long   GPIOQ2_Handler              /* GPIO Port Q2 */
    .long   GPIOQ3_Handler              /* GPIO Port Q3 */
    .long   GPIOQ4_Handler              /* GPIO Port Q4 */
    .long   GPIOQ5_Handler              /* GPIO Port Q5 */
    .long   GPIOQ6_Handler              /* GPIO Port Q6 */
    .long   GPIOQ7_Handler              /* GPIO Port Q7 */
    .long   GPIOR_Handler               /* GPIO Port R */
    .long   GPIOS_Handler               /* GPIO Port S */
    .long   PMW1_0_Handler              /* PWM 1 Generator 0 */
    .long   PWM1_1_Handler              /* PWM 1 Generator 1 */
    .long   PWM1_2_Handler              /* PWM 1 Generator 2 */
    .long   PWM1_3_Handler              /* PWM 1 Generator 3 */
    .long   PWM1_FAULT_Handler          /* PWM 1 Fault */

    .size   __cs3_interrupt_vector_cortex_m, . - __cs3_interrupt_vector_cortex_m


    .thumb


/* Reset Handler */

    .section .cs3.reset,"x",%progbits
    .thumb_func
    .globl  __cs3_reset_cortex_m
    .type   __cs3_reset_cortex_m, %function
__cs3_reset_cortex_m:
    .fnstart
    LDR     R0, =SystemInit
    BLX     R0
    LDR     R0,=_start
    BX      R0
    .pool
    .cantunwind
    .fnend
    .size   __cs3_reset_cortex_m,.-__cs3_reset_cortex_m

    .section ".text"

/* Exception Handlers */

    .weak   NMI_Handler
    .type   NMI_Handler, %function
NMI_Handler:
    B       .
    .size   NMI_Handler, . - NMI_Handler

    .weak   HardFault_Handler
    .type   HardFault_Handler, %function
HardFault_Handler:
    B       .
    .size   HardFault_Handler, . - HardFault_Handler

    .weak   MemManage_Handler
    .type   MemManage_Handler, %function
MemManage_Handler:
    B       .
    .size   MemManage_Handler, . - MemManage_Handler

    .weak   BusFault_Handler
    .type   BusFault_Handler, %function
BusFault_Handler:
    B       .
    .size   BusFault_Handler, . - BusFault_Handler

    .weak   UsageFault_Handler
    .type   UsageFault_Handler, %function
UsageFault_Handler:
    B       .
    .size   UsageFault_Handler, . - UsageFault_Handler

    .weak   SVC_Handler
    .type   SVC_Handler, %function
SVC_Handler:
    B       .
    .size   SVC_Handler, . - SVC_Handler

    .weak   DebugMon_Handler
    .type   DebugMon_Handler, %function
DebugMon_Handler:
    B       .
    .size   DebugMon_Handler, . - DebugMon_Handler

    .weak   PendSV_Handler
    .type   PendSV_Handler, %function
PendSV_Handler:
    B       .
    .size   PendSV_Handler, . - PendSV_Handler

    .weak   SysTick_Handler
    .type   SysTick_Handler, %function
SysTick_Handler:
    B       .
    .size   SysTick_Handler, . - SysTick_Handler


/* IRQ Handlers */

    .globl  Default_Handler
    .type   Default_Handler, %function
Default_Handler:
    B       .
    .size   Default_Handler, . - Default_Handler

    .macro  IRQ handler
    .weak   \handler
    .set    \handler, Default_Handler
    .endm

    IRQ     GPIOA_Handler
    IRQ     GPIOB_Handler
    IRQ     GPIOC_Handler
    IRQ     GPIOD_Handler
    IRQ     GPIOE_Handler
    IRQ     UART0_Handler
    IRQ     UART1_Handler
    IRQ     SSI0_Handler
    IRQ     I2C0_Handler
    IRQ     PMW0_FAULT_Handler
    IRQ     PWM0_0_Handler
    IRQ     PWM0_1_Handler
    IRQ     PWM0_2_Handler
    IRQ     QEI0_Handler
    IRQ     ADC0SS0_Handler
    IRQ     ADC0SS1_Handler
    IRQ     ADC0SS2_Handler
    IRQ     ADC0SS3_Handler
    IRQ     WDT0_Handler
    IRQ     TIMER0A_Handler
    IRQ     TIMER0B_Handler
    IRQ     TIMER1A_Handler
    IRQ     TIMER1B_Handler
    IRQ     TIMER2A_Handler
    IRQ     TIMER2B_Handler
    IRQ     COMP0_Handler
    IRQ     COMP1_Handler
    IRQ     COMP2_Handler
    IRQ     SYSCTL_Handler
    IRQ     FLASH_Handler
    IRQ     GPIOF_Handler
    IRQ     GPIOG_Handler
    IRQ     GPIOH_Handler
    IRQ     UART2_Handler
    IRQ     SSI1_Handler
    IRQ     TIMER3A_Handler
    IRQ     TIMER3B_Handler
    IRQ     I2C1_Handler
    IRQ     QEI1_Handler
    IRQ     CAN0_Handler
    IRQ     CAN1_Handler
    IRQ     CAN2_Handler
    IRQ     ETH0_Handler
    IRQ     HIB_Handler
    IRQ     USB0_Handler
    IRQ     PWM0_3_Handler
    IRQ     UDMA_Handler
    IRQ     UDMAERR_Handler
    IRQ     ADC1SS0_Handler
    IRQ     ADC1SS1_Handler
    IRQ     ADC1SS2_Handler
    IRQ     ADC1SS3_Handler
    IRQ     I2S0_Handler
    IRQ     EPI0_Handler
    IRQ     GPIOJ_Handler

    IRQ     GPIOK_Handler
    IRQ     GPIOL_Handler
    IRQ     SSI2_Handler
    IRQ     SSI3_Handler
    IRQ     UART3_Handler
    IRQ     UART4_Handler
    IRQ     UART5_Handler
    IRQ     UART6_Handler
    IRQ     UART7_Handler
    IRQ     I2C2_Handler
    IRQ     I2C3_Handler
    IRQ     TIMER4A_Handler
    IRQ     TIMER4B_Handler
    IRQ     TIMER5A_Handler
    IRQ     TIMER5B_Handler
    IRQ     WTIMER0A_Handler
    IRQ     WTIMER0B_Handler
    IRQ     WTIMER1A_Handler
    IRQ     WTIMER1B_Handler
    IRQ     WTIMER2A_Handler
    IRQ     WTIMER2B_Handler
    IRQ     WTIMER3A_Handler
    IRQ     WTIMER3B_Handler
    IRQ     WTIMER4A_Handler
    IRQ     WTIMER4B_Handler
    IRQ     WTIMER5A_Handler
    IRQ     WTIMER5B_Handler
    IRQ     FPU_Handler
    IRQ     PECI0_Handler
    IRQ     LPC0_Handler
    IRQ     I2C4_Handler
    IRQ     I2C5_Handler
    IRQ     GPIOM_Handler
    IRQ     GPION_Handler
    IRQ     QEI2_Handler
    IRQ     FAN0_Handler
    IRQ     GPIOP0_Handler
    IRQ     GPIOP1_Handler
    IRQ     GPIOP2_Handler
    IRQ     GPIOP3_Handler
    IRQ     GPIOP4_Handler
    IRQ     GPIOP5_Handler
    IRQ     GPIOP6_Handler
    IRQ     GPIOP7_Handler
    IRQ     GPIOQ0_Handler
    IRQ     GPIOQ1_Handler
    IRQ     GPIOQ2_Handler
    IRQ     GPIOQ3_Handler
    IRQ     GPIOQ4_Handler
    IRQ     GPIOQ5_Handler
    IRQ     GPIOQ6_Handler
    IRQ     GPIOQ7_Handler
    IRQ     GPIOR_Handler
    IRQ     GPIOS_Handler
    IRQ     PMW1_0_Handler
    IRQ     PWM1_1_Handler
    IRQ     PWM1_2_Handler
    IRQ     PWM1_3_Handler
    IRQ     PWM1_FAULT_Handler

    .end
