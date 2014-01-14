/*****************************************************************************/
/* Startup_XMC1100.s: Startup file for XMC1100 device series                 */
/*****************************************************************************/

/* ********************* Version History *********************************** */
/* ***************************************************************************
V1.0, Oct, 02, 2012 PKB:Startup file for XMC1  
V1.1, Oct, 19, 2012 PKB:ERU and MATH interrupt handlers  
V1.2, Nov, 02, 2012 PKB:Renamed AllowPLLInitByStartup to AllowClkInitByStartup  
V1.3, Dec, 11, 2012 PKB:Attributes of .XmcVeneerCode section changed  
V1.4, Dec, 13, 2012 PKB:Removed unwanted interrupts/veneers  
V1.5, Jan, 26, 2013 PKB:Corrected the SSW related entries  
V1.6, Feb, 13, 2013 PKB:Relative path to Device_Data.h  
V1.7, Feb, 19, 2013 PKB:Included XMC1100_SCU.inc  
**************************************************************************** */
/**
* @file     Startup_XMC1100.s
*           XMC1100 Device Series
* @version  V1.7
* @date     Feb 2013
*
Copyright (C) 2013 Infineon Technologies AG. All rights reserved.
*
*
* @par
* Infineon Technologies AG (Infineon) is supplying this software for use with 
* Infineon's microcontrollers.  This file can be freely distributed
* within development tools that are supporting such microcontrollers.
*
* @par
* THIS SOFTWARE IS PROVIDED AS IS.  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
* ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
* CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
*
******************************************************************************/
#ifdef DAVE_CE
#include <XMC1100_SCU.inc>
#include "../Dave/Generated/inc/DAVESupport/Device_Data.h"
#else
#define CLKVAL1_SSW 0x80000000
#define CLKVAL2_SSW 0x80000000
#endif

/* A macro to define vector table entries */
.macro Entry Handler
   .long \Handler
.endm

/* A couple of macros to ease definition of the various handlers */
.macro Insert_ExceptionHandler Handler_Func 
   .weak \Handler_Func
   .type \Handler_Func, %function
   \Handler_Func:
   B  .
   .size \Handler_Func, . - \Handler_Func
.endm    

/* ================== START OF VECTOR TABLE DEFINITION ====================== */
/* Vector Table - This is indirectly branched to through the veneers */
    .syntax unified   
    .cpu cortex-m0

    .section ".Xmc1100.reset"
    .globl  __Xmc1100_interrupt_vector_cortex_m
    .type   __Xmc1100_interrupt_vector_cortex_m, %object

__Xmc1100_interrupt_vector_cortex_m:
    .long   __Xmc1100_stack             /* Top of Stack                 */
    .long   __Xmc1100_reset_cortex_m    /* Reset Handler                */
/* 
 * All entries below are redundant for M0, but are retained because they can
 * in the future be directly ported to M0 Plus devices.
 */
    Entry   NMI_Handler                 /* NMI Handler                  */
    Entry   HardFault_Handler           /* Hard Fault Handler           */
    .long   CLKVAL1_SSW                 /* Reserved                     */
    .long   CLKVAL2_SSW                 /* Reserved                     */
#ifdef RETAIN_VECTOR_TABLE
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    Entry   SVC_Handler                 /* SVCall Handler               */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    Entry   PendSV_Handler              /* PendSV Handler               */
    Entry   SysTick_Handler             /* SysTick Handler              */

    /* Interrupt Handlers for Service Requests (SR) from XMC1100 Peripherals */
    Entry   SCU_0_IRQHandler            /* Handler name for SR SCU_0     */
    Entry   SCU_1_IRQHandler            /* Handler name for SR SCU_1     */
    Entry   SCU_2_IRQHandler            /* Handler name for SR SCU_2     */
    Entry   ERU0_0_IRQHandler           /* Handler name for SR ERU0_0    */
    Entry   ERU0_1_IRQHandler           /* Handler name for SR ERU0_1    */
    Entry   ERU0_2_IRQHandler           /* Handler name for SR ERU0_2    */
    Entry   ERU0_3_IRQHandler           /* Handler name for SR ERU0_3    */
    .long   0                           /* Not Available                 */
    .long   0                           /* Not Available                 */
    Entry   USIC0_0_IRQHandler          /* Handler name for SR USIC0_0   */
    Entry   USIC0_1_IRQHandler          /* Handler name for SR USIC0_1   */
    Entry   USIC0_2_IRQHandler          /* Handler name for SR USIC0_2   */
    Entry   USIC0_3_IRQHandler          /* Handler name for SR USIC0_3   */
    Entry   USIC0_4_IRQHandler          /* Handler name for SR USIC0_4   */
    Entry   USIC0_5_IRQHandler          /* Handler name for SR USIC0_5   */
    Entry   VADC0_C0_0_IRQHandler       /* Handler name for SR VADC0_C0_0  */
    Entry   VADC0_C0_1_IRQHandler       /* Handler name for SR VADC0_C0_1  */
    .long   0                           /* Not Available                 */
    .long   0                           /* Not Available                 */
    .long   0                           /* Not Available                 */
    .long   0                           /* Not Available                 */
    Entry   CCU40_0_IRQHandler          /* Handler name for SR CCU40_0   */
    Entry   CCU40_1_IRQHandler          /* Handler name for SR CCU40_1   */
    Entry   CCU40_2_IRQHandler          /* Handler name for SR CCU40_2   */
    Entry   CCU40_3_IRQHandler          /* Handler name for SR CCU40_3   */
    .long   0                           /* Not Available                 */
    .long   0                           /* Not Available                 */
    .long   0                           /* Not Available                 */
    .long   0                           /* Not Available                 */
    .long   0                           /* Not Available                 */
    .long   0                           /* Not Available                 */
    .long   0                           /* Not Available                 */
#endif

    .size  __Xmc1100_interrupt_vector_cortex_m, . - __Xmc1100_interrupt_vector_cortex_m
/* ================== END OF VECTOR TABLE DEFINITION ======================= */

/* ================== START OF VECTOR ROUTINES ============================= */
    .thumb 
/* ======================================================================== */
/* Reset Handler */

    .thumb_func 
    .globl  __Xmc1100_reset_cortex_m
    .type   __Xmc1100_reset_cortex_m, %function
__Xmc1100_reset_cortex_m:
    .fnstart

    /* C routines are likely to be called. Setup the stack now */
    /* This is already setup by BootROM,hence this step is optional */ 
    LDR R0,=__Xmc1100_stack
    MOV SP,R0

    /* Clock tree, External memory setup etc may be done here */    
    LDR     R0, =SystemInit
    BLX     R0

/* 
   SystemInit_DAVE3() is provided by DAVE3 code generation engine. It is  
   weakly defined here though for a potential override.
*/
    LDR     R0, =SystemInit_DAVE3 	
    BLX     R0

    B       __Xmc1100_Program_Loader 
    
    .pool
    .cantunwind
    .fnend
    .size   __Xmc1100_reset_cortex_m,.-__Xmc1100_reset_cortex_m
/* ======================================================================== */
/* __Xmc1100_reset must yield control to __Xmc1100_Program_Loader before control
   to C land is given */
   .section .Xmc1100.postreset,"x",%progbits
   __Xmc1100_Program_Loader:
   .fnstart
   /* Memories are accessible now*/
   
   /* DATA COPY */
   /* R0 = Start address, R1 = Destination address, R2 = Size */
   LDR R0, =DataLoadAddr
   LDR R1, =__Xmc1100_sData
   LDR R2, =__Xmc1100_Data_Size

   /* Is there anything to be copied? */
   CMP R2,#0
   BEQ SKIPCOPY
   
   /* For bytecount less than 4, at least 1 word must be copied */
   CMP R2,#4
   BCS STARTCOPY
   
   /* Byte count < 4 ; so bump it up */
   MOVS R2,#4

STARTCOPY:
   /* 
      R2 contains byte count. Change it to word count. It is ensured in the 
      linker script that the length is always word aligned.
   */
   LSRS R2,R2,#2 /* Divide by 4 to obtain word count */

   /* The proverbial loop from the schooldays */
COPYLOOP:
   LDR R3,[R0]
   STR R3,[R1]
   SUBS R2,#1
   BEQ SKIPCOPY
   ADDS R0,#4
   ADDS R1,#4
   B COPYLOOP
    
SKIPCOPY:
   /* BSS CLEAR */
   LDR R0, =__Xmc1100_sBSS     /* Start of BSS */
   LDR R1, =__Xmc1100_BSS_Size /* BSS size in bytes */

   /* Find out if there are items assigned to BSS */   
   CMP R1,#0 
   BEQ SKIPCLEAR

   /* At least 1 word must be copied */
   CMP R1,#4
   BCS STARTCLEAR
   
   /* Byte count < 4 ; so bump it up to a word*/
   MOVS R1,#4

STARTCLEAR:
   LSRS R1,R1,#2            /* BSS size in words */
   
   MOVS R2,#0
CLEARLOOP:
   STR R2,[R0]
   SUBS R1,#1
   BEQ SKIPCLEAR
   ADDS R0,#4
   B CLEARLOOP
    
SKIPCLEAR:

   /* VENEER COPY */
   /* R0 = Start address, R1 = Destination address, R2 = Size */
   LDR R0, =VeneerLoadAddr
   LDR R1, =VeneerStart
   LDR R2, =VeneerSize

STARTVENEERCOPY:
   /* 
      R2 contains byte count. Change it to word count. It is ensured in the 
      linker script that the length is always word aligned.
   */
   LSRS R2,R2,#2 /* Divide by 4 to obtain word count */

   /* The proverbial loop from the schooldays */
VENEERCOPYLOOP:
   LDR R3,[R0]
   STR R3,[R1]
   SUBS R2,#1
   BEQ SKIPVENEERCOPY
   ADDS R0,#4
   ADDS R1,#4
   B VENEERCOPYLOOP
    
SKIPVENEERCOPY:
   /* Update System Clock */
   LDR R0,=SystemCoreClockUpdate
   BLX R0

   /* Reset stack pointer before zipping off to user application, Optional */
   LDR R0,=__Xmc1100_stack 
   MOV SP,R0

   MOVS R0,#0
   MOVS R1,#0
   LDR R2, =main
   MOV PC,R2

   .pool
   .cantunwind
   .fnend
   .size   __Xmc1100_Program_Loader,.-__Xmc1100_Program_Loader
/* ======================================================================== */
/* ========== START OF EXCEPTION HANDLER DEFINITION ======================== */

/* Default exception Handlers - Users may override this default functionality by
   defining handlers of the same name in their C code */
    .thumb 
    .text

    Insert_ExceptionHandler NMI_Handler
/* ======================================================================== */
    Insert_ExceptionHandler HardFault_Handler
/* ======================================================================== */
    Insert_ExceptionHandler SVC_Handler
/* ======================================================================== */
    Insert_ExceptionHandler PendSV_Handler
/* ======================================================================== */
    Insert_ExceptionHandler SysTick_Handler

/* ============= END OF EXCEPTION HANDLER DEFINITION ======================== */

/* ============= START OF INTERRUPT HANDLER DEFINITION ====================== */

/* IRQ Handlers */
    Insert_ExceptionHandler SCU_0_IRQHandler
/* ======================================================================== */
    Insert_ExceptionHandler SCU_1_IRQHandler
/* ======================================================================== */
    Insert_ExceptionHandler SCU_2_IRQHandler
/* ======================================================================== */
    Insert_ExceptionHandler ERU0_0_IRQHandler
/* ======================================================================== */
    Insert_ExceptionHandler ERU0_1_IRQHandler
/* ======================================================================== */
    Insert_ExceptionHandler ERU0_2_IRQHandler
/* ======================================================================== */
    Insert_ExceptionHandler ERU0_3_IRQHandler
/* ======================================================================== */
    Insert_ExceptionHandler VADC0_C0_0_IRQHandler
/* ======================================================================== */
    Insert_ExceptionHandler VADC0_C0_1_IRQHandler
/* ======================================================================== */
    Insert_ExceptionHandler CCU40_0_IRQHandler
/* ======================================================================== */
    Insert_ExceptionHandler CCU40_1_IRQHandler
/* ======================================================================== */
    Insert_ExceptionHandler CCU40_2_IRQHandler
/* ======================================================================== */
    Insert_ExceptionHandler CCU40_3_IRQHandler
/* ======================================================================== */
    Insert_ExceptionHandler USIC0_0_IRQHandler
/* ======================================================================== */
    Insert_ExceptionHandler USIC0_1_IRQHandler
/* ======================================================================== */
    Insert_ExceptionHandler USIC0_2_IRQHandler
/* ======================================================================== */
    Insert_ExceptionHandler USIC0_3_IRQHandler
/* ======================================================================== */
    Insert_ExceptionHandler USIC0_4_IRQHandler
/* ======================================================================== */
    Insert_ExceptionHandler USIC0_5_IRQHandler
/* ======================================================================== */
/* ======================================================================== */
/* ======================================================================== */

/* ==================VENEERS VENEERS VENEERS VENEERS VENEERS=============== */
    .section ".XmcVeneerCode","ax",%progbits
.globl HardFault_Veneer
HardFault_Veneer:
    LDR R0, =HardFault_Handler
    MOV PC,R0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    
/* ======================================================================== */
.globl SVC_Veneer
SVC_Veneer:
    LDR R0, =SVC_Handler
    MOV PC,R0
    .long 0
    .long 0
/* ======================================================================== */
.globl PendSV_Veneer
PendSV_Veneer:
    LDR R0, =PendSV_Handler
    MOV PC,R0
/* ======================================================================== */
.globl SysTick_Veneer 
SysTick_Veneer:
    LDR R0, =SysTick_Handler
    MOV PC,R0
/* ======================================================================== */
.globl SCU_0_Veneer 
SCU_0_Veneer:
    LDR R0, =SCU_0_IRQHandler
    MOV PC,R0
/* ======================================================================== */
.globl SCU_1_Veneer 
SCU_1_Veneer:
    LDR R0, =SCU_1_IRQHandler
    MOV PC,R0
/* ======================================================================== */
.globl SCU_2_Veneer
SCU_2_Veneer:
    LDR R0, =SCU_2_IRQHandler
    MOV PC,R0
/* ======================================================================== */
.globl SCU_3_Veneer 
SCU_3_Veneer:
    LDR R0, =ERU0_0_IRQHandler
    MOV PC,R0
/* ======================================================================== */
.globl SCU_4_Veneer 
SCU_4_Veneer:
    LDR R0, =ERU0_1_IRQHandler
    MOV PC,R0
/* ======================================================================== */
.globl SCU_5_Veneer 
SCU_5_Veneer:
    LDR R0, =ERU0_2_IRQHandler
    MOV PC,R0
/* ======================================================================== */
.globl SCU_6_Veneer 
SCU_6_Veneer:
    LDR R0, =ERU0_3_IRQHandler
    MOV PC,R0
    .long 0
    .long 0
/* ======================================================================== */
.globl USIC0_0_Veneer
USIC0_0_Veneer:
    LDR R0, =USIC0_0_IRQHandler
    MOV PC,R0
/* ======================================================================== */
.globl USIC0_1_Veneer
USIC0_1_Veneer:
    LDR R0, =USIC0_1_IRQHandler
    MOV PC,R0
/* ======================================================================== */
.globl USIC0_2_Veneer
USIC0_2_Veneer:
    LDR R0, =USIC0_2_IRQHandler
    MOV PC,R0
/* ======================================================================== */
.globl USIC0_3_Veneer
USIC0_3_Veneer:
    LDR R0, =USIC0_3_IRQHandler
    MOV PC,R0
/* ======================================================================== */
.globl USIC0_4_Veneer
USIC0_4_Veneer:
    LDR R0, =USIC0_4_IRQHandler
    MOV PC,R0
/* ======================================================================== */
.globl USIC0_5_Veneer
USIC0_5_Veneer:
    LDR R0, =USIC0_5_IRQHandler
    MOV PC,R0
/* ======================================================================== */
.globl VADC0_C0_0_Veneer 
VADC0_C0_0_Veneer:
    LDR R0, =VADC0_C0_0_IRQHandler
    MOV PC,R0
/* ======================================================================== */
.globl VADC0_C0_1_Veneer
VADC0_C0_1_Veneer:
    LDR R0, =VADC0_C0_1_IRQHandler
    MOV PC,R0
    .long 0
    .long 0
    .long 0
    .long 0
/* ======================================================================== */
.globl CCU40_0_Veneer
CCU40_0_Veneer:
    LDR R0, =CCU40_0_IRQHandler
    MOV PC,R0
/* ======================================================================== */
.globl CCU40_1_Veneer
CCU40_1_Veneer:
    LDR R0, =CCU40_1_IRQHandler
    MOV PC,R0
/* ======================================================================== */
.globl CCU40_2_Veneer
CCU40_2_Veneer:
    LDR R0, =CCU40_2_IRQHandler
    MOV PC,R0
/* ======================================================================== */
.globl CCU40_3_Veneer
CCU40_3_Veneer:
    LDR R0, =CCU40_3_IRQHandler
    MOV PC,R0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
/* ======================================================================== */

/* ============= END OF INTERRUPT HANDLER DEFINITION ======================== */

/* ===== Decision function queried by CMSIS startup for Clock tree setup === */
/* In the absence of DAVE code engine, CMSIS SystemInit() must perform clock 
   tree setup. 
   
   This decision routine defined here will always return TRUE.
   
   When overridden by a definition defined in DAVE code engine, this routine
   returns FALSE indicating that the code engine has performed the clock setup
*/   
     .section ".XmcStartup"
    .weak   AllowClkInitByStartup
    .type   AllowClkInitByStartup, %function
AllowClkInitByStartup:
    MOVS R0,#1
    BX LR
    .size   AllowClkInitByStartup, . - AllowClkInitByStartup

/* ======  Definition of the default weak SystemInit_DAVE3 function =========
If DAVE3 requires an extended SystemInit it will create its own version of
SystemInit_DAVE3 which overrides this weak definition. Example includes
setting up of external memory interfaces.
*/
     .weak SystemInit_DAVE3
     .type SystemInit_DAVE3, %function
SystemInit_DAVE3:
     NOP
     BX LR
     .size SystemInit_DAVE3, . - SystemInit_DAVE3
/* ======================================================================== */
/* ======================================================================== */

/* ======================== Data references =============================== */

    .end
