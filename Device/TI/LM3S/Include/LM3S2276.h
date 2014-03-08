
/****************************************************************************************************//**
 * @file     LM3S2276.h
 *
 * @brief    CMSIS Cortex-M3 Core Peripheral Access Layer Header File for
 *           default LM3S2276 Device Series
 *
 * @version  V8636
 * @date     15. February 2012
 *
 * @note     Generated with SVDConv V2.73b  on Wednesday, 15.02.2012 18:03:05
 *           from CMSIS SVD File 'lm3s2276.svd.xml' Version 8636,
 *           created on Thursday, 16.02.2012 00:03:05, last modified on Thursday, 16.02.2012 00:03:05
 *******************************************************************************************************/



/** @addtogroup TI
  * @{
  */

/** @addtogroup LM3S2276
  * @{
  */

#ifndef LM3S2276_H
#define LM3S2276_H

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------  Interrupt Number Definition  ------------------------ */

typedef enum {
/* -------------------  Cortex-M3 Processor Exceptions Numbers  ------------------- */
  Reset_IRQn                    = -15,              /*!<   1  Reset Vector, invoked on Power up and warm reset                 */
  NonMaskableInt_IRQn           = -14,              /*!<   2  Non maskable Interrupt, cannot be stopped or preempted           */
  HardFault_IRQn                = -13,              /*!<   3  Hard Fault, all classes of Fault                                 */
  MemoryManagement_IRQn         = -12,              /*!<   4  Memory Management, MPU mismatch, including Access Violation
                                                         and No Match                                                          */
  BusFault_IRQn                 = -11,              /*!<   5  Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory
                                                         related Fault                                                         */
  UsageFault_IRQn               = -10,              /*!<   6  Usage Fault, i.e. Undef Instruction, Illegal State Transition    */
  SVCall_IRQn                   =  -5,              /*!<  11  System Service Call via SVC instruction                          */
  DebugMonitor_IRQn             =  -4,              /*!<  12  Debug Monitor                                                    */
  PendSV_IRQn                   =  -2,              /*!<  14  Pendable request for system service                              */
  SysTick_IRQn                  =  -1,              /*!<  15  System Tick Timer                                                */
/* ---------------------  LM3S2276 Specific Interrupt Numbers  -------------------- */
  GPIOA_IRQn                    =   0,              /*!<   0  GPIOA                                                            */
  GPIOB_IRQn                    =   1,              /*!<   1  GPIOB                                                            */
  GPIOC_IRQn                    =   2,              /*!<   2  GPIOC                                                            */
  GPIOD_IRQn                    =   3,              /*!<   3  GPIOD                                                            */
  GPIOE_IRQn                    =   4,              /*!<   4  GPIOE                                                            */
  UART0_IRQn                    =   5,              /*!<   5  UART0                                                            */
  SSI0_IRQn                     =   7,              /*!<   7  SSI0                                                             */
  I2C0_IRQn                     =   8,              /*!<   8  I2C0                                                             */
  PWM0_FAULT_IRQn               =   9,              /*!<   9  PWM0_FAULT                                                       */
  PWM0_0_IRQn                   =  10,              /*!<  10  PWM0_0                                                           */
  PWM0_1_IRQn                   =  11,              /*!<  11  PWM0_1                                                           */
  PWM0_2_IRQn                   =  12,              /*!<  12  PWM0_2                                                           */
  ADC0SS0_IRQn                  =  14,              /*!<  14  ADC0SS0                                                          */
  ADC0SS1_IRQn                  =  15,              /*!<  15  ADC0SS1                                                          */
  ADC0SS2_IRQn                  =  16,              /*!<  16  ADC0SS2                                                          */
  ADC0SS3_IRQn                  =  17,              /*!<  17  ADC0SS3                                                          */
  WATCHDOG0_IRQn                =  18,              /*!<  18  WATCHDOG0                                                        */
  TIMER0A_IRQn                  =  19,              /*!<  19  TIMER0A                                                          */
  TIMER0B_IRQn                  =  20,              /*!<  20  TIMER0B                                                          */
  TIMER1A_IRQn                  =  21,              /*!<  21  TIMER1A                                                          */
  TIMER1B_IRQn                  =  22,              /*!<  22  TIMER1B                                                          */
  TIMER2A_IRQn                  =  23,              /*!<  23  TIMER2A                                                          */
  TIMER2B_IRQn                  =  24,              /*!<  24  TIMER2B                                                          */
  SYSCTL_IRQn                   =  28,              /*!<  28  SYSCTL                                                           */
  FLASH_CTRL_IRQn               =  29,              /*!<  29  FLASH_CTRL                                                       */
  CAN0_IRQn                     =  39,              /*!<  39  CAN0                                                             */
  HIB_IRQn                      =  43,              /*!<  43  HIB                                                              */
  PWM0_3_IRQn                   =  45,              /*!<  45  PWM0_3                                                           */
  UDMA_IRQn                     =  46,              /*!<  46  UDMA                                                             */
  UDMAERR_IRQn                  =  47               /*!<  47  UDMAERR                                                          */
} IRQn_Type;


/** @addtogroup Configuration_of_CMSIS
  * @{
  */


/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* ----------------Configuration of the Cortex-M3 Processor and Core Peripherals---------------- */
#define __CM3_REV                 0x0202            /*!< Cortex-M3 Core Revision                                               */
#define __MPU_PRESENT                  1            /*!< MPU present or not                                                    */
#define __NVIC_PRIO_BITS               3            /*!< Number of Bits used for Priority Levels                               */
#define __Vendor_SysTickConfig         0            /*!< Set to 1 if different SysTick Config is used                          */
/** @} */ /* End of group Configuration_of_CMSIS */

#include <core_cm3.h>                               /*!< Cortex-M3 processor and core peripherals                              */
#include "system_LM3S.h"                            /*!< LM3S2276 System                                                       */


/* ================================================================================ */
/* ================       Device Specific Peripheral Section       ================ */
/* ================================================================================ */


/** @addtogroup Device_Peripheral_Registers
  * @{
  */


/* -------------------  Start of section using anonymous unions  ------------------ */
#if defined(__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined(__ICCARM__)
  #pragma language=extended
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning 586
#else
  #warning Not supported compiler type
#endif



/* ================================================================================ */
/* ================                    WATCHDOG0                   ================ */
/* ================================================================================ */


/**
  * @brief Register map for WATCHDOG0 peripheral (WATCHDOG0)
  */

typedef struct {                                    /*!< WATCHDOG0 Structure                                                   */
  __IO uint32_t  LOAD;                              /*!< Watchdog Load                                                         */
  __IO uint32_t  VALUE;                             /*!< Watchdog Value                                                        */
  __IO uint32_t  CTL;                               /*!< Watchdog Control                                                      */
  __O  uint32_t  ICR;                               /*!< Watchdog Interrupt Clear                                              */
  __IO uint32_t  RIS;                               /*!< Watchdog Raw Interrupt Status                                         */
  __IO uint32_t  MIS;                               /*!< Watchdog Masked Interrupt Status                                      */
  __I  uint32_t  RESERVED0[256];
  __IO uint32_t  TEST;                              /*!< Watchdog Test                                                         */
  __I  uint32_t  RESERVED1[505];
  __IO uint32_t  LOCK;                              /*!< Watchdog Lock                                                         */
} WATCHDOG0_Type;


/* ================================================================================ */
/* ================                      GPIOA                     ================ */
/* ================================================================================ */


/**
  * @brief Register map for GPIOA peripheral (GPIOA)
  */

typedef struct {                                    /*!< GPIOA Structure                                                       */
  __I  uint32_t  RESERVED0[255];
  __IO uint32_t  DATA;                              /*!< GPIO Data                                                             */
  __IO uint32_t  DIR;                               /*!< GPIO Direction                                                        */
  __IO uint32_t  IS;                                /*!< GPIO Interrupt Sense                                                  */
  __IO uint32_t  IBE;                               /*!< GPIO Interrupt Both Edges                                             */
  __IO uint32_t  IEV;                               /*!< GPIO Interrupt Event                                                  */
  __IO uint32_t  IM;                                /*!< GPIO Interrupt Mask                                                   */
  __IO uint32_t  RIS;                               /*!< GPIO Raw Interrupt Status                                             */
  __IO uint32_t  MIS;                               /*!< GPIO Masked Interrupt Status                                          */
  __O  uint32_t  ICR;                               /*!< GPIO Interrupt Clear                                                  */
  __IO uint32_t  AFSEL;                             /*!< GPIO Alternate Function Select                                        */
  __I  uint32_t  RESERVED1[55];
  __IO uint32_t  DR2R;                              /*!< GPIO 2-mA Drive Select                                                */
  __IO uint32_t  DR4R;                              /*!< GPIO 4-mA Drive Select                                                */
  __IO uint32_t  DR8R;                              /*!< GPIO 8-mA Drive Select                                                */
  __IO uint32_t  ODR;                               /*!< GPIO Open Drain Select                                                */
  __IO uint32_t  PUR;                               /*!< GPIO Pull-Up Select                                                   */
  __IO uint32_t  PDR;                               /*!< GPIO Pull-Down Select                                                 */
  __IO uint32_t  SLR;                               /*!< GPIO Slew Rate Control Select                                         */
  __IO uint32_t  DEN;                               /*!< GPIO Digital Enable                                                   */
  __IO uint32_t  LOCK;                              /*!< GPIO Lock                                                             */
  __I  uint32_t  CR;                                /*!< GPIO Commit                                                           */
  __IO uint32_t  AMSEL;                             /*!< GPIO Analog Mode Select                                               */
} GPIOA_Type;


/* ================================================================================ */
/* ================                      SSI0                      ================ */
/* ================================================================================ */


/**
  * @brief Register map for SSI0 peripheral (SSI0)
  */

typedef struct {                                    /*!< SSI0 Structure                                                        */
  __IO uint32_t  CR0;                               /*!< SSI Control 0                                                         */
  __IO uint32_t  CR1;                               /*!< SSI Control 1                                                         */
  __IO uint32_t  DR;                                /*!< SSI Data                                                              */
  __IO uint32_t  SR;                                /*!< SSI Status                                                            */
  __IO uint32_t  CPSR;                              /*!< SSI Clock Prescale                                                    */
  __IO uint32_t  IM;                                /*!< SSI Interrupt Mask                                                    */
  __IO uint32_t  RIS;                               /*!< SSI Raw Interrupt Status                                              */
  __IO uint32_t  MIS;                               /*!< SSI Masked Interrupt Status                                           */
  __O  uint32_t  ICR;                               /*!< SSI Interrupt Clear                                                   */
  __IO uint32_t  DMACTL;                            /*!< SSI DMA Control                                                       */
} SSI0_Type;


/* ================================================================================ */
/* ================                      UART0                     ================ */
/* ================================================================================ */


/**
  * @brief Register map for UART0 peripheral (UART0)
  */

typedef struct {                                    /*!< UART0 Structure                                                       */
  __IO uint32_t  DR;                                /*!< UART Data                                                             */
  
  union {
    __IO uint32_t  UART_ALT_ECR;                    /*!< UART Receive Status/Error Clear                                       */
    __IO uint32_t  RSR;                             /*!< UART Receive Status/Error Clear                                       */
  } ;
  __I  uint32_t  RESERVED0[4];
  __IO uint32_t  FR;                                /*!< UART Flag                                                             */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  ILPR;                              /*!< UART IrDA Low-Power Register                                          */
  __IO uint32_t  IBRD;                              /*!< UART Integer Baud-Rate Divisor                                        */
  __IO uint32_t  FBRD;                              /*!< UART Fractional Baud-Rate Divisor                                     */
  __IO uint32_t  LCRH;                              /*!< UART Line Control                                                     */
  __IO uint32_t  CTL;                               /*!< UART Control                                                          */
  __IO uint32_t  IFLS;                              /*!< UART Interrupt FIFO Level Select                                      */
  __IO uint32_t  IM;                                /*!< UART Interrupt Mask                                                   */
  __IO uint32_t  RIS;                               /*!< UART Raw Interrupt Status                                             */
  __IO uint32_t  MIS;                               /*!< UART Masked Interrupt Status                                          */
  __O  uint32_t  ICR;                               /*!< UART Interrupt Clear                                                  */
  __IO uint32_t  DMACTL;                            /*!< UART DMA Control                                                      */
} UART0_Type;


/* ================================================================================ */
/* ================                      I2C0                      ================ */
/* ================================================================================ */


/**
  * @brief Register map for I2C0 peripheral (I2C0)
  */

typedef struct {                                    /*!< I2C0 Structure                                                        */
  __IO uint32_t  MSA;                               /*!< I2C Master Slave Address                                              */
  
  union {
    __IO uint32_t  I2C0_ALT_MCS;                    /*!< I2C Master Control/Status                                             */
    __IO uint32_t  MCS;                             /*!< I2C Master Control/Status                                             */
  } ;
  __IO uint32_t  MDR;                               /*!< I2C Master Data                                                       */
  __IO uint32_t  MTPR;                              /*!< I2C Master Timer Period                                               */
  __IO uint32_t  MIMR;                              /*!< I2C Master Interrupt Mask                                             */
  __IO uint32_t  MRIS;                              /*!< I2C Master Raw Interrupt Status                                       */
  __IO uint32_t  MMIS;                              /*!< I2C Master Masked Interrupt Status                                    */
  __O  uint32_t  MICR;                              /*!< I2C Master Interrupt Clear                                            */
  __IO uint32_t  MCR;                               /*!< I2C Master Configuration                                              */
  __I  uint32_t  RESERVED0[503];
  __IO uint32_t  SOAR;                              /*!< I2C Slave Own Address                                                 */
  
  union {
    __IO uint32_t  I2C0_ALT_SCSR;                   /*!< I2C Slave Control/Status                                              */
    __IO uint32_t  SCSR;                            /*!< I2C Slave Control/Status                                              */
  } ;
  __IO uint32_t  SDR;                               /*!< I2C Slave Data                                                        */
  __IO uint32_t  SIMR;                              /*!< I2C Slave Interrupt Mask                                              */
  __IO uint32_t  SRIS;                              /*!< I2C Slave Raw Interrupt Status                                        */
  __IO uint32_t  SMIS;                              /*!< I2C Slave Masked Interrupt Status                                     */
  __O  uint32_t  SICR;                              /*!< I2C Slave Interrupt Clear                                             */
} I2C0_Type;


/* ================================================================================ */
/* ================                      PWM0                      ================ */
/* ================================================================================ */


/**
  * @brief Register map for PWM0 peripheral (PWM0)
  */

typedef struct {                                    /*!< PWM0 Structure                                                        */
  __IO uint32_t  CTL;                               /*!< PWM Master Control                                                    */
  __IO uint32_t  SYNC;                              /*!< PWM Time Base Sync                                                    */
  __IO uint32_t  ENABLE;                            /*!< PWM Output Enable                                                     */
  __IO uint32_t  INVERT;                            /*!< PWM Output Inversion                                                  */
  __IO uint32_t  FAULT;                             /*!< PWM Output Fault                                                      */
  __IO uint32_t  INTEN;                             /*!< PWM Interrupt Enable                                                  */
  __IO uint32_t  RIS;                               /*!< PWM Raw Interrupt Status                                              */
  __IO uint32_t  ISC;                               /*!< PWM Interrupt Status and Clear                                        */
  __IO uint32_t  STATUS;                            /*!< PWM Status                                                            */
  __IO uint32_t  FAULTVAL;                          /*!< PWM Fault Condition Value                                             */
  __I  uint32_t  RESERVED0[6];
  __IO uint32_t  _0_CTL;                            /*!< PWM0 Control                                                          */
  __IO uint32_t  _0_INTEN;                          /*!< PWM0 Interrupt and Trigger Enable                                     */
  __IO uint32_t  _0_RIS;                            /*!< PWM0 Raw Interrupt Status                                             */
  __IO uint32_t  _0_ISC;                            /*!< PWM0 Interrupt Status and Clear                                       */
  __IO uint32_t  _0_LOAD;                           /*!< PWM0 Load                                                             */
  __IO uint32_t  _0_COUNT;                          /*!< PWM0 Counter                                                          */
  __IO uint32_t  _0_CMPA;                           /*!< PWM0 Compare A                                                        */
  __IO uint32_t  _0_CMPB;                           /*!< PWM0 Compare B                                                        */
  __IO uint32_t  _0_GENA;                           /*!< PWM0 Generator A Control                                              */
  __IO uint32_t  _0_GENB;                           /*!< PWM0 Generator B Control                                              */
  __IO uint32_t  _0_DBCTL;                          /*!< PWM0 Dead-Band Control                                                */
  __IO uint32_t  _0_DBRISE;                         /*!< PWM0 Dead-Band Rising-Edge Delay                                      */
  __IO uint32_t  _0_DBFALL;                         /*!< PWM0 Dead-Band Falling-Edge-Delay                                     */
  __IO uint32_t  _0_FLTSRC0;                        /*!< PWM0 Fault Source 0                                                   */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  _0_MINFLTPER;                      /*!< PWM0 Minimum Fault Period                                             */
  __IO uint32_t  _1_CTL;                            /*!< PWM1 Control                                                          */
  __IO uint32_t  _1_INTEN;                          /*!< PWM1 Interrupt and Trigger Enable                                     */
  __IO uint32_t  _1_RIS;                            /*!< PWM1 Raw Interrupt Status                                             */
  __IO uint32_t  _1_ISC;                            /*!< PWM1 Interrupt Status and Clear                                       */
  __IO uint32_t  _1_LOAD;                           /*!< PWM1 Load                                                             */
  __IO uint32_t  _1_COUNT;                          /*!< PWM1 Counter                                                          */
  __IO uint32_t  _1_CMPA;                           /*!< PWM1 Compare A                                                        */
  __IO uint32_t  _1_CMPB;                           /*!< PWM1 Compare B                                                        */
  __IO uint32_t  _1_GENA;                           /*!< PWM1 Generator A Control                                              */
  __IO uint32_t  _1_GENB;                           /*!< PWM1 Generator B Control                                              */
  __IO uint32_t  _1_DBCTL;                          /*!< PWM1 Dead-Band Control                                                */
  __IO uint32_t  _1_DBRISE;                         /*!< PWM1 Dead-Band Rising-Edge Delay                                      */
  __IO uint32_t  _1_DBFALL;                         /*!< PWM1 Dead-Band Falling-Edge-Delay                                     */
  __IO uint32_t  _1_FLTSRC0;                        /*!< PWM1 Fault Source 0                                                   */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  _1_MINFLTPER;                      /*!< PWM1 Minimum Fault Period                                             */
  __IO uint32_t  _2_CTL;                            /*!< PWM2 Control                                                          */
  __IO uint32_t  _2_INTEN;                          /*!< PWM2 Interrupt and Trigger Enable                                     */
  __IO uint32_t  _2_RIS;                            /*!< PWM2 Raw Interrupt Status                                             */
  __IO uint32_t  _2_ISC;                            /*!< PWM2 Interrupt Status and Clear                                       */
  __IO uint32_t  _2_LOAD;                           /*!< PWM2 Load                                                             */
  __IO uint32_t  _2_COUNT;                          /*!< PWM2 Counter                                                          */
  __IO uint32_t  _2_CMPA;                           /*!< PWM2 Compare A                                                        */
  __IO uint32_t  _2_CMPB;                           /*!< PWM2 Compare B                                                        */
  __IO uint32_t  _2_GENA;                           /*!< PWM2 Generator A Control                                              */
  __IO uint32_t  _2_GENB;                           /*!< PWM2 Generator B Control                                              */
  __IO uint32_t  _2_DBCTL;                          /*!< PWM2 Dead-Band Control                                                */
  __IO uint32_t  _2_DBRISE;                         /*!< PWM2 Dead-Band Rising-Edge Delay                                      */
  __IO uint32_t  _2_DBFALL;                         /*!< PWM2 Dead-Band Falling-Edge-Delay                                     */
  __IO uint32_t  _2_FLTSRC0;                        /*!< PWM2 Fault Source 0                                                   */
  __I  uint32_t  RESERVED3;
  __IO uint32_t  _2_MINFLTPER;                      /*!< PWM2 Minimum Fault Period                                             */
  __IO uint32_t  _3_CTL;                            /*!< PWM3 Control                                                          */
  __IO uint32_t  _3_INTEN;                          /*!< PWM3 Interrupt and Trigger Enable                                     */
  __IO uint32_t  _3_RIS;                            /*!< PWM3 Raw Interrupt Status                                             */
  __IO uint32_t  _3_ISC;                            /*!< PWM3 Interrupt Status and Clear                                       */
  __IO uint32_t  _3_LOAD;                           /*!< PWM3 Load                                                             */
  __IO uint32_t  _3_COUNT;                          /*!< PWM3 Counter                                                          */
  __IO uint32_t  _3_CMPA;                           /*!< PWM3 Compare A                                                        */
  __IO uint32_t  _3_CMPB;                           /*!< PWM3 Compare B                                                        */
  __IO uint32_t  _3_GENA;                           /*!< PWM3 Generator A Control                                              */
  __IO uint32_t  _3_GENB;                           /*!< PWM3 Generator B Control                                              */
  __IO uint32_t  _3_DBCTL;                          /*!< PWM3 Dead-Band Control                                                */
  __IO uint32_t  _3_DBRISE;                         /*!< PWM3 Dead-Band Rising-Edge Delay                                      */
  __IO uint32_t  _3_DBFALL;                         /*!< PWM3 Dead-Band Falling-Edge-Delay                                     */
  __IO uint32_t  _3_FLTSRC0;                        /*!< PWM3 Fault Source 0                                                   */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  _3_MINFLTPER;                      /*!< PWM3 Minimum Fault Period                                             */
  __I  uint32_t  RESERVED5[432];
  __IO uint32_t  _0_FLTSEN;                         /*!< PWM0 Fault Pin Logic Sense                                            */
  __I  uint32_t  _0_FLTSTAT0;                       /*!< PWM0 Fault Status 0                                                   */
  __I  uint32_t  RESERVED6[30];
  __IO uint32_t  _1_FLTSEN;                         /*!< PWM1 Fault Pin Logic Sense                                            */
  __I  uint32_t  _1_FLTSTAT0;                       /*!< PWM1 Fault Status 0                                                   */
  __I  uint32_t  RESERVED7[30];
  __IO uint32_t  _2_FLTSEN;                         /*!< PWM2 Fault Pin Logic Sense                                            */
  __I  uint32_t  _2_FLTSTAT0;                       /*!< PWM2 Fault Status 0                                                   */
  __I  uint32_t  RESERVED8[31];
  __I  uint32_t  _3_FLTSTAT0;                       /*!< PWM3 Fault Status 0                                                   */
} PWM0_Type;


/* ================================================================================ */
/* ================                     TIMER0                     ================ */
/* ================================================================================ */


/**
  * @brief Register map for TIMER0 peripheral (TIMER0)
  */

typedef struct {                                    /*!< TIMER0 Structure                                                      */
  __IO uint32_t  CFG;                               /*!< GPTM Configuration                                                    */
  __IO uint32_t  TAMR;                              /*!< GPTM Timer A Mode                                                     */
  __IO uint32_t  TBMR;                              /*!< GPTM Timer B Mode                                                     */
  __IO uint32_t  CTL;                               /*!< GPTM Control                                                          */
  __I  uint32_t  RESERVED0[2];
  __IO uint32_t  IMR;                               /*!< GPTM Interrupt Mask                                                   */
  __IO uint32_t  RIS;                               /*!< GPTM Raw Interrupt Status                                             */
  __IO uint32_t  MIS;                               /*!< GPTM Masked Interrupt Status                                          */
  __O  uint32_t  ICR;                               /*!< GPTM Interrupt Clear                                                  */
  __IO uint32_t  TAILR;                             /*!< GPTM Timer A Interval Load                                            */
  __IO uint32_t  TBILR;                             /*!< GPTM Timer B Interval Load                                            */
  __IO uint32_t  TAMATCHR;                          /*!< GPTM Timer A Match                                                    */
  __IO uint32_t  TBMATCHR;                          /*!< GPTM Timer B Match                                                    */
  __IO uint32_t  TAPR;                              /*!< GPTM Timer A Prescale                                                 */
  __IO uint32_t  TBPR;                              /*!< GPTM Timer B Prescale                                                 */
  __I  uint32_t  RESERVED1[2];
  __IO uint32_t  TAR;                               /*!< GPTM Timer A                                                          */
  __IO uint32_t  TBR;                               /*!< GPTM Timer B                                                          */
} TIMER0_Type;


/* ================================================================================ */
/* ================                      ADC0                      ================ */
/* ================================================================================ */


/**
  * @brief Register map for ADC0 peripheral (ADC0)
  */

typedef struct {                                    /*!< ADC0 Structure                                                        */
  __IO uint32_t  ACTSS;                             /*!< ADC Active Sample Sequencer                                           */
  __IO uint32_t  RIS;                               /*!< ADC Raw Interrupt Status                                              */
  __IO uint32_t  IM;                                /*!< ADC Interrupt Mask                                                    */
  __IO uint32_t  ISC;                               /*!< ADC Interrupt Status and Clear                                        */
  __IO uint32_t  OSTAT;                             /*!< ADC Overflow Status                                                   */
  __IO uint32_t  EMUX;                              /*!< ADC Event Multiplexer Select                                          */
  __IO uint32_t  USTAT;                             /*!< ADC Underflow Status                                                  */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  SSPRI;                             /*!< ADC Sample Sequencer Priority                                         */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  PSSI;                              /*!< ADC Processor Sample Sequence Initiate                                */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  SAC;                               /*!< ADC Sample Averaging Control                                          */
  __I  uint32_t  RESERVED3[3];
  __IO uint32_t  SSMUX0;                            /*!< ADC Sample Sequence Input Multiplexer Select 0                        */
  __IO uint32_t  SSCTL0;                            /*!< ADC Sample Sequence Control 0                                         */
  __IO uint32_t  SSFIFO0;                           /*!< ADC Sample Sequence Result FIFO 0                                     */
  __IO uint32_t  SSFSTAT0;                          /*!< ADC Sample Sequence FIFO 0 Status                                     */
  __I  uint32_t  RESERVED4[4];
  __IO uint32_t  SSMUX1;                            /*!< ADC Sample Sequence Input Multiplexer Select 1                        */
  __IO uint32_t  SSCTL1;                            /*!< ADC Sample Sequence Control 1                                         */
  __IO uint32_t  SSFIFO1;                           /*!< ADC Sample Sequence Result FIFO 1                                     */
  __IO uint32_t  SSFSTAT1;                          /*!< ADC Sample Sequence FIFO 1 Status                                     */
  __I  uint32_t  RESERVED5[4];
  __IO uint32_t  SSMUX2;                            /*!< ADC Sample Sequence Input Multiplexer Select 2                        */
  __IO uint32_t  SSCTL2;                            /*!< ADC Sample Sequence Control 2                                         */
  __IO uint32_t  SSFIFO2;                           /*!< ADC Sample Sequence Result FIFO 2                                     */
  __IO uint32_t  SSFSTAT2;                          /*!< ADC Sample Sequence FIFO 2 Status                                     */
  __I  uint32_t  RESERVED6[4];
  __IO uint32_t  SSMUX3;                            /*!< ADC Sample Sequence Input Multiplexer Select 3                        */
  __IO uint32_t  SSCTL3;                            /*!< ADC Sample Sequence Control 3                                         */
  __IO uint32_t  SSFIFO3;                           /*!< ADC Sample Sequence Result FIFO 3                                     */
  __IO uint32_t  SSFSTAT3;                          /*!< ADC Sample Sequence FIFO 3 Status                                     */
} ADC0_Type;


/* ================================================================================ */
/* ================                      CAN0                      ================ */
/* ================================================================================ */


/**
  * @brief Register map for CAN0 peripheral (CAN0)
  */

typedef struct {                                    /*!< CAN0 Structure                                                        */
  __IO uint32_t  CTL;                               /*!< CAN Control                                                           */
  __IO uint32_t  STS;                               /*!< CAN Status                                                            */
  __IO uint32_t  ERR;                               /*!< CAN Error Counter                                                     */
  __IO uint32_t  BIT;                               /*!< CAN Bit Timing                                                        */
  __IO uint32_t  INT;                               /*!< CAN Interrupt                                                         */
  __IO uint32_t  TST;                               /*!< CAN Test                                                              */
  __IO uint32_t  BRPE;                              /*!< CAN Baud Rate Prescaler Extension                                     */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  IF1CRQ;                            /*!< CAN IF1 Command Request                                               */
  
  union {
    __IO uint32_t  CAN0_ALT_IF1CMSK;                /*!< CAN IF1 Command Mask                                                  */
    __IO uint32_t  IF1CMSK;                         /*!< CAN IF1 Command Mask                                                  */
  } ;
  __IO uint32_t  IF1MSK1;                           /*!< CAN IF1 Mask 1                                                        */
  __IO uint32_t  IF1MSK2;                           /*!< CAN IF1 Mask 2                                                        */
  __IO uint32_t  IF1ARB1;                           /*!< CAN IF1 Arbitration 1                                                 */
  __IO uint32_t  IF1ARB2;                           /*!< CAN IF1 Arbitration 2                                                 */
  __IO uint32_t  IF1MCTL;                           /*!< CAN IF1 Message Control                                               */
  __IO uint32_t  IF1DA1;                            /*!< CAN IF1 Data A1                                                       */
  __IO uint32_t  IF1DA2;                            /*!< CAN IF1 Data A2                                                       */
  __IO uint32_t  IF1DB1;                            /*!< CAN IF1 Data B1                                                       */
  __IO uint32_t  IF1DB2;                            /*!< CAN IF1 Data B2                                                       */
  __I  uint32_t  RESERVED1[13];
  __IO uint32_t  IF2CRQ;                            /*!< CAN IF2 Command Request                                               */
  
  union {
    __IO uint32_t  CAN0_ALT_IF2CMSK;                /*!< CAN IF2 Command Mask                                                  */
    __IO uint32_t  IF2CMSK;                         /*!< CAN IF2 Command Mask                                                  */
  } ;
  __IO uint32_t  IF2MSK1;                           /*!< CAN IF2 Mask 1                                                        */
  __IO uint32_t  IF2MSK2;                           /*!< CAN IF2 Mask 2                                                        */
  __IO uint32_t  IF2ARB1;                           /*!< CAN IF2 Arbitration 1                                                 */
  __IO uint32_t  IF2ARB2;                           /*!< CAN IF2 Arbitration 2                                                 */
  __IO uint32_t  IF2MCTL;                           /*!< CAN IF2 Message Control                                               */
  __IO uint32_t  IF2DA1;                            /*!< CAN IF2 Data A1                                                       */
  __IO uint32_t  IF2DA2;                            /*!< CAN IF2 Data A2                                                       */
  __IO uint32_t  IF2DB1;                            /*!< CAN IF2 Data B1                                                       */
  __IO uint32_t  IF2DB2;                            /*!< CAN IF2 Data B2                                                       */
  __I  uint32_t  RESERVED2[21];
  __IO uint32_t  TXRQ1;                             /*!< CAN Transmission Request 1                                            */
  __IO uint32_t  TXRQ2;                             /*!< CAN Transmission Request 2                                            */
  __I  uint32_t  RESERVED3[6];
  __IO uint32_t  NWDA1;                             /*!< CAN New Data 1                                                        */
  __IO uint32_t  NWDA2;                             /*!< CAN New Data 2                                                        */
  __I  uint32_t  RESERVED4[6];
  __IO uint32_t  MSG1INT;                           /*!< CAN Message 1 Interrupt Pending                                       */
  __IO uint32_t  MSG2INT;                           /*!< CAN Message 2 Interrupt Pending                                       */
  __I  uint32_t  RESERVED5[6];
  __IO uint32_t  MSG1VAL;                           /*!< CAN Message 1 Valid                                                   */
  __IO uint32_t  MSG2VAL;                           /*!< CAN Message 2 Valid                                                   */
} CAN0_Type;


/* ================================================================================ */
/* ================                       HIB                      ================ */
/* ================================================================================ */


/**
  * @brief Register map for HIB peripheral (HIB)
  */

typedef struct {                                    /*!< HIB Structure                                                         */
  __IO uint32_t  RTCC;                              /*!< Hibernation RTC Counter                                               */
  __IO uint32_t  RTCM0;                             /*!< Hibernation RTC Match 0                                               */
  __IO uint32_t  RTCM1;                             /*!< Hibernation RTC Match 1                                               */
  __IO uint32_t  RTCLD;                             /*!< Hibernation RTC Load                                                  */
  __IO uint32_t  CTL;                               /*!< Hibernation Control                                                   */
  __IO uint32_t  IM;                                /*!< Hibernation Interrupt Mask                                            */
  __IO uint32_t  RIS;                               /*!< Hibernation Raw Interrupt Status                                      */
  __IO uint32_t  MIS;                               /*!< Hibernation Masked Interrupt Status                                   */
  __IO uint32_t  IC;                                /*!< Hibernation Interrupt Clear                                           */
  __IO uint32_t  RTCT;                              /*!< Hibernation RTC Trim                                                  */
  __I  uint32_t  RESERVED0[2];
  __IO uint32_t  DATA;                              /*!< Hibernation Data                                                      */
} HIB_Type;


/* ================================================================================ */
/* ================                   FLASH_CTRL                   ================ */
/* ================================================================================ */


/**
  * @brief Register map for FLASH_CTRL peripheral (FLASH_CTRL)
  */

typedef struct {                                    /*!< FLASH_CTRL Structure                                                  */
  __IO uint32_t  FMA;                               /*!< Flash Memory Address                                                  */
  __IO uint32_t  FMD;                               /*!< Flash Memory Data                                                     */
  __IO uint32_t  FMC;                               /*!< Flash Memory Control                                                  */
  __IO uint32_t  FCRIS;                             /*!< Flash Controller Raw Interrupt Status                                 */
  __IO uint32_t  FCIM;                              /*!< Flash Controller Interrupt Mask                                       */
  __IO uint32_t  FCMISC;                            /*!< Flash Controller Masked Interrupt Status and Clear                    */
  __I  uint32_t  RESERVED0[1078];
  __IO uint32_t  RMCTL;                             /*!< ROM Control                                                           */
  __I  uint32_t  RESERVED1[19];
  __IO uint32_t  USECRL;                            /*!< USec Reload                                                           */
  __I  uint32_t  RESERVED2[35];
  __IO uint32_t  FLASH_ALT_USERDBG;                 /*!< User Debug                                                            */
  __I  uint32_t  RESERVED3[3];
  __IO uint32_t  USERREG0;                          /*!< User Register 0                                                       */
  __IO uint32_t  USERREG1;                          /*!< User Register 1                                                       */
  __IO uint32_t  USERREG2;                          /*!< User Register 2                                                       */
  __IO uint32_t  USERREG3;                          /*!< User Register 3                                                       */
  __I  uint32_t  RESERVED4[4];
  __IO uint32_t  FMPRE0;                            /*!< Flash Memory Protection Read Enable 0                                 */
  __IO uint32_t  FMPRE1;                            /*!< Flash Memory Protection Read Enable 1                                 */
  __IO uint32_t  FMPRE2;                            /*!< Flash Memory Protection Read Enable 2                                 */
  __IO uint32_t  FMPRE3;                            /*!< Flash Memory Protection Read Enable 3                                 */
  __I  uint32_t  RESERVED5[124];
  __IO uint32_t  FMPPE0;                            /*!< Flash Memory Protection Program Enable 0                              */
  __IO uint32_t  FMPPE1;                            /*!< Flash Memory Protection Program Enable 1                              */
  __IO uint32_t  FMPPE2;                            /*!< Flash Memory Protection Program Enable 2                              */
  __IO uint32_t  FMPPE3;                            /*!< Flash Memory Protection Program Enable 3                              */
} FLASH_CTRL_Type;


/* ================================================================================ */
/* ================                     SYSCTL                     ================ */
/* ================================================================================ */


/**
  * @brief Register map for SYSCTL peripheral (SYSCTL)
  */

typedef struct {                                    /*!< SYSCTL Structure                                                      */
  __IO uint32_t  DID0;                              /*!< Device Identification 0                                               */
  __IO uint32_t  DID1;                              /*!< Device Identification 1                                               */
  __IO uint32_t  DC0;                               /*!< Device Capabilities 0                                                 */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  DC1;                               /*!< Device Capabilities 1                                                 */
  __IO uint32_t  DC2;                               /*!< Device Capabilities 2                                                 */
  __IO uint32_t  DC3;                               /*!< Device Capabilities 3                                                 */
  __IO uint32_t  DC4;                               /*!< Device Capabilities 4                                                 */
  __IO uint32_t  DC5;                               /*!< Device Capabilities 5                                                 */
  __IO uint32_t  DC6;                               /*!< Device Capabilities 6                                                 */
  __IO uint32_t  DC7;                               /*!< Device Capabilities 7                                                 */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  PBORCTL;                           /*!< Brown-Out Reset Control                                               */
  __IO uint32_t  LDOPCTL;                           /*!< LDO Power Control                                                     */
  __I  uint32_t  RESERVED2[2];
  __IO uint32_t  SRCR0;                             /*!< Software Reset Control 0                                              */
  __IO uint32_t  SRCR1;                             /*!< Software Reset Control 1                                              */
  __IO uint32_t  SRCR2;                             /*!< Software Reset Control 2                                              */
  __I  uint32_t  RESERVED3;
  __IO uint32_t  RIS;                               /*!< Raw Interrupt Status                                                  */
  __IO uint32_t  IMC;                               /*!< Interrupt Mask Control                                                */
  __IO uint32_t  MISC;                              /*!< Masked Interrupt Status and Clear                                     */
  __IO uint32_t  RESC;                              /*!< Reset Cause                                                           */
  __IO uint32_t  RCC;                               /*!< Run-Mode Clock Configuration                                          */
  __IO uint32_t  PLLCFG;                            /*!< XTAL to PLL Translation                                               */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  GPIOHBCTL;                         /*!< GPIO High-Performance Bus Control                                     */
  __IO uint32_t  RCC2;                              /*!< Run-Mode Clock Configuration 2                                        */
  __I  uint32_t  RESERVED5[2];
  __IO uint32_t  MOSCCTL;                           /*!< Main Oscillator Control                                               */
  __I  uint32_t  RESERVED6[32];
  __IO uint32_t  RCGC0;                             /*!< Run Mode Clock Gating Control Register 0                              */
  __IO uint32_t  RCGC1;                             /*!< Run Mode Clock Gating Control Register 1                              */
  __IO uint32_t  RCGC2;                             /*!< Run Mode Clock Gating Control Register 2                              */
  __I  uint32_t  RESERVED7;
  __IO uint32_t  SCGC0;                             /*!< Sleep Mode Clock Gating Control Register 0                            */
  __IO uint32_t  SCGC1;                             /*!< Sleep Mode Clock Gating Control Register 1                            */
  __IO uint32_t  SCGC2;                             /*!< Sleep Mode Clock Gating Control Register 2                            */
  __I  uint32_t  RESERVED8;
  __IO uint32_t  DCGC0;                             /*!< Deep Sleep Mode Clock Gating Control Register 0                       */
  __IO uint32_t  DCGC1;                             /*!< Deep-Sleep Mode Clock Gating Control Register 1                       */
  __IO uint32_t  DCGC2;                             /*!< Deep Sleep Mode Clock Gating Control Register 2                       */
  __I  uint32_t  RESERVED9[6];
  __IO uint32_t  DSLPCLKCFG;                        /*!< Deep Sleep Clock Configuration                                        */
} SYSCTL_Type;


/* ================================================================================ */
/* ================                      UDMA                      ================ */
/* ================================================================================ */


/**
  * @brief Register map for UDMA peripheral (UDMA)
  */

typedef struct {                                    /*!< UDMA Structure                                                        */
  __IO uint32_t  STAT;                              /*!< DMA Status                                                            */
  __O  uint32_t  CFG;                               /*!< DMA Configuration                                                     */
  __IO uint32_t  CTLBASE;                           /*!< DMA Channel Control Base Pointer                                      */
  __IO uint32_t  ALTBASE;                           /*!< DMA Alternate Channel Control Base Pointer                            */
  __IO uint32_t  WAITSTAT;                          /*!< DMA Channel Wait-on-Request Status                                    */
  __O  uint32_t  SWREQ;                             /*!< DMA Channel Software Request                                          */
  __IO uint32_t  USEBURSTSET;                       /*!< DMA Channel Useburst Set                                              */
  __O  uint32_t  USEBURSTCLR;                       /*!< DMA Channel Useburst Clear                                            */
  __IO uint32_t  REQMASKSET;                        /*!< DMA Channel Request Mask Set                                          */
  __O  uint32_t  REQMASKCLR;                        /*!< DMA Channel Request Mask Clear                                        */
  __IO uint32_t  ENASET;                            /*!< DMA Channel Enable Set                                                */
  __O  uint32_t  ENACLR;                            /*!< DMA Channel Enable Clear                                              */
  __IO uint32_t  ALTSET;                            /*!< DMA Channel Primary Alternate Set                                     */
  __O  uint32_t  ALTCLR;                            /*!< DMA Channel Primary Alternate Clear                                   */
  __IO uint32_t  PRIOSET;                           /*!< DMA Channel Priority Set                                              */
  __O  uint32_t  PRIOCLR;                           /*!< DMA Channel Priority Clear                                            */
  __I  uint32_t  RESERVED0[3];
  __IO uint32_t  ERRCLR;                            /*!< DMA Bus Error Clear                                                   */
} UDMA_Type;


/* --------------------  End of section using anonymous unions  ------------------- */
#if defined(__CC_ARM)
  #pragma pop
#elif defined(__ICCARM__)
  /* leave anonymous unions enabled */
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning restore
#else
  #warning Not supported compiler type
#endif




/* ================================================================================ */
/* ================              Peripheral memory map             ================ */
/* ================================================================================ */

#define WATCHDOG0_BASE                  0x40000000UL
#define GPIOA_BASE                      0x40004000UL
#define GPIOB_BASE                      0x40005000UL
#define GPIOC_BASE                      0x40006000UL
#define GPIOD_BASE                      0x40007000UL
#define SSI0_BASE                       0x40008000UL
#define UART0_BASE                      0x4000C000UL
#define I2C0_BASE                       0x40020000UL
#define GPIOE_BASE                      0x40024000UL
#define PWM0_BASE                       0x40028000UL
#define TIMER0_BASE                     0x40030000UL
#define TIMER1_BASE                     0x40031000UL
#define TIMER2_BASE                     0x40032000UL
#define ADC0_BASE                       0x40038000UL
#define CAN0_BASE                       0x40040000UL
#define GPIOA_AHB_BASE                  0x40058000UL
#define GPIOB_AHB_BASE                  0x40059000UL
#define GPIOC_AHB_BASE                  0x4005A000UL
#define GPIOD_AHB_BASE                  0x4005B000UL
#define GPIOE_AHB_BASE                  0x4005C000UL
#define HIB_BASE                        0x400FC000UL
#define FLASH_CTRL_BASE                 0x400FD000UL
#define SYSCTL_BASE                     0x400FE000UL
#define UDMA_BASE                       0x400FF000UL


/* ================================================================================ */
/* ================             Peripheral declaration             ================ */
/* ================================================================================ */

#define WATCHDOG0                       ((WATCHDOG0_Type          *) WATCHDOG0_BASE)
#define GPIOA                           ((GPIOA_Type              *) GPIOA_BASE)
#define GPIOB                           ((GPIOA_Type              *) GPIOB_BASE)
#define GPIOC                           ((GPIOA_Type              *) GPIOC_BASE)
#define GPIOD                           ((GPIOA_Type              *) GPIOD_BASE)
#define SSI0                            ((SSI0_Type               *) SSI0_BASE)
#define UART0                           ((UART0_Type              *) UART0_BASE)
#define I2C0                            ((I2C0_Type               *) I2C0_BASE)
#define GPIOE                           ((GPIOA_Type              *) GPIOE_BASE)
#define PWM0                            ((PWM0_Type               *) PWM0_BASE)
#define TIMER0                          ((TIMER0_Type             *) TIMER0_BASE)
#define TIMER1                          ((TIMER0_Type             *) TIMER1_BASE)
#define TIMER2                          ((TIMER0_Type             *) TIMER2_BASE)
#define ADC0                            ((ADC0_Type               *) ADC0_BASE)
#define CAN0                            ((CAN0_Type               *) CAN0_BASE)
#define GPIOA_AHB                       ((GPIOA_Type              *) GPIOA_AHB_BASE)
#define GPIOB_AHB                       ((GPIOA_Type              *) GPIOB_AHB_BASE)
#define GPIOC_AHB                       ((GPIOA_Type              *) GPIOC_AHB_BASE)
#define GPIOD_AHB                       ((GPIOA_Type              *) GPIOD_AHB_BASE)
#define GPIOE_AHB                       ((GPIOA_Type              *) GPIOE_AHB_BASE)
#define HIB                             ((HIB_Type                *) HIB_BASE)
#define FLASH_CTRL                      ((FLASH_CTRL_Type         *) FLASH_CTRL_BASE)
#define SYSCTL                          ((SYSCTL_Type             *) SYSCTL_BASE)
#define UDMA                            ((UDMA_Type               *) UDMA_BASE)


/** @} */ /* End of group Device_Peripheral_Registers */
/** @} */ /* End of group LM3S2276 */
/** @} */ /* End of group TI */

#ifdef __cplusplus
}
#endif


#endif  /* LM3S2276_H */

