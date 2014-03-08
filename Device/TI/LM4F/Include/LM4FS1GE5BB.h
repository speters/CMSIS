
/****************************************************************************************************//**
 * @file     LM4FS1GE5BB.h
 *
 * @brief    CMSIS Cortex-M4 Core Peripheral Access Layer Header File for
 *           default LM4FS1GE5BB Device Series
 *
 * @version  V8636
 * @date     15. February 2012
 *
 * @note     Generated with SVDConv V2.73b  on Wednesday, 15.02.2012 18:06:25
 *           from CMSIS SVD File 'lm4fs1ge5bb.svd.xml' Version 8636,
 *           created on Thursday, 16.02.2012 00:06:25, last modified on Thursday, 16.02.2012 00:06:25
 *******************************************************************************************************/



/** @addtogroup TI
  * @{
  */

/** @addtogroup LM4FS1GE5BB
  * @{
  */

#ifndef LM4FS1GE5BB_H
#define LM4FS1GE5BB_H

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------  Interrupt Number Definition  ------------------------ */

typedef enum {
/* -------------------  Cortex-M4 Processor Exceptions Numbers  ------------------- */
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
/* -------------------  LM4FS1GE5BB Specific Interrupt Numbers  ------------------- */
  GPIOA_IRQn                    =   0,              /*!<   0  GPIOA                                                            */
  GPIOB_IRQn                    =   1,              /*!<   1  GPIOB                                                            */
  GPIOC_IRQn                    =   2,              /*!<   2  GPIOC                                                            */
  GPIOD_IRQn                    =   3,              /*!<   3  GPIOD                                                            */
  GPIOE_IRQn                    =   4,              /*!<   4  GPIOE                                                            */
  UART0_IRQn                    =   5,              /*!<   5  UART0                                                            */
  UART1_IRQn                    =   6,              /*!<   6  UART1                                                            */
  SSI0_IRQn                     =   7,              /*!<   7  SSI0                                                             */
  I2C0_IRQn                     =   8,              /*!<   8  I2C0                                                             */
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
  COMP0_IRQn                    =  25,              /*!<  25  COMP0                                                            */
  COMP1_IRQn                    =  26,              /*!<  26  COMP1                                                            */
  COMP2_IRQn                    =  27,              /*!<  27  COMP2                                                            */
  SYSCTL_IRQn                   =  28,              /*!<  28  SYSCTL                                                           */
  FLASH_CTRL_IRQn               =  29,              /*!<  29  FLASH_CTRL                                                       */
  GPIOF_IRQn                    =  30,              /*!<  30  GPIOF                                                            */
  GPIOG_IRQn                    =  31,              /*!<  31  GPIOG                                                            */
  GPIOH_IRQn                    =  32,              /*!<  32  GPIOH                                                            */
  UART2_IRQn                    =  33,              /*!<  33  UART2                                                            */
  SSI1_IRQn                     =  34,              /*!<  34  SSI1                                                             */
  TIMER3A_IRQn                  =  35,              /*!<  35  TIMER3A                                                          */
  TIMER3B_IRQn                  =  36,              /*!<  36  TIMER3B                                                          */
  I2C1_IRQn                     =  37,              /*!<  37  I2C1                                                             */
  HIB_IRQn                      =  43,              /*!<  43  HIB                                                              */
  UDMA_IRQn                     =  46,              /*!<  46  UDMA                                                             */
  UDMAERR_IRQn                  =  47,              /*!<  47  UDMAERR                                                          */
  ADC1SS0_IRQn                  =  48,              /*!<  48  ADC1SS0                                                          */
  ADC1SS1_IRQn                  =  49,              /*!<  49  ADC1SS1                                                          */
  ADC1SS2_IRQn                  =  50,              /*!<  50  ADC1SS2                                                          */
  ADC1SS3_IRQn                  =  51,              /*!<  51  ADC1SS3                                                          */
  GPIOJ_IRQn                    =  54,              /*!<  54  GPIOJ                                                            */
  GPIOK_IRQn                    =  55,              /*!<  55  GPIOK                                                            */
  GPIOL_IRQn                    =  56,              /*!<  56  GPIOL                                                            */
  SSI2_IRQn                     =  57,              /*!<  57  SSI2                                                             */
  SSI3_IRQn                     =  58,              /*!<  58  SSI3                                                             */
  UART3_IRQn                    =  59,              /*!<  59  UART3                                                            */
  UART4_IRQn                    =  60,              /*!<  60  UART4                                                            */
  UART5_IRQn                    =  61,              /*!<  61  UART5                                                            */
  UART6_IRQn                    =  62,              /*!<  62  UART6                                                            */
  UART7_IRQn                    =  63,              /*!<  63  UART7                                                            */
  I2C2_IRQn                     =  68,              /*!<  68  I2C2                                                             */
  I2C3_IRQn                     =  69,              /*!<  69  I2C3                                                             */
  TIMER4A_IRQn                  =  70,              /*!<  70  TIMER4A                                                          */
  TIMER4B_IRQn                  =  71,              /*!<  71  TIMER4B                                                          */
  TIMER5A_IRQn                  =  92,              /*!<  92  TIMER5A                                                          */
  TIMER5B_IRQn                  =  93,              /*!<  93  TIMER5B                                                          */
  WTIMER0A_IRQn                 =  94,              /*!<  94  WTIMER0A                                                         */
  WTIMER0B_IRQn                 =  95,              /*!<  95  WTIMER0B                                                         */
  WTIMER1A_IRQn                 =  96,              /*!<  96  WTIMER1A                                                         */
  WTIMER1B_IRQn                 =  97,              /*!<  97  WTIMER1B                                                         */
  WTIMER2A_IRQn                 =  98,              /*!<  98  WTIMER2A                                                         */
  WTIMER2B_IRQn                 =  99,              /*!<  99  WTIMER2B                                                         */
  WTIMER3A_IRQn                 = 100,              /*!< 100  WTIMER3A                                                         */
  WTIMER3B_IRQn                 = 101,              /*!< 101  WTIMER3B                                                         */
  WTIMER4A_IRQn                 = 102,              /*!< 102  WTIMER4A                                                         */
  WTIMER4B_IRQn                 = 103,              /*!< 103  WTIMER4B                                                         */
  WTIMER5A_IRQn                 = 104,              /*!< 104  WTIMER5A                                                         */
  WTIMER5B_IRQn                 = 105,              /*!< 105  WTIMER5B                                                         */
  SYSEXC_IRQn                   = 106,              /*!< 106  SYSEXC                                                           */
  PECI0_IRQn                    = 107,              /*!< 107  PECI0                                                            */
  LPC0_IRQn                     = 108,              /*!< 108  LPC0                                                             */
  I2C4_IRQn                     = 109,              /*!< 109  I2C4                                                             */
  I2C5_IRQn                     = 110,              /*!< 110  I2C5                                                             */
  GPIOM_IRQn                    = 111,              /*!< 111  GPIOM                                                            */
  GPION_IRQn                    = 112,              /*!< 112  GPION                                                            */
  FAN0_IRQn                     = 114,              /*!< 114  FAN0                                                             */
  GPIOP0_IRQn                   = 116,              /*!< 116  GPIOP0                                                           */
  GPIOP1_IRQn                   = 117,              /*!< 117  GPIOP1                                                           */
  GPIOP2_IRQn                   = 118,              /*!< 118  GPIOP2                                                           */
  GPIOP3_IRQn                   = 119,              /*!< 119  GPIOP3                                                           */
  GPIOP4_IRQn                   = 120,              /*!< 120  GPIOP4                                                           */
  GPIOP5_IRQn                   = 121,              /*!< 121  GPIOP5                                                           */
  GPIOP6_IRQn                   = 122,              /*!< 122  GPIOP6                                                           */
  GPIOP7_IRQn                   = 123,              /*!< 123  GPIOP7                                                           */
  GPIOQ0_IRQn                   = 124,              /*!< 124  GPIOQ0                                                           */
  GPIOQ1_IRQn                   = 125,              /*!< 125  GPIOQ1                                                           */
  GPIOQ2_IRQn                   = 126,              /*!< 126  GPIOQ2                                                           */
  GPIOQ3_IRQn                   = 127,              /*!< 127  GPIOQ3                                                           */
  GPIOQ4_IRQn                   = 128,              /*!< 128  GPIOQ4                                                           */
  GPIOQ5_IRQn                   = 129,              /*!< 129  GPIOQ5                                                           */
  GPIOQ6_IRQn                   = 130,              /*!< 130  GPIOQ6                                                           */
  GPIOQ7_IRQn                   = 131               /*!< 131  GPIOQ7                                                           */
} IRQn_Type;


/** @addtogroup Configuration_of_CMSIS
  * @{
  */


/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* ----------------Configuration of the Cortex-M4 Processor and Core Peripherals---------------- */
#define __CM4_REV                 0x0102            /*!< Cortex-M4 Core Revision                                               */
#define __MPU_PRESENT                  1            /*!< MPU present or not                                                    */
#define __NVIC_PRIO_BITS               3            /*!< Number of Bits used for Priority Levels                               */
#define __Vendor_SysTickConfig         0            /*!< Set to 1 if different SysTick Config is used                          */
#define __FPU_PRESENT                  1            /*!< FPU present or not                                                    */
/** @} */ /* End of group Configuration_of_CMSIS */

#include <core_cm4.h>                               /*!< Cortex-M4 processor and core peripherals                              */
#include "system_LM4F.h"                            /*!< LM4FS1GE5BB System                                                    */


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
  __IO uint32_t  PCTL;                              /*!< GPIO Port Control                                                     */
  __IO uint32_t  ADCCTL;                            /*!< GPIO ADC Control                                                      */
  __IO uint32_t  DMACTL;                            /*!< GPIO DMA Control                                                      */
  __IO uint32_t  SI;                                /*!< GPIO Select Interrupt                                                 */
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
  __I  uint32_t  RESERVED0[1000];
  __IO uint32_t  CC;                                /*!< SSI Clock Configuration                                               */
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
  __I  uint32_t  RESERVED2[17];
  __IO uint32_t  LCTL;                              /*!< UART LIN Control                                                      */
  __IO uint32_t  LSS;                               /*!< UART LIN Snap Shot                                                    */
  __IO uint32_t  LTIM;                              /*!< UART LIN Timer                                                        */
  __I  uint32_t  RESERVED3[2];
  __IO uint32_t  _9BITADDR;                         /*!< UART 9-Bit Self Address                                               */
  __IO uint32_t  _9BITAMASK;                        /*!< UART 9-Bit Self Address Mask                                          */
  __I  uint32_t  RESERVED4[965];
  __IO uint32_t  PP;                                /*!< UART Peripheral Properties                                            */
  __I  uint32_t  RESERVED5;
  __IO uint32_t  CC;                                /*!< UART Clock Configuration                                              */
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
    __IO uint32_t  MCS;                             /*!< I2C Master Control/Status                                             */
    __IO uint32_t  I2C0_ALT_MCS;                    /*!< I2C Master Control/Status                                             */
  } ;
  __IO uint32_t  MDR;                               /*!< I2C Master Data                                                       */
  __IO uint32_t  MTPR;                              /*!< I2C Master Timer Period                                               */
  __IO uint32_t  MIMR;                              /*!< I2C Master Interrupt Mask                                             */
  __IO uint32_t  MRIS;                              /*!< I2C Master Raw Interrupt Status                                       */
  __IO uint32_t  MMIS;                              /*!< I2C Master Masked Interrupt Status                                    */
  __O  uint32_t  MICR;                              /*!< I2C Master Interrupt Clear                                            */
  __IO uint32_t  MCR;                               /*!< I2C Master Configuration                                              */
  __IO uint32_t  MCLKOCNT;                          /*!< I2C Master Clock Low Timeout Count                                    */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  MBMON;                             /*!< I2C Master Bus Monitor                                                */
  __I  uint32_t  RESERVED1[500];
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
  __IO uint32_t  SOAR2;                             /*!< I2C Slave Own Address 2                                               */
  __IO uint32_t  SACKCTL;                           /*!< I2C ACK Control                                                       */
  __I  uint32_t  RESERVED2[487];
  __IO uint32_t  PP;                                /*!< I2C Peripheral Properties                                             */
} I2C0_Type;


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
  __IO uint32_t  SYNC;                              /*!< GPTM Synchronize                                                      */
  __I  uint32_t  RESERVED0;
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
  __IO uint32_t  TAPMR;                             /*!< GPTM TimerA Prescale Match                                            */
  __IO uint32_t  TBPMR;                             /*!< GPTM TimerB Prescale Match                                            */
  __IO uint32_t  TAR;                               /*!< GPTM Timer A                                                          */
  __IO uint32_t  TBR;                               /*!< GPTM Timer B                                                          */
  __IO uint32_t  TAV;                               /*!< GPTM Timer A Value                                                    */
  __IO uint32_t  TBV;                               /*!< GPTM Timer B Value                                                    */
  __IO uint32_t  RTCPD;                             /*!< GPTM RTC Predivide                                                    */
  __IO uint32_t  TAPS;                              /*!< GPTM Timer A Prescale Snapshot                                        */
  __IO uint32_t  TBPS;                              /*!< GPTM Timer B Prescale Snapshot                                        */
  __IO uint32_t  TAPV;                              /*!< GPTM Timer A Prescale Value                                           */
  __IO uint32_t  TBPV;                              /*!< GPTM Timer B Prescale Value                                           */
  __I  uint32_t  RESERVED1[981];
  __IO uint32_t  PP;                                /*!< GPTM Peripheral Properties                                            */
} TIMER0_Type;


/* ================================================================================ */
/* ================                     WTIMER0                    ================ */
/* ================================================================================ */


/**
  * @brief Register map for WTIMER0 peripheral (WTIMER0)
  */

typedef struct {                                    /*!< WTIMER0 Structure                                                     */
  __IO uint32_t  CFG;                               /*!< GPTM Configuration                                                    */
  __IO uint32_t  TAMR;                              /*!< GPTM Timer A Mode                                                     */
  __IO uint32_t  TBMR;                              /*!< GPTM Timer B Mode                                                     */
  __IO uint32_t  CTL;                               /*!< GPTM Control                                                          */
  __IO uint32_t  SYNC;                              /*!< GPTM Synchronize                                                      */
  __I  uint32_t  RESERVED0;
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
  __IO uint32_t  TAPMR;                             /*!< GPTM TimerA Prescale Match                                            */
  __IO uint32_t  TBPMR;                             /*!< GPTM TimerB Prescale Match                                            */
  __IO uint32_t  TAR;                               /*!< GPTM Timer A                                                          */
  __IO uint32_t  TBR;                               /*!< GPTM Timer B                                                          */
  __IO uint32_t  TAV;                               /*!< GPTM Timer A Value                                                    */
  __IO uint32_t  TBV;                               /*!< GPTM Timer B Value                                                    */
  __IO uint32_t  RTCPD;                             /*!< GPTM RTC Predivide                                                    */
  __IO uint32_t  TAPS;                              /*!< GPTM Timer A Prescale Snapshot                                        */
  __IO uint32_t  TBPS;                              /*!< GPTM Timer B Prescale Snapshot                                        */
  __IO uint32_t  TAPV;                              /*!< GPTM Timer A Prescale Value                                           */
  __IO uint32_t  TBPV;                              /*!< GPTM Timer B Prescale Value                                           */
  __I  uint32_t  RESERVED1[981];
  __IO uint32_t  PP;                                /*!< GPTM Peripheral Properties                                            */
} WTIMER0_Type;


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
  __IO uint32_t  SPC;                               /*!< ADC Sample Phase Control                                              */
  __IO uint32_t  PSSI;                              /*!< ADC Processor Sample Sequence Initiate                                */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  SAC;                               /*!< ADC Sample Averaging Control                                          */
  __IO uint32_t  DCISC;                             /*!< ADC Digital Comparator Interrupt Status and Clear                     */
  __IO uint32_t  CTL;                               /*!< ADC Control                                                           */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  SSMUX0;                            /*!< ADC Sample Sequence Input Multiplexer Select 0                        */
  __IO uint32_t  SSCTL0;                            /*!< ADC Sample Sequence Control 0                                         */
  __IO uint32_t  SSFIFO0;                           /*!< ADC Sample Sequence Result FIFO 0                                     */
  __IO uint32_t  SSFSTAT0;                          /*!< ADC Sample Sequence FIFO 0 Status                                     */
  __IO uint32_t  SSOP0;                             /*!< ADC Sample Sequence 0 Operation                                       */
  __IO uint32_t  SSDC0;                             /*!< ADC Sample Sequence 0 Digital Comparator Select                       */
  __IO uint32_t  SSEMUX0;                           /*!< ADC Sample Sequence Extended Input Multiplexer Select 0               */
  __I  uint32_t  RESERVED3;
  __IO uint32_t  SSMUX1;                            /*!< ADC Sample Sequence Input Multiplexer Select 1                        */
  __IO uint32_t  SSCTL1;                            /*!< ADC Sample Sequence Control 1                                         */
  __IO uint32_t  SSFIFO1;                           /*!< ADC Sample Sequence Result FIFO 1                                     */
  __IO uint32_t  SSFSTAT1;                          /*!< ADC Sample Sequence FIFO 1 Status                                     */
  __IO uint32_t  SSOP1;                             /*!< ADC Sample Sequence 1 Operation                                       */
  __IO uint32_t  SSDC1;                             /*!< ADC Sample Sequence 1 Digital Comparator Select                       */
  __IO uint32_t  SSEMUX1;                           /*!< ADC Sample Sequence Extended Input Multiplexer Select 1               */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  SSMUX2;                            /*!< ADC Sample Sequence Input Multiplexer Select 2                        */
  __IO uint32_t  SSCTL2;                            /*!< ADC Sample Sequence Control 2                                         */
  __IO uint32_t  SSFIFO2;                           /*!< ADC Sample Sequence Result FIFO 2                                     */
  __IO uint32_t  SSFSTAT2;                          /*!< ADC Sample Sequence FIFO 2 Status                                     */
  __IO uint32_t  SSOP2;                             /*!< ADC Sample Sequence 2 Operation                                       */
  __IO uint32_t  SSDC2;                             /*!< ADC Sample Sequence 2 Digital Comparator Select                       */
  __IO uint32_t  SSEMUX2;                           /*!< ADC Sample Sequence Extended Input Multiplexer Select 2               */
  __I  uint32_t  RESERVED5;
  __IO uint32_t  SSMUX3;                            /*!< ADC Sample Sequence Input Multiplexer Select 3                        */
  __IO uint32_t  SSCTL3;                            /*!< ADC Sample Sequence Control 3                                         */
  __IO uint32_t  SSFIFO3;                           /*!< ADC Sample Sequence Result FIFO 3                                     */
  __IO uint32_t  SSFSTAT3;                          /*!< ADC Sample Sequence FIFO 3 Status                                     */
  __IO uint32_t  SSOP3;                             /*!< ADC Sample Sequence 3 Operation                                       */
  __IO uint32_t  SSDC3;                             /*!< ADC Sample Sequence 3 Digital Comparator Select                       */
  __IO uint32_t  SSEMUX3;                           /*!< ADC Sample Sequence Extended Input Multiplexer Select 3               */
  __I  uint32_t  RESERVED6[785];
  __IO uint32_t  DCRIC;                             /*!< ADC Digital Comparator Reset Initial Conditions                       */
  __I  uint32_t  RESERVED7[63];
  __IO uint32_t  DCCTL0;                            /*!< ADC Digital Comparator Control 0                                      */
  __IO uint32_t  DCCTL1;                            /*!< ADC Digital Comparator Control 1                                      */
  __IO uint32_t  DCCTL2;                            /*!< ADC Digital Comparator Control 2                                      */
  __IO uint32_t  DCCTL3;                            /*!< ADC Digital Comparator Control 3                                      */
  __IO uint32_t  DCCTL4;                            /*!< ADC Digital Comparator Control 4                                      */
  __IO uint32_t  DCCTL5;                            /*!< ADC Digital Comparator Control 5                                      */
  __IO uint32_t  DCCTL6;                            /*!< ADC Digital Comparator Control 6                                      */
  __IO uint32_t  DCCTL7;                            /*!< ADC Digital Comparator Control 7                                      */
  __I  uint32_t  RESERVED8[8];
  __IO uint32_t  DCCMP0;                            /*!< ADC Digital Comparator Range 0                                        */
  __IO uint32_t  DCCMP1;                            /*!< ADC Digital Comparator Range 1                                        */
  __IO uint32_t  DCCMP2;                            /*!< ADC Digital Comparator Range 2                                        */
  __IO uint32_t  DCCMP3;                            /*!< ADC Digital Comparator Range 3                                        */
  __IO uint32_t  DCCMP4;                            /*!< ADC Digital Comparator Range 4                                        */
  __IO uint32_t  DCCMP5;                            /*!< ADC Digital Comparator Range 5                                        */
  __IO uint32_t  DCCMP6;                            /*!< ADC Digital Comparator Range 6                                        */
  __IO uint32_t  DCCMP7;                            /*!< ADC Digital Comparator Range 7                                        */
  __I  uint32_t  RESERVED9[88];
  __IO uint32_t  PP;                                /*!< ADC Peripheral Properties                                             */
  __IO uint32_t  PC;                                /*!< ADC Peripheral Configuration                                          */
  __IO uint32_t  CC;                                /*!< ADC Clock Configuration                                               */
} ADC0_Type;


/* ================================================================================ */
/* ================                      COMP                      ================ */
/* ================================================================================ */


/**
  * @brief Register map for COMP peripheral (COMP)
  */

typedef struct {                                    /*!< COMP Structure                                                        */
  __IO uint32_t  ACMIS;                             /*!< Analog Comparator Masked Interrupt Status                             */
  __IO uint32_t  ACRIS;                             /*!< Analog Comparator Raw Interrupt Status                                */
  __IO uint32_t  ACINTEN;                           /*!< Analog Comparator Interrupt Enable                                    */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  ACREFCTL;                          /*!< Analog Comparator Reference Voltage Control                           */
  __I  uint32_t  RESERVED1[3];
  __IO uint32_t  ACSTAT0;                           /*!< Analog Comparator Status 0                                            */
  __IO uint32_t  ACCTL0;                            /*!< Analog Comparator Control 0                                           */
  __I  uint32_t  RESERVED2[6];
  __IO uint32_t  ACSTAT1;                           /*!< Analog Comparator Status 1                                            */
  __IO uint32_t  ACCTL1;                            /*!< Analog Comparator Control 1                                           */
  __I  uint32_t  RESERVED3[6];
  __IO uint32_t  ACSTAT2;                           /*!< Analog Comparator Status 2                                            */
  __IO uint32_t  ACCTL2;                            /*!< Analog Comparator Control 2                                           */
  __I  uint32_t  RESERVED4[982];
  __IO uint32_t  PP;                                /*!< Analog Comparator Peripheral Properties                               */
} COMP_Type;


/* ================================================================================ */
/* ================                      LPC0                      ================ */
/* ================================================================================ */


/**
  * @brief Register map for LPC0 peripheral (LPC0)
  */

typedef struct {                                    /*!< LPC0 Structure                                                        */
  __IO uint32_t  CTL;                               /*!< LPC Control                                                           */
  __IO uint32_t  STS;                               /*!< LPC Status                                                            */
  __IO uint32_t  IRQCTL;                            /*!< LPC IRQ Control                                                       */
  __IO uint32_t  IRQST;                             /*!< LPC IRQ Status                                                        */
  
  union {
    __IO uint32_t  LPC0_ALT_CH0CTL;                 /*!< LPC Channel 0 Control                                                 */
    __IO uint32_t  CH0CTL;                          /*!< LPC Channel 0 Control                                                 */
  } ;
  __IO uint32_t  CH0ST;                             /*!< LPC Channel 0 Status                                                  */
  __IO uint32_t  CH0ADR;                            /*!< LPC Channel 0 Address                                                 */
  __I  uint32_t  RESERVED0;
  
  union {
    __IO uint32_t  LPC0_ALT_CH1CTL;                 /*!< LPC Channel 1 Control                                                 */
    __IO uint32_t  CH1CTL;                          /*!< LPC Channel 1 Control                                                 */
  } ;
  __IO uint32_t  CH1ST;                             /*!< LPC Channel 1 Status                                                  */
  __IO uint32_t  CH1ADR;                            /*!< LPC Channel 1 Address                                                 */
  __I  uint32_t  RESERVED1;
  
  union {
    __IO uint32_t  LPC0_ALT_CH2CTL;                 /*!< LPC Channel 2 Control                                                 */
    __IO uint32_t  CH2CTL;                          /*!< LPC Channel 2 Control                                                 */
  } ;
  __IO uint32_t  CH2ST;                             /*!< LPC Channel 2 Status                                                  */
  __IO uint32_t  CH2ADR;                            /*!< LPC Channel 2 Address                                                 */
  __I  uint32_t  RESERVED2;
  
  union {
    __IO uint32_t  LPC0_ALT_CH3CTL;                 /*!< LPC Channel 3 Control                                                 */
    __IO uint32_t  CH3CTL;                          /*!< LPC Channel 3 Control                                                 */
  } ;
  __IO uint32_t  CH3ST;                             /*!< LPC Channel 3 Status                                                  */
  __IO uint32_t  CH3ADR;                            /*!< LPC Channel 3 Address                                                 */
  __I  uint32_t  RESERVED3;
  
  union {
    __IO uint32_t  LPC0_ALT_CH4CTL;                 /*!< LPC Channel 4 Control                                                 */
    __IO uint32_t  CH4CTL;                          /*!< LPC Channel 4 Control                                                 */
  } ;
  __IO uint32_t  CH4ST;                             /*!< LPC Channel 4 Status                                                  */
  __IO uint32_t  CH4ADR;                            /*!< LPC Channel 4 Address                                                 */
  __I  uint32_t  RESERVED4;
  
  union {
    __IO uint32_t  LPC0_ALT_CH5CTL;                 /*!< LPC Channel 5 Control                                                 */
    __IO uint32_t  CH5CTL;                          /*!< LPC Channel 5 Control                                                 */
  } ;
  __IO uint32_t  CH5ST;                             /*!< LPC Channel 5 Status                                                  */
  __IO uint32_t  CH5ADR;                            /*!< LPC Channel 5 Address                                                 */
  __I  uint32_t  RESERVED5;
  
  union {
    __IO uint32_t  LPC0_ALT_CH6CTL;                 /*!< LPC Channel 6 Control                                                 */
    __IO uint32_t  CH6CTL;                          /*!< LPC Channel 6 Control                                                 */
  } ;
  __IO uint32_t  CH6ST;                             /*!< LPC Channel 6 Status                                                  */
  __IO uint32_t  CH6ADR;                            /*!< LPC Channel 6 Address                                                 */
  __I  uint32_t  RESERVED6;
  
  union {
    __IO uint32_t  LPC0_ALT_CH7CTL;                 /*!< LPC Channel 7 / COMx Control                                          */
    __IO uint32_t  CH7CTL;                          /*!< LPC Channel 7 / COMx Control                                          */
  } ;
  __IO uint32_t  CH7ST;                             /*!< LPC Channel 7 / COMx Status                                           */
  __IO uint32_t  CH7ADR;                            /*!< LPC Channel 7 / COMx Address                                          */
  __I  uint32_t  RESERVED7[5];
  __IO uint32_t  STSADDR;                           /*!< LPC Status Block Address                                              */
  __I  uint32_t  RESERVED8[23];
  __IO uint32_t  IM;                                /*!< LPC Interrupt Mask                                                    */
  __IO uint32_t  RIS;                               /*!< LPC Raw Interrupt Status                                              */
  __IO uint32_t  MIS;                               /*!< LPC Masked Interrupt Status                                           */
  __O  uint32_t  IC;                                /*!< LPC Interrupt Clear                                                   */
  __I  uint32_t  RESERVED9[4];
  __IO uint32_t  DMACX;                             /*!< LPC DMA and COMx Control                                              */
  __I  uint32_t  RESERVED10[183];
  __IO uint32_t  POOL;                              /*!< LPC Register Pool                                                     */
  __I  uint32_t  RESERVED11[751];
  __IO uint32_t  PP;                                /*!< LPC Peripheral Properties                                             */
} LPC0_Type;


/* ================================================================================ */
/* ================                      FAN0                      ================ */
/* ================================================================================ */


/**
  * @brief Register map for FAN0 peripheral (FAN0)
  */

typedef struct {                                    /*!< FAN0 Structure                                                        */
  __IO uint32_t  STS;                               /*!< FAN Status                                                            */
  __IO uint32_t  CTL;                               /*!< FAN Control                                                           */
  __I  uint32_t  RESERVED0[2];
  __IO uint32_t  CH0;                               /*!< FAN Channel Command                                                   */
  __IO uint32_t  CMD0;                              /*!< FAN Channel Command                                                   */
  __IO uint32_t  CST0;                              /*!< FAN Channel Status                                                    */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  CH1;                               /*!< FAN Channel Setup                                                     */
  __IO uint32_t  CMD1;                              /*!< FAN Channel Command                                                   */
  __IO uint32_t  CST1;                              /*!< FAN Channel Status                                                    */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  CH2;                               /*!< FAN Channel Setup                                                     */
  __IO uint32_t  CMD2;                              /*!< FAN Channel Command                                                   */
  __IO uint32_t  CST2;                              /*!< FAN Channel Status                                                    */
  __I  uint32_t  RESERVED3;
  __IO uint32_t  CH3;                               /*!< FAN Channel Setup                                                     */
  __IO uint32_t  CMD3;                              /*!< FAN Channel Command                                                   */
  __IO uint32_t  CST3;                              /*!< FAN Channel Status                                                    */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  CH4;                               /*!< FAN Channel Setup                                                     */
  __IO uint32_t  CMD4;                              /*!< FAN Channel Command                                                   */
  __IO uint32_t  CST4;                              /*!< FAN Channel Status                                                    */
  __I  uint32_t  RESERVED5;
  __IO uint32_t  CH5;                               /*!< FAN Channel Setup                                                     */
  __IO uint32_t  CMD5;                              /*!< FAN Channel Command                                                   */
  __IO uint32_t  CST5;                              /*!< FAN Channel Status                                                    */
  __I  uint32_t  RESERVED6[9];
  __IO uint32_t  IM;                                /*!< FAN Interrupt Mask                                                    */
  __IO uint32_t  RIS;                               /*!< FAN Raw Interrupt Status                                              */
  __IO uint32_t  MIS;                               /*!< FAN Masked Interrupt Status                                           */
  __IO uint32_t  IC;                                /*!< FAN Interrupt Clear                                                   */
  __I  uint32_t  RESERVED7[968];
  __IO uint32_t  PP;                                /*!< FAN Peripheral Properties                                             */
} FAN0_Type;


/* ================================================================================ */
/* ================                     EEPROM                     ================ */
/* ================================================================================ */


/**
  * @brief Register map for EEPROM peripheral (EEPROM)
  */

typedef struct {                                    /*!< EEPROM Structure                                                      */
  __IO uint32_t  EESIZE;                            /*!< EEPROM Size Information                                               */
  __IO uint32_t  EEBLOCK;                           /*!< EEPROM Current Block                                                  */
  __IO uint32_t  EEOFFSET;                          /*!< EEPROM Current Offset                                                 */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  EERDWR;                            /*!< EEPROM Read-Write                                                     */
  __IO uint32_t  EERDWRINC;                         /*!< EEPROM Read-Write with Increment                                      */
  __IO uint32_t  EEDONE;                            /*!< EEPROM Done Status                                                    */
  __IO uint32_t  EESUPP;                            /*!< EEPROM Support Control and Status                                     */
  __IO uint32_t  EEUNLOCK;                          /*!< EEPROM Unlock                                                         */
  __I  uint32_t  RESERVED1[3];
  __IO uint32_t  EEPROT;                            /*!< EEPROM Protection                                                     */
  __IO uint32_t  EEPASS0;                           /*!< EEPROM Password                                                       */
  __IO uint32_t  EEPASS1;                           /*!< EEPROM Password                                                       */
  __IO uint32_t  EEPASS2;                           /*!< EEPROM Password                                                       */
  __IO uint32_t  EEINT;                             /*!< EEPROM Interrupt                                                      */
  __I  uint32_t  RESERVED2[3];
  __IO uint32_t  EEHIDE;                            /*!< EEPROM Block Hide                                                     */
  __I  uint32_t  RESERVED3[11];
  __IO uint32_t  EEDBGME;                           /*!< EEPROM Debug Mass Erase                                               */
  __I  uint32_t  RESERVED4[975];
  __IO uint32_t  EEPROMPP;                          /*!< EEPROM                                                                */
} EEPROM_Type;


/* ================================================================================ */
/* ================                      PECI0                     ================ */
/* ================================================================================ */


/**
  * @brief Register map for PECI0 peripheral (PECI0)
  */

typedef struct {                                    /*!< PECI0 Structure                                                       */
  __IO uint32_t  CTL;                               /*!< PECI Control                                                          */
  __IO uint32_t  DIV;                               /*!< PECI Poll-Rate Divider                                                */
  __IO uint32_t  CMP;                               /*!< PECI Comparator                                                       */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  M0D0C;                             /*!< PECI Microprocessor 0 / Domain 0 Control                              */
  __IO uint32_t  M0D1C;                             /*!< PECI Microprocessor 0 / Domain 1 Control                              */
  __IO uint32_t  M1D0C;                             /*!< PECI Microprocessor 1 / Domain 0 Control                              */
  __IO uint32_t  M1D1C;                             /*!< PECI Microprocessor 1 / Domain 1 Control                              */
  __I  uint32_t  RESERVED1[8];
  __IO uint32_t  M0D0;                              /*!< PECI Microprocessor 0 / Domain 0 Value                                */
  __IO uint32_t  M0D1;                              /*!< PECI Microprocessor 0 / Domain 1 Value                                */
  __IO uint32_t  M1D0;                              /*!< PECI Microprocessor 1 / Domain 0 Value                                */
  __IO uint32_t  M1D1;                              /*!< PECI Microprocessor 1 / Domain 1 Value                                */
  __I  uint32_t  RESERVED2[12];
  __IO uint32_t  IM;                                /*!< PECI Interrupt Mask                                                   */
  __IO uint32_t  RIS;                               /*!< PECI Raw Interrupt Status                                             */
  __IO uint32_t  MIS;                               /*!< PECI Masked Interrupt Status                                          */
  __O  uint32_t  IC;                                /*!< PECI Interrupt Clear                                                  */
  __I  uint32_t  RESERVED3[28];
  __IO uint32_t  ACADDR;                            /*!< PECI Advanced Command Address                                         */
  __IO uint32_t  ACARG;                             /*!< PECI Advanced Command Argument                                        */
  __IO uint32_t  ACRDWR0;                           /*!< PECI Advanced Command Data 0                                          */
  __IO uint32_t  ACRDWR1;                           /*!< PECI Advanced Command Data 1                                          */
  __IO uint32_t  ACCMD;                             /*!< PECI Advanced Command                                                 */
  __IO uint32_t  ACCODE;                            /*!< PECI Advanced Command Completion Code                                 */
} PECI0_Type;


/* ================================================================================ */
/* ================                     SYSEXC                     ================ */
/* ================================================================================ */


/**
  * @brief Register map for SYSEXC peripheral (SYSEXC)
  */

typedef struct {                                    /*!< SYSEXC Structure                                                      */
  __IO uint32_t  RIS;                               /*!< System Exception Raw Interrupt Status                                 */
  __IO uint32_t  IM;                                /*!< System Exception Interrupt Mask                                       */
  __IO uint32_t  MIS;                               /*!< System Exception Raw Interrupt Status                                 */
  __O  uint32_t  IC;                                /*!< System Exception Interrupt Clear                                      */
} SYSEXC_Type;


/* ================================================================================ */
/* ================                       HIB                      ================ */
/* ================================================================================ */


/**
  * @brief Register map for HIB peripheral (HIB)
  */

typedef struct {                                    /*!< HIB Structure                                                         */
  __IO uint32_t  RTCC;                              /*!< Hibernation RTC Counter                                               */
  __IO uint32_t  RTCM0;                             /*!< Hibernation RTC Match 0                                               */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  RTCLD;                             /*!< Hibernation RTC Load                                                  */
  __IO uint32_t  CTL;                               /*!< Hibernation Control                                                   */
  __IO uint32_t  IM;                                /*!< Hibernation Interrupt Mask                                            */
  __IO uint32_t  RIS;                               /*!< Hibernation Raw Interrupt Status                                      */
  __IO uint32_t  MIS;                               /*!< Hibernation Masked Interrupt Status                                   */
  __IO uint32_t  IC;                                /*!< Hibernation Interrupt Clear                                           */
  __IO uint32_t  RTCT;                              /*!< Hibernation RTC Trim                                                  */
  __IO uint32_t  RTCSS;                             /*!< Hibernation RTC Sub Seconds                                           */
  __I  uint32_t  RESERVED1;
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
  __I  uint32_t  RESERVED0[2];
  __IO uint32_t  FMC2;                              /*!< Flash Memory Control 2                                                */
  __I  uint32_t  RESERVED1[3];
  __IO uint32_t  FWBVAL;                            /*!< Flash Write Buffer Valid                                              */
  __I  uint32_t  RESERVED2[51];
  __IO uint32_t  FWBN;                              /*!< Flash Write Buffer n                                                  */
  __I  uint32_t  RESERVED3[943];
  __IO uint32_t  FSIZE;                             /*!< Flash Size                                                            */
  __IO uint32_t  SSIZE;                             /*!< SRAM Size                                                             */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  ROMSWMAP;                          /*!< ROM Software Map                                                      */
  __I  uint32_t  RESERVED5[72];
  __IO uint32_t  RMCTL;                             /*!< ROM Control                                                           */
  __I  uint32_t  RESERVED6[55];
  __IO uint32_t  BOOTCFG;                           /*!< Boot Configuration                                                    */
  __I  uint32_t  RESERVED7[3];
  __IO uint32_t  USERREG0;                          /*!< User Register 0                                                       */
  __IO uint32_t  USERREG1;                          /*!< User Register 1                                                       */
  __IO uint32_t  USERREG2;                          /*!< User Register 2                                                       */
  __IO uint32_t  USERREG3;                          /*!< User Register 3                                                       */
  __I  uint32_t  RESERVED8[4];
  __IO uint32_t  FMPRE0;                            /*!< Flash Memory Protection Read Enable 0                                 */
  __IO uint32_t  FMPRE1;                            /*!< Flash Memory Protection Read Enable 1                                 */
  __I  uint32_t  RESERVED9[126];
  __IO uint32_t  FMPPE0;                            /*!< Flash Memory Protection Program Enable 0                              */
  __IO uint32_t  FMPPE1;                            /*!< Flash Memory Protection Program Enable 1                              */
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
  __IO uint32_t  DC8;                               /*!< Device Capabilities 8 ADC Channels                                    */
  __IO uint32_t  PBORCTL;                           /*!< Brown-Out Reset Control                                               */
  __I  uint32_t  RESERVED1[3];
  __IO uint32_t  SRCR0;                             /*!< Software Reset Control 0                                              */
  __IO uint32_t  SRCR1;                             /*!< Software Reset Control 1                                              */
  __IO uint32_t  SRCR2;                             /*!< Software Reset Control 2                                              */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  RIS;                               /*!< Raw Interrupt Status                                                  */
  __IO uint32_t  IMC;                               /*!< Interrupt Mask Control                                                */
  __IO uint32_t  MISC;                              /*!< Masked Interrupt Status and Clear                                     */
  __IO uint32_t  RESC;                              /*!< Reset Cause                                                           */
  __IO uint32_t  RCC;                               /*!< Run-Mode Clock Configuration                                          */
  __I  uint32_t  RESERVED3[2];
  __IO uint32_t  GPIOHBCTL;                         /*!< GPIO High-Performance Bus Control                                     */
  __IO uint32_t  RCC2;                              /*!< Run-Mode Clock Configuration 2                                        */
  __I  uint32_t  RESERVED4[2];
  __IO uint32_t  MOSCCTL;                           /*!< Main Oscillator Control                                               */
  __I  uint32_t  RESERVED5[32];
  __IO uint32_t  RCGC0;                             /*!< Run Mode Clock Gating Control Register 0                              */
  __IO uint32_t  RCGC1;                             /*!< Run Mode Clock Gating Control Register 1                              */
  __IO uint32_t  RCGC2;                             /*!< Run Mode Clock Gating Control Register 2                              */
  __I  uint32_t  RESERVED6;
  __IO uint32_t  SCGC0;                             /*!< Sleep Mode Clock Gating Control Register 0                            */
  __IO uint32_t  SCGC1;                             /*!< Sleep Mode Clock Gating Control Register 1                            */
  __IO uint32_t  SCGC2;                             /*!< Sleep Mode Clock Gating Control Register 2                            */
  __I  uint32_t  RESERVED7;
  __IO uint32_t  DCGC0;                             /*!< Deep Sleep Mode Clock Gating Control Register 0                       */
  __IO uint32_t  DCGC1;                             /*!< Deep-Sleep Mode Clock Gating Control Register 1                       */
  __IO uint32_t  DCGC2;                             /*!< Deep Sleep Mode Clock Gating Control Register 2                       */
  __I  uint32_t  RESERVED8[6];
  __IO uint32_t  DSLPCLKCFG;                        /*!< Deep Sleep Clock Configuration                                        */
  __I  uint32_t  RESERVED9;
  __IO uint32_t  SYSPROP;                           /*!< System Properties                                                     */
  __IO uint32_t  PIOSCCAL;                          /*!< Precision Internal Oscillator Calibration                             */
  __IO uint32_t  PIOSCSTAT;                         /*!< Precision Internal Oscillator Statistics                              */
  __I  uint32_t  RESERVED10[2];
  __IO uint32_t  PLLFREQ0;                          /*!< PLL Frequency 0                                                       */
  __IO uint32_t  PLLFREQ1;                          /*!< PLL Frequency                                                         */
  __IO uint32_t  PLLSTAT;                           /*!< PLL Status                                                            */
  __I  uint32_t  RESERVED11[9];
  __IO uint32_t  DC9;                               /*!< Device Capabilities 9 ADC Digital Comparators                         */
  __I  uint32_t  RESERVED12[3];
  __IO uint32_t  NVMSTAT;                           /*!< Non-Volatile Memory Information                                       */
  __I  uint32_t  RESERVED13[87];
  __IO uint32_t  PPWD;                              /*!< Watchdog Timer Peripheral Present                                     */
  __IO uint32_t  PPTIMER;                           /*!< Timer Peripheral Present                                              */
  __IO uint32_t  PPGPIO;                            /*!< General-Purpose Input/Output Peripheral Present                       */
  __IO uint32_t  PPDMA;                             /*!< Micro Direct Memory Access Peripheral Present                         */
  __I  uint32_t  RESERVED14;
  __IO uint32_t  PPHIB;                             /*!< Hibernation Peripheral Present                                        */
  __IO uint32_t  PPUART;                            /*!< Universal Asynchronous Receiver/Transmitter Peripheral Present        */
  __IO uint32_t  PPSSI;                             /*!< Synchronous Serial Interface Peripheral Present                       */
  __IO uint32_t  PPI2C;                             /*!< Inter-Integrated Circuit Peripheral Present                           */
  __I  uint32_t  RESERVED15;
  __IO uint32_t  PPUSB;                             /*!< Universal Serial Bus Peripheral Present                               */
  __I  uint32_t  RESERVED16[2];
  __IO uint32_t  PPCAN;                             /*!< Controller Area Network Peripheral Present                            */
  __IO uint32_t  PPADC;                             /*!< Analog-to-Digital Converter Peripheral Present                        */
  __IO uint32_t  PPACMP;                            /*!< Analog Comparator Peripheral Present                                  */
  __IO uint32_t  PPPWM;                             /*!< Pulse Width Modulator Peripheral Present                              */
  __IO uint32_t  PPQEI;                             /*!< Quadrature Encoder Interface Peripheral Present                       */
  __IO uint32_t  PPLPC;                             /*!< Low Pin Count Interface Peripheral Present                            */
  __I  uint32_t  RESERVED17;
  __IO uint32_t  PPPECI;                            /*!< Platform Environment Control Interface Peripheral Present             */
  __IO uint32_t  PPFAN;                             /*!< FAN Peripheral Present                                                */
  __IO uint32_t  PPEEPROM;                          /*!< EEPROM Peripheral Present                                             */
  __IO uint32_t  PPWTIMER;                          /*!< Wide Timer Peripheral Present                                         */
  __I  uint32_t  RESERVED18[104];
  __IO uint32_t  SRWD;                              /*!< Watchdog Timer Software Reset                                         */
  __IO uint32_t  SRTIMER;                           /*!< Timer Software Reset                                                  */
  __IO uint32_t  SRGPIO;                            /*!< General-Purpose Input/Output Software Reset                           */
  __IO uint32_t  SRDMA;                             /*!< Micro Direct Memory Access Software Reset                             */
  __I  uint32_t  RESERVED19;
  __IO uint32_t  SRHIB;                             /*!< Hibernation Software Reset                                            */
  __IO uint32_t  SRUART;                            /*!< Universal Asynchronous Receiver/Transmitter Software Reset            */
  __IO uint32_t  SRSSI;                             /*!< Synchronous Serial Interface Software Reset                           */
  __IO uint32_t  SRI2C;                             /*!< Inter-Integrated Circuit Software Reset                               */
  __I  uint32_t  RESERVED20[5];
  __IO uint32_t  SRADC;                             /*!< Analog-to-Digital Converter Software Reset                            */
  __IO uint32_t  SRACMP;                            /*!< Analog Comparator Software Reset                                      */
  __I  uint32_t  RESERVED21[2];
  __IO uint32_t  SRLPC;                             /*!< Low Pin Count Interface Software Reset                                */
  __I  uint32_t  RESERVED22;
  __IO uint32_t  SRPECI;                            /*!< Platform Environment Control Interface Software Reset                 */
  __IO uint32_t  SRFAN;                             /*!< FAN Software Reset                                                    */
  __IO uint32_t  SREEPROM;                          /*!< EEPROM Software Reset                                                 */
  __IO uint32_t  SRWTIMER;                          /*!< Wide Timer Software Reset                                             */
  __I  uint32_t  RESERVED23[40];
  __IO uint32_t  RCGCWD;                            /*!< Watchdog Timer Run Mode Clock Gating Control                          */
  __IO uint32_t  RCGCTIMER;                         /*!< Timer Run Mode Clock Gating Control                                   */
  __IO uint32_t  RCGCGPIO;                          /*!< General-Purpose Input/Output Run Mode Clock Gating Control            */
  __IO uint32_t  RCGCDMA;                           /*!< Micro Direct Memory Access Run Mode Clock Gating Control              */
  __I  uint32_t  RESERVED24;
  __IO uint32_t  RCGCHIB;                           /*!< Hibernation Run Mode Clock Gating Control                             */
  __IO uint32_t  RCGCUART;                          /*!< Universal Asynchronous Receiver/Transmitter Run Mode Clock Gating
                                                         Control                                                               */
  __IO uint32_t  RCGCSSI;                           /*!< Synchronous Serial Interface Run Mode Clock Gating Control            */
  __IO uint32_t  RCGCI2C;                           /*!< Inter-Integrated Circuit Run Mode Clock Gating Control                */
  __I  uint32_t  RESERVED25[5];
  __IO uint32_t  RCGCADC;                           /*!< Analog-to-Digital Converter Run Mode Clock Gating Control             */
  __IO uint32_t  RCGCACMP;                          /*!< Analog Comparator Run Mode Clock Gating Control                       */
  __I  uint32_t  RESERVED26[2];
  __IO uint32_t  RCGCLPC;                           /*!< Low Pin Count Interface Run Mode Clock Gating Control                 */
  __I  uint32_t  RESERVED27;
  __IO uint32_t  RCGCPECI;                          /*!< Platform Environment Control Interface Run Mode Clock Gating
                                                         Control                                                               */
  __IO uint32_t  RCGCFAN;                           /*!< FAN Run Mode Clock Gating Control                                     */
  __IO uint32_t  RCGCEEPROM;                        /*!< EEPROM Run Mode Clock Gating Control                                  */
  __IO uint32_t  RCGCWTIMER;                        /*!< Wide Timer Run Mode Clock Gating Control                              */
  __I  uint32_t  RESERVED28[40];
  __IO uint32_t  SCGCWD;                            /*!< Watchdog Timer Sleep Mode Clock Gating Control                        */
  __IO uint32_t  SCGCTIMER;                         /*!< Timer Sleep Mode Clock Gating Control                                 */
  __IO uint32_t  SCGCGPIO;                          /*!< General-Purpose Input/Output Sleep Mode Clock Gating Control          */
  __IO uint32_t  SCGCDMA;                           /*!< Micro Direct Memory Access Sleep Mode Clock Gating Control            */
  __I  uint32_t  RESERVED29;
  __IO uint32_t  SCGCHIB;                           /*!< Hibernation Sleep Mode Clock Gating Control                           */
  __IO uint32_t  SCGCUART;                          /*!< Universal Asynchronous Receiver/Transmitter Sleep Mode Clock
                                                         Gating Control                                                        */
  __IO uint32_t  SCGCSSI;                           /*!< Synchronous Serial Interface Sleep Mode Clock Gating Control          */
  __IO uint32_t  SCGCI2C;                           /*!< Inter-Integrated Circuit Sleep Mode Clock Gating Control              */
  __I  uint32_t  RESERVED30[5];
  __IO uint32_t  SCGCADC;                           /*!< Analog-to-Digital Converter Sleep Mode Clock Gating Control           */
  __IO uint32_t  SCGCACMP;                          /*!< Analog Comparator Sleep Mode Clock Gating Control                     */
  __I  uint32_t  RESERVED31[2];
  __IO uint32_t  SCGCLPC;                           /*!< Low Pin Count Interface Sleep Mode Clock Gating Control               */
  __I  uint32_t  RESERVED32;
  __IO uint32_t  SCGCPECI;                          /*!< Platform Environment Control Interface Sleep Mode Clock Gating
                                                         Control                                                               */
  __IO uint32_t  SCGCFAN;                           /*!< FAN Sleep Mode Clock Gating Control                                   */
  __IO uint32_t  SCGCEEPROM;                        /*!< EEPROM Sleep Mode Clock Gating Control                                */
  __IO uint32_t  SCGCWTIMER;                        /*!< Wide Timer Sleep Mode Clock Gating Control                            */
  __I  uint32_t  RESERVED33[40];
  __IO uint32_t  DCGCWD;                            /*!< Watchdog Timer Deep-Sleep Mode Clock Gating Control                   */
  __IO uint32_t  DCGCTIMER;                         /*!< Timer Deep-Sleep Mode Clock Gating Control                            */
  __IO uint32_t  DCGCGPIO;                          /*!< General-Purpose Input/Output Deep-Sleep Mode Clock Gating Control     */
  __IO uint32_t  DCGCDMA;                           /*!< Micro Direct Memory Access Deep-Sleep Mode Clock Gating Control       */
  __I  uint32_t  RESERVED34;
  __IO uint32_t  DCGCHIB;                           /*!< Hibernation Deep-Sleep Mode Clock Gating Control                      */
  __IO uint32_t  DCGCUART;                          /*!< Universal Asynchronous Receiver/Transmitter Deep-Sleep Mode
                                                         Clock Gating Control                                                  */
  __IO uint32_t  DCGCSSI;                           /*!< Synchronous Serial Interface Deep-Sleep Mode Clock Gating Control     */
  __IO uint32_t  DCGCI2C;                           /*!< Inter-Integrated Circuit Deep-Sleep Mode Clock Gating Control         */
  __I  uint32_t  RESERVED35[5];
  __IO uint32_t  DCGCADC;                           /*!< Analog-to-Digital Converter Deep-Sleep Mode Clock Gating Control      */
  __IO uint32_t  DCGCACMP;                          /*!< Analog Comparator Deep-Sleep Mode Clock Gating Control                */
  __I  uint32_t  RESERVED36[2];
  __IO uint32_t  DCGCLPC;                           /*!< Low Pin Count Interface Deep-Sleep Mode Clock Gating Control          */
  __I  uint32_t  RESERVED37;
  __IO uint32_t  DCGCPECI;                          /*!< Platform Environment Control Interface Deep-Sleep Mode Clock
                                                         Gating Control                                                        */
  __IO uint32_t  DCGCFAN;                           /*!< FAN Deep-Sleep Mode Clock Gating Control                              */
  __IO uint32_t  DCGCEEPROM;                        /*!< EEPROM Deep-Sleep Mode Clock Gating Control                           */
  __IO uint32_t  DCGCWTIMER;                        /*!< Wide Timer Deep-Sleep Mode Clock Gating Control                       */
  __I  uint32_t  RESERVED38[40];
  __IO uint32_t  PCWD;                              /*!< Watchdog Timer Power Control                                          */
  __IO uint32_t  PCTIMER;                           /*!< Timer Power Control                                                   */
  __IO uint32_t  PCGPIO;                            /*!< General-Purpose Input/Output Power Control                            */
  __IO uint32_t  PCDMA;                             /*!< Micro Direct Memory Access Power Control                              */
  __I  uint32_t  RESERVED39;
  __IO uint32_t  PCHIB;                             /*!< Hibernation Power Control                                             */
  __IO uint32_t  PCUART;                            /*!< Universal Asynchronous Receiver/Transmitter Power Control             */
  __IO uint32_t  PCSSI;                             /*!< Synchronous Serial Interface Power Control                            */
  __IO uint32_t  PCI2C;                             /*!< Inter-Integrated Circuit Power Control                                */
  __I  uint32_t  RESERVED40[5];
  __IO uint32_t  PCADC;                             /*!< Analog-to-Digital Converter Power Control                             */
  __IO uint32_t  PCACMP;                            /*!< Analog Comparator Power Control                                       */
  __I  uint32_t  RESERVED41[2];
  __IO uint32_t  PCLPC;                             /*!< Low Pin Count Interface Power Control                                 */
  __I  uint32_t  RESERVED42;
  __IO uint32_t  PCPECI;                            /*!< Platform Environment Control Interface Power Control                  */
  __IO uint32_t  PCFAN;                             /*!< Fan Power Control                                                     */
  __IO uint32_t  PCEEPROM;                          /*!< EEPROM Power Control                                                  */
  __IO uint32_t  PCWTIMER;                          /*!< Wide Timer Power Control                                              */
  __I  uint32_t  RESERVED43[40];
  __IO uint32_t  PRWD;                              /*!< Watchdog Timer Peripheral Ready                                       */
  __IO uint32_t  PRTIMER;                           /*!< Timer Peripheral Ready                                                */
  __IO uint32_t  PRGPIO;                            /*!< General-Purpose Input/Output Peripheral Ready                         */
  __IO uint32_t  PRDMA;                             /*!< Micro Direct Memory Access Peripheral Ready                           */
  __I  uint32_t  RESERVED44;
  __IO uint32_t  PRHIB;                             /*!< Hibernation Peripheral Ready                                          */
  __IO uint32_t  PRUART;                            /*!< Universal Asynchronous Receiver/Transmitter Peripheral Ready          */
  __IO uint32_t  PRSSI;                             /*!< Synchronous Serial Interface Peripheral Ready                         */
  __IO uint32_t  PRI2C;                             /*!< Inter-Integrated Circuit Peripheral Ready                             */
  __I  uint32_t  RESERVED45[5];
  __IO uint32_t  PRADC;                             /*!< Analog-to-Digital Converter Peripheral Ready                          */
  __IO uint32_t  PRACMP;                            /*!< Analog Comparator Peripheral Ready                                    */
  __I  uint32_t  RESERVED46[2];
  __IO uint32_t  PRLPC;                             /*!< Low Pin Count Interface Peripheral Ready                              */
  __I  uint32_t  RESERVED47;
  __IO uint32_t  PRPECI;                            /*!< Platform Environment Control Interface Peripheral Ready               */
  __IO uint32_t  PRFAN;                             /*!< FAN Peripheral Ready                                                  */
  __IO uint32_t  PREEPROM;                          /*!< EEPROM Peripheral Ready                                               */
  __IO uint32_t  PRWTIMER;                          /*!< Wide Timer Peripheral Ready                                           */
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
  __I  uint32_t  RESERVED1[300];
  __IO uint32_t  CHASGN;                            /*!< DMA Channel Assignment                                                */
  __IO uint32_t  CHIS;                              /*!< DMA Channel Interrupt Status                                          */
  __I  uint32_t  RESERVED2[2];
  __IO uint32_t  CHMAP0;                            /*!< DMA Channel Map Select 0                                              */
  __IO uint32_t  CHMAP1;                            /*!< DMA Channel Map Select 1                                              */
  __IO uint32_t  CHMAP2;                            /*!< DMA Channel Map Select 2                                              */
  __IO uint32_t  CHMAP3;                            /*!< DMA Channel Map Select 3                                              */
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
#define WATCHDOG1_BASE                  0x40001000UL
#define GPIOA_BASE                      0x40004000UL
#define GPIOB_BASE                      0x40005000UL
#define GPIOC_BASE                      0x40006000UL
#define GPIOD_BASE                      0x40007000UL
#define SSI0_BASE                       0x40008000UL
#define SSI1_BASE                       0x40009000UL
#define SSI2_BASE                       0x4000A000UL
#define SSI3_BASE                       0x4000B000UL
#define UART0_BASE                      0x4000C000UL
#define UART1_BASE                      0x4000D000UL
#define UART2_BASE                      0x4000E000UL
#define UART3_BASE                      0x4000F000UL
#define UART4_BASE                      0x40010000UL
#define UART5_BASE                      0x40011000UL
#define UART6_BASE                      0x40012000UL
#define UART7_BASE                      0x40013000UL
#define I2C0_BASE                       0x40020000UL
#define I2C1_BASE                       0x40021000UL
#define I2C2_BASE                       0x40022000UL
#define I2C3_BASE                       0x40023000UL
#define GPIOE_BASE                      0x40024000UL
#define GPIOF_BASE                      0x40025000UL
#define GPIOG_BASE                      0x40026000UL
#define GPIOH_BASE                      0x40027000UL
#define TIMER0_BASE                     0x40030000UL
#define TIMER1_BASE                     0x40031000UL
#define TIMER2_BASE                     0x40032000UL
#define TIMER3_BASE                     0x40033000UL
#define TIMER4_BASE                     0x40034000UL
#define TIMER5_BASE                     0x40035000UL
#define WTIMER0_BASE                    0x40036000UL
#define WTIMER1_BASE                    0x40037000UL
#define ADC0_BASE                       0x40038000UL
#define ADC1_BASE                       0x40039000UL
#define COMP_BASE                       0x4003C000UL
#define GPIOJ_BASE                      0x4003D000UL
#define WTIMER2_BASE                    0x4004C000UL
#define WTIMER3_BASE                    0x4004D000UL
#define WTIMER4_BASE                    0x4004E000UL
#define WTIMER5_BASE                    0x4004F000UL
#define GPIOA_AHB_BASE                  0x40058000UL
#define GPIOB_AHB_BASE                  0x40059000UL
#define GPIOC_AHB_BASE                  0x4005A000UL
#define GPIOD_AHB_BASE                  0x4005B000UL
#define GPIOE_AHB_BASE                  0x4005C000UL
#define GPIOF_AHB_BASE                  0x4005D000UL
#define GPIOG_AHB_BASE                  0x4005E000UL
#define GPIOH_AHB_BASE                  0x4005F000UL
#define GPIOJ_AHB_BASE                  0x40060000UL
#define GPIOK_BASE                      0x40061000UL
#define GPIOL_BASE                      0x40062000UL
#define GPIOM_BASE                      0x40063000UL
#define GPION_BASE                      0x40064000UL
#define GPIOP_BASE                      0x40065000UL
#define GPIOQ_BASE                      0x40066000UL
#define LPC0_BASE                       0x40080000UL
#define FAN0_BASE                       0x40084000UL
#define EEPROM_BASE                     0x400AF000UL
#define PECI0_BASE                      0x400B0000UL
#define I2C4_BASE                       0x400C0000UL
#define I2C5_BASE                       0x400C1000UL
#define SYSEXC_BASE                     0x400F9000UL
#define HIB_BASE                        0x400FC000UL
#define FLASH_CTRL_BASE                 0x400FD000UL
#define SYSCTL_BASE                     0x400FE000UL
#define UDMA_BASE                       0x400FF000UL


/* ================================================================================ */
/* ================             Peripheral declaration             ================ */
/* ================================================================================ */

#define WATCHDOG0                       ((WATCHDOG0_Type          *) WATCHDOG0_BASE)
#define WATCHDOG1                       ((WATCHDOG0_Type          *) WATCHDOG1_BASE)
#define GPIOA                           ((GPIOA_Type              *) GPIOA_BASE)
#define GPIOB                           ((GPIOA_Type              *) GPIOB_BASE)
#define GPIOC                           ((GPIOA_Type              *) GPIOC_BASE)
#define GPIOD                           ((GPIOA_Type              *) GPIOD_BASE)
#define SSI0                            ((SSI0_Type               *) SSI0_BASE)
#define SSI1                            ((SSI0_Type               *) SSI1_BASE)
#define SSI2                            ((SSI0_Type               *) SSI2_BASE)
#define SSI3                            ((SSI0_Type               *) SSI3_BASE)
#define UART0                           ((UART0_Type              *) UART0_BASE)
#define UART1                           ((UART0_Type              *) UART1_BASE)
#define UART2                           ((UART0_Type              *) UART2_BASE)
#define UART3                           ((UART0_Type              *) UART3_BASE)
#define UART4                           ((UART0_Type              *) UART4_BASE)
#define UART5                           ((UART0_Type              *) UART5_BASE)
#define UART6                           ((UART0_Type              *) UART6_BASE)
#define UART7                           ((UART0_Type              *) UART7_BASE)
#define I2C0                            ((I2C0_Type               *) I2C0_BASE)
#define I2C1                            ((I2C0_Type               *) I2C1_BASE)
#define I2C2                            ((I2C0_Type               *) I2C2_BASE)
#define I2C3                            ((I2C0_Type               *) I2C3_BASE)
#define GPIOE                           ((GPIOA_Type              *) GPIOE_BASE)
#define GPIOF                           ((GPIOA_Type              *) GPIOF_BASE)
#define GPIOG                           ((GPIOA_Type              *) GPIOG_BASE)
#define GPIOH                           ((GPIOA_Type              *) GPIOH_BASE)
#define TIMER0                          ((TIMER0_Type             *) TIMER0_BASE)
#define TIMER1                          ((TIMER0_Type             *) TIMER1_BASE)
#define TIMER2                          ((TIMER0_Type             *) TIMER2_BASE)
#define TIMER3                          ((TIMER0_Type             *) TIMER3_BASE)
#define TIMER4                          ((TIMER0_Type             *) TIMER4_BASE)
#define TIMER5                          ((TIMER0_Type             *) TIMER5_BASE)
#define WTIMER0                         ((WTIMER0_Type            *) WTIMER0_BASE)
#define WTIMER1                         ((TIMER0_Type             *) WTIMER1_BASE)
#define ADC0                            ((ADC0_Type               *) ADC0_BASE)
#define ADC1                            ((ADC0_Type               *) ADC1_BASE)
#define COMP                            ((COMP_Type               *) COMP_BASE)
#define GPIOJ                           ((GPIOA_Type              *) GPIOJ_BASE)
#define WTIMER2                         ((TIMER0_Type             *) WTIMER2_BASE)
#define WTIMER3                         ((TIMER0_Type             *) WTIMER3_BASE)
#define WTIMER4                         ((TIMER0_Type             *) WTIMER4_BASE)
#define WTIMER5                         ((TIMER0_Type             *) WTIMER5_BASE)
#define GPIOA_AHB                       ((GPIOA_Type              *) GPIOA_AHB_BASE)
#define GPIOB_AHB                       ((GPIOA_Type              *) GPIOB_AHB_BASE)
#define GPIOC_AHB                       ((GPIOA_Type              *) GPIOC_AHB_BASE)
#define GPIOD_AHB                       ((GPIOA_Type              *) GPIOD_AHB_BASE)
#define GPIOE_AHB                       ((GPIOA_Type              *) GPIOE_AHB_BASE)
#define GPIOF_AHB                       ((GPIOA_Type              *) GPIOF_AHB_BASE)
#define GPIOG_AHB                       ((GPIOA_Type              *) GPIOG_AHB_BASE)
#define GPIOH_AHB                       ((GPIOA_Type              *) GPIOH_AHB_BASE)
#define GPIOJ_AHB                       ((GPIOA_Type              *) GPIOJ_AHB_BASE)
#define GPIOK                           ((GPIOA_Type              *) GPIOK_BASE)
#define GPIOL                           ((GPIOA_Type              *) GPIOL_BASE)
#define GPIOM                           ((GPIOA_Type              *) GPIOM_BASE)
#define GPION                           ((GPIOA_Type              *) GPION_BASE)
#define GPIOP                           ((GPIOA_Type              *) GPIOP_BASE)
#define GPIOQ                           ((GPIOA_Type              *) GPIOQ_BASE)
#define LPC0                            ((LPC0_Type               *) LPC0_BASE)
#define FAN0                            ((FAN0_Type               *) FAN0_BASE)
#define EEPROM                          ((EEPROM_Type             *) EEPROM_BASE)
#define PECI0                           ((PECI0_Type              *) PECI0_BASE)
#define I2C4                            ((I2C0_Type               *) I2C4_BASE)
#define I2C5                            ((I2C0_Type               *) I2C5_BASE)
#define SYSEXC                          ((SYSEXC_Type             *) SYSEXC_BASE)
#define HIB                             ((HIB_Type                *) HIB_BASE)
#define FLASH_CTRL                      ((FLASH_CTRL_Type         *) FLASH_CTRL_BASE)
#define SYSCTL                          ((SYSCTL_Type             *) SYSCTL_BASE)
#define UDMA                            ((UDMA_Type               *) UDMA_BASE)


/** @} */ /* End of group Device_Peripheral_Registers */
/** @} */ /* End of group LM4FS1GE5BB */
/** @} */ /* End of group TI */

#ifdef __cplusplus
}
#endif


#endif  /* LM4FS1GE5BB_H */

