/*************************************************************************** */
/**
* @file     XMC1300_SCU.inc
*           Infineon XMC Device Series
* @version  V1.0
* @date     18 Feb 2013
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

/*****************************************************************************
************************* Version history ************************************
V1.0, 18 Feb 2013, PKB, Draft
*****************************************************************************/

/* ================================================================================ */
/* ================        struct 'SCU_CLK' Position & Mask        ================ */
/* ================================================================================ */


/* --------------------------------  SCU_CLK_CLKCR  ------------------------------- */
#define SCU_CLK_CLKCR_FDIV_Pos                0                                                       /*!< SCU_CLK CLKCR: FDIV Position            */
#define SCU_CLK_CLKCR_FDIV_Msk                (0x000000ff << SCU_CLK_CLKCR_FDIV_Pos)                /*!< SCU_CLK CLKCR: FDIV Mask                */
#define SCU_CLK_CLKCR_IDIV_Pos                8                                                       /*!< SCU_CLK CLKCR: IDIV Position            */
#define SCU_CLK_CLKCR_IDIV_Msk                (0x000000ff << SCU_CLK_CLKCR_IDIV_Pos)               /*!< SCU_CLK CLKCR: IDIV Mask                */
#define SCU_CLK_CLKCR_PCLKSEL_Pos             16                                                      /*!< SCU_CLK CLKCR: PCLKSEL Position         */
#define SCU_CLK_CLKCR_PCLKSEL_Msk             (0x01 << SCU_CLK_CLKCR_PCLKSEL_Pos)                   /*!< SCU_CLK CLKCR: PCLKSEL Mask             */
#define SCU_CLK_CLKCR_RTCCLKSEL_Pos           17                                                      /*!< SCU_CLK CLKCR: RTCCLKSEL Position       */
#define SCU_CLK_CLKCR_RTCCLKSEL_Msk           (0x07 << SCU_CLK_CLKCR_RTCCLKSEL_Pos)                 /*!< SCU_CLK CLKCR: RTCCLKSEL Mask           */
#define SCU_CLK_CLKCR_CNTADJ_Pos              20                                                      /*!< SCU_CLK CLKCR: CNTADJ Position          */
#define SCU_CLK_CLKCR_CNTADJ_Msk              (0x000003ff << SCU_CLK_CLKCR_CNTADJ_Pos)              /*!< SCU_CLK CLKCR: CNTADJ Mask              */
#define SCU_CLK_CLKCR_VDDC2LOW_Pos            30                                                      /*!< SCU_CLK CLKCR: VDDC2LOW Position        */
#define SCU_CLK_CLKCR_VDDC2LOW_Msk            (0x01 << SCU_CLK_CLKCR_VDDC2LOW_Pos)                  /*!< SCU_CLK CLKCR: VDDC2LOW Mask            */
#define SCU_CLK_CLKCR_VDDC2HIGH_Pos           31                                                      /*!< SCU_CLK CLKCR: VDDC2HIGH Position       */
#define SCU_CLK_CLKCR_VDDC2HIGH_Msk           (0x01 << SCU_CLK_CLKCR_VDDC2HIGH_Pos)                 /*!< SCU_CLK CLKCR: VDDC2HIGH Mask           */

/* ------------------------------  SCU_CLK_CGATCLR0  ------------------------------ */
#define SCU_CLK_CGATCLR0_VADC_Pos             0                                                       /*!< SCU_CLK CGATCLR0: VADC Position         */
#define SCU_CLK_CGATCLR0_VADC_Msk             (0x01 << SCU_CLK_CGATCLR0_VADC_Pos)                   /*!< SCU_CLK CGATCLR0: VADC Mask             */
#define SCU_CLK_CGATCLR0_CCU80_Pos            1                                                       /*!< SCU_CLK CGATCLR0: CCU80 Position        */
#define SCU_CLK_CGATCLR0_CCU80_Msk            (0x01 << SCU_CLK_CGATCLR0_CCU80_Pos)                  /*!< SCU_CLK CGATCLR0: CCU80 Mask            */
#define SCU_CLK_CGATCLR0_CCU40_Pos            2                                                       /*!< SCU_CLK CGATCLR0: CCU40 Position        */
#define SCU_CLK_CGATCLR0_CCU40_Msk            (0x01 << SCU_CLK_CGATCLR0_CCU40_Pos)                  /*!< SCU_CLK CGATCLR0: CCU40 Mask            */
#define SCU_CLK_CGATCLR0_USIC0_Pos            3                                                       /*!< SCU_CLK CGATCLR0: USIC0 Position        */
#define SCU_CLK_CGATCLR0_USIC0_Msk            (0x01 << SCU_CLK_CGATCLR0_USIC0_Pos)                  /*!< SCU_CLK CGATCLR0: USIC0 Mask            */
#define SCU_CLK_CGATCLR0_BCCU0_Pos            4                                                       /*!< SCU_CLK CGATCLR0: BCCU0 Position        */
#define SCU_CLK_CGATCLR0_BCCU0_Msk            (0x01 << SCU_CLK_CGATCLR0_BCCU0_Pos)                  /*!< SCU_CLK CGATCLR0: BCCU0 Mask            */
#define SCU_CLK_CGATCLR0_POSIF0_Pos           7                                                       /*!< SCU_CLK CGATCLR0: POSIF0 Position       */
#define SCU_CLK_CGATCLR0_POSIF0_Msk           (0x01 << SCU_CLK_CGATCLR0_POSIF0_Pos)                 /*!< SCU_CLK CGATCLR0: POSIF0 Mask           */
#define SCU_CLK_CGATCLR0_MATH_Pos             8                                                       /*!< SCU_CLK CGATCLR0: MATH Position         */
#define SCU_CLK_CGATCLR0_MATH_Msk             (0x01 << SCU_CLK_CGATCLR0_MATH_Pos)                   /*!< SCU_CLK CGATCLR0: MATH Mask             */
#define SCU_CLK_CGATCLR0_WDT_Pos              9                                                       /*!< SCU_CLK CGATCLR0: WDT Position          */
#define SCU_CLK_CGATCLR0_WDT_Msk              (0x01 << SCU_CLK_CGATCLR0_WDT_Pos)                    /*!< SCU_CLK CGATCLR0: WDT Mask              */
#define SCU_CLK_CGATCLR0_RTC_Pos              10                                                      /*!< SCU_CLK CGATCLR0: RTC Position          */
#define SCU_CLK_CGATCLR0_RTC_Msk              (0x01 << SCU_CLK_CGATCLR0_RTC_Pos)                    /*!< SCU_CLK CGATCLR0: RTC Mask              */
