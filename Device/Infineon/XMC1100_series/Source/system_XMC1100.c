/******************************************************************************
 * @file     system_XMC1100.c
 * @brief    Device specific initialization for the XMC1100-Series according 
 * to CMSIS
 * @version  V1.6
 * @date     19 Feb 2014
 *
 * @note
 * Copyright (C) 2012-2014 Infineon Technologies AG. All rights reserved.

 *
 * @par
 * Infineon Technologies AG (Infineon) is supplying this software for use with 
 * Infineon’s microcontrollers.
 *   
 * This file can be freely distributed within development tools that are 
 * supporting such microcontrollers.
 *  
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/
/*
 * *************************** Change history ********************************
 * V1.2, 13 Dec 2012, PKB : Created change history table
 * V1.3, 20 Dec 2012, PKB : Fixed SystemCoreClock computation
 * V1.4, 02 Feb 2013, PKB : SCU_CLOCK -> SCU_CLK
 * V1.5, 27 Nov 2013, DNE : Comments added in SystemInit function for MCLK support 
 * V1.6, 19 Feb 2014, JFT : Fixed SystemCoreClock when FDIV != 0
 */

#include "system_XMC1100.h"
#include <XMC1100.h>

/*----------------------------------------------------------------------------
  Clock Global defines
 *----------------------------------------------------------------------------*/
#define DCO_DCLK                  64000000UL
#define MCLK_MHZ                  32000000UL

/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
/*!< System Clock Frequency (Core Clock) (MCLK on TIMM1) */
uint32_t SystemCoreClock;

/**
  * @brief  Setup the microcontroller system.
  * @param  None
  * @retval None
  */
void SystemInit(void)
{    
  /*
   * Clock tree setup by CMSIS routines is allowed only in the absence of DAVE
   * Clock app.
   */ 
  /* Do not change default values of IDIV,FDIV and RTCCLKSEL */
  /* ====== Default configuration ======= */
  /*
   * MCLK    = DCO_DCLK
   * PCLK    = MCLK
   * RTC CLK = Standby clock
   */
  
   /* In the absence of DAVE Clock app, user can choose to change the MCLK
   * and PCLK setting in this routine. Using the following set of code.
   * This changes the MCLK to 16MHz and PCLK to 32MHz.
   *
   * SCU_GENERAL->PASSWD = 0x000000C0UL; // disable bit protection
   * SCU_CLK->CLKCR = 0x3FF01200UL; 	 // MCLK = 16MHz, PCLK = 32MHz
   * while((SCU_CLK->CLKCR & SCU_CLK_CLKCR_VDDC2LOW_Msk));
   * SCU_GENERAL->PASSWD = 0x000000C3UL; // enable bit protection
   * SystemCoreClockUpdate();
   *
   */
  SystemCoreClockUpdate();
}

/**
  * @brief  Update SystemCoreClock according to Clock Register Values
  * @note   -  
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate(void)
{
  uint32_t IDIV, FDIV;

  IDIV = ((SCU_CLK->CLKCR) & SCU_CLK_CLKCR_IDIV_Msk) >> SCU_CLK_CLKCR_IDIV_Pos;
  FDIV = ((SCU_CLK->CLKCR) & SCU_CLK_CLKCR_FDIV_Msk) >> SCU_CLK_CLKCR_FDIV_Pos;
  
  if(IDIV)
  {
    /* Fractional divider is enabled and used */
    SystemCoreClock = ((MCLK_MHZ << 7) / ((IDIV << 8) + FDIV)) << 1;
  }
  else
  {
    /* Fractional divider bypassed. Simply divide DCO_DCLK by 2 */
    SystemCoreClock = MCLK_MHZ;
  }
}
