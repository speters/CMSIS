/*****************************************************************************
 *                                                                           *
 * Copyright (c) 2013 Rowley Associates Limited.                             *
 *                                                                           *
 * This file may be distributed under the terms of the License Agreement     *
 * provided with this software.                                              *
 *                                                                           *
 * THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND, INCLUDING THE   *
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. *
 *****************************************************************************/

#include <stdint.h>
#include "TM4C129.h"

#define CLOCK_SETUP 1

#if (CLOCK_SETUP)
uint32_t SystemCoreClock = 120000000; // (25 * 96) / (4 + 1) / 4
#else
uint32_t SystemCoreClock = 16000000;
#endif

void SystemInit (void)
{
#if (CLOCK_SETUP)
  SYSCTL->MOSCCTL = 0x10; // Enable HF MOSC
  SYSCTL->RSCLKCFG = 0x03300000; // MOSC is PLL input and used when bypassed 
  SYSCTL->PLLFREQ1 = 0x4; // N = 4
  SYSCTL->PLLFREQ0 = 0x60; // M = 96
  SYSCTL->MEMTIM0 = 0x01960196;
  SYSCTL->PLLFREQ0 |= (1<<23);
  while (!(SYSCTL->PLLSTAT & 1));
  SYSCTL->RSCLKCFG = 0x93300003; // MEMTIMU | USEPLL | / 4
#endif
}
