/*************************************************************************** */
/**
* @file     uc_regs.h
*           Infineon XMC Device Series
* @version  V1.1
* @date     12 Feb 2013
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
V1.0, 25 Jan 2013, PKB, First version for XMC4 family
V1.1, 12 Feb 2013, RD, Addition of XMC1 devices  
*****************************************************************************/
#ifndef __UCREGS_H__
#define __UCREGS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <uc_id.h>

#if (UC_SERIES == XMC45)
#include <XMC4500_series/Include/XMC4500.h>
#elif (UC_SERIES == XMC44)
#include <XMC4400_series/Include/XMC4400.h>
#elif ((UC_SERIES == XMC42) ||(UC_SERIES == XMC41) )
#include <XMC4200-4100_series/Include/XMC4200.h>
#elif (UC_SERIES == XMC11)
#include <XMC1100_series/Include/XMC1100.h>
#elif (UC_SERIES == XMC12)
#include <XMC1200_series/Include/XMC1200.h>
#elif (UC_SERIES == XMC13)
#include <XMC1300_series/Include/XMC1300.h>
#else
#error "Unsupported XMC family"
#endif

#ifdef __cplusplus
}
#endif


#endif /* __UCREGS_H__ */
