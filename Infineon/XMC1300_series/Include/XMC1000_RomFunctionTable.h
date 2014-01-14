/*************************************************************************** */
/**
* @file     XMC1000_RomFunctionTable.h
*           Infineon XMC Device Series
* @version  V1.0
* @date     08 April 2013
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
V1.0, 08 Jan 2013, GD, First version with XMC1000 devices
*****************************************************************************/

#ifndef ROM_FUNCTION_TABLE_H
#define ROM_FUNCTION_TABLE_H

#ifdef __cplusplus
extern "C" {
#endif

/* ***************************************************************************
******************************* Constants *******************************
*************************************************************************** */
/* Start address of the ROM function table */
#define ROM_FUNCTION_TABLE_START    (0x00000100)

/* Pointer to Erase Flash Page routine */
#define _NvmErase                    (ROM_FUNCTION_TABLE_START + 0x00) 

/* Pointer to Erase, Program & Verify Flash Page routine */
#define _NvmProgVerify               (ROM_FUNCTION_TABLE_START + 0x04) 

/* Pointer to Request BMI installation routine */
#define _BmiInstallationReq          (ROM_FUNCTION_TABLE_START + 0x08) 


/* ***************************************************************************
******************************** Enumerations ********************************
*************************************************************************** */
typedef enum TagNVMStatus
{

/* The function succeeded */
 NVM_PASS               = (signed long) 0x00010000, 
 
/* Generic error code */ 
 NVM_E_FAIL             = (signed long) 0x80010001, 
 
/* Source data not in RAM */ 
 NVM_E_SRC_AREA_EXCCEED = (signed long) 0x80010003, 
 
/* Source data is not 4 byte aligned */ 
 NVM_E_SRC_ALIGNMENT    = (signed long) 0x80010004, 
 
/* NVM module cannot be physically accessed */ 
 NVM_E_NVM_FAIL         = (signed long) 0x80010005, 
 
/* Verification of written page not successful */ 
 NVM_E_VERIFY           = (signed long) 0x80010006, 
 
/* Destination data is not (completely) located in NVM */ 
 NVM_E_DST_AREA_EXCEED  = (signed long) 0x80010009, 
 
/* Destination data is not properly aligned */ 
 NVM_E_DST_ALIGNMENT    = (signed long) 0x80010010, 

} NVM_STATUS;


/* ***************************************************************************
*********************************** Macros ***********************************
*************************************************************************** */

/* ***************************************************************************
Description: Erase granularity = 1 Page of  16 blocks of 16 Bytes
                               = Equivalent to 256 Bytes using this routine.

Input parameters:                                                                                                              
– Logical address of the Flash Page to be erased which must be page aligned 
and in NVM address range

Return status:
– OK (NVM_PASS)
– Invalid address (NVM_E_DST_ALIGNMENT or NVM_E_DST_AREA_EXCEED)
– Operation failed (Error during low level NVM programming driver):
                                                                NVM_E_FAIL 
                                                                NVM_E_VERIFY 
                                                                NVM_E_NVM_FAIL

Prototype: 
    NVM_STATUS XMC1000_NvmErasePage(unsigned long* pageAddr)
*************************************************************************** */
#define XMC1000_NvmErasePage (*((NVM_STATUS (**) (unsigned long * )) \
                                                                   _NvmErase))

/* ***************************************************************************
Description: This procedure performs erase (skipped if not necessary), program 
and verify of selected Flash page.

Input parameter:                                                                                                              
– Logical address of the target Flash Page, must be page aligned and in NVM 
address range

– Address in SRAM where the data starts, must be 4-byte aligned

Return status:
–  OK (NVM_PASS)
–  Invalid addresses 
                    NVM_E_DST_ALIGNMENT 
                    NVM_E_SRC_ALIGNMENT 
                    NVM_E_DST_AREA_EXCEED 
                    NVM_E_SRC_AREA_EXCCEED
–  Operation failed (Error during low level NVM programming driver): 
                                                                 NVM_E_FAIL
                                                                 NVM_E_VERIFY 
                                                                 NVM_E_NVM_FAIL

Prototype: 
  NVM_STATUS XMC1000_NvmProgVerify(unsigned long* srcAddr, unsigned long* dstAddr)
*************************************************************************** */
#define XMC1000_NvmProgVerify (*((NVM_STATUS (**) (unsigned long * ,\
                                                   unsigned long * ))\
                                                               _NvmProgVerify))


/* ***************************************************************************
Description: This procedure initiates installation of a new BMI value. In 
particular, it can be used as well as to restore the state upon delivery for a
device already in User Productive mode. 

Input parameter:                                                                                                              
– BMI value to be installed

Return status:
–  wrong input BMI value (0x01) - only upon error, if OK the procedure triggers
a reset respectively does not return to calling routine !

Prototype: 
  unsigned long XMC1000_BmiInstallationReq(unsigned short requestedBmiValue)
**************************************************************************** */
#define XMC1000_BmiInstallationReq (*((unsigned long (**) (unsigned short)) \
                                                          _BmiInstallationReq))

#ifdef __cplusplus
}
#endif

#endif /* ROM_FUNCTION_TABLE_H */
