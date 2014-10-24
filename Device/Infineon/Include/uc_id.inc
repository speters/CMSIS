/*************************************************************************** */
/**
* @file     uc_id.inc
*           Infineon XMC Device Series
* @version  V1.2
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
V1.0, 25 Jan 2013, PKB, First version with XMC4 devices
V1.1, 12 Feb 2013, RD, Addition of XMC1 devices
V1.2, 18 Feb 2013, RD, Updated Package definitions
V1.3, 21 Jan 2014, RD, Addition of VQFN Packages for XMC1 devices
*****************************************************************************/

/* ***************************************************************************
******************************* ID definitions *******************************
*************************************************************************** */
/* Family definitions  */
#define    XMC4       (4)
#define    XMC1       (1)

/* Series definitions  */
#define    XMC45      (45)
#define    XMC44      (44)
#define    XMC42      (42)
#define    XMC41      (41)

#define    XMC13      (13)
#define    XMC12      (12)
#define    XMC11      (11)

/* Device definitions  */
#define    XMC4500    (4500)
#define    XMC4502    (4502)
#define    XMC4504    (4504)
#define    XMC4400    (4400)
#define    XMC4402    (4402)
#define    XMC4200    (4200)
#define    XMC4100    (4100)
#define    XMC4104    (4104)

#define    XMC1300    (1300)
#define    XMC1301    (1301)
#define    XMC1302    (1302)
#define    XMC1200    (1200)
#define    XMC1201    (1201)
#define    XMC1202    (1202)
#define    XMC1203    (1203)
#define    XMC1100    (1100)

/* STEP definitions    */
#define    STEP_AA    (1)
#define    STEP_AB    (2)
#define    STEP_AC    (3)
#define    STEP_BA    (4)
#define    STEP_BB    (5)
#define    STEP_BC    (6)

/* Package definitions */
#define    BGA144     (1)
#define    LQFP144    (2)
#define    LQFP100    (3)
#define    BGA64      (4)
#define    LQFP64     (5)
#define    VQFN48     (6)
#define    TSSOP38    (7)
#define    TSSOP28    (8)
#define    TSSOP16    (9) 
#define    VQFN24     (10)
#define    VQFN40     (11)

/************************* End of ID definitions ************************** */

/* *************** README README README README README README ************** */
/*
 * Within a family and specifically within a series, SW ID uniquely identifies
 * a device by available functionality, silicon revision (aka STEP) and 
 * packaging information.
 *
 * The early engineering samples of XMC45/44/42/41 series and initial steps of 
 * productive XMC45 series devices had a simple 4 digit SW ID.
 *
 * In order to make this numbering scheme more scalable, a 7 digit encoding
 * scheme is adopted for later steps of productive samples of XMC45/44/42/41
 */

/* ***************************************************************************
**************** Mapping between IDs and supported SW IDs ********************
*************************************************************************** */

/* ***************** Start of section for XMC45 devices ******************* */
#if (UC_ID == 4501)         /* SW ID : 45.0.1 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC45
#define UC_DEVICE    XMC4500
#define UC_STEP      STEP_AA
#define UC_PACKAGE   BGA144

#elif (UC_ID == 4502)       /* SW ID : 45.0.2 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC45
#define UC_DEVICE    XMC4500
#define UC_STEP      STEP_AA
#define UC_PACKAGE   LQFP144

#elif (UC_ID == 4503)       /* SW ID : 45.0.3 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC45
#define UC_DEVICE    XMC4500
#define UC_STEP      STEP_AA
#define UC_PACKAGE   LQFP100

#elif (UC_ID == 4523)       /* SW ID : 45.2.3 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC45
#define UC_DEVICE    XMC4502
#define UC_STEP      STEP_AA
#define UC_PACKAGE   LQFP100

#elif (UC_ID == 4543)       /* SW ID : 45.4.3 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC45
#define UC_DEVICE    XMC4504
#define UC_STEP      STEP_AA
#define UC_PACKAGE   LQFP100

#elif (UC_ID == 4542)       /* SW ID : 45.4.2 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC45
#define UC_DEVICE    XMC4504
#define UC_STEP      STEP_AA
#define UC_PACKAGE   LQFP144

#elif (UC_ID == 4502001)    /* SW ID : 45.0200.1 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC45
#define UC_DEVICE    XMC4500
#define UC_STEP      STEP_AB
#define UC_PACKAGE   BGA144

#elif (UC_ID == 4502002)    /* SW ID : 45.0200.2 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC45
#define UC_DEVICE    XMC4500
#define UC_STEP      STEP_AB
#define UC_PACKAGE   LQFP144

#elif (UC_ID == 4502003)    /* SW ID : 45.0200.3 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC45
#define UC_DEVICE    XMC4500
#define UC_STEP      STEP_AB
#define UC_PACKAGE   LQFP100

#elif (UC_ID == 4502023)    /* SW ID : 45.0202.3 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC45
#define UC_DEVICE    XMC4502
#define UC_STEP      STEP_AB
#define UC_PACKAGE   LQFP100

#elif (UC_ID == 4502043)    /* SW ID : 45.0204.3 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC45
#define UC_DEVICE    XMC4504
#define UC_STEP      STEP_AB
#define UC_PACKAGE   LQFP100

#elif (UC_ID == 4502042)    /* SW ID : 45.0204.2 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC45
#define UC_DEVICE    XMC4504
#define UC_STEP      STEP_AB
#define UC_PACKAGE   LQFP144

#elif (UC_ID == 4503001)    /* SW ID : 45.0300.1 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC45
#define UC_DEVICE    XMC4500
#define UC_STEP      STEP_AC
#define UC_PACKAGE   BGA144

#elif (UC_ID == 4503002)    /* SW ID : 45.0300.2 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC45
#define UC_DEVICE    XMC4500
#define UC_STEP      STEP_AC
#define UC_PACKAGE   LQFP144

#elif (UC_ID == 4503003)    /* SW ID : 45.0300.3 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC45
#define UC_DEVICE    XMC4500
#define UC_STEP      STEP_AC
#define UC_PACKAGE   LQFP100

#elif (UC_ID == 4503023)    /* SW ID : 45.0302.3 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC45
#define UC_DEVICE    XMC4502
#define UC_STEP      STEP_AC
#define UC_PACKAGE   LQFP100

#elif (UC_ID == 4503043)    /* SW ID : 45.0304.3 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC45
#define UC_DEVICE    XMC4504
#define UC_STEP      STEP_AC
#define UC_PACKAGE   LQFP100

#elif (UC_ID == 4503042)    /* SW ID : 45.0304.2 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC45
#define UC_DEVICE    XMC4504
#define UC_STEP      STEP_AC
#define UC_PACKAGE   LQFP144

/* ***************** Start of section for XMC44 devices ******************* */
#elif (UC_ID == 4403)       /* SW ID : 44.0.3 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC44
#define UC_DEVICE    XMC4400
#define UC_STEP      STEP_AA
#define UC_PACKAGE   LQFP100

#elif (UC_ID == 44005)      /* SW ID : 44.00.5 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC44
#define UC_DEVICE    XMC4400
#define UC_STEP      STEP_AA
#define UC_PACKAGE   LQFP64

#elif (UC_ID == 44023)      /* SW ID : 44.02.3 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC44
#define UC_DEVICE    XMC4402
#define UC_STEP      STEP_AA
#define UC_PACKAGE   LQFP100

#elif (UC_ID == 44025)      /* SW ID : 44.02.5 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC44
#define UC_DEVICE    XMC4402
#define UC_STEP      STEP_AA
#define UC_PACKAGE   LQFP64

#elif (UC_ID == 4402003)       /* SW ID : 44.0200.3 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC44
#define UC_DEVICE    XMC4400
#define UC_STEP      STEP_AB
#define UC_PACKAGE   LQFP100

#elif (UC_ID == 4402005)      /* SW ID : 44.0200.5 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC44
#define UC_DEVICE    XMC4400
#define UC_STEP      STEP_AB
#define UC_PACKAGE   LQFP64

#elif (UC_ID == 4402023)      /* SW ID : 44.0202.3 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC44
#define UC_DEVICE    XMC4402
#define UC_STEP      STEP_AB
#define UC_PACKAGE   LQFP100

#elif (UC_ID == 4402025)      /* SW ID : 44.0202.5 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC44
#define UC_DEVICE    XMC4402
#define UC_STEP      STEP_AB
#define UC_PACKAGE   LQFP64

/* **************** Start of section for XMC42/41 devices ****************** */
#elif (UC_ID == 42005)      /* SW ID : 42.00.5 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC42
#define UC_DEVICE    XMC4200
#define UC_STEP      STEP_AA
#define UC_PACKAGE   LQFP64

#elif (UC_ID == 4206)       /* SW ID : 42.0.6 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC42
#define UC_DEVICE    XMC4200
#define UC_STEP      STEP_AA
#define UC_PACKAGE   VQFN48

#elif (UC_ID == 42045)      /* SW ID : 42.04.5 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC41
#define UC_DEVICE    XMC4104
#define UC_STEP      STEP_AA
#define UC_PACKAGE   LQFP64

#elif (UC_ID == 42046)      /* SW ID : 42.04.6 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC41
#define UC_DEVICE    XMC4104
#define UC_STEP      STEP_AA
#define UC_PACKAGE   VQFN48

#elif (UC_ID == 4202005)      /* SW ID : 42.0200.5 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC42
#define UC_DEVICE    XMC4200
#define UC_STEP      STEP_AB
#define UC_PACKAGE   LQFP64

#elif (UC_ID == 4202006)       /* SW ID : 42.0200.6 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC42
#define UC_DEVICE    XMC4200
#define UC_STEP      STEP_AB
#define UC_PACKAGE   VQFN48

#elif (UC_ID == 4202045)      /* SW ID : 42.0204.5 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC41
#define UC_DEVICE    XMC4104
#define UC_STEP      STEP_AB
#define UC_PACKAGE   LQFP64

#elif (UC_ID == 4202046)      /* SW ID : 42.0204.6 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC41
#define UC_DEVICE    XMC4104
#define UC_STEP      STEP_AB
#define UC_PACKAGE   VQFN48

#elif (UC_ID == 4202086)      /* SW ID : 42.0208.6 */
#define UC_FAMILY    XMC4
#define UC_SERIES    XMC41
#define UC_DEVICE    XMC4108
#define UC_STEP      STEP_AB
#define UC_PACKAGE   VQFN48


/* **************** Start of section for XMC13 devices ****************** */
#elif (UC_ID == 1301017)       /* SW ID : 13.0101.7 */
#define UC_FAMILY    XMC1
#define UC_SERIES    XMC13
#define UC_DEVICE    XMC1301
#define UC_STEP      STEP_AA
#define UC_PACKAGE   TSSOP38

#elif (UC_ID == 1301019)       /* SW ID : 13.0101.9 */
#define UC_FAMILY    XMC1
#define UC_SERIES    XMC13
#define UC_DEVICE    XMC1301
#define UC_STEP      STEP_AA
#define UC_PACKAGE   TSSOP16

#elif (UC_ID == 13010110)      /* SW ID : 13.0101.10 */
#define UC_FAMILY    XMC1
#define UC_SERIES    XMC13
#define UC_DEVICE    XMC1301
#define UC_STEP      STEP_AA
#define UC_PACKAGE   VQFN24

#elif (UC_ID == 13010111)      /* SW ID : 13.0101.11 */
#define UC_FAMILY    XMC1
#define UC_SERIES    XMC13
#define UC_DEVICE    XMC1301
#define UC_STEP      STEP_AA
#define UC_PACKAGE   VQFN40

#elif (UC_ID == 1301027)      /* SW ID : 13.0102.7 */
#define UC_FAMILY    XMC1
#define UC_SERIES    XMC13
#define UC_DEVICE    XMC1302
#define UC_STEP      STEP_AA
#define UC_PACKAGE   TSSOP38

#elif (UC_ID == 1301029)       /* SW ID : 13.0102.9 */
#define UC_FAMILY    XMC1
#define UC_SERIES    XMC13
#define UC_DEVICE    XMC1302
#define UC_STEP      STEP_AA
#define UC_PACKAGE   TSSOP16

#elif (UC_ID == 13010210)      /* SW ID : 13.0102.10 */
#define UC_FAMILY    XMC1
#define UC_SERIES    XMC13
#define UC_DEVICE    XMC1302
#define UC_STEP      STEP_AA
#define UC_PACKAGE   VQFN24

#elif (UC_ID == 13010211)      /* SW ID : 13.0102.11 */
#define UC_FAMILY    XMC1
#define UC_SERIES    XMC13
#define UC_DEVICE    XMC1302
#define UC_STEP      STEP_AA
#define UC_PACKAGE   VQFN40

/* **************** Start of section for XMC12 devices ****************** */
#elif (UC_ID == 1201007)      /* SW ID : 12.0100.7 */
#define UC_FAMILY    XMC1
#define UC_SERIES    XMC12
#define UC_DEVICE    XMC1200
#define UC_STEP      STEP_AA
#define UC_PACKAGE   TSSOP38

#elif (UC_ID == 1201017)       /* SW ID : 12.0101.7 */
#define UC_FAMILY    XMC1
#define UC_SERIES    XMC12
#define UC_DEVICE    XMC1201
#define UC_STEP      STEP_AA
#define UC_PACKAGE   TSSOP38

#elif (UC_ID == 1201018)       /* SW ID : 12.0101.8 */
#define UC_FAMILY    XMC1
#define UC_SERIES    XMC12
#define UC_DEVICE    XMC1201
#define UC_STEP      STEP_AA
#define UC_PACKAGE   TSSOP28

#elif (UC_ID == 12010111)       /* SW ID : 12.0101.11 */
#define UC_FAMILY    XMC1
#define UC_SERIES    XMC12
#define UC_DEVICE    XMC1201
#define UC_STEP      STEP_AA
#define UC_PACKAGE   VQFN40

#elif (UC_ID == 1201027)      /* SW ID : 12.0102.7 */
#define UC_FAMILY    XMC1
#define UC_SERIES    XMC12
#define UC_DEVICE    XMC1202
#define UC_STEP      STEP_AA
#define UC_PACKAGE   TSSOP38

#elif (UC_ID == 1201028)      /* SW ID : 12.0102.8 */
#define UC_FAMILY    XMC1
#define UC_SERIES    XMC12
#define UC_DEVICE    XMC1202
#define UC_STEP      STEP_AA
#define UC_PACKAGE   TSSOP28

#elif (UC_ID == 1201029)      /* SW ID : 12.0102.9 */
#define UC_FAMILY    XMC1
#define UC_SERIES    XMC12
#define UC_DEVICE    XMC1202
#define UC_STEP      STEP_AA
#define UC_PACKAGE   TSSOP16

#elif (UC_ID == 12010210)      /* SW ID : 12.0102.10 */
#define UC_FAMILY    XMC1
#define UC_SERIES    XMC12
#define UC_DEVICE    XMC1202
#define UC_STEP      STEP_AA
#define UC_PACKAGE   VQFN24

#elif (UC_ID == 12010211)      /* SW ID : 12.0102.11 */
#define UC_FAMILY    XMC1
#define UC_SERIES    XMC12
#define UC_DEVICE    XMC1202
#define UC_STEP      STEP_AA
#define UC_PACKAGE   VQFN40

#elif (UC_ID == 1201039)      /* SW ID : 12.0103.9 */
#define UC_FAMILY    XMC1
#define UC_SERIES    XMC12
#define UC_DEVICE    XMC1203
#define UC_STEP      STEP_AA
#define UC_PACKAGE   TSSOP16

#elif (UC_ID == 12010310)      /* SW ID : 12.0103.10 */
#define UC_FAMILY    XMC1
#define UC_SERIES    XMC12
#define UC_DEVICE    XMC1203
#define UC_STEP      STEP_AA
#define UC_PACKAGE   VQFN24

/* **************** Start of section for XMC11 devices ****************** */
#elif (UC_ID == 1101007)      /* SW ID : 11.0100.7 */
#define UC_FAMILY    XMC1
#define UC_SERIES    XMC11
#define UC_DEVICE    XMC1100
#define UC_STEP      STEP_AA
#define UC_PACKAGE   TSSOP38

#elif (UC_ID == 1101009)      /* SW ID : 11.0100.9 */
#define UC_FAMILY    XMC1
#define UC_SERIES    XMC11
#define UC_DEVICE    XMC1100
#define UC_STEP      STEP_AA
#define UC_PACKAGE   TSSOP16

#elif (UC_ID == 11010010)      /* SW ID : 11.0100.10 */
#define UC_FAMILY    XMC1
#define UC_SERIES    XMC11
#define UC_DEVICE    XMC1100
#define UC_STEP      STEP_AA
#define UC_PACKAGE   VQFN24

#elif (UC_ID == 11010011)      /* SW ID : 11.0100.11 */
#define UC_FAMILY    XMC1
#define UC_SERIES    XMC11
#define UC_DEVICE    XMC1100
#define UC_STEP      STEP_AA
#define UC_PACKAGE   VQFN40

#else
#error "Unsupported XMC device"
#endif

#define __TARGET_DEVICE__ UC_SERIES

