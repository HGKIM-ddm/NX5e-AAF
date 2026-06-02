/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
* LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR
* ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability
* of this software. By using this software, you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2018, 2024 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name        : Config_TAUJ1.h
* Component Version: 1.7.0
* Device(s)        : R7F701695
* Description      : This file implements device driver for Config_TAUJ1.
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_tauj.h"

#ifndef CFG_Config_TAUJ1_H
#define CFG_Config_TAUJ1_H

/***********************************************************************************************************************
Macro definitions (Register bit)
***********************************************************************************************************************/

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
#define _TAUJ1_CHANNEL0_COMPARE_VALUE                            (0x0000056CUL) /* Data register for compare values */
#define _TAUJ1_CHANNEL1_COMPARE_VALUE                            (0x000002B7UL) /* Data register for compare values */

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/
void R_Config_TAUJ1_Create(void);
void R_Config_TAUJ1_Start(void);
void R_Config_TAUJ1_Stop(void);
void R_Config_TAUJ1_Create_UserInit(void);
/* Start user code for function. Do not edit comment generated here */
// 1/32STEP 4.5RPM 0x0000056CUL, 0x000002B7UL    1/8STEP 5RPM 0x00001387UL, 0x000009C4UL
// 1/8STEP 4.5RPM 0x000015B3UL, 0x00000ADAUL 1/64STEP 4.5RPM 0x000002B5UL, 0x0000015BUL
/* End user code. Do not edit comment generated here */
#endif
