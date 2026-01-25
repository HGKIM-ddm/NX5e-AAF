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
* File Name        : Config_TAUD0_3_user.c
* Component Version: 1.7.0
* Device(s)        : R7F701695
* Description      : This file implements device driver for Config_TAUD0_3.
***********************************************************************************************************************/
/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
extern unsigned int time_1ms_init_move;
extern unsigned int time_1ms_init_move_flag;
extern unsigned int time_1ms_adc;
extern unsigned int time_1ms_adc_flag;
extern unsigned int time_1ms_external_10s_chk;
extern unsigned int time_1ms_external_10s_chk_flag;

extern unsigned int time_1ms_adc_1s_chk;
extern unsigned int time_1ms_adc_1s_chk_flag;


extern unsigned int time_1ms_fdl_flag;
extern unsigned int timer_1ms_lin_bus_inactive;

extern unsigned int time_1ms_spi;
extern unsigned int time_1ms_spi_flag;
extern unsigned int time_1ms_3minute;
extern unsigned int time_1s_3minute;
extern unsigned int time_1ms_3minute_flag;
extern unsigned int time_1ms_init_chk;
extern unsigned int time_1ms_init_chk_flag;
extern unsigned int time_1ms_stall_chk;
extern unsigned int time_1ms_stall_chk_flag;
extern unsigned int time_1ms_protection_mode;
extern unsigned int time_1ms_protection_mode_flag;
extern unsigned int stall_chk_cnt;
extern unsigned int stall_chk_time_1ms;
extern unsigned int time_1ms_motor_wait;
extern unsigned int time_1ms_motor_wait_flag;
extern unsigned int time_1ms_voltage_chk_delay;
extern unsigned int time_1ms_voltage_chk_delay_flag;
extern unsigned int time_1ms_diag_auto;
extern unsigned int time_1ms_diag_auto_flag;
extern unsigned int time_1ms_voltage_status_change_wait;
extern unsigned int time_1ms_voltage_status_change_wait_flag;
extern unsigned int time_1ms_motor_acceleration;
extern unsigned int time_1ms_motor_acceleration_flag;
extern unsigned int timer_1ms_init_fail_chk;
extern unsigned int timer_1ms_init_fail_chk_flag;
extern unsigned int timer_1ms_lin_sleep;
extern unsigned int timer_1ms_lin_sleep_flag;
extern unsigned int time_1ms_antipinch;
extern unsigned int time_1ms_antipinch_flag;
extern unsigned int time_1ms_spi_error_chk;
extern unsigned int time_1ms_spi_error_chk_flag;
extern unsigned int time_1ms_adc_error_chk;
extern unsigned int time_1ms_adc_error_chk_flag;
extern unsigned int time_1ms_fdl_error_chk;
extern unsigned int time_1ms_fdl_error_chk_flag;

extern unsigned int time_1ms_protection_chk;
extern unsigned int time_1ms_protection_chk_flag;

extern unsigned int time_1ms_Moving_chk;
extern unsigned int time_1ms_Moving_chk_flag;

extern unsigned int time_1ms_step_chk;
extern unsigned int time_1ms_step_chk_flag;

extern unsigned int time_1ms_wdg_chk;
extern unsigned int time_1ms_wdg_chk_flag;

extern unsigned int time_1ms_LIN_GndShort;
extern unsigned int time_1ms_LIN_GndShort_flag;

extern unsigned int time_1ms_LIN_BatShort;
extern unsigned int time_1ms_LIN_BatShort_flag;

extern unsigned int time_1ms_Error_chk;
extern unsigned int time_1ms_Error_chk_flag;

extern unsigned int time_1ms_motor_Short_chk;
extern unsigned int time_1ms_motor_Short_chk_flag;

extern unsigned int time_1ms_motor_Open_chk;
extern unsigned int time_1ms_motor_Open_chk_flag;

extern unsigned int time_1ms_IGN_chk;
extern unsigned int time_1ms_IGN_chk_flag;

/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_userdefine.h"
#include "Config_TAUD0_3.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Config_TAUD0_3_Create_UserInit
* Description  : This function adds user code after initializing the TAUD03 channel
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_Config_TAUD0_3_Create_UserInit(void)
{
    /* Start user code for user init. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_TAUD0_3_interrupt
* Description  : This function is TAUD03 interrupt service routine
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#pragma ghs interrupt
void r_Config_TAUD0_3_interrupt(void)
{
    /* Start user code for r_Config_TAUD0_3_interrupt. Do not edit comment generated here */

	 timer_1ms_lin_bus_inactive++;
    

    if(time_1ms_init_move_flag == 1)
    {
	    time_1ms_init_move++;
    }

    if(time_1ms_adc_flag == 1)
    {
	    time_1ms_adc++;
    }
    
    if(time_1ms_external_10s_chk_flag == 1)
    {
	    time_1ms_external_10s_chk++;
    }

    if(time_1ms_spi_flag == 1)
    {
	    time_1ms_spi++;
    }

    if(time_1ms_3minute_flag == 1)
    {
	    time_1ms_3minute++;

        if(time_1ms_3minute == 1000)
        {
            time_1s_3minute++;
            time_1ms_3minute = 0;
        }
    }

    if(time_1ms_init_chk_flag == 1)
    {
	    time_1ms_init_chk++;
    }

    if(time_1ms_stall_chk_flag == 1)
    {
	    time_1ms_stall_chk++;
    }

    if(time_1ms_protection_mode_flag == 1)
    {
	    time_1ms_protection_mode++;
    }

    if(stall_chk_cnt == 1)
    {
	    stall_chk_time_1ms++;
    }

    if(time_1ms_motor_wait_flag == 1)
    {
	    time_1ms_motor_wait++;
    }

    if(time_1ms_voltage_chk_delay_flag == 1)
    {
	    time_1ms_voltage_chk_delay++;
    }

    if(time_1ms_diag_auto_flag == 1)
    {
	    time_1ms_diag_auto++;
    }

    if(time_1ms_voltage_status_change_wait_flag == 1)
    {
	    time_1ms_voltage_status_change_wait++;
    }

    if(time_1ms_motor_acceleration_flag == 1)
    {
	    time_1ms_motor_acceleration++;
    }

    if(timer_1ms_init_fail_chk_flag == 1)
    {
	    timer_1ms_init_fail_chk++;
    }

    if(timer_1ms_lin_sleep_flag == 1)
    {
	    timer_1ms_lin_sleep++;
    }

    if(time_1ms_antipinch_flag == 1)
    {
        time_1ms_antipinch++;
    }

    if(time_1ms_spi_error_chk_flag == 1)
    {
        time_1ms_spi_error_chk++;
    }

    if(time_1ms_adc_error_chk_flag == 1)
    {
        time_1ms_adc_error_chk++;
    }

    if(time_1ms_fdl_error_chk_flag == 1)
    {
        time_1ms_fdl_error_chk++;
    }
    if(time_1ms_protection_chk_flag == 1)
    {
        time_1ms_protection_chk++;
    }

    if(time_1ms_Moving_chk_flag==1)
    {
	time_1ms_Moving_chk++;
    }
    if(time_1ms_step_chk_flag == 1)
    {
	time_1ms_step_chk++;
    }
    if(time_1ms_wdg_chk_flag == 1)
    {
	time_1ms_wdg_chk++;
    }
    if(time_1ms_LIN_GndShort_flag == 1)
    {
	time_1ms_LIN_GndShort++;
    }
    if(time_1ms_LIN_BatShort_flag == 1)
    {
	time_1ms_LIN_BatShort++;
    }
    
    if(time_1ms_Error_chk_flag==1)
    {
	time_1ms_Error_chk++;
    }

    if(time_1ms_motor_Short_chk_flag==1)
    {
	time_1ms_motor_Short_chk++;
    }
    if(time_1ms_motor_Open_chk_flag==1)
    {
	time_1ms_motor_Open_chk++;
    }
    if(time_1ms_adc_1s_chk_flag==1)
    {
       time_1ms_adc_1s_chk++;
    }
    if(time_1ms_IGN_chk_flag==1)
    {
       time_1ms_IGN_chk++;
    }

    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
