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
 * File Name        : r_cg_main.c
 * Version          : 1.0.151
 * Device(s)        : R7F701695
 * Description      : This file implements main function.
 ***********************************************************************************************************************/
/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_smc_entry.h"
/* Start user code for include. Do not edit comment generated here */
#include "target.h"
#include "fdl_user.h"
#include "fdl_function.h"
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/***********************************************************************************************************************
 * Section 2: Global Variable Declarations
 * Description : Buffers, Flags, Counters, and State Variables
 ***********************************************************************************************************************/

/* 2.1 Communication Buffers (LIN / SPI) */
static uint8_t GetIDbuffer;
static uint8_t Slave_RxData1[8]; /*reception data store array*/
static uint8_t Slave_TxData[8] = {
    0,
}; /*Transmission data store array*/
static uint8_t ID_chk_rxdata[11] = {
    0,
};
static uint8_t w_buff[18] = {
    0,
};
static uint32_t r_buff[5] = {
    0,
};

static uint8_t Slave_SwData[8] = {
    0,
}; /* Transmission data store array  */

static uint16_t tx_16bit_spi[11] = {
    0x4000, // [0] (R) FAULT Status
    0x4200, // [1] (R) DIAG Status 1
    0x4400, // [2] (R) DIAG Status 2
    0x0691, // [3] ?  瑜섏젣?
    0x080F, // [4] (RW)CTRL 2
    0x0A05, // [5] (RW)CTRL 3
    0x0C3E, // [6] (RW)CTRL 4   3E (open load on) 0C36 0C3A
    0x0E10, // [7] (RW)CTRL 5
    0x1000, // [8] (RW)CTRL 6   stall threshold
    0x5200, // [9] (R) CTRL 7   stall count
    0x5400  // [10](R) CTRL 8
};

static uint16_t tx_16bit_spi_current_limit[16] = {
    0x0601,
    0x0611,
    0x0621,
    0x0631,
    0x0641,
    0x0651,
    0x0661,
    0x0671,
    0x0681,
    0x0691,
    0x06A1,
    0x06B1,
    0x06C1,
    0x06D1,
    0x06E1,
    0x06F1};

static uint16_t rx_16bit_spi_id[11] = {
    0x4000,
    0x4200,
    0x4400,
    0x4600,
    0x4800,
    0x4A00,
    0x4C00,
    0x4E00,
    0x5000,
    0x5200,
    0x5400};

static uint16_t rx_16bit_spi[11] = {
    0,
};
uint16_t fault_clear[1] = {
    0x0CBA};

/* 2.2 Motor Control Variables */
static unsigned int motor_start = OFF;
static unsigned int motor_wait_chk = 0;
static unsigned long long step_position = 0;
static unsigned int dir_state = 0;
static unsigned int step_toggle_flag = 0;
static unsigned int init_move_step = 0;
static unsigned int motor_open_load = 0;
static unsigned int motor_step_value = 0;
static volatile unsigned int softstart_complete = 0;
static unsigned int motor_stall_value = 255;
static unsigned int motor_stall_flag = 0;
static unsigned int motor_cw_stall_value = 0;
static unsigned int motor_ccw_stall_value = 0;
static unsigned int step_start_flag = 0;

/* 2.3 AAF Application Variables */
static unsigned int aaf_step = 0;
static unsigned int aaf_action = 0;
static unsigned int aaf_init_step = 0;
static unsigned int aaf_action_complete_chk = 0;
static unsigned int flap_move = FLAP_STOP;
static unsigned int step_position_open = 0;
static unsigned int step_position_close = 0;
static unsigned int limit_step_position = 0;
static unsigned int open_1st_step_position = 0;
static unsigned int open_2nd_step_position = 0;
static unsigned int protection_function = 0;
static unsigned int protection_Mode_step = 0;
static unsigned int AAF_location_type = 0;
static unsigned int AAF_OverCurrent = 0;
static unsigned int AAF_LINOut = 0;
static unsigned int AAFx_Type = 0;
static unsigned int AAFx_InitStatus = 0;
static unsigned int AAFx_Index = 0;
static unsigned int TotalNumOfAAF = 0;
static unsigned int AAFx_Circuit_Open = 0;
static unsigned int AAFx_Circuit_Short = 0;
static unsigned int AAFx_Motor_Fault = 0;
static unsigned int AAFx_Low_Volt = 0;
static unsigned int AAFx_Over_Volt = 0;

static unsigned int AAF1_TargetPosition = 0;
static unsigned int AAF2_TargetPosition = 0;
static unsigned int AAF3_TargetPosition = 0;
static unsigned int AAF1_TargetPosition_select = 0;
static unsigned int AAF2_TargetPosition_select = 0;
static unsigned int AAF3_TargetPosition_select = 0;

static unsigned int AAF_ProtectionMode_Rx = 0;
static volatile unsigned int AAF_ProtectionMode_Tx = 0;
static unsigned int AAF_Tx_Position = 0;
static unsigned int AAF_Tx_Position_LIN = 0;
static volatile unsigned int AAF_Maximum_Torque_Test_Mode = 0;
static unsigned int torque_test_position = 0;

static unsigned int ReqRespAAFID = 0;
static unsigned int ReqAAF1DiagMode = 0;
static unsigned int ReqAAF2DiagMode = 0;
static unsigned int ReqAAF3DiagMode = 0;
static unsigned int EngRunSta = 0;
static unsigned int HevRdy = 0;
static unsigned int Req_ChkSum_Rx = 0;
static unsigned int Req_Alive_Rx = 0;
static unsigned int AAFx_Mode = 0;

static unsigned int AAFx_SNSR_SCG = 0;
static unsigned int AAFx_SNSR_SCB = 0;
static unsigned int AAFx_SNSR_OC = 0;
static unsigned int AAFx_Position_Status = 0;
static unsigned int AAFx_ErrorStatus = 0;
static unsigned int TotalNumOfAAFSensor = 0;
static unsigned int AAFx_SNSR1_Position = 0;
static unsigned int AAFx_SNSR2_Position = 0;
static unsigned int AAFx_SNSR3_Position = 0;
static unsigned int AAFx_SNSR4_Position = 0;
static unsigned int Req_ChkSum_Tx = 0;
static unsigned int Req_Alive_Tx = 0;

/* 2.4 Communication Flags & Status */
static volatile uint8_t error_status;
static unsigned int lin_aaf_command = 0;
static unsigned int lin_rx_pass_flag = 0;
static unsigned int lin_rx_chk_flag = 0;
static unsigned int AAF_LIN_ChkSum_CHK = 0;
static unsigned int AAF_LIN_ChkSum_CHK_value = 0;
unsigned char spi_send_flag = 0;
unsigned char spi_receive_flag = 0;
unsigned char spi_error_flag = 0;
static unsigned int spi_action_step = 0;
static char ret = 0;
unsigned int timer_1ms_lin_bus_inactive = 0;
static unsigned int lin_bus_inactive_flag = 0;
static unsigned int lin_sleep_step = 0;
unsigned int timer_1ms_lin_sleep = 0;
unsigned int timer_1ms_lin_sleep_flag = 0;
static void LIN_Diag_Rx(void);     /* [cite: 9, 194] */
static void LIN_Diag_Action(void); /*  */
/* 2.5 ADC & Power Variables */
static uint16_t bat_adc = 0;
static unsigned int adc_chk[10] = {
    0,
};
static unsigned int adc_sum = 0;
static unsigned int adc_avr = 0;
static unsigned int adc_chk_ok_flag = 0;
static unsigned int adc_chk_ready = 0;
uint16_t scan_results[3];

static unsigned int voltage_status_spi = 0;
static unsigned int voltage_status_change = 0;
static unsigned int voltage_status_change_complete = 0;
static unsigned int voltage_chk_delay_complete = 0;
static unsigned int Under_Voltage_Deceted = 0U;
static unsigned int Over_Voltage_Deceted = 0U;
static unsigned int First_Powerchk = 0U;

/* 2.6 Fault & Diagnosis */
static unsigned int fail_safety_flag = 0;
static unsigned int fail_safety_1_cycle_flag = 0;
static unsigned int fail_safety_step = 0;
unsigned int stall_chk_cnt = 0;
unsigned int stall_chk_time_1ms = 0;
static unsigned int stall_cnt = STALL_CNT_DEFAULT;
static unsigned int stall_test_mode = 0;
static unsigned int evrdy_on_flag = 0;
static unsigned int diag_mode_auto_dir = 0;
static unsigned int diag_mode_auto_action = 0;
static unsigned int fdl_fail = 0;
static volatile unsigned int wake_up_motor_range_init_chk = 0;
static unsigned int LIMP_HOME_Count = 0;
static unsigned int LIMP_HOME_step = 0;
static uint8_t DTC_Status = 0;
static unsigned int motor_fault_chk = 0U;
static unsigned int power_chk = 0U;
static unsigned int Diag_Mode = 0U;
static unsigned int Diag_Mode_chk = 0U;
unsigned int Short_chk = 0U;
unsigned int LIN_Short_Sleep = 0U;
static unsigned int Short_Detected = 0U;
static unsigned int Open_Detected = 0U;
static unsigned int Short_fault_check = 0U;
static unsigned int Open_fault_check = 0U;
static unsigned int motor_Short_chk_count = 0U;
static unsigned int motor_Open_chk_count = 0U;

/* 2.7 Antipinch */
static unsigned int antipinch_step = 0;
static unsigned int antipinch_previous_action = INITIALIZATION;
static unsigned int antipinch_action_on = 0;

/* 2.8 Flash Memory Variables (Shadow RAM) */
static unsigned int close_memory_write = 0; // close step
static unsigned int close_memory_read = 0;
static unsigned int open_memory_write = 0; // open step
static unsigned int open_memory_read = 0;
static unsigned int now_step_memory_write = 0; // now step
static unsigned int now_step_memory_read = 0;
static unsigned int position_memory_write = 0; // AAFx_Position
static unsigned int position_memory_read = 0;
static unsigned int Initial_memory_write = 0; // evrdy flag
static unsigned int Initial_memory_read = 0;
static unsigned int position_Initial_combined_read = 0; // AAFx_Position+evrdy flag
static unsigned int limit_memory_write = 0; // limit step(5%)
static unsigned int limit_memory_read = 0;
static unsigned int position_status_memory_write = 0; // AAFx_Position_Status
static unsigned int position_status_memory_read = 0;
static unsigned int AAFx_InitStatus_memory_write = 0; // AAFx_InitStatus
static unsigned int AAFx_InitStatus_memory_read = 0;
static unsigned int position_Initstatus_combined_read = 0; // AAFx_Position_Status+AAFx_InitStatus
static unsigned int DTC_memory_write = 0; // DTC
static unsigned int DTC_memory_read = 0;
static unsigned int power_chk_memory_write = 0; // power chk
static unsigned int power_chk_memory_read = 0;
static unsigned int First_Powerchk_memory_write = 0U;
static unsigned int First_Powerchk_memory_read = 0U;

/* 2.9 Timers (1us / 1ms Counters) */
unsigned int time_1us_motor = 0;
unsigned int time_1us_motor_flag = 0;
unsigned int time_1us_spi = 0;
unsigned int time_1us_spi_flag = 0;
unsigned int time_1ms_init_move = 0;
unsigned int time_1ms_init_move_flag = 0;
unsigned int time_1ms_adc = 0;
unsigned int time_1ms_adc_flag = 0;
unsigned int time_1ms_external_10s_chk = 0;
unsigned int time_1ms_external_10s_chk_flag = 0;
unsigned int time_1ms_3minute = 0;
unsigned int time_1s_3minute = 0;
unsigned int time_1ms_3minute_flag = 0;
unsigned int time_1ms_spi = 0;
unsigned int time_1ms_spi_flag = 0;
unsigned int time_1ms_init_chk = 0;
unsigned int time_1ms_init_chk_flag = 0;
unsigned int time_1ms_stall_chk = 0;
unsigned int time_1ms_stall_chk_flag = 0;
unsigned int time_1ms_protection_mode = 0;
unsigned int time_1ms_protection_mode_flag = 0;
unsigned int time_1ms_motor_wait = 0;
unsigned int time_1ms_motor_wait_flag = 0;
unsigned int time_1ms_voltage_chk_delay = 0;
unsigned int time_1ms_volt_chk_dly_flag = 0;
unsigned int time_1ms_diag_auto = 0;
unsigned int time_1ms_diag_auto_flag = 0;
unsigned int time_1ms_volt_stat_chg_wait = 0;
unsigned int time_1ms_volt_stat_chg_wait_flag = 0;
unsigned int time_1ms_antipinch = 0;
unsigned int time_1ms_antipinch_flag = 0;
unsigned int time_1ms_motor_acceleration = 0;
unsigned int time_1ms_motor_acceleration_flag = 0;
unsigned int time_1ms_spi_error_chk = 0;
unsigned int time_1ms_spi_error_chk_flag = 0;
unsigned int time_1ms_adc_error_chk = 0;
unsigned int time_1ms_adc_error_chk_flag = 0;
unsigned int time_1ms_fdl_error_chk = 0;
unsigned int time_1ms_fdl_error_chk_flag = 0;
unsigned int time_1ms_protection_chk = 0;
unsigned int time_1ms_protection_chk_flag = 0;
unsigned int timer_1ms_init_fail_chk = 0;
unsigned int timer_1ms_init_fail_chk_flag = 0;
unsigned int time_1ms_step_chk = 0;
unsigned int time_1ms_step_chk_flag = 0;
static unsigned int step_check_flag = 0;
unsigned int time_1ms_Moving_chk = 0;
unsigned int time_1ms_Moving_chk_flag = 0;
unsigned int time_1ms_wdg_chk = 0U;
unsigned int time_1ms_wdg_chk_flag = 0U;
unsigned int time_1ms_LIN_GndShort_flag = 0U;
unsigned int time_1ms_LIN_GndShort = 0U;
unsigned int time_1ms_LIN_BatShort_flag = 0U;
unsigned int time_1ms_LIN_BatShort = 0U;
unsigned int time_1ms_Error_chk = 0U;
unsigned int time_1ms_Error_chk_flag = 0U;
unsigned int time_1ms_motor_Short_chk = 0U;
unsigned int time_1ms_motor_Short_chk_flag = 0U;
unsigned int time_1ms_motor_Open_chk = 0U;
unsigned int time_1ms_motor_Open_chk_flag = 0U;
unsigned int time_1ms_adc_1s_chk = 0U;
unsigned int time_1ms_adc_1s_chk_flag = 0U;
unsigned int time_1ms_IGN_chk = 0U;
unsigned int time_1ms_IGN_chk_flag = 0U;

//development
static unsigned int SW_Chk = 0U;
static unsigned int Open_Min_Limit = 0U;
static unsigned int Open_Max_Limit = 0U;
static unsigned int Close_Min_Limit = 0U;
static unsigned int Close_Max_Limit = 0U;
static unsigned int test = 0U;
static uint8_t Slave_RxSwData1[8];      /*  */
static unsigned int wakeup_chk = 1;      /*  */
static unsigned int spi_fail = 0;        /*  */
static unsigned int LIN_Short_Ok = 0U;   /*  */

/***********************************************************************************************************************
 * Section 3: Function Prototypes
 * Description : Declaration of Internal and External Functions
 ***********************************************************************************************************************/

/* 3.1 Driver / Hardware Control Functions */
static void DRV8899_Init(void);
static void DRV8899_Wakeup(void);
static void DRV8899_Sleep(void);
static void DRV_On(void);
static void DRV_Off(void);
static void Motor_Open(void);
static void Motor_Close(void);
static void Motor_Action(void);
static void Motor_SPI_Init(void);
static void SPI_select_pin_Off(void);
static void VDC_adc(void);
static void Flash_memory_write(void);
static void Flash_memory_read(void);
static void IGN_On_Memory_read(void);

/* 3.2 Communication Functions (LIN / SPI) */
void RLIN3_slave_init(void);
static void Lin_Transceiver_On(void);
static void Lin_Transceiver_Off(void);
void Clear_DataBuffer(void);
void RLIN_Slave_Receive(uint8_t Data_length);
void RLIN_Slave_Transmit(uint8_t *databuf, uint8_t Data_length);
void Get_reponse_RxData(uint8_t *RxData);
void RLIN_Slave_NoResponse(void);
static void Lin_rx_data_chk(void); // V
static void Lin_tx_data_chk(void); // V
static void SPI_chk(void);
static void Lin_bus_chk(void);
static void Lin_sleep(void);      // V

/* 3.3 Application Logic Functions */
static void Operating_mode(void);
static void MCU_sleep(void);       // V
static void Stall_chk(void);
static void CHK_external_factors(void);
static void Fail_safety_mode(void);
static void Protection_Mode(void);
static void Voltage_chk_current_limit_init(void);
static void Current_limiting_select(void);
static void Antipinch_move(void); // V
static void Torque_Test_Mode(void);
static void LIMP_HOME(void);          // V
static void Init_move(void);
static void step_check(void);         // V
static void Re_Init(void);            // V
static void LIN_Short_Chk(void);
static void Error_FaultClear(void);
static void ERROR_chk(void);
static void Tx_position_complete_chk(void);

/* 3.4 AAF Application Functions */
static void AAF_Init(void);
static void AAF_communicate_chk(void);
static void AAF_mode_chk(void);
static void AAF_App(void);

/* 3.5 Interrupt Service Routines */
void RLIN30_interrupt(void);
void RLIN30_transmit_interrupt(void);
void RLIN30_receive_complete_interrupt(void);
void RLIN30_status_interrupt(void);

/* End user code. Do not edit comment generated here */
void r_main_userinit(void);

/***********************************************************************************************************************
 * Function Name: main
 * Description  : This function implements main function.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
int main(void)
{
	r_main_userinit();
	/* Start user code for main. Do not edit comment generated here */

	R_Config_TAUD0_3_Start();
	R_Config_TAUD0_13_Start();
	R_Config_ADCA0_ScanGroup1_OperationOn();
	R_Config_CSIH0_Start();
	RLIN3_slave_init();
	R_Config_WDT0_Create();
	R_Config_WDT0_Restart();

	protected_write(WPROTR.PROTCMD0, WPROTR.PROTS0, STBC_IOHOLD.IOHOLD, 0x00u);
	while (STBC_IOHOLD.IOHOLD != 0x00u)
	{
	}

	// flash memory setup

	FDL_Open();
	ret = function_FDL_init();

	time_1ms_fdl_error_chk_flag = 1;
	if (ret < 0) // error
	{
		while (1)
		{
			if (time_1ms_fdl_error_chk >= 100U)
			{
				fdl_fail += 1;
				break;
			}
		}
	}

	time_1ms_fdl_error_chk_flag = 0;
	time_1ms_fdl_error_chk = 0;

	Lin_Transceiver_On();
	DRV8899_Init();

	AAFx_Type = EXTERNAL_TYPE; // NX5e AAF1, 2 EXTERNAL_TYPE

	// AAFx_Type = INTERNAL_TYPE; //  NX5e AAF3 INTERNAL_TYPE(LOWER)

	// AAFx_Index = AAF_1;			 //
	// AAF_location_type = LH_TYPE; // CW

	AAFx_Index = AAF_2;			 //
	AAF_location_type = RH_TYPE; // CCW

	// AAFx_Index = AAF_3;			 //
	// AAF_location_type = LH_TYPE; // CW

	TotalNumOfAAF = AAFx3; // aaf 3 ea

	TotalNumOfAAFSensor = 0;

	aaf_step = AAF_INITIALIZATION;
	aaf_init_step = WAIT_INITIALIZATION;
	AAF_Tx_Position = UNKOWN_POSITION;
	AAFx_Position_Status = Unknown_Status;
	AAFx_InitStatus = DURING_INITIALIZATION;
	time_1ms_spi_flag = 1; // spi timer on
	while (1)			   // 1ms wait (spi error prevention)
	{
		if (time_1ms_spi >= 1U)
		{
			break;
		}
	}
	Motor_SPI_Init();

	motor_stall_value = (unsigned int)(rx_16bit_spi[9] & 0xFFU);
	time_1ms_volt_chk_dly_flag = 1; // POWER ON AFTER 500ms

	motor_cw_stall_value = MOTOR_CW_STALL_CHK_VALUE_NORMAL_VOLTAGE;
	motor_ccw_stall_value = MOTOR_CCW_STALL_CHK_VALUE_NORMAL_VOLTAGE;

	diag_mode_auto_dir = OPEN;
	time_1ms_protection_chk_flag = 1;

	voltage_status_change_complete = COMPLETE;

	while (1)
	{
		if (time_1ms_protection_chk >= 550) // POWER ON AFTER 550ms NOT OP
		{
			time_1ms_protection_chk_flag = 0;
			time_1ms_protection_chk = 550;
		}
		// -------------- stop mode
		if (time_1ms_protection_chk >= 50)
		{
			Motor_Action();
		}
		if (((EngRunSta == 0x01) || (HevRdy == 0x01)) && (time_1ms_IGN_chk <= 500))
		{
			time_1ms_IGN_chk_flag = 1;
			if (time_1ms_IGN_chk >= 500)
			{
				time_1ms_IGN_chk_flag = 0;
				time_1ms_IGN_chk = 500;
				ERROR_chk();
			}
		}

		VDC_adc();

		if ((protection_function == OFF) && (lin_bus_inactive_flag == OFF) && (AAF_Maximum_Torque_Test_Mode == OFF) && (time_1ms_protection_chk == 550))
		{
			Operating_mode();
		}
		else if (AAF_Maximum_Torque_Test_Mode == ON) //-----------------------------------------development----------------------------------------
		{
			Torque_Test_Mode();
		}
		else if (lin_bus_inactive_flag == ON)
		{
			Lin_sleep();
		}
		else
		{
		}
		Lin_rx_data_chk();
		Lin_tx_data_chk();
		SPI_chk();
		CHK_external_factors();

		if ((protection_function == OFF) && (lin_bus_inactive_flag == OFF) && (AAF_Maximum_Torque_Test_Mode == OFF))
		{
			Fail_safety_mode();
		}

		if ((antipinch_previous_action == OPEN) || (antipinch_previous_action == CLOSE))
		{
			Antipinch_move();
		}

		Lin_bus_chk();

		if ((protection_function == ON) && (lin_bus_inactive_flag == OFF) && (AAF_Maximum_Torque_Test_Mode == OFF))
		{
			Protection_Mode();
		}
		if ((step_check_flag == 0) && ((EngRunSta == 0x01) || (HevRdy == 0x01))) // IGN > POSITION CHECK
		{
			Flash_memory_read();
			IGN_On_Memory_read();

			time_1ms_step_chk_flag = 1;
			if ((time_1ms_step_chk >= 50))
			{
				step_check();
				time_1ms_step_chk_flag = 0;
				time_1ms_step_chk = 50;
				step_check_flag = 1;
			}
			if ((step_check_flag == 1))
			{
				power_chk = Shutdown_Check;
				Flash_memory_write();
			}
		}
		R_Config_WDT0_Restart(); // wdt0 feed
	}

	/* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
 * Function Name: r_main_userinit
 * Description  : This function adds user code before implementing main function.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void r_main_userinit(void)
{
	DI();
	/* Start user code for r_main_userinit. Do not edit comment generated here */
	/* End user code. Do not edit comment generated here */
	R_Systeminit();
	EI();
}

/* Start user code for adding. Do not edit comment generated here */
/***********************************************************************************************************************
 * Function Name: Clear_DataBuffer
 * Description  : This function setting all data buffer to some value
 * Arguments    : uint8_t x : setting data buff value
 * Return Value : None
 ***********************************************************************************************************************/
void Clear_DataBuffer(void)
{
	uint8_t i;
	// uint32_t Databuf_adr;
	uint8_t *Databuf_adr;

	// Databuf_adr = &RLN30.LDBR1;
	Databuf_adr = (uint8_t *)&RLN30.LDBR1;
	for (i = 0; i < 8; i++)
	{
		*((uint8_t *)(Databuf_adr + i)) = 0U;
	}
}

/***********************************************************************************************************************
 * Function Name: RLIN_Slave_Receive(void)
 * Description  : This function clear data buffer for response reception start
 * Arguments    : uint8_t Data_length : receive data length.
 * Return Value : None
 ***********************************************************************************************************************/
void RLIN_Slave_Receive(uint8_t Data_length)
{
	Clear_DataBuffer();
	// RLN30.LDFC = 0x20;		   /*b5=1:enhanced checksum mode; b4=0:Reception*/
	if (GetIDbuffer == 0x3C) // 0x3C / 0x3D
	{
		RLN30.LDFC = 0x00; // b5=0 → Classic checksum
	}
	else
	{
		RLN30.LDFC = 0x20; // b5=1 → Enhanced checksum
	}
	RLN30.LDFC |= Data_length; /* b4-b0=Data_length: response data lengh select byte*/
	RLN30.LTRC = 0x02;		   /*setting RTS=1,response reception is started*/
}

/***********************************************************************************************************************
* Function Name: RLIN_Slave_Transmit(void)
* Description  : This function seting data buffer for response transmission start
* Arguments    : uint8_t* databuf    : variable array data.
				 uint8_t Data_length : transmit data length.
* Return Value : None
***********************************************************************************************************************/
void RLIN_Slave_Transmit(uint8_t *databuf, uint8_t Data_length)
{
	uint8_t i;
	// uint32_t  Databuf_adr;
	uint8_t *Databuf_adr;

	// RLN30.LDFC = 0x30;		   /*b5=1:enhanced checksum mode; b4=1:transmission*/
	if ((GetIDbuffer == 0x7D)) // 0x3C / 0x3D
	{
		RLN30.LDFC = 0x10; // b5=0 → Classic checksum
	}
	else
	{
		RLN30.LDFC = 0x30; // b5=1 → Enhanced checksum
	}
	RLN30.LDFC |= Data_length; /* b4-b0=Data_length: response data lengh select byte*/
	// Databuf_adr = &RLN30.LDBR1;    /* get the data buffer address*/
	Databuf_adr = (uint8_t *)&RLN30.LDBR1; /* get the data buffer address*/

	for (i = 0; i < Data_length; i++) /* setting tansmission data to date buffer*/
	{
		*((uint8_t *)(Databuf_adr + i)) = databuf[i];
	}

	RLN30.LTRC = 0x02; /*setting RTS=1;Response transmission start*/
}

/***********************************************************************************************************************
 * Function Name: Get_reponse_RxData
 * Description  : This function get data buffer value to a variable array
 * Arguments    : uint8_t * RxData : a avriable array for store Data
 * Return Value : None
 ***********************************************************************************************************************/
void Get_reponse_RxData(uint8_t *RxData)
{
	uint8_t i, k;
	// uint32_t Databuf_adr;
	uint8_t *Databuf_adr;

	k = RLN30.LDFC & 0x0F;
	// Databuf_adr = &RLN30.LDBR1;
	Databuf_adr = (uint8_t *)&RLN30.LDBR1;
	for (i = 0; i < k; i++)
	{
		RxData[i] = (*((uint8_t *)(Databuf_adr + i)));
	}
}

/***********************************************************************************************************************
 * Function Name: RLIN_Slave_Init(void)
 * Description  : This function initializes the RLIN Slave node, setting clock supply,baud rate,ect.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void RLIN3_slave_init(void)
{
	R_PORT_SetAltFunc(Port10, 10, Alt2, Output);
	R_PORT_SetAltFunc(Port10, 9, Alt2, Input);

	do
	{
		/*LIN control register*/
		RLN30.LCUC = 0x00; // LIN reset mode
	} while (RLN30.LMST != 0u); // LIN mode status register

	// Sets a baud rate
	RLN30.LWBR = 0x34; // 4sampling_1/4 Prescaler
	// RLN30.LWBR = 0x00; //jdy_test
	RLN30.LBRP01.UINT16 = 0x0081; /*  0X81=129D, Baud rate= 40M / 4 / (129+1) / 4 = 19230 bps*/
	// Sets noise filter ON/OFF
	RLN30.LMD = 0x12; // The noise filter is enabled; transmission interrupt, reception interrupt, status interrupt are used ;LIN Slave mode (auto baud rate)
					  // Enables interrupt

#if 0 // for RLN30.LMD LIOS bit set 0
	  // RLIN30 interrupt
	INTC2.ICRLIN30.BIT.MKRLIN30 = _INT_PROCESSING_DISABLED;
	INTC2.ICRLIN30.BIT.RFRLIN30 = _INT_REQUEST_NOT_OCCUR;
	INTC2.ICRLIN30.BIT.TBRLIN30 = _INT_TABLE_VECTOR;
	INTC2.ICRLIN30.UINT16 &= _INT_PRIORITY_LOWEST;
#else // for RLN30.LMD LIOS bit set 1
	  // RLIN30 transmit interrupt
	INTC2.ICRLIN30UR0.BIT.MKRLIN30UR0 = _INT_PROCESSING_DISABLED;
	INTC2.ICRLIN30UR0.BIT.RFRLIN30UR0 = _INT_REQUEST_NOT_OCCUR;
	INTC2.ICRLIN30UR0.BIT.TBRLIN30UR0 = _INT_TABLE_VECTOR;
	INTC2.ICRLIN30UR0.UINT16 &= _INT_PRIORITY_LOWEST;

	// RLIN30 receive complete interrupt
	INTC2.ICRLIN30UR1.BIT.MKRLIN30UR1 = _INT_PROCESSING_DISABLED;
	INTC2.ICRLIN30UR1.BIT.RFRLIN30UR1 = _INT_REQUEST_NOT_OCCUR;
	INTC2.ICRLIN30UR1.BIT.TBRLIN30UR1 = _INT_TABLE_VECTOR;
	INTC2.ICRLIN30UR1.UINT16 &= _INT_PRIORITY_LOWEST;

	// RLIN30 status interrupt
	INTC2.ICRLIN30UR2.BIT.MKRLIN30UR2 = _INT_PROCESSING_DISABLED;
	INTC2.ICRLIN30UR2.BIT.RFRLIN30UR2 = _INT_REQUEST_NOT_OCCUR;
	INTC2.ICRLIN30UR2.BIT.TBRLIN30UR2 = _INT_TABLE_VECTOR;
	INTC2.ICRLIN30UR2.UINT16 &= _INT_PRIORITY_LOWEST;

	RLN30.LIE = 0x0f; // enable all interrupt
	// Enables error detection
	RLN30.LEDE = 0xdd; // enable all error

	// Sets frame configuration parameters
	RLN30.LBFC = 0x00; // Reception break of 9.5/10 or more Tbits
	RLN30.LSC = 0x11;  // inter-byte space 1bit;Response space 1bit
	RLN30.LWUP = 0x30; // Wake-up Transmission Low level Width Select 4 Tbits
	RLN30.LIDB = 0x00; // Clear the ID buffer
#endif

	// Transitions to the LIN slave mode: LIN operation mode
	do
	{
		RLN30.LCUC = 0x03u; // LIN Opeation mode, Exit LIN reset mode.
	} while (RLN30.LMST != 0x03u);

	// Sets the FTS bit in the RLN3nLTRC register to 1 (header reception or wake-up transmission/reception started)
	RLN30.LTRC = 0x01;

	// Interrupt processing enable
	INTC2.ICRLIN30UR0.BIT.RFRLIN30UR0 = _INT_REQUEST_NOT_OCCUR;
	INTC2.ICRLIN30UR0.BIT.MKRLIN30UR0 = _INT_PROCESSING_ENABLED;
	INTC2.ICRLIN30UR1.BIT.RFRLIN30UR1 = _INT_REQUEST_NOT_OCCUR;
	INTC2.ICRLIN30UR1.BIT.MKRLIN30UR1 = _INT_PROCESSING_ENABLED;
	INTC2.ICRLIN30UR2.BIT.RFRLIN30UR2 = _INT_REQUEST_NOT_OCCUR;
	INTC2.ICRLIN30UR2.BIT.MKRLIN30UR2 = _INT_PROCESSING_ENABLED;
}

/***********************************************************************************************************************
 * Function Name: RLIN_Slave_NoResponse(void)
 * Description  : This function perform no-response for the reception ID
 * Arguments    : uint8_t Data_length : receive data length.
 * Return Value : None
 ***********************************************************************************************************************/
static void RLIN_Slave_NoResponse(void)
{
	RLN30.LTRC = 0x04; /* setting LNRR=1, No response request*/
}

/***********************************************************************************************************************
 * Function Name: r_Config_TAUB0_0_interrupt
 * Description  : This function is TAUB00 interrupt service routine
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
#pragma ghs interrupt
void RLIN30_interrupt(void)
{
}

/***********************************************************************************************************************
 * Function Name: r_Config_TAUB0_0_interrupt
 * Description  : This function is TAUB00 interrupt service routine
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
#pragma ghs interrupt
void RLIN30_transmit_interrupt(void)
{
	RLN30.LST &= 0xFE;
}

/***********************************************************************************************************************
 * Function Name: Wakeup_from_lin_sleep
 * Description  : Wake up in sleep mode when LIN communication is detected and initialize the associated variables
 * Called By    : Handle_lin_header_received
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Wakeup_from_lin_sleep(void)
{
    if (lin_bus_inactive_flag == ON)
    {
        lin_bus_inactive_flag = OFF;
        lin_sleep_step = 0;
        timer_1ms_lin_sleep_flag = 0;
        timer_1ms_lin_sleep = 0;

        DRV_Off();
        motor_start = OFF;
        stall_chk_cnt = 0;
        stall_chk_time_1ms = 0;
        timer_1ms_init_fail_chk_flag = 0;
        timer_1ms_init_fail_chk = 0;

        if (AAFx_Position_Status == FlapMoving_Status)
        {
            aaf_step = AAF_OPERATE;
        }
    }
}

/***********************************************************************************************************************
 * Function Name: Handle_lin_header_received
 * Description  : Check ID on LIN header reception and set send/receive mode
 * Called By    : RLIN30_receive_complete_interrupt
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Handle_lin_header_received(void)
{
    RLN30.LST = 0x00U; // Clear Status
    GetIDbuffer = RLN30.LIDB; // Get ID

	if (GetIDbuffer == 0x25U)
	{
		RLIN_Slave_Receive(6); // 0x25
	}
	else if ((GetIDbuffer == 0xA6U) && (AAFx_Index == ReqRespAAFID))
	{
		RLIN_Slave_Transmit(Slave_TxData, 7); // 0x26
	}
	else if (GetIDbuffer == 0x3CU)
	{
		RLIN_Slave_Receive(8); // 0x3C
	}
	else if ((GetIDbuffer == 0x7DU) && ((SW_Chk == 1U) || (SW_Chk == 3U)))
	{
		RLIN_Slave_Transmit(Slave_SwData, 8); // 0x3D
	}
	else
	{
		RLIN_Slave_NoResponse();
	}

    Wakeup_from_lin_sleep();
}

/***********************************************************************************************************************
 * Function Name: Handle_lin_response_received
 * Description  : LIN data reception completion flag processing and receiving buffer read
 * Called By    : RLIN30_receive_complete_interrupt
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Handle_lin_response_received(void)
{
    RLN30.LST &= 0xFD; // Clear successful response reception flag

	switch (GetIDbuffer)
		{
	case 0x25:
		Get_reponse_RxData(Slave_RxData1);
		lin_rx_chk_flag = ON;
		break;
	case 0x3C:
		Get_reponse_RxData(Slave_RxSwData1);
		break;
	default:
		break;
	}
}

/***********************************************************************************************************************
 * Function Name: Calculate_and_verify_checksum
 * Description  : Manage LIMP HOME counts by calculating and validating the checksum of received data
 * Called By    : RLIN30_receive_complete_interrupt
 * Arguments    : is_response_received - Response received flag (0: Not received, 2: Received)
 * Return Value : void
 ***********************************************************************************************************************/
static void Calculate_and_verify_checksum(uint8_t is_response_received)
{
    unsigned int sum_val = 0U;

    ReqRespAAFID = WAIT;
    
    Req_ChkSum_Rx = (unsigned int)((Slave_RxData1[5] & 0xF0U) >> 4U);
    Req_Alive_Rx = (unsigned int)(Slave_RxData1[5] & 0x0FU);
    Req_Alive_Tx = Req_Alive_Rx;

    AAF_LIN_ChkSum_CHK = WAIT;

    // Checksum Calculation Logic
    sum_val = (unsigned int)((Slave_RxData1[0] >> 4U) + (Slave_RxData1[0] & 0x0FU) +
                             (Slave_RxData1[1] >> 4U) + (Slave_RxData1[1] & 0x0FU) +
                             (Slave_RxData1[2] >> 4U) + (Slave_RxData1[2] & 0x0FU) +
                             (Slave_RxData1[3] >> 4U) + (Slave_RxData1[3] & 0x0FU) +
                             (Slave_RxData1[4] >> 4U) + (Slave_RxData1[4] & 0x0FU) +
                             Req_Alive_Rx);


    AAF_LIN_ChkSum_CHK_value = (unsigned int)((16U - (sum_val & 0x0FU)) & 0x0FU);

    if ((is_response_received == 0x02U) && (AAF_LIN_ChkSum_CHK_value == Req_ChkSum_Rx))
    {
        AAF_LIN_ChkSum_CHK = PASS;

        if (LIMP_HOME_Count >= 4U) LIMP_HOME_Count -= 4U; 
        else                       LIMP_HOME_Count = 0U;  
    }
    else if ((is_response_received == 0x02U) && (AAF_LIN_ChkSum_CHK_value != Req_ChkSum_Rx))
    {
        AAF_LIN_ChkSum_CHK = FAIL;

        if (LIMP_HOME_Count <= 158U) LIMP_HOME_Count += 2U; 
        else                         LIMP_HOME_Count = 160U; 
    }
    else
    {
        // Waiting
    }
}


/***********************************************************************************************************************
 * Function Name: RLIN30_receive_complete_interrupt
 * Description  : RLIN30 Received Interrupt Handler (Header/Response Processing)
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
#pragma ghs interrupt
void RLIN30_receive_complete_interrupt(void)
{
    uint8_t receive_header_flag;
    uint8_t receive_response_flag; // Typo Fixed: reponse -> response

    // 1. Get Status Flags
    receive_header_flag = (uint8_t)(RLN30.LST & 0x80U);  /* 1: Header transmission completed */
    receive_response_flag = (uint8_t)(RLN30.LST & 0x02U); /* 1: Frame/Wake-up reception completed */

    // 2. Handle Header Reception
    if (receive_header_flag != 0U)
    {
        Handle_lin_header_received();
    }

    // 3. Handle Response Reception
    if (receive_response_flag != 0U)
    {
        Handle_lin_response_received();
    }

    // 4. Calculate Checksum & Update Status (Always executed in original logic)
    Calculate_and_verify_checksum(receive_response_flag);

    // 5. Reset Timer & Update Hardware Status
    timer_1ms_lin_bus_inactive = 0U; // Reset LIN timeout timer
    RLN30.LTRC = 0x01U; // Set FTS bit (Ready for next frame)
}

/***********************************************************************************************************************
 * Function Name: r_Config_TAUB0_0_interrupt
 * Description  : This function is TAUB00 interrupt service routine
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
#pragma ghs interrupt
void RLIN30_status_interrupt(void)
{
	NOP();
	error_status = RLN30.LEST; // check status
	if (time_1ms_IGN_chk >= 500)
	{
		if (LIMP_HOME_Count <= 160)
		{
			LIMP_HOME_Count += 2;
		}
	}

#if 1 // LEST Clear
	while (1)
	{
		if ((RLN30.LTRC & 0x01) == 1)
		{
			RLN30.LEST = 0;
			NOP();
			NOP();
			NOP();
			NOP();
			NOP();
			NOP();
			NOP();
			NOP();
			NOP();
			NOP();
			NOP();
			NOP();
			NOP();
			NOP();
			NOP();
			NOP();
			NOP();
			NOP();
			NOP();
			NOP();
			NOP();
			error_status = RLN30.LEST;
			break;
		}
	}
#endif
}

static void DRV8899_Init(void)
{
	PORT.P10 |= _PORT_Pn0_OUTPUT_HIGH;	// MCU_DRVOFF
	PORT.P10 |= _PORT_Pn2_OUTPUT_HIGH;	// MCU_DRV_SLEEP_MODE
	PORT.P9 |= _PORT_Pn1_OUTPUT_HIGH;	// VREF
	PORT.P10 &= ~_PORT_Pn4_OUTPUT_HIGH; // MCU_DIR
	PORT.P9 &= ~_PORT_Pn0_OUTPUT_HIGH;	// MCU_STEP
}

static void DRV8899_Wakeup(void)
{
	PORT.P10 |= _PORT_Pn0_OUTPUT_HIGH; // MCU_DRVOFF
	PORT.P10 |= _PORT_Pn2_OUTPUT_HIGH; // MCU_DRV_SLEEP_MODE
	PORT.P9 |= _PORT_Pn1_OUTPUT_HIGH;  // VREF
}

static void DRV8899_Sleep(void)
{
	PORT.P10 |= _PORT_Pn0_OUTPUT_HIGH;	// MCU_DRVOFF
	PORT.P10 &= ~_PORT_Pn2_OUTPUT_HIGH; // MCU_DRV_SLEEP_MODE
	PORT.P9 &= ~_PORT_Pn1_OUTPUT_HIGH;	// VREF
	PORT.P10 &= ~_PORT_Pn4_OUTPUT_HIGH; // MCU_DIR
	PORT.P9 &= ~_PORT_Pn0_OUTPUT_HIGH;	// MCU_STEP
}

static void DRV_On(void)
{
	PORT.P10 &= ~_PORT_Pn0_OUTPUT_HIGH; // MCU_DRVOFF
}

static void DRV_Off(void)
{
	PORT.P10 |= _PORT_Pn0_OUTPUT_HIGH; // MCU_DRVOFF
	stall_chk_cnt = 0;
	stall_chk_time_1ms = 0;
}

static void DRV8899_On(void)
{
	PORT.P10 &= ~_PORT_Pn0_OUTPUT_HIGH; // MCU_DRVON
}

static void DRV8899_Off(void)
{
	PORT.P10 |= _PORT_Pn0_OUTPUT_HIGH; // MCU_DRVOFF

	stall_chk_cnt = 0;
	stall_chk_time_1ms = 0;
}


static void SPI_select_pin_Off(void)
{
	PORT.P8 &= ~_PORT_Pn0_OUTPUT_HIGH;
}

static void Lin_Transceiver_On(void)
{
	PORT.P10 |= _PORT_Pn3_OUTPUT_HIGH; // MCU_LIN_SLEEP_MODE
}

static void Lin_Transceiver_Off(void)
{
	PORT.P10 &= ~_PORT_Pn3_OUTPUT_HIGH; // MCU_LIN_SLEEP_MODE
}

static void Motor_Open(void)
{
	if (AAF_location_type == RH_TYPE)
	{
		PORT.P10 &= ~_PORT_Pn4_OUTPUT_HIGH; // CCW
		dir_state = OPEN;
	}
	else if (AAF_location_type == LH_TYPE)
	{
		PORT.P10 |= _PORT_Pn4_OUTPUT_HIGH; // CW
		dir_state = OPEN;
	}
	else
	{
	}
}

static void Motor_dir_open(void)
{
	if (AAF_location_type == RH_TYPE)
	{
		PORT.P10 &= ~_PORT_Pn4_OUTPUT_HIGH; // CCW
		dir_state = OPEN;
	}
	else if (AAF_location_type == LH_TYPE)
	{
		PORT.P10 |= _PORT_Pn4_OUTPUT_HIGH; // CW
		dir_state = OPEN;
	}
	else
	{
		//invalid
	}
}

static void Motor_Close(void)
{
	if (AAF_location_type == RH_TYPE)
	{
		PORT.P10 |= _PORT_Pn4_OUTPUT_HIGH; // MCU_DIR
		dir_state = CLOSE;
	}
	else if (AAF_location_type == LH_TYPE)
	{
		PORT.P10 &= ~_PORT_Pn4_OUTPUT_HIGH; // MCU_DIR
		dir_state = CLOSE;
	}
	else
	{
	}
}

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> bdd6f71 (Init move)
static void Motor_dir_close(void)
{
	if (AAF_location_type == RH_TYPE)
	{
		PORT.P10 |= _PORT_Pn4_OUTPUT_HIGH; // MCU_DIR
		dir_state = CLOSE;
	}
	else if (AAF_location_type == LH_TYPE)
	{
		PORT.P10 &= ~_PORT_Pn4_OUTPUT_HIGH; // MCU_DIR
		dir_state = CLOSE;
	}
	else
	{
		//invalid
	}
}

<<<<<<< HEAD
=======
>>>>>>> d4c07df (Motor Action)
=======
>>>>>>> bdd6f71 (Init move)
/***********************************************************************************************************************
 * Function Name: Manage_motor_start_status
 * Description  : Manages flags and timers based on the motor start state (ON/OFF).
 * Called By    : Motor_Action
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Manage_motor_start_status(void)
{
    if (motor_start == ON)
    {
        time_1ms_motor_wait_flag = 1;
    }
    else if (motor_start == OFF)
    {
        time_1ms_motor_wait_flag = 0;
        time_1ms_motor_wait = 0;
        time_1us_motor_flag = 0;
        time_1us_motor = 0;
        time_1ms_motor_acceleration_flag = 0;
        time_1ms_motor_acceleration = 0;
    }
    else
    {
        // Invalid
    }
}

/***********************************************************************************************************************
 * Function Name: Execute_legacy_soft_start
 * Description  : Handles the legacy detailed soft-start acceleration logic (Previously commented out).
 * Called By    : Process_active_acceleration (Currently commented out)
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Execute_legacy_soft_start(void)
{
    // if (((motor_step_value <= STEP_TIME_1000RPM) && (motor_step_value > STEP_TIME_1250RPM)) && (time_1ms_motor_acceleration >= 4))
    // {
    //     motor_step_value--;
    //     time_1ms_motor_acceleration = 0;
    // }
    // else if (((motor_step_value <= STEP_TIME_1250RPM) && (motor_step_value > STEP_TIME_1500RPM)) && (time_1ms_motor_acceleration >= 6))
    // {
    //     motor_step_value--;
    //     time_1ms_motor_acceleration = 0;
    // }
    // else if (((motor_step_value <= STEP_TIME_1500RPM) && (motor_step_value > STEP_TIME_1575RPM)) && (time_1ms_motor_acceleration >= 8))
    // {
    //     motor_step_value--;
    //     time_1ms_motor_acceleration = 0;
    // }
    // else if (motor_step_value <= STEP_TIME_1575RPM)
    // {
    //     motor_step_value = STEP_TIME_1575RPM;
    //     softstart_complete = ON;
    // }
    // else
    // {
    //     // Invalid
    // }
}

/***********************************************************************************************************************
 * Function Name: Process_active_acceleration
 * Description  : Handles the currently active motor acceleration logic (1000RPM -> 1250RPM).
 * Called By    : Motor_Action
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Process_active_acceleration(void)
{
    // Development START
    if ((AAF_Tx_Position != UNKOWN_POSITION) && (Diag_Mode == 0)) 
    {
        AAFx_Position_Status = FlapMoving_Status;
    }
    else if (AAF_Tx_Position == UNKOWN_POSITION)
    {
        AAFx_Position_Status = Unknown_Status;
    }
    else
    {
        // Invalid
    }
    // Development END

    motor_wait_chk = ON;
    stall_chk_cnt = ON;
    time_1ms_motor_acceleration_flag = 1;

    Execute_legacy_soft_start(); //not used

    // Active acceleration logic
    if (((motor_step_value <= STEP_TIME_1000RPM) && (motor_step_value > STEP_TIME_1250RPM)) && (time_1ms_motor_acceleration >= 10))
    {
        motor_step_value--;
        time_1ms_motor_acceleration = 0;
    }
    else
    {
        motor_step_value = STEP_TIME_1250RPM;
        time_1ms_motor_acceleration = 0;
        softstart_complete = ON;
    }

    time_1ms_motor_wait = MOTOR_WAIT_TIME;
}

/***********************************************************************************************************************
 * Function Name: Generate_step_pulse
 * Description  : Toggles the GPIO pin to generate motor steps and updates the position counter.
 * Called By    : Motor_Action
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Generate_step_pulse(void)
{
    if ((motor_wait_chk == ON) && (voltage_status_change_complete == COMPLETE))
    {
        time_1us_motor_flag = 1;

        if ((time_1us_motor >= motor_step_value * 2U) && (step_toggle_flag == 1U))
        {
            PORT.P9 &= ~_PORT_Pn0_OUTPUT_HIGH;
            step_toggle_flag = 0;
            time_1us_motor = 0;
        }
        else if ((time_1us_motor >= motor_step_value) && (step_toggle_flag == 0U))
        {
            PORT.P9 |= _PORT_Pn0_OUTPUT_HIGH;
            step_toggle_flag = 1;

            if (dir_state == OPEN)
            {
                step_position--;
            }
            else if (dir_state == CLOSE)
            {
                step_position++;
            }
            else
            {
                // Invalid
            }
        }
        else
        {
            // Invalid
        }
    }
}

/***********************************************************************************************************************
 * Function Name: Motor_Action
 * Description  : Main function for motor control (State, Acceleration, Step Generation).
 * Called By    : Main Loop : AAF_App()
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Motor_Action(void)
{
    // 1. Manage start/stop flags
    Manage_motor_start_status();
<<<<<<< HEAD

    // 2. Handle wait time and acceleration
    if (time_1ms_motor_wait >= MOTOR_WAIT_TIME)
    {
        Process_active_acceleration();
    }
    else 
    {
        motor_wait_chk = OFF;
    }

    // 3. Generate step pulses
    Generate_step_pulse();
}

/***********************************************************************************************************************
 * Function Name: Start_motor_move_init
 * Description  : Start the motor drive, initialize the relevant variables and move on to the next step
 * Called By    : Process_init_step_0_to_9
 * Arguments    : next_step - Next Case
 * dir       - Motor Drive Direction (OPEN / CLOSE)
 * is_case0  - first entry (Case 0) or not (TRUE: Perform additional initialization / FALSE: not)
 * Return Value : void
 ***********************************************************************************************************************/
static void Start_motor_move_init(uint8_t next_step, uint8_t dir, uint8_t is_case0)
{
    if (dir == OPEN) Motor_dir_open();
    else             Motor_dir_close();

    DRV8899_On();
    motor_start = ON;
    
    // (Case 0, 6, 9)
    motor_stall_flag = MOTOR_NORMAL;
    stall_chk_time_1ms = 0;
    motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE;
    time_1ms_spi = 0;
    AAF_Tx_Position = UNKOWN_POSITION;
    AAFx_Position_Status = Unknown_Status;
    antipinch_previous_action = INITIALIZATION;
    time_1ms_init_chk = 0;
    time_1ms_init_chk_flag = 1; 

    // (Case 0)
    if (is_case0 == TRUE)
    {
        time_1ms_external_10s_chk_flag = OFF;
        time_1ms_external_10s_chk = 0;
        step_position = REFERENCE_POSITION;
    }

    init_move_step = next_step;
}


/***********************************************************************************************************************
 * Function Name: Check_stall_and_move
 * Called By: Init_move (Case 4, 7, 10)
 * next_step: next case
 * retry_step: Steps to move in case of failure (timeout)
 * dir: OPEN, CLOSE DIRECTION
 ***********************************************************************************************************************/
static void Check_stall_and_move(uint8_t next_step, uint8_t retry_step, uint8_t dir)
{
    if ((motor_stall_flag == MOTOR_STALL) || (time_1ms_init_chk >= 4500U))
    {
    
        DRV8899_Off();
        motor_start = OFF;
        
        if (dir == CLOSE) step_position_close = step_position;
        else              step_position_open = step_position;

        stall_chk_cnt = 0;
        stall_chk_time_1ms = 0;
        softstart_complete = OFF;
        motor_step_value = STEP_TIME_1000RPM;
        timer_1ms_init_fail_chk_flag = 0;
        timer_1ms_init_fail_chk = 0;

        init_move_step = next_step;
    }
    else
    {
        timer_1ms_init_fail_chk_flag = 1;

        if (timer_1ms_init_fail_chk >= 5000U)
        {
            init_move_step = retry_step;

            timer_1ms_init_fail_chk_flag = 0;
            timer_1ms_init_fail_chk = 0;
        }
    }
}

/***********************************************************************************************************************
 * Function Name: Wait_delay_move
 * Description  : Wait 100 ms and move to the next step (Step 5, 8, 11 common)
 * Called By    : Process_init_step_0_to_9, Process_init_step_10_to_15
 * Arguments    : next_step - next case
 * Return Value : void
 ***********************************************************************************************************************/
static void Wait_delay_move(uint8_t next_step)
{
    time_1ms_init_move_flag = 1;

    if (time_1ms_init_move >= 100U)
    {
        time_1ms_init_move_flag = 0;
        time_1ms_init_move = 0;
        init_move_step = next_step;

        time_1ms_init_chk_flag = 0; 
        time_1ms_init_chk = 0;      
    }
}

/***********************************************************************************************************************
 * Function Name: Move_to_limit_position
 * Description  : Calculate the target position based on the entire learned stroke and start moving to that position
 * Called By    : Init_move (Case 13)
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Move_to_limit_position(void)
{
	if (step_position <= step_position_open + limit_step_position)
	{
		Motor_dir_close();						 // dir CLOSE
		DRV8899_On();							 // drv on
		motor_start = ON;					 // step start
		time_1ms_external_10s_chk_flag = ON; // 10s chk timer on

		motor_stall_flag = MOTOR_NORMAL; // stall reset
		// stall_chk_cnt = 0;			 stall reset
		stall_chk_time_1ms = 0;							  // stall reset
		motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset
		time_1ms_spi = 0;

		init_move_step = 14;
	}
	else
	{
		time_1ms_external_10s_chk_flag = ON; // 10s chk timer on

		init_move_step = 14;
	}
}

/***********************************************************************************************************************
 * Function Name: Check_limit_arrival
 * Description  : Monitor for target position reach or abnormal stall occurrence on the move
 * Called By    : Init_move (Case 14)
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Check_limit_arrival(void)
{
	if (((motor_stall_flag == MOTOR_STALL) || ((step_position_close - step_position_open) <= STEP_POSITION_MINIMUM_RANGE)) && (stall_test_mode == 0U))
	{
		DRV8899_Off();
		motor_start = OFF;
		fail_safety_1_cycle_flag = OFF;
		softstart_complete = OFF;
		motor_step_value = STEP_TIME_1000RPM;
		timer_1ms_init_fail_chk_flag = 0;
		timer_1ms_init_fail_chk = 0;
	}
	else if (step_position >= step_position_open + limit_step_position)
	{
		DRV8899_Off();
		motor_start = OFF;
		// step_position_open = step_position;
		stall_chk_cnt = 0;
		stall_chk_time_1ms = 0; // stall reset
		time_1ms_external_10s_chk_flag = OFF;
		time_1ms_external_10s_chk = 0;
		softstart_complete = OFF;
		motor_step_value = STEP_TIME_1000RPM;
		timer_1ms_init_fail_chk_flag = 0;
		timer_1ms_init_fail_chk = 0;

		init_move_step = 15;
	}
	else
	{
		timer_1ms_init_fail_chk_flag = 1;

		if (timer_1ms_init_fail_chk >= 5000U)
		{
			init_move_step = 0;

			timer_1ms_init_fail_chk_flag = 0;
			timer_1ms_init_fail_chk = 0;
		}
	}
}

/***********************************************************************************************************************
 * Function Name: Init_move_0_to_9
 * Description  : case 0 ~ 9
 * Called By    : Init_move 
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Init_move_0_to_9(void)
=======

    // 2. Handle wait time and acceleration
    if (time_1ms_motor_wait >= MOTOR_WAIT_TIME)
    {
        Process_active_acceleration();
    }
    else 
    {
        motor_wait_chk = OFF;
    }

    // 3. Generate step pulses
    Generate_step_pulse();
}

<<<<<<< HEAD
static void Init_move(void)
>>>>>>> d4c07df (Motor Action)
{
    switch (init_move_step)
    {
    case 0:
        Start_motor_move_init(4, OPEN, TRUE); // Case 0 TRUE
        break;
    case 4:
        Check_stall_and_move(5, 0, OPEN);
        break;
    case 5:
        Wait_delay_move(6);
        break;
    case 6:
        Start_motor_move_init(7, CLOSE, FALSE);
        break;
    case 7:
        Check_stall_and_move(8, 6, CLOSE);
        break;
    case 8:
        Wait_delay_move(9);
        break;
    case 9:
        Start_motor_move_init(10, OPEN, FALSE);
        break;
    default:
        break;
    }
}

=======
/***********************************************************************************************************************
 * Function Name: Start_motor_move_init
 * Description  : Start the motor drive, initialize the relevant variables and move on to the next step
 * Called By    : Process_init_step_0_to_9
 * Arguments    : next_step - Next Case
 * dir       - Motor Drive Direction (OPEN / CLOSE)
 * is_case0  - first entry (Case 0) or not (TRUE: Perform additional initialization / FALSE: not)
 * Return Value : void
 ***********************************************************************************************************************/
static void Start_motor_move_init(uint8_t next_step, uint8_t dir, uint8_t is_case0)
{
    if (dir == OPEN) Motor_dir_open();
    else             Motor_dir_close();

    DRV8899_On();
    motor_start = ON;
    
    // (Case 0, 6, 9)
    motor_stall_flag = MOTOR_NORMAL;
    stall_chk_time_1ms = 0;
    motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE;
    time_1ms_spi = 0;
    AAF_Tx_Position = UNKOWN_POSITION;
    AAFx_Position_Status = Unknown_Status;
    antipinch_previous_action = INITIALIZATION;
    time_1ms_init_chk = 0;
    time_1ms_init_chk_flag = 1; 

    // (Case 0)
    if (is_case0 == TRUE)
    {
        time_1ms_external_10s_chk_flag = OFF;
        time_1ms_external_10s_chk = 0;
        step_position = REFERENCE_POSITION;
    }

    init_move_step = next_step;
}


/***********************************************************************************************************************
 * Function Name: Check_stall_and_move
 * Called By: Init_move (Case 4, 7, 10)
 * next_step: next case
 * retry_step: Steps to move in case of failure (timeout)
 * dir: OPEN, CLOSE DIRECTION
 ***********************************************************************************************************************/
static void Check_stall_and_move(uint8_t next_step, uint8_t retry_step, uint8_t dir)
{
    if ((motor_stall_flag == MOTOR_STALL) || (time_1ms_init_chk >= 4500U))
    {
    
        DRV8899_Off();
        motor_start = OFF;
        
        if (dir == CLOSE) step_position_close = step_position;
        else              step_position_open = step_position;

        stall_chk_cnt = 0;
        stall_chk_time_1ms = 0;
        softstart_complete = OFF;
        motor_step_value = STEP_TIME_1000RPM;
        timer_1ms_init_fail_chk_flag = 0;
        timer_1ms_init_fail_chk = 0;

        init_move_step = next_step;
    }
    else
    {
        timer_1ms_init_fail_chk_flag = 1;

        if (timer_1ms_init_fail_chk >= 5000U)
        {
            init_move_step = retry_step;

            timer_1ms_init_fail_chk_flag = 0;
            timer_1ms_init_fail_chk = 0;
        }
    }
}

/***********************************************************************************************************************
 * Function Name: Wait_delay_move
 * Description  : Wait 100 ms and move to the next step (Step 5, 8, 11 common)
 * Called By    : Process_init_step_0_to_9, Process_init_step_10_to_15
 * Arguments    : next_step - next case
 * Return Value : void
 ***********************************************************************************************************************/
static void Wait_delay_move(uint8_t next_step)
{
    time_1ms_init_move_flag = 1;

    if (time_1ms_init_move >= 100U)
    {
        time_1ms_init_move_flag = 0;
        time_1ms_init_move = 0;
        init_move_step = next_step;

        time_1ms_init_chk_flag = 0; 
        time_1ms_init_chk = 0;      
    }
}

/***********************************************************************************************************************
 * Function Name: Move_to_limit_position
 * Description  : Calculate the target position based on the entire learned stroke and start moving to that position
 * Called By    : Init_move (Case 13)
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Move_to_limit_position(void)
{
	if (step_position <= step_position_open + limit_step_position)
	{
		Motor_dir_close();						 // dir CLOSE
		DRV8899_On();							 // drv on
		motor_start = ON;					 // step start
		time_1ms_external_10s_chk_flag = ON; // 10s chk timer on

		motor_stall_flag = MOTOR_NORMAL; // stall reset
		// stall_chk_cnt = 0;			 stall reset
		stall_chk_time_1ms = 0;							  // stall reset
		motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset
		time_1ms_spi = 0;

		init_move_step = 14;
	}
	else
	{
		time_1ms_external_10s_chk_flag = ON; // 10s chk timer on

		init_move_step = 14;
	}
}

/***********************************************************************************************************************
 * Function Name: Check_limit_arrival
 * Description  : Monitor for target position reach or abnormal stall occurrence on the move
 * Called By    : Init_move (Case 14)
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Check_limit_arrival(void)
{
	if (((motor_stall_flag == MOTOR_STALL) || ((step_position_close - step_position_open) <= STEP_POSITION_MINIMUM_RANGE)) && (stall_test_mode == 0U))
	{
		DRV8899_Off();
		motor_start = OFF;
		fail_safety_1_cycle_flag = OFF;
		softstart_complete = OFF;
		motor_step_value = STEP_TIME_1000RPM;
		timer_1ms_init_fail_chk_flag = 0;
		timer_1ms_init_fail_chk = 0;
	}
	else if (step_position >= step_position_open + limit_step_position)
	{
		DRV8899_Off();
		motor_start = OFF;
		// step_position_open = step_position;
		stall_chk_cnt = 0;
		stall_chk_time_1ms = 0; // stall reset
		time_1ms_external_10s_chk_flag = OFF;
		time_1ms_external_10s_chk = 0;
		softstart_complete = OFF;
		motor_step_value = STEP_TIME_1000RPM;
		timer_1ms_init_fail_chk_flag = 0;
		timer_1ms_init_fail_chk = 0;

		init_move_step = 15;
	}
	else
	{
		timer_1ms_init_fail_chk_flag = 1;

		if (timer_1ms_init_fail_chk >= 5000U)
		{
			init_move_step = 0;

			timer_1ms_init_fail_chk_flag = 0;
			timer_1ms_init_fail_chk = 0;
		}
	}
}

/***********************************************************************************************************************
 * Function Name: Init_move_0_to_9
 * Description  : case 0 ~ 9
 * Called By    : Init_move 
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Init_move_0_to_9(void)
{
    switch (init_move_step)
    {
    case 0:
        Start_motor_move_init(4, OPEN, TRUE); // Case 0 TRUE
        break;
    case 4:
        Check_stall_and_move(5, 0, OPEN);
        break;
    case 5:
        Wait_delay_move(6);
        break;
    case 6:
        Start_motor_move_init(7, CLOSE, FALSE);
        break;
    case 7:
        Check_stall_and_move(8, 6, CLOSE);
        break;
    case 8:
        Wait_delay_move(9);
        break;
    case 9:
        Start_motor_move_init(10, OPEN, FALSE);
        break;
    default:
        break;
    }
}

>>>>>>> bdd6f71 (Init move)
/***********************************************************************************************************************
 * Function Name: Init_move_10_to_15
 * Description  : case 10 ~ 15
 * Called By    : Init_move 
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Init_move_10_to_15(void)
{
    switch (init_move_step)
    {
    case 10:
        Check_stall_and_move(11, 9, OPEN);
        break;
    case 11:
        Wait_delay_move(12);
        break;
    case 12:
        limit_step_position = (step_position_close - step_position_open) * AAF_ERROR_ANGLE / AAF_FULL_ANGLE;
        // open_1st_step_position = (step_position_close - step_position_open) * AAF_1ST_OPEN_ANGLE / AAF_FULL_ANGLE;  ICE NOT USED
        // open_2nd_step_position = (step_position_close - step_position_open) * AAF_2ST_OPEN_ANGLE / AAF_FULL_ANGLE;  ICE NOT USED
		init_move_step = 13;
        break;
    case 13:
        Move_to_limit_position();
        break;
    case 14:
        Check_limit_arrival();
        break;
    case 15:
        time_1ms_init_move_flag = 1;
        if (time_1ms_init_move >= 100U)
        {
			wake_up_motor_range_init_chk = COMPLETE;
			time_1ms_init_move_flag = 0;
			time_1ms_init_move = 0;
			evrdy_on_flag = ON;
			motor_Short_chk_count = 0U;
			motor_Open_chk_count = 0U;
			init_move_step = 19;
        }
        break;
    default:
        break;
    }
}

static void Init_move(void)
{
    if (init_move_step < 10)
    {
        Init_move_0_to_9();
    }
    else
    {
        Init_move_10_to_15();
    }
}

static void VDC_adc(void)
{
	time_1ms_adc_flag = 1;
#if 0
	if ((time_1ms_adc >= 10U) && (voltage_chk_delay_complete == 1U))
	{
		time_1ms_adc_error_chk_flag = 1;

		while (INTC1.ICADCA0I0.BIT.RFADCA0I0 == 0)
		{
			if (time_1ms_adc_error_chk >= 100)
			{
				time_1ms_adc_error_chk_flag = 0;
				time_1ms_adc_error_chk = 0;
				adc_fail = 1;
				break;
			}
		}

		time_1ms_adc_error_chk_flag = 0;
		time_1ms_adc_error_chk = 0;

		/*
		while (!INTC1.ICADCA0I0.BIT.RFADCA0I0)
		{

		}
		*/

		INTC1.ICADCA0I0.BIT.RFADCA0I0 = 0;
		R_Config_ADCA0_ScanGroup1_GetResult(&bat_adc, 8);

		adc_sum = 0;

		for (int i = 8; i >= 0; i--)
		{
			adc_chk[i + 1] = adc_chk[i];
			adc_sum += adc_chk[i];
		}

		adc_chk[0] = bat_adc;

		adc_sum += adc_chk[0];

		adc_avr = adc_sum / 10U;

		adc_chk_ok_flag++;

		if (adc_chk_ok_flag >= 10U)
		{
			adc_chk_ok_flag = 10;
		}

		time_1ms_adc = 0;
	}
#endif
	if (adc_chk_ready == 1U)
	{
		INTC1.ICADCA0I0.BIT.RFADCA0I0 = 0;
		// R_Config_ADCA0_ScanGroup1_GetResult(&bat_adc, 8);
		R_Config_ADCA0_ScanGroup1_GetResult(&scan_results, 1);
		bat_adc = scan_results[0];
		adc_sum = 0;

		for (int i = 8; i >= 0; i--)
		{
			adc_chk[i + 1] = adc_chk[i];
			adc_sum += adc_chk[i];
		}

		adc_chk[0] = bat_adc;

		adc_sum += adc_chk[0];

		adc_avr = adc_sum / 10U;

		adc_chk_ok_flag++;

		if (adc_chk_ok_flag >= 10U)
		{
			adc_chk_ok_flag = 10;
		}

		adc_chk_ready = 0;
		time_1ms_adc = 0;
	}
	else if ((time_1ms_adc >= 10U) && (voltage_chk_delay_complete == 1U) && (adc_chk_ready == 0U))
	{
		if (INTC1.ICADCA0I0.BIT.RFADCA0I0 == 1U)
		{
			adc_chk_ready = 1;
			time_1ms_adc = 0;
		}
	}
	else if (time_1ms_voltage_chk_delay >= 500U)
	{
		voltage_chk_delay_complete = 1;
		time_1ms_voltage_chk_delay = 0;
		time_1ms_volt_chk_dly_flag = 0;
		Voltage_chk_current_limit_init();
	}
	else
	{
	}

	if ((time_1ms_volt_stat_chg_wait >= 10U) && (motor_start == OFF))
	{
		voltage_status_change_complete = COMPLETE;
		time_1ms_volt_stat_chg_wait = 0;
		time_1ms_volt_stat_chg_wait_flag = 0;
	}
}

static void ERROR_chk(void)
{
	if (AAF_ProtectionMode_Rx == ON)
	{
		protection_function = ON;
		AAF_ProtectionMode_Tx = ON;
	}
	else if ((AAF_ProtectionMode_Rx == OFF) && (stall_test_mode == 0U))
	{
		if (adc_chk_ok_flag == 10U)
		{
			if (adc_avr <= ADC_UNDER_VOLTAGE_7V)
			{
				DRV_Off();
				motor_start = OFF;
				stall_chk_cnt = 0;
				stall_chk_time_1ms = 0;
				softstart_complete = OFF;
				motor_step_value = STEP_TIME_1000RPM;
				AAFx_InitStatus = DURING_INITIALIZATION;
				AAF_Tx_Position = UNKOWN_POSITION;
				AAFx_Position_Status = Unknown_Status;
				AAFx_Low_Volt = UNDER_VOLTAGE;

				return;
			}
			if ((AAFx_Low_Volt == UNDER_VOLTAGE) && (protection_function == ON))
			{
				if (adc_avr >= ADC_UNDER_VOLTAGE_9V)
				{
					AAFx_Low_Volt = NO_ERROR;
					protection_function = OFF;
					protection_Mode_step = 0;
					Under_Voltage_Deceted = 0;
					time_1ms_adc_1s_chk = 0;
					time_1ms_adc_1s_chk_flag = 0;
					Re_Init();
				}
			}
			else if (adc_avr <= ADC_UNDER_VOLTAGE_8_5V)
			{
				if (Under_Voltage_Deceted == 0)
				{
					Under_Voltage_Deceted = 1;
					time_1ms_adc_1s_chk = 0;
					time_1ms_adc_1s_chk_flag = 1;
				}
				if ((Under_Voltage_Deceted == 1) && (time_1ms_adc_1s_chk >= 1000))
				{
					AAFx_Low_Volt = UNDER_VOLTAGE;
					protection_function = ON;
					DTC_Status |= 0x20;
					time_1ms_adc_1s_chk = 1000;
					time_1ms_adc_1s_chk_flag = 0;
				}
			}
			else
			{
				if (Under_Voltage_Deceted == 1)
				{
					Under_Voltage_Deceted = 0;
					time_1ms_adc_1s_chk = 0;
					time_1ms_adc_1s_chk_flag = 0;
				}
			}
			if (adc_avr >= ADC_OVER_VOLTAGE_18V)
			{
				DRV_Off();
				motor_start = OFF;
				stall_chk_cnt = 0;
				stall_chk_time_1ms = 0;
				softstart_complete = OFF;
				motor_step_value = STEP_TIME_1000RPM;
				AAFx_InitStatus = DURING_INITIALIZATION;
				AAF_Tx_Position = UNKOWN_POSITION;
				AAFx_Position_Status = Unknown_Status;
				AAFx_Over_Volt = OVER_VOLTAGE;

				return;
			}

			if ((AAFx_Over_Volt == OVER_VOLTAGE) && (protection_function == ON))
			{
				if (adc_avr <= ADC_OVER_VOLTAGE_16V)
				{
					AAFx_Over_Volt = NO_ERROR;
					protection_function = OFF;
					protection_Mode_step = 0;
					Re_Init();
				}
			}
			else
			{
				if (adc_avr >= ADC_OVER_VOLTAGE_16_5V)
				{
					if (Over_Voltage_Deceted == 0)
					{
						Over_Voltage_Deceted = 1;
						time_1ms_adc_1s_chk = 0;
						time_1ms_adc_1s_chk_flag = 1;
					}

					if ((Over_Voltage_Deceted == 1) && (time_1ms_adc_1s_chk >= 1000))
					{
						AAFx_Over_Volt = OVER_VOLTAGE;
						protection_function = ON;
						DTC_Status |= 0x40;
						time_1ms_adc_1s_chk = 1000;
						time_1ms_adc_1s_chk_flag = 0;
					}

					else
					{
					}
				}
				else
				{
					if (Over_Voltage_Deceted == 1)
					{
						Over_Voltage_Deceted = 0;
						time_1ms_adc_1s_chk = 0;
						time_1ms_adc_1s_chk_flag = 0;
					}
				}
			}
		}

		if (((AAFx_InitStatus == ABNORMAL_FINISHED_INITIALIZATION)) && (motor_fault_chk == 1))
		{
			DRV8899_Sleep();
			AAFx_Motor_Fault = 1;
			// protection_function = ON;
			DTC_Status |= 0x10;
			// DTC_memory_write = DTC_memory_write | 0x01;
			// AAF_ProtectionMode_Tx = ON;	// protection mode?占쏙옙 vpc ?占쏙옙?占쏙옙
		}
		else
		{
			AAFx_Motor_Fault = NO_ERROR;
		}
		// =========================================================
		// 1. (Initial Detection)
		// =========================================================
		if (AAFx_Circuit_Short == AAF_CIRCUIT_SHORT)
		{
			return;
		}

		if ((AAF_OverCurrent == OVER_CURRENT) && (Short_Detected == 0))
		{
			time_1ms_motor_Short_chk = 0;
			time_1ms_motor_Short_chk_flag = 1;
			Short_Detected = 1;
			Short_fault_check = 0;
			motor_Short_chk_count++;
			DRV_Off();
			motor_start = OFF;
			stall_chk_cnt = 0;
			stall_chk_time_1ms = 0;
			softstart_complete = OFF;
			motor_step_value = STEP_TIME_1000RPM;
			AAF_Tx_Position = UNKOWN_POSITION;
			AAFx_Position_Status = Unknown_Status;
		}

		// =========================================================
		// 2. (Monitoring & Retry)
		// =========================================================

		else if (Short_Detected == 1)
		{

			if ((time_1ms_motor_Short_chk >= 200) && (Short_fault_check == 0))
			{
				Error_FaultClear();
				// motor_Short_chk_count++;
				Short_fault_check = 1;
			}

			AAF_OverCurrent = (unsigned int)(rx_16bit_spi[9] & 0x800U);

			if (time_1ms_motor_Short_chk >= 1000)
			{
				if ((AAF_OverCurrent == NO_ERROR) && (Short_fault_check == 1))
				{
					Re_Init();
					time_1ms_motor_Short_chk = 0;
					time_1ms_motor_Short_chk_flag = 0;
					Short_Detected = 0;
				}

				else if ((AAF_OverCurrent == OVER_CURRENT) && (motor_Short_chk_count < 10))
				{
					// time_1ms_motor_Short_chk = 0;
					// Short_fault_check = 0;
					Short_Detected = 0;
				}

				else
				{
					// invalid
				}
			}

			if ((motor_Short_chk_count >= 10))
			{
				DRV_Off();
				motor_start = OFF;
				AAFx_Circuit_Short = AAF_CIRCUIT_SHORT;
				DRV8899_Sleep();
				DTC_Status |= 0x04;
				// time_1ms_motor_Short_chk = 0;
				time_1ms_motor_Short_chk_flag = 0;
				// Short_fault_check = 0;
				Short_Detected = 0;
				Flash_memory_write();
			}
		}

		else
		{
			// invalid
		}
		 //LIN_Short_Chk();

		if (AAFx_Motor_Fault == 1)
		{
			return;
		}
		if ((motor_open_load == MOTOR_FAULT) && (Open_Detected == 0))
		{
			time_1ms_motor_Open_chk = 0;
			time_1ms_motor_Open_chk_flag = 1;
			Open_Detected = 1;
			Open_fault_check = 0;
			motor_Open_chk_count++;
			DRV_Off();
			motor_start = OFF;
			stall_chk_cnt = 0;
			stall_chk_time_1ms = 0;
			softstart_complete = OFF;
			motor_step_value = STEP_TIME_1000RPM;
			AAF_Tx_Position = UNKOWN_POSITION;
			AAFx_Position_Status = Unknown_Status;
		}

		else if (Open_Detected == 1)
		{
			if ((time_1ms_motor_Open_chk >= 200) && (Open_fault_check == 0))
			{
				Error_FaultClear();
				Open_fault_check = 1;
			}

			motor_open_load = (unsigned int)(rx_16bit_spi[9] & 0x100U);

			if (time_1ms_motor_Open_chk >= 1000)
			{
				if ((motor_open_load == NO_ERROR) && (Open_fault_check == 1))
				{
					Re_Init();
					// motor_Open_chk_count++;
					time_1ms_motor_Open_chk = 0;
					time_1ms_motor_Open_chk_flag = 0;
					Open_Detected = 0;
				}

				else if ((motor_open_load == MOTOR_FAULT) && (motor_Open_chk_count < 10))
				{
					// time_1ms_motor_Open_chk = 0;
					// Open_fault_check = 0;
					Open_Detected = 0;
				}

				else
				{
					// invalid
				}
			}

			else
			{
				// invalid
			}
		}
		if ((motor_Open_chk_count >= 10))
		{
			DRV_Off();
			motor_start = OFF;
			AAFx_Motor_Fault = 1;
			DRV8899_Sleep();
			DTC_Status |= 0x10;
			// time_1ms_motor_Open_chk = 0;
			time_1ms_motor_Open_chk_flag = 0;

			// Open_fault_check = 1;
			Flash_memory_write();
		}

		if (First_Powerchk == 1)
		{
			if (fdl_fail >= 10)
			{
				AAFx_Circuit_Open = AAF_CIRCUIT_OPEN;
				protection_function = ON;
			}
			else
			{
				AAFx_Circuit_Open = NO_ERROR;
			}
		}

		LIMP_HOME();
	}
	else
	{
	}
}

static void Tx_position_complete_chk(void)
{
	if (AAFx_Index == AAF_1)
	{
		AAF_Tx_Position = aaf_action;
	}
	else if (AAFx_Index == AAF_2)
	{
		AAF_Tx_Position = aaf_action;
	}
	else if (AAFx_Index == AAF_3)
	{
		AAF_Tx_Position = aaf_action;
	}
	else
	{
	}
}

static void Flash_memory_write(void)
{
	time_1ms_fdl_error_chk_flag = 0;
	time_1ms_fdl_error_chk = 0;
	close_memory_write = step_position_close;
	open_memory_write = step_position_open;
	now_step_memory_write = step_position;
	position_memory_write = AAF_Tx_Position;
	Initial_memory_write = evrdy_on_flag;
	limit_memory_write = limit_step_position;

	position_status_memory_write = AAFx_Position_Status;
	AAFx_InitStatus_memory_write = AAFx_InitStatus;
	DTC_memory_write |= DTC_Status;
	power_chk_memory_write = power_chk;
	First_Powerchk_memory_write = First_Powerchk;

	w_buff[0] = close_memory_write & 0x00FFU; // write 2byte read 4byte ?븯?쐞
	w_buff[1] = (close_memory_write & 0xFF00U) >> 8U;

	w_buff[2] = open_memory_write & 0x00FFU; // write 2byte read 4byte ?긽?쐞
	w_buff[3] = (open_memory_write & 0xFF00U) >> 8U;

	w_buff[4] = now_step_memory_write & 0x00FFU; // write 2byte read 4byte
	w_buff[5] = (now_step_memory_write & 0xFF00U) >> 8U;

	w_buff[6] = position_memory_write & 0xFFU; // write 2byte read 4byte position+evrdy
	w_buff[7] = (Initial_memory_write & 0xFFU);

	w_buff[8] = limit_memory_write & 0x00FFU; // write 2byte read 4byte limitstep
	w_buff[9] = (limit_memory_write & 0xFF00U) >> 8U;

	w_buff[10] = position_status_memory_write & 0xFFU; // write 2byte read 4byte limitstep
	w_buff[11] = (AAFx_InitStatus_memory_write & 0xFFU);

	w_buff[12] = DTC_memory_write & 0x00FFU; // write 2byte read 4byte limitstep
	w_buff[13] = (DTC_memory_write & 0xFF00U) >> 8U;

	w_buff[14] = power_chk_memory_write & 0x00FFU; // write 2byte read 4byte limitstep
	w_buff[15] = (power_chk_memory_write & 0xFF00U) >> 8U;

	w_buff[16] = First_Powerchk_memory_write & 0x00FFU; // write 2byte read 4byte limitstep
	w_buff[17] = (First_Powerchk_memory_write & 0xFF00U) >> 8U;

	ret = function_FDL_erease(0, 1);

	time_1ms_fdl_error_chk_flag = 1;

	if (ret < 0) // error
	{
		while (1)
		{
			if (time_1ms_fdl_error_chk >= 100U)
			{
				fdl_fail += 1;
				break;
			}
		}
	}

	ret = function_FDL_write(w_buff, 0, 9); // size = word(2byte)

	time_1ms_fdl_error_chk_flag = 1;

	if (ret < 0) // error
	{
		while (1)
		{
			if (time_1ms_fdl_error_chk >= 100U)
			{
				fdl_fail += 1;
				break;
			}
		}
	}

	time_1ms_fdl_error_chk_flag = 0;
	time_1ms_fdl_error_chk = 0;
}

static void Flash_memory_read(void)
{

	ret = function_FDL_read(r_buff, 0, 9); // size = word(2byte) read 4byte

	time_1ms_fdl_error_chk_flag = 1;

	if (ret < 0) // error
	{
		while (1)
		{
			if (time_1ms_fdl_error_chk >= 100U)
			{
				fdl_fail += 1;
				break;
			}
		}
	}

	time_1ms_fdl_error_chk_flag = 0;
	time_1ms_fdl_error_chk = 0;

	close_memory_read = (unsigned int)r_buff[0] & 0xFFFF;		 // 4byte 源뚯?
	open_memory_read = (unsigned int)(r_buff[0] >> 16) & 0xFFFF; // 4byte 源뚯?

	now_step_memory_read = (unsigned int)r_buff[1] & 0xFFFF; // 4byte 源뚯?

	position_Initial_combined_read = (unsigned int)(r_buff[1] >> 16) & 0xFFFF;		  // position_memory_read+Initial_memory_read
	position_memory_read = (unsigned int)position_Initial_combined_read & 0xFF;		  // 4byte 源뚯?
	Initial_memory_read = (unsigned int)(position_Initial_combined_read >> 8) & 0xFF; // 4byte 源뚯?

	limit_memory_read = (unsigned int)r_buff[2] & 0xFFFF;

	position_Initstatus_combined_read = (unsigned int)(r_buff[2] >> 16) & 0xFFFF;
	position_status_memory_read = (unsigned int)position_Initstatus_combined_read & 0xFF;
	AAFx_InitStatus_memory_read = (unsigned int)(position_Initstatus_combined_read >> 8) & 0xFF;

	DTC_memory_read = (unsigned int)(r_buff[3]) & 0xFFFF;

	power_chk_memory_read = (unsigned int)(r_buff[3] >> 16) & 0xF;
	First_Powerchk_memory_read = (unsigned int)(r_buff[4]) & 0xF;
}
void Re_Init(void)
{
	/*DRV_Off();							  // drv of
	motor_start = OFF;					  // step stop
	time_1ms_external_10s_chk_flag = OFF; // 10s chk timer off
	time_1ms_external_10s_chk = 0;
	aaf_action = FLAP_STOP;
	aaf_action_complete_chk = FLAP_STOP;
	time_1ms_init_chk = 0;
	time_1ms_init_chk_flag = 0;	 // test
	time_1ms_stall_chk = 0;		 // test
	time_1ms_stall_chk_flag = 0; // test

	stall_chk_cnt = 0;		// stall reset
	stall_chk_time_1ms = 0; // stall reset

	motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset*/
	time_1ms_diag_auto = 0;
	time_1ms_diag_auto_flag = OFF;
	diag_mode_auto_action = OFF;
	aaf_action = 0;
	motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset
	aaf_step = AAF_INITIALIZATION;					  // MCU is reset, AAF is initialized.
	aaf_init_step = START_INITIALIZATION;			  // MCU is reset, AAF is initialized.
	AAFx_Position_Status = Unknown_Status;
	AAF_Tx_Position = UNKOWN_POSITION;
	init_move_step = 0;
	AAFx_ErrorStatus = No_ErrorStatus;
	step_position = REFERENCE_POSITION;
	step_position_open = 0;
	step_position_close = 0;
	lin_aaf_command = OPEN;
	Diag_Mode = 0;
	Diag_Mode_chk = 0;
	evrdy_on_flag = OFF;
}

static void Operating_mode(void)
{
	switch (aaf_step)
	{
	case AAF_WAITING:
		if ((step_start_flag == ON) && (lin_bus_inactive_flag == OFF))
		{
			if (lin_aaf_command == OPEN) // lin open command chk
			{
				DRV8899_Wakeup();

				time_1ms_diag_auto = 0;
				time_1ms_diag_auto_flag = OFF;
				diag_mode_auto_action = OFF;

				aaf_action = OPEN;
				aaf_step = AAF_OPERATE;
			}
			else if (lin_aaf_command == CLOSE) // lin close command chk
			{
				DRV8899_Wakeup();

				time_1ms_diag_auto = 0;
				time_1ms_diag_auto_flag = OFF;
				diag_mode_auto_action = OFF;

				aaf_action = CLOSE;
				aaf_step = AAF_OPERATE;
			}
			else if (lin_aaf_command == OPEN_1ST) // lin init command chk
			{
				DRV8899_Wakeup();

				time_1ms_diag_auto = 0;
				time_1ms_diag_auto_flag = OFF;
				diag_mode_auto_action = OFF;

				aaf_action = OPEN_1ST;
				aaf_step = AAF_OPERATE;
			}
			else if (lin_aaf_command == OPEN_2ND) // lin init command chk
			{
				DRV8899_Wakeup();

				time_1ms_diag_auto = 0;
				time_1ms_diag_auto_flag = OFF;
				diag_mode_auto_action = OFF;

				aaf_action = OPEN_2ND;
				aaf_step = AAF_OPERATE;
			}
			else if ((lin_aaf_command == DIAG_MODE_OPEN) && (AAF_Tx_Position != DIAG_MODE_OPEN)) // lin init command chk
			{
				DRV8899_Wakeup();

				time_1ms_diag_auto = 0;
				time_1ms_diag_auto_flag = OFF;
				diag_mode_auto_action = OFF;

				aaf_action = DIAG_MODE_OPEN;
				aaf_step = AAF_OPERATE;
			}
			else if ((lin_aaf_command == DIAG_MODE_CLOSE) && (AAF_Tx_Position != DIAG_MODE_CLOSE)) // lin init command chk
			{
				DRV8899_Wakeup();

				time_1ms_diag_auto = 0;
				time_1ms_diag_auto_flag = OFF;
				diag_mode_auto_action = OFF;

				aaf_action = DIAG_MODE_CLOSE;
				aaf_step = AAF_OPERATE;
			}
			else if (lin_aaf_command == DIAG_MODE_AUTO) // lin init command chk
			{
				DRV8899_Wakeup();

				if (diag_mode_auto_action == ON)
				{
					if (diag_mode_auto_dir == OPEN)
					{
						diag_mode_auto_dir = CLOSE;
					}
					else if (diag_mode_auto_dir == CLOSE)
					{
						diag_mode_auto_dir = OPEN;
					}
					else
					{
					}
				}
				else
				{
					diag_mode_auto_action = ON;
					diag_mode_auto_dir = OPEN;
				}

				time_1ms_diag_auto_flag = ON;
				aaf_action = DIAG_MODE_AUTO;
				aaf_step = AAF_OPERATE;
			}
			else
			{
			}

			aaf_action_complete_chk = FLAP_START;

			step_start_flag = OFF;
		}
		else if ((diag_mode_auto_action == ON) && (lin_bus_inactive_flag == OFF))
		{
			if (diag_mode_auto_dir == OPEN)
			{
				diag_mode_auto_dir = CLOSE;
			}
			else if (diag_mode_auto_dir == CLOSE)
			{
				diag_mode_auto_dir = OPEN;
			}
			else
			{
			}

			aaf_action = DIAG_MODE_AUTO;
			aaf_step = AAF_OPERATE;

			aaf_action_complete_chk = FLAP_START;
			time_1ms_diag_auto_flag = ON;
			step_start_flag = OFF;
		}
		else
		{
		}

		break;

	case AAF_OPERATE:

		if (aaf_action == OPEN)
		{
			if (step_position >= (step_position_open + limit_step_position))
			{
				Motor_Open();						 // dir OPEN
				DRV_On();							 // drv on
				motor_start = ON;					 // step start
				time_1ms_external_10s_chk_flag = ON; // 10s chk timer on
				time_1ms_stall_chk_flag = ON;		 // test

				stall_chk_time_1ms = 0;							  // stall reset
				motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset

				flap_move = OPEN;
				Diag_Mode = 0;
				aaf_step = TRAVEL_RANGE_COMPLETE_CHECK;
			}
			else
			{
				aaf_step = TRAVEL_RANGE_COMPLETE_CHECK;
			}
		}
		else if (aaf_action == CLOSE)
		{
			if (step_position <= (step_position_close - limit_step_position))
			{
				Motor_Close();						 // dir CLOSE
				DRV_On();							 // drv on
				motor_start = ON;					 // step start
				time_1ms_external_10s_chk_flag = ON; // 10s chk timer on
				time_1ms_stall_chk_flag = ON;		 // test

				stall_chk_time_1ms = 0;							  // stall reset
				motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset

				flap_move = CLOSE;
				Diag_Mode = 0;
				aaf_step = TRAVEL_RANGE_COMPLETE_CHECK;
			}
			else
			{
				aaf_step = TRAVEL_RANGE_COMPLETE_CHECK;
			}
		}
		else if (aaf_action == OPEN_1ST)
		{
			if (step_position <= (step_position_open + open_1st_step_position))
			{
				Motor_Close();						 // dir CLOSE
				DRV_On();							 // drv on
				motor_start = ON;					 // step start
				time_1ms_external_10s_chk_flag = ON; // 10s chk timer on
				time_1ms_stall_chk_flag = ON;		 // test

				stall_chk_time_1ms = 0;							  // stall reset
				motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset

				flap_move = CLOSE;
				Diag_Mode = 0;
				aaf_step = TRAVEL_RANGE_COMPLETE_CHECK;
			}
			else if (step_position >= step_position_open + open_1st_step_position)
			{
				Motor_Open();						 // dir OPEN
				DRV_On();							 // drv on
				motor_start = ON;					 // step start
				time_1ms_external_10s_chk_flag = ON; // 10s chk timer on
				time_1ms_stall_chk_flag = ON;		 // test

				stall_chk_time_1ms = 0;							  // stall reset
				motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset

				flap_move = OPEN;
				Diag_Mode = 0;
				aaf_step = TRAVEL_RANGE_COMPLETE_CHECK;
			}
			else
			{
				Diag_Mode = 0;
				aaf_step = TRAVEL_RANGE_COMPLETE_CHECK;
			}
		}
		else if (aaf_action == OPEN_2ND)
		{
			if (step_position <= (step_position_open + open_2nd_step_position))
			{
				Motor_Close();						 // dir CLOSE
				DRV_On();							 // drv on
				motor_start = ON;					 // step start
				time_1ms_external_10s_chk_flag = ON; // 10s chk timer on
				time_1ms_stall_chk_flag = ON;		 // test

				stall_chk_time_1ms = 0;							  // stall reset
				motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset

				flap_move = CLOSE;
				Diag_Mode = 0;
				aaf_step = TRAVEL_RANGE_COMPLETE_CHECK;
			}
			else if (step_position >= (step_position_open + open_2nd_step_position))
			{
				Motor_Open();						 // dir OPEN
				DRV_On();							 // drv on
				motor_start = ON;					 // step start
				time_1ms_external_10s_chk_flag = ON; // 10s chk timer on
				time_1ms_stall_chk_flag = ON;		 // test

				stall_chk_time_1ms = 0;							  // stall reset
				motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset

				flap_move = OPEN;
				Diag_Mode = 0;
				aaf_step = TRAVEL_RANGE_COMPLETE_CHECK;
			}
			else
			{
				Diag_Mode = 0;
				aaf_step = TRAVEL_RANGE_COMPLETE_CHECK;
			}
		}
		else if (aaf_action == DIAG_MODE_OPEN)
		{

			Motor_Open();				  // dir OPEN
			DRV_On();					  // drv on
			motor_start = ON;			  // step start
			time_1ms_stall_chk_flag = ON; // test

			stall_chk_time_1ms = 0;							  // stall reset
			motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset
			Diag_Mode = 1;
			flap_move = OPEN;
			// time_1ms_init_chk_flag = 1;
			aaf_step = TRAVEL_RANGE_COMPLETE_CHECK;
		}
		else if (aaf_action == DIAG_MODE_CLOSE)
		{

			Motor_Close();				  // dir CLOSE
			DRV_On();					  // drv on
			motor_start = ON;			  // step start
			time_1ms_stall_chk_flag = ON; // test

			stall_chk_time_1ms = 0;							  // stall reset
			motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset
			Diag_Mode = 1;
			flap_move = CLOSE;
			// time_1ms_init_chk_flag = 1;
			aaf_step = TRAVEL_RANGE_COMPLETE_CHECK;
		}
		else if ((aaf_action == DIAG_MODE_AUTO) && (diag_mode_auto_dir == OPEN))
		{

			Motor_Open();				  // dir OPEN
			DRV_On();					  // drv on
			motor_start = ON;			  // step start
			time_1ms_stall_chk_flag = ON; // test

			stall_chk_time_1ms = 0;							  // stall reset
			motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset
			Diag_Mode = 1;
			flap_move = OPEN;
			// time_1ms_init_chk_flag = 1;
			time_1ms_diag_auto_flag = 1;

			aaf_step = TRAVEL_RANGE_COMPLETE_CHECK;
		}
		else if ((aaf_action == DIAG_MODE_AUTO) && (diag_mode_auto_dir == CLOSE))
		{

			Motor_Close();				  // dir CLOSE
			DRV_On();					  // drv on
			motor_start = ON;			  // step start
			time_1ms_stall_chk_flag = ON; // test
			Diag_Mode = 1;
			stall_chk_time_1ms = 0;							  // stall reset
			motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset

			flap_move = CLOSE;
			// time_1ms_init_chk_flag = 1;
			time_1ms_diag_auto_flag = 1;

			aaf_step = TRAVEL_RANGE_COMPLETE_CHECK;
		}
		else
		{
		}
		aaf_action_complete_chk = FLAP_MOVING;

		break;

	case TRAVEL_RANGE_COMPLETE_CHECK:

		if ((aaf_action == OPEN) && (step_position <= (step_position_open + limit_step_position))) //
		{
			AAF_Tx_Position = OPEN;
			Tx_position_complete_chk();
			aaf_step = FINISHED_OPERATE;
		}
		else if ((aaf_action == CLOSE) && (step_position >= (step_position_close - limit_step_position)))
		{
			AAF_Tx_Position = CLOSE;
			Tx_position_complete_chk();
			aaf_step = FINISHED_OPERATE;
		}
		else if ((aaf_action == OPEN_1ST) && ((step_position >= (step_position_open + open_1st_step_position - ERROR_RANGE)) && (step_position <= (step_position_open + open_1st_step_position + ERROR_RANGE))))
		{
			AAF_Tx_Position = OPEN_1ST;
			Tx_position_complete_chk();
			aaf_step = FINISHED_OPERATE;
		}
		else if ((aaf_action == OPEN_2ND) && ((step_position >= (step_position_open + open_2nd_step_position - ERROR_RANGE)) && (step_position <= (step_position_open + open_2nd_step_position + ERROR_RANGE))))
		{
			AAF_Tx_Position = OPEN_2ND;
			Tx_position_complete_chk();
			aaf_step = FINISHED_OPERATE;
		}
		else if ((aaf_action == DIAG_MODE_OPEN) && (step_position <= (step_position_open + limit_step_position)))
		{
			AAF_Tx_Position = DIAG_MODE_OPEN;
			aaf_step = FINISHED_OPERATE;
		}
		else if ((aaf_action == DIAG_MODE_CLOSE) && (step_position >= (step_position_close - limit_step_position)))
		{
			AAF_Tx_Position = DIAG_MODE_CLOSE;
			aaf_step = FINISHED_OPERATE;
		}
		else if ((aaf_action == DIAG_MODE_AUTO) && (diag_mode_auto_dir == OPEN) && (step_position <= (step_position_open + limit_step_position)))
		{
			AAF_Tx_Position = DIAG_MODE_AUTO;
			aaf_step = FINISHED_OPERATE;
		}
		else if ((aaf_action == DIAG_MODE_AUTO) && (diag_mode_auto_dir == CLOSE) && (step_position >= (step_position_close - limit_step_position)))
		{
			AAF_Tx_Position = DIAG_MODE_AUTO;
			aaf_step = FINISHED_OPERATE;
		}
		// else if(((motor_stall_flag == MOTOR_STALL) && (time_1ms_stall_chk >= 100)) || (time_1ms_init_chk >= 4800))
		else if ((motor_stall_flag == MOTOR_STALL) && (time_1ms_stall_chk >= 100U))
		{
			DRV_Off();
			motor_start = OFF;
			softstart_complete = OFF;
			motor_step_value = STEP_TIME_1000RPM;
			if (aaf_action == DIAG_MODE_OPEN)
			{
				AAF_Tx_Position = DIAG_MODE_OPEN;
				aaf_step = FINISHED_OPERATE;
			}
			else if (aaf_action == DIAG_MODE_CLOSE)
			{
				AAF_Tx_Position = DIAG_MODE_CLOSE;
				aaf_step = FINISHED_OPERATE;
			}
			else if (aaf_action == DIAG_MODE_AUTO)
			{
				AAF_Tx_Position = DIAG_MODE_AUTO;
				aaf_step = FINISHED_OPERATE;
			}
			else
			{
			}
			time_1ms_external_10s_chk_flag = OFF; // 10s chk timer on
			time_1ms_external_10s_chk = 0;
			aaf_step = CHECK_AAF_CONDITION;
			time_1ms_init_chk_flag = 0; // test
			time_1ms_init_chk = 0;		// test

			aaf_action = FLAP_STOP;
		}
		else
		{
		}
		break;

	case CHECK_AAF_CONDITION:
		if (flap_move == OPEN)
		{
			antipinch_previous_action = OPEN;
			antipinch_action_on = ON;

			motor_start = OFF;
			stall_chk_cnt = 0;
			stall_chk_time_1ms = 0; // stall reset

			flap_move = FLAP_STOP;
		}
		else if (flap_move == CLOSE)
		{
			antipinch_previous_action = CLOSE;
			antipinch_action_on = ON;

			motor_start = OFF;
			stall_chk_cnt = 0;
			stall_chk_time_1ms = 0; // stall reset

			flap_move = FLAP_STOP;
		}

		time_1ms_stall_chk = 0;		 // test
		time_1ms_stall_chk_flag = 0; // test
		break;

	case AAF_INITIALIZATION:

		// init action

		switch (aaf_init_step)
		{
		case START_INITIALIZATION:
			AAFx_InitStatus = DURING_INITIALIZATION;

			if (fail_safety_flag == ON)
			{
				init_move_step = 0;
			}

			aaf_init_step = CHECK_TRAVELRANGE;

			break;

		case CHECK_TRAVELRANGE:
			Init_move();

			if ((init_move_step == 19U) && (fail_safety_flag == OFF))
			{
				init_move_step = 0;
				aaf_init_step = NORMAL_INITIALIZATION;
			}

			break;

		case TRAVEL_RANGE_ERROR:

			break;

		case NORMAL_INITIALIZATION:
			aaf_step = FINISHED_OPERATE;
			AAF_Tx_Position = OPEN;
			AAFx_Position_Status = Open_Status;
			AAFx_InitStatus = NORMAL_FINISHED_INITIALIZATION;

			break;

		default:
			break;
		}

		break;

	case FINISHED_OPERATE:

		DRV_Off();							  // drv of
		motor_start = OFF;					  // step stop
		time_1ms_external_10s_chk_flag = OFF; // 10s chk timer off
		time_1ms_external_10s_chk = 0;

		if ((AAF_Tx_Position == OPEN) && (Diag_Mode == 0))
		{
			AAFx_Position_Status = Open_Status;
		}
		else if ((AAF_Tx_Position == CLOSE) && (Diag_Mode == 0))
		{
			AAFx_Position_Status = Close_Status;
		}
		else if ((AAF_Tx_Position == DIAG_MODE_OPEN) || (AAF_Tx_Position == DIAG_MODE_CLOSE) || (AAF_Tx_Position == DIAG_MODE_AUTO))
		{
			AAFx_Position_Status = Unknown_Status;
		}
		else
		{
		}

		time_1ms_stall_chk = 0;		 // test
		time_1ms_stall_chk_flag = 0; // test
		softstart_complete = OFF;
		motor_step_value = STEP_TIME_1000RPM;
		time_1ms_init_chk_flag = 0;						  // test
		time_1ms_init_chk = 0;							  // test
		stall_chk_cnt = 0;								  // stall reset
		stall_chk_time_1ms = 0;							  // stall reset
		motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset
		if (aaf_action == DIAG_MODE_AUTO)
		{
			if (time_1ms_diag_auto >= 5000U)
			{
				time_1ms_diag_auto = 0;
				time_1ms_diag_auto_flag = 0;

				AAF_Tx_Position = DIAG_MODE_AUTO;

				aaf_action = FLAP_STOP;
				aaf_action_complete_chk = FLAP_STOP;
				aaf_step = AAF_WAITING;
			}
		}
		else
		{
			aaf_action = FLAP_STOP;
			aaf_action_complete_chk = FLAP_STOP;
			aaf_step = AAF_WAITING;
		}

		break;

	default:
		break;
	}
}

static void Lin_rx_data_chk(void)
{
	LIN_Diag_Rx();
	ReqRespAAFID = (unsigned int)((Slave_RxData1[0] & 0xC0U) >> 6U);
	if (AAF_LIN_ChkSum_CHK == PASS)
	{
		// ReqRespAAFID = (unsigned int)((Slave_RxData1[0] & 0xC0U) >> 6U);

		if (protection_function == OFF) //
		{
			if (lin_rx_chk_flag == ON)
			{
				for (int i = 0; i < 8; i++)
				{
					ID_chk_rxdata[i] = Slave_RxData1[i];
				}

				lin_rx_chk_flag = OFF;
				lin_rx_pass_flag = PASS;
			}

			if (lin_rx_pass_flag == PASS)
			{
				if (AAFx_InitStatus != DURING_INITIALIZATION)
				{
					ReqAAF3DiagMode = (unsigned int)((ID_chk_rxdata[0] & 0x30U) >> 4U);
					ReqAAF2DiagMode = (unsigned int)((ID_chk_rxdata[0] & 0x0CU) >> 2U);
					ReqAAF1DiagMode = (unsigned int)(ID_chk_rxdata[0] & 0x03U);
					AAF1_TargetPosition = (unsigned int)(ID_chk_rxdata[1] & 0x7FU);
					AAF2_TargetPosition = (unsigned int)(ID_chk_rxdata[2] & 0x7FU);
					AAF3_TargetPosition = (unsigned int)(ID_chk_rxdata[3] & 0x7FU);
					EngRunSta = (unsigned int)((ID_chk_rxdata[4] & 0x30U) >> 4U);
					HevRdy = (unsigned int)((ID_chk_rxdata[4] & 0x0CU) >> 2U);
					AAF_LINOut = (unsigned int)(ID_chk_rxdata[4] & 0x03U);

					//-----------------------------------------development START----------------------------------------
					// AAF_Init_Flag = (unsigned int)((ID_chk_rxdata[1] & 0x80U) >> 7U);
					// AAF_Flap_Fixation_Test_Mode = (unsigned int)((ID_chk_rxdata[2] & 0x80U) >> 7U);
					// AAF_Maximum_Torque_Test_Mode = (unsigned int)((ID_chk_rxdata[3] & 0x80U) >> 7U);
					// Re_Init_check= (unsigned int)((ID_chk_rxdata[4] & 0x80U) >> 7U);
					// AAF_Init_Flag = 0;
					// AAF_Flap_Fixation_Test_Mode = 0;
					// AAF_Maximum_Torque_Test_Mode = 0;
					/*if (Re_Init_check == 0x01)
					{
						Re_Init_check_flag = 1;
					}
					else
					{
						Re_Init_check_flag = 0;
					}
					if ((Re_Init_check_flag == 1)&&(aaf_step == AAF_WAITING)&&(Re_Init_check_prev==0))
					{
						Re_Init();
					}
					Re_Init_check_prev = Re_Init_check;*/

					//-----------------------------------------development END----------------------------------------
					//-----------------------------------------DIAG development START----------------------------------------
					if (AAFx_Index == AAF_1)
					{
						if (ReqAAF1DiagMode == 0x00)
						{
							if (AAF1_TargetPosition == 0x00)
							{
								AAF1_TargetPosition_select = OPEN;
							}
							else if (AAF1_TargetPosition == 0x64)
							{
								AAF1_TargetPosition_select = CLOSE;
							}
							else if (AAF1_TargetPosition == 0x7F)
							{
								AAF1_TargetPosition_select = UNKOWN_POSITION;
							}
						}
						else if (ReqAAF1DiagMode == 0x01)
						{
							AAF1_TargetPosition_select = DIAG_MODE_AUTO;
						}
						else if (ReqAAF1DiagMode == 0x02)
						{
							AAF1_TargetPosition_select = DIAG_MODE_OPEN;
						}
						else if (ReqAAF1DiagMode == 0x03)
						{
							AAF1_TargetPosition_select = DIAG_MODE_CLOSE;
						}
						else
						{
						}

						if (aaf_action_complete_chk == FLAP_STOP)
						{
							if ((AAF1_TargetPosition_select != AAF_Tx_Position) && (AAF_Tx_Position != UNKOWN_POSITION) && (AAF1_TargetPosition_select != UNKOWN_POSITION))
							{
								step_start_flag = ON;
							}
						}
						else if (aaf_action == DIAG_MODE_AUTO)
						{
							if (AAF1_TargetPosition_select != aaf_action)
							{
								step_start_flag = ON;
							}
						}
						else
						{
						}

						switch (AAF1_TargetPosition_select)
						{
						case CLOSE:
							lin_aaf_command = CLOSE;
							break;
						case OPEN_1ST:
							lin_aaf_command = OPEN_1ST;
							break;
						case OPEN_2ND:
							lin_aaf_command = OPEN_2ND;
							break;
						case OPEN:
							lin_aaf_command = OPEN;
							break;
						case DIAG_MODE_OPEN:
							lin_aaf_command = DIAG_MODE_OPEN;
							break;
						case DIAG_MODE_CLOSE:
							lin_aaf_command = DIAG_MODE_CLOSE;
							break;
						case DIAG_MODE_AUTO:
							lin_aaf_command = DIAG_MODE_AUTO;
							break;
						case UNKOWN_POSITION:
							lin_aaf_command = UNKOWN_POSITION;
							break;
						default:
							break;
						}
					}
					else if (AAFx_Index == AAF_2)
					{
						if (ReqAAF2DiagMode == 0x00)
						{
							if (AAF2_TargetPosition == 0x00)
							{
								AAF2_TargetPosition_select = OPEN;
							}
							else if (AAF2_TargetPosition == 0x64)
							{
								AAF2_TargetPosition_select = CLOSE;
							}
							else if (AAF2_TargetPosition == 0x7F)
							{
								AAF2_TargetPosition_select = UNKOWN_POSITION;
							}
						}
						else if (ReqAAF2DiagMode == 0x01)
						{
							AAF2_TargetPosition_select = DIAG_MODE_AUTO;
						}
						else if (ReqAAF2DiagMode == 0x02)
						{
							AAF2_TargetPosition_select = DIAG_MODE_OPEN;
						}
						else
						{
							AAF2_TargetPosition_select = DIAG_MODE_CLOSE;
						}

						if (aaf_action_complete_chk == FLAP_STOP)
						{
							if ((AAF2_TargetPosition_select != AAF_Tx_Position) && (AAF_Tx_Position != UNKOWN_POSITION) && (AAF2_TargetPosition_select != UNKOWN_POSITION))
							{
								step_start_flag = ON;
							}
						}
						else if (aaf_action == DIAG_MODE_AUTO)
						{
							if (AAF2_TargetPosition_select != aaf_action)
							{
								step_start_flag = ON;
							}
						}
						else
						{
						}

						switch (AAF2_TargetPosition_select)
						{
						case CLOSE:
							lin_aaf_command = CLOSE;
							break;
						case OPEN_1ST:
							lin_aaf_command = OPEN_1ST;
							break;
						case OPEN_2ND:
							lin_aaf_command = OPEN_2ND;
							break;
						case OPEN:
							lin_aaf_command = OPEN;
							break;
						case DIAG_MODE_OPEN:
							lin_aaf_command = DIAG_MODE_OPEN;
							break;
						case DIAG_MODE_CLOSE:
							lin_aaf_command = DIAG_MODE_CLOSE;
							break;
						case DIAG_MODE_AUTO:
							lin_aaf_command = DIAG_MODE_AUTO;
							break;
						case UNKOWN_POSITION:
							lin_aaf_command = UNKOWN_POSITION;
							break;
						default:
							break;
						}
					}
					else if (AAFx_Index == AAF_3)
					{
						if (ReqAAF3DiagMode == 0x00)
						{
							if (AAF3_TargetPosition == 0x00)
							{
								AAF3_TargetPosition_select = OPEN;
							}
							else if (AAF3_TargetPosition == 0x64)
							{
								AAF3_TargetPosition_select = CLOSE;
							}
							else if (AAF3_TargetPosition == 0x7F)
							{
								AAF3_TargetPosition_select = UNKOWN_POSITION;
							}
						}
						else if (ReqAAF3DiagMode == 0x01)
						{
							AAF3_TargetPosition_select = DIAG_MODE_AUTO;
						}
						else if (ReqAAF3DiagMode == 0x02)
						{
							AAF3_TargetPosition_select = DIAG_MODE_OPEN;
						}
						else
						{
							AAF3_TargetPosition_select = DIAG_MODE_CLOSE;
						}

						if (aaf_action_complete_chk == FLAP_STOP)
						{
							if ((AAF3_TargetPosition_select != AAF_Tx_Position) && (AAF_Tx_Position != UNKOWN_POSITION) && (AAF3_TargetPosition_select != UNKOWN_POSITION))
							{
								step_start_flag = ON;
							}
						}
						else if (aaf_action == DIAG_MODE_AUTO)
						{
							if (AAF3_TargetPosition_select != aaf_action)
							{
								step_start_flag = ON;
							}
						}
						else
						{
						}

						switch (AAF3_TargetPosition_select)
						{
						case CLOSE:
							lin_aaf_command = CLOSE;
							break;
						case OPEN_1ST:
							lin_aaf_command = OPEN_1ST;
							break;
						case OPEN_2ND:
							lin_aaf_command = OPEN_2ND;
							break;
						case OPEN:
							lin_aaf_command = OPEN;
							break;
						case DIAG_MODE_OPEN:
							lin_aaf_command = DIAG_MODE_OPEN;
							break;
						case DIAG_MODE_CLOSE:
							lin_aaf_command = DIAG_MODE_CLOSE;
							break;
						case DIAG_MODE_AUTO:
							lin_aaf_command = DIAG_MODE_AUTO;
							break;
						case UNKOWN_POSITION:
							lin_aaf_command = UNKOWN_POSITION;
							break;
						default:
							break;
						}
					}
					//-----------------------------------------DIAG development END----------------------------------------
					//-----------------------------------------development START----------------------------------------
					/*if ((AAF_Init_Flag_tog == OFF) && (AAF_Init_Flag == ON))
					{
						if ((AAF1_TargetPosition == 0x7F) || (AAF2_TargetPosition == 0x7F) || (AAF3_TargetPosition == 0x7F))
						{
							wake_up_motor_range_init_chk = 0;
							evrdy_on_flag = OFF;
							Re_Init();
						}
						AAF_Init_Flag_tog = ON;
					}
					else if ((AAF_Init_Flag_tog == ON) && (AAF_Init_Flag == OFF))
					{
						AAF_Init_Flag_tog = OFF;
					}
					else
					{
					}

					if ((AAF_Flap_Fixation_Test_Mode_tog == OFF) && (AAF_Flap_Fixation_Test_Mode == ON))
					{
						AAF_Flap_Fixation_Test_Mode_tog = ON;
					}
					else if ((AAF_Flap_Fixation_Test_Mode_tog == ON) && (AAF_Flap_Fixation_Test_Mode == OFF))
					{
						wake_up_motor_range_init_chk = 0;
						evrdy_on_flag = OFF;
						Re_Init();
						AAF_Flap_Fixation_Test_Mode_tog = OFF;
					}
					else
					{
					}

					if ((AAF_Maximum_Torque_Test_Mode_tog == OFF) && (AAF_Maximum_Torque_Test_Mode == ON))
					{
						AAF_Maximum_Torque_Test_Mode_tog = ON;
					}
					else if ((AAF_Maximum_Torque_Test_Mode_tog == ON) && (AAF_Maximum_Torque_Test_Mode == OFF))
					{
						wake_up_motor_range_init_chk = 0;
						evrdy_on_flag = OFF;
						Re_Init();
						AAF_Maximum_Torque_Test_Mode_tog = OFF;
					}
					else
					{
					}*/
					//-----------------------------------------development END----------------------------------------
				}
				else if (AAFx_InitStatus == DURING_INITIALIZATION)
				{
					ReqRespAAFID = (unsigned int)((ID_chk_rxdata[0] & 0xC0U) >> 6U);
					EngRunSta = (unsigned int)((ID_chk_rxdata[4] & 0x30U) >> 4U);
					HevRdy = (unsigned int)((ID_chk_rxdata[4] & 0x0CU) >> 2U);
					AAF_LINOut = (unsigned int)(ID_chk_rxdata[4] & 0x03U);
				}
				else
				{
				}
			}
		}
		else if (protection_function == ON)
		{
			if (lin_rx_chk_flag == ON)
			{
				for (int i = 0; i < 8; i++)
				{
					ID_chk_rxdata[i] = Slave_RxData1[i];
				}

				lin_rx_chk_flag = OFF;
				lin_rx_pass_flag = PASS;
			}

			if (lin_rx_pass_flag == PASS)
			{
				AAF_ProtectionMode_Rx = (unsigned int)((ID_chk_rxdata[7] & 0x40U) >> 6U);
			}
		}
		else
		{
		}
	}
}

static void Lin_tx_data_chk(void)
{

	/*
	Slave_TxData[0] = (uint8_t)((AAFx_Type << 7) | AAF_Tx_Position);
	Slave_TxData[1] = (uint8_t)((TotalNumOfAAF << 6) | (AAFx_Index << 4) | AAFx_InitStatus);
	Slave_TxData[2] = (uint8_t)((AAF_ProtectionMode_Tx << 7) | (AAFx_Over_Volt << 6) | (AAFx_Low_Volt << 5) | (AAFx_Motor_Fault << 4) | (AAFx_Circuit_Short << 2) | (AAFx_Circuit_Open << 1));
	*/
	/*
		if (init_move_step >= 15U)
		{
			Slave_TxData[3] = (uint8_t)((0xF0U) | (stall_test_mode << 2) | Calibration_chk);
		}
		else
		{
			Slave_TxData[3] = (uint8_t)((init_move_step << 4) | (wakeup_chk << 3) | (stall_test_mode << 2) | Calibration_chk);
		}	// wakeup chk ?占쏙옙 ?占쏙옙?占쏙옙?占쏙옙

		Slave_TxData[4] = (uint8_t)((protection_Mode_step << 4) | current_value);
	*/
	/*
		if ((rx_16bit_spi[9] & 0xFFU) >= 200U)
		{
			if(fail_safety_step <= 15U)
			{
				Slave_TxData[4] = (uint8_t)((fail_safety_step << 4) | 0x02);
			}
			else if(fail_safety_step >= 16U)
			{
				Slave_TxData[4] = 0xF2U;
			}
			else
			{

			}

			Slave_TxData[5] = (uint8_t)(((((rx_16bit_spi[9] & 0xFFU) - 200) / 10) << 4) | ((rx_16bit_spi[9] & 0xFFU) % 10));
		}
		else if ((rx_16bit_spi[9] & 0xFFU) >= 100U)
		{
			if(fail_safety_step <= 15U)
			{
				Slave_TxData[4] = (uint8_t)((fail_safety_step << 4) | 0x01U);
			}
			else if(fail_safety_step >= 16U)
			{
				Slave_TxData[4] = 0xF1U;
			}
			else
			{

			}

			Slave_TxData[5] = (uint8_t)(((((rx_16bit_spi[9] & 0xFFU) - 100) / 10) << 4) | ((rx_16bit_spi[9] & 0xFFU) % 10));
		}
		else
		{
			if(fail_safety_step <= 15U)
			{
				Slave_TxData[4] = (uint8_t)((fail_safety_step << 4) | 0x00U);
			}
			else if(fail_safety_step >= 16U)
			{
				Slave_TxData[4] = 0xF0U;
			}
			else
			{

			}

			Slave_TxData[5] = (uint8_t)((((rx_16bit_spi[9] & 0xFFU) / 10) << 4) | ((rx_16bit_spi[9] & 0xFFU) % 10));
		}
	*/

	/*if (motor_stall_value >= 200U)
	{
		if(fail_safety_step <= 15U)
		{
			Slave_TxData[4] = (uint8_t)((fail_safety_step << 4) | 0x02);
		}
		else if(fail_safety_step >= 16U)
		{
			Slave_TxData[4] = 0xF2U;
		}
		else
		{

		}

		Slave_TxData[5] = (uint8_t)((((motor_stall_value - 200) / 10) << 4) | (motor_stall_value % 10));
	}
	else if (motor_stall_value >= 100U)
	{
		if(fail_safety_step <= 15U)
		{
			Slave_TxData[4] = (uint8_t)((fail_safety_step << 4) | 0x01U);
		}
		else if(fail_safety_step >= 16U)
		{
			Slave_TxData[4] = 0xF1U;
		}
		else
		{

		}

		Slave_TxData[5] = (uint8_t)((((motor_stall_value - 100) / 10) << 4) | (motor_stall_value % 10));
	}
	else
	{
		if(fail_safety_step <= 15U)
		{
			Slave_TxData[4] = (uint8_t)((fail_safety_step << 4) | 0x00U);
		}
		else if(fail_safety_step >= 16U)
		{
			Slave_TxData[4] = 0xF0U;
		}
		else
		{

		}

		Slave_TxData[5] = (uint8_t)(((motor_stall_value / 10) << 4) | (motor_stall_value % 10));
	}*/
	switch (AAF_Tx_Position)
	{
	case CLOSE:
		AAF_Tx_Position_LIN = 0x64;
		AAFx_Mode = 0x00;
		Diag_Mode_chk = 0;
		break;
	case OPEN:
		AAF_Tx_Position_LIN = 0x00;
		AAFx_Mode = 0x00;
		Diag_Mode_chk = 0;
		break;
	case DIAG_MODE_OPEN:
		AAF_Tx_Position_LIN = 0x7F;
		AAFx_Mode = 0x02;
		Diag_Mode_chk = 1;
		break;
	case DIAG_MODE_CLOSE:
		AAF_Tx_Position_LIN = 0x7F;
		AAFx_Mode = 0x03;
		Diag_Mode_chk = 1;
		break;
	case DIAG_MODE_AUTO:
		AAF_Tx_Position_LIN = 0x7F;
		AAFx_Mode = 0x01;
		Diag_Mode_chk = 1;
		break;
	case UNKOWN_POSITION:
		AAF_Tx_Position_LIN = 0x7F;
		break;
	default:
		break;
	}

	if (((AAFx_Mode == 0x01) || (AAFx_Mode == 0x02) || (AAFx_Mode == 0x03)) && (Diag_Mode_chk == 1))
	{
		if ((ReqAAF1DiagMode == 0) && (ReqAAF2DiagMode == 0) && (ReqAAF3DiagMode == 0))
		{
			Re_Init();
		}
	}

	Slave_TxData[0] = (uint8_t)((AAFx_Type << 7) | AAF_Tx_Position_LIN);
	Slave_TxData[1] = (uint8_t)((TotalNumOfAAF << 6) | (AAFx_Index << 4) | (AAFx_Mode << 2) | AAFx_InitStatus);
	Slave_TxData[2] = (uint8_t)((1U << 7) | (AAFx_Over_Volt << 6) | (AAFx_Low_Volt << 5) | (AAFx_Motor_Fault << 4) | (1U << 3) | (AAFx_Circuit_Short << 2) | (AAFx_Circuit_Open << 1) | 1U);
	Slave_TxData[3] = (uint8_t)((AAFx_SNSR_SCG << 7) | (AAFx_SNSR_SCB << 6) | (AAFx_SNSR_OC << 5) | (AAFx_Position_Status << 2) | AAFx_ErrorStatus);
	Slave_TxData[4] = (uint8_t)((0x1F << 3) | (TotalNumOfAAFSensor));
	Slave_TxData[5] = (uint8_t)((AAFx_SNSR4_Position << 6) | (AAFx_SNSR3_Position << 4) | (AAFx_SNSR2_Position << 2) | AAFx_SNSR1_Position);

	Req_ChkSum_Tx = (unsigned int)((16 - ((((Slave_TxData[0] / 16) + (Slave_TxData[1] / 16) + (Slave_TxData[2] / 16) + (Slave_TxData[3] / 16) + (Slave_TxData[4] / 16) + (Slave_TxData[5] / 16) + Req_Alive_Tx) + ((Slave_TxData[0] % 16) + (Slave_TxData[1] % 16) + (Slave_TxData[2] % 16) + (Slave_TxData[3] % 16) + (Slave_TxData[4] % 16) + (Slave_TxData[5] % 16))) & 0x0F) & 0x0F));

	Slave_TxData[6] = (uint8_t)((Req_ChkSum_Tx << 4) | Req_Alive_Tx);
	LIN_Diag_Action();

	lin_rx_pass_flag = WAITING;
}

static void MCU_sleep(void)
{
	power_chk = Normal_Shutdown;
	First_Powerchk = 1;
	if (step_check_flag == 1)
	{
		Flash_memory_write();
	}
	DRV8899_Sleep();
	Lin_Transceiver_Off();
	SPI_select_pin_Off();

	R_PORT_ResetAltFunc(Port10, 10, Output);
	R_PORT_ResetAltFunc(Port10, 9, Input);

	PORT.P10 &= ~_PORT_Pn10_OUTPUT_HIGH; // MCU_LIN_Tx_Low Sleep go

	R_Config_INTC_Create();
	// R_Config_INTC_INTP5_Start();

	R_Config_CSIH0_Stop();
	R_Config_ADCA0_Halt();
	R_Config_TAUD0_13_Stop();
	R_Config_TAUD0_3_Stop();
	time_1ms_spi_flag = 0;
	time_1ms_spi = 0;

	R_CGC_Create_sleepmode();

	R_Config_STBC_Prepare_Deep_Stop_Mode();
	R_Config_STBC_Start_Deep_Stop_Mode();
}

static void SPI_chk(void)
{
	if ((voltage_status_change == ON) && (motor_start == OFF) && (time_1ms_spi >= 20U))
	{
		PORT.P8 &= ~_PORT_Pn0_OUTPUT_HIGH;

		if (voltage_status_spi == HIGH_VOLTAGE_1ST)
		{
			R_Config_CSIH0_Send_Receive(&tx_16bit_spi_current_limit[9], 1, &rx_16bit_spi[3], _CSIH_SELECT_CHIP_0);
			motor_cw_stall_value = MOTOR_CW_STALL_CHK_VALUE_HIGH_VOLTAGE_1ST;
			motor_ccw_stall_value = MOTOR_CCW_STALL_CHK_VALUE_HIGH_VOLTAGE_1ST;
			// current_value = 9;
		}
		else if (voltage_status_spi == NORMAL_VOLTAGE)
		{
			R_Config_CSIH0_Send_Receive(&tx_16bit_spi_current_limit[9], 1, &rx_16bit_spi[3], _CSIH_SELECT_CHIP_0);
			motor_cw_stall_value = MOTOR_CW_STALL_CHK_VALUE_NORMAL_VOLTAGE;
			motor_ccw_stall_value = MOTOR_CCW_STALL_CHK_VALUE_NORMAL_VOLTAGE;
			// current_value = 9;
		}
		else if (voltage_status_spi == LOW_VOLTAGE_1ST)
		{
			R_Config_CSIH0_Send_Receive(&tx_16bit_spi_current_limit[9], 1, &rx_16bit_spi[3], _CSIH_SELECT_CHIP_0);
			motor_cw_stall_value = MOTOR_CW_STALL_CHK_VALUE_LOW_VOLTAGE_1ST;
			motor_ccw_stall_value = MOTOR_CCW_STALL_CHK_VALUE_LOW_VOLTAGE_1ST;
			// current_value = 9;
		}

		voltage_status_change_complete = WAIT;

		time_1ms_volt_stat_chg_wait_flag = 1;

		spi_action_step = 1;

		voltage_status_change = OFF;
	}
	else if ((motor_start == ON) && (time_1ms_spi >= 2U) && (time_1us_motor <= (STEP_TIME_1250RPM / 2)) && (spi_action_step == 0U))
	{
		PORT.P8 &= ~_PORT_Pn0_OUTPUT_HIGH;

		R_Config_CSIH0_Send_Receive(&rx_16bit_spi_id[9], 1, &rx_16bit_spi[9], _CSIH_SELECT_CHIP_0);

		spi_action_step = 1;
	}
	else if ((time_1ms_spi >= 50U) && (time_1us_motor == 0U) && (spi_action_step == 0U))
	{

		// Current_limiting_select();

		if (voltage_status_change == OFF)
		{
			PORT.P8 &= ~_PORT_Pn0_OUTPUT_HIGH;

			R_Config_CSIH0_Send_Receive(&rx_16bit_spi_id[9], 1, &rx_16bit_spi[9], _CSIH_SELECT_CHIP_0);

			spi_action_step = 1;
		}
	}
	else
	{
	}

	if (spi_action_step == 1U)
	{
		time_1ms_spi_error_chk_flag = 1;

		if ((spi_receive_flag >= 1U) && (spi_send_flag >= 1U))
		{
			spi_receive_flag = 0;
			spi_send_flag = 0;

			PORT.P8 |= _PORT_Pn0_OUTPUT_HIGH;

			time_1ms_spi_error_chk = 0;
			time_1ms_spi_error_chk_flag = 0;

			spi_action_step = 2;
		}

		if (time_1ms_spi_error_chk >= 100U)
		{
			spi_action_step = 2;
			// DRV_Off();
			spi_fail = 1;
			time_1ms_spi_error_chk = 0;
			time_1ms_spi_error_chk_flag = 0;
		}
	}
	else if (spi_action_step == 2U)
	{
		time_1us_spi_flag = 1;

		if (time_1us_spi >= 2U)
		{
			time_1us_spi_flag = 0;
			time_1us_spi = 0;

			spi_action_step = 3;
		}
	}
	else if (spi_action_step == 3U)
	{
		motor_stall_value = (unsigned int)(rx_16bit_spi[9] & 0xFFU);
		motor_open_load = (unsigned int)(rx_16bit_spi[9] & 0x100U);
		AAF_OverCurrent = (unsigned int)(rx_16bit_spi[9] & 0x800U);

		time_1ms_spi = 0;

		if (AAF_Maximum_Torque_Test_Mode == OFF)
		{
			Stall_chk();
		}
		else
		{
			motor_stall_flag = MOTOR_NORMAL;
		}

		spi_action_step = 0;
	}
	else
	{
	}
}

static void Stall_chk(void)
{
	if (motor_start == ON)
	{
		if (dir_state == CLOSE)
		{
			if (AAF_location_type == RH_TYPE)
			{
				if ((motor_stall_value <= motor_ccw_stall_value) || (motor_stall_value >= MOTOR_CCW_STALL_CHK_HIGH_VALUE))
				{

					if ((stall_chk_time_1ms >= STALL_CHK_WAIT_TIME) || (stall_test_mode == 1U))
					{
						stall_cnt++;
					}

					if (stall_cnt >= STALL_CNT_DEFAULT + STALL_CNT_COMPARISON_VAL)
					{
						motor_stall_flag = MOTOR_STALL;
						stall_cnt = STALL_CNT_DEFAULT;
					}
				}
				else
				{
					motor_stall_flag = MOTOR_NORMAL;

					stall_cnt = STALL_CNT_DEFAULT;
					// stall_cnt--;

					if (stall_cnt < STALL_CNT_DEFAULT)
					{
						stall_cnt = STALL_CNT_DEFAULT;
					}
				}
			}
			else if (AAF_location_type == LH_TYPE)
			{
				if ((motor_stall_value <= motor_cw_stall_value) || (motor_stall_value >= MOTOR_CW_STALL_CHK_HIGH_VALUE))
				{

					if ((stall_chk_time_1ms >= STALL_CHK_WAIT_TIME) || (stall_test_mode == 1U))
					{
						stall_cnt++;
					}

					if (stall_cnt >= STALL_CNT_DEFAULT + STALL_CNT_COMPARISON_VAL)
					{
						motor_stall_flag = MOTOR_STALL;
						stall_cnt = STALL_CNT_DEFAULT;
					}
				}
				else
				{
					motor_stall_flag = MOTOR_NORMAL;

					stall_cnt = STALL_CNT_DEFAULT;
					// stall_cnt--;

					if (stall_cnt < STALL_CNT_DEFAULT)
					{
						stall_cnt = STALL_CNT_DEFAULT;
					}
				}
			}
			else
			{
			}
		}
		else if (dir_state == OPEN)
		{
			if (AAF_location_type == RH_TYPE)
			{
				if ((motor_stall_value <= motor_cw_stall_value) || (motor_stall_value >= MOTOR_CW_STALL_CHK_HIGH_VALUE))
				{
					if ((stall_chk_time_1ms >= STALL_CHK_WAIT_TIME) || (stall_test_mode == 1U))
					{
						stall_cnt++;
					}

					if (stall_cnt >= STALL_CNT_DEFAULT + STALL_CNT_COMPARISON_VAL)
					{
						motor_stall_flag = MOTOR_STALL;
						stall_cnt = STALL_CNT_DEFAULT;
					}
				}
				else
				{
					motor_stall_flag = MOTOR_NORMAL;

					stall_cnt = STALL_CNT_DEFAULT;
					// stall_cnt--;

					if (stall_cnt < STALL_CNT_DEFAULT)
					{
						stall_cnt = STALL_CNT_DEFAULT;
					}
				}
			}
			else if (AAF_location_type == LH_TYPE)
			{
				if ((motor_stall_value <= motor_ccw_stall_value) || (motor_stall_value >= MOTOR_CCW_STALL_CHK_HIGH_VALUE))
				{

					if ((stall_chk_time_1ms >= STALL_CHK_WAIT_TIME) || (stall_test_mode == 1U))
					{
						stall_cnt++;
					}

					if (stall_cnt >= STALL_CNT_DEFAULT + STALL_CNT_COMPARISON_VAL)
					{
						motor_stall_flag = MOTOR_STALL;
						stall_cnt = STALL_CNT_DEFAULT;
					}
				}
				else
				{
					motor_stall_flag = MOTOR_NORMAL;

					stall_cnt = STALL_CNT_DEFAULT;
					// stall_cnt--;

					if (stall_cnt < STALL_CNT_DEFAULT)
					{
						stall_cnt = STALL_CNT_DEFAULT;
					}
				}
			}
			else
			{
			}
		}
		else
		{
		}
	}
}

static void CHK_external_factors(void)
{
	if ((time_1ms_external_10s_chk >= 10000U) && (fail_safety_1_cycle_flag == OFF))
	{
		if ((fail_safety_step == 0) && (antipinch_previous_action == INITIALIZATION))
		{
			fail_safety_flag = ON;
		}
		else if (fail_safety_step == 1U)
		{
			fail_safety_step = 2;
		}
		else if (fail_safety_step == 7U)
		{
			fail_safety_step = 8;
		}
		else if (fail_safety_step == 13U)
		{
			fail_safety_step = 14;
		}
		else
		{
		}
	}
}

static void Fail_safety_mode(void)
{
	if (fail_safety_flag == ON)
	{
		switch (fail_safety_step)
		{
		case 0:									  // 1st cycle start
			DRV_Off();							  // drv of
			motor_start = OFF;					  // step stop
			time_1ms_external_10s_chk_flag = OFF; // 10s chk timer off
			time_1ms_external_10s_chk = 0;
			aaf_action = FLAP_STOP;
			aaf_action_complete_chk = FLAP_STOP;

			time_1ms_stall_chk = 0;		 // test
			time_1ms_stall_chk_flag = 0; // test

			stall_chk_cnt = 0;		// stall reset
			stall_chk_time_1ms = 0; // stall reset

			motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset

			aaf_step = AAF_INITIALIZATION;		  // MCU is reset, AAF is initialized.
			aaf_init_step = START_INITIALIZATION; // MCU is reset, AAF is initialized.
			AAF_Tx_Position = UNKOWN_POSITION;
			AAFx_Position_Status = Unknown_Status;
			lin_aaf_command = OPEN;
			softstart_complete = OFF;
			motor_step_value = STEP_TIME_1000RPM;

			if (AAFx_Index == AAF_1)
			{
				ID_chk_rxdata[4] = 0x03;
			}
			else if (AAFx_Index == AAF_2)
			{
				ID_chk_rxdata[4] = 0x18;
			}
			else if (AAFx_Index == AAF_3)
			{
				ID_chk_rxdata[5] = 0x03;
			}
			else
			{
			}

			init_move_step = START_INITIALIZATION_OPEN; // MCU is reset, AAF is initialized.

			fail_safety_step = 1;
			fail_safety_1_cycle_flag = ON;
			break;

		case 1:
			if (init_move_step == 19U)
			{
				init_move_step = 0;
				aaf_init_step = NORMAL_INITIALIZATION;
				fail_safety_step = 0;
				fail_safety_flag = OFF;
				fail_safety_1_cycle_flag = OFF;
				AAFx_ErrorStatus = No_ErrorStatus;
			}
			break;

		case 2:
			if (lin_aaf_command == OPEN) // lin open command chk
			{
				DRV8899_Wakeup();
				aaf_action = OPEN;
				motor_stall_flag = MOTOR_NORMAL;				  // stall reset
				stall_chk_time_1ms = 0;							  // stall reset
				motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset
				fail_safety_step = 3;
			}
			else if (lin_aaf_command == CLOSE) // lin close command chk
			{
				DRV8899_Wakeup();
				aaf_action = CLOSE;
				motor_stall_flag = MOTOR_NORMAL;				  // stall reset
				stall_chk_time_1ms = 0;							  // stall reset
				motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset
				fail_safety_step = 3;
			}
			else
			{
			}
			break;

		case 3:
			if (aaf_action == OPEN)
			{
				Motor_Open();				// dir OPEN
				DRV_On();					// drv on
				motor_start = ON;			// step start
				time_1ms_init_chk_flag = 1; // test
				fail_safety_step = 4;
			}
			else if (aaf_action == CLOSE)
			{
				Motor_Close();				// dir CLOSE
				DRV_On();					// drv on
				motor_start = ON;			// step start
				time_1ms_init_chk_flag = 1; // test
				fail_safety_step = 4;
			}
			else
			{
			}
			break;

		case 4:
			if ((motor_stall_flag == MOTOR_STALL) || (time_1ms_init_chk >= 4500U))
			{
				DRV_Off();
				motor_start = OFF;
				stall_chk_cnt = 0;
				stall_chk_time_1ms = 0; // stall reset
				fail_safety_step = 5;
				aaf_action = FLAP_STOP;
				time_1ms_init_chk_flag = 0;
				time_1ms_init_chk = 0;
				softstart_complete = OFF;
				motor_step_value = STEP_TIME_1000RPM;

				if (aaf_action == OPEN)
				{
					AAF_Tx_Position = OPEN;
				}
				else if (aaf_action == CLOSE)
				{
					AAF_Tx_Position = CLOSE;
				}
			}
			break;

		case 5:
			time_1ms_3minute_flag = 1;
			antipinch_step = 0;

			if (time_1s_3minute >= MINUTE_3)
			{
				time_1ms_3minute_flag = 0;
				time_1s_3minute = 0;
				time_1ms_3minute = 0;
				fail_safety_step = 6;
			}
			break;

		case 6:									  // 1st cycle start
			DRV_Off();							  // drv of
			motor_start = OFF;					  // step stop
			time_1ms_external_10s_chk_flag = OFF; // 10s chk timer off
			time_1ms_external_10s_chk = 0;
			aaf_action = FLAP_STOP;
			aaf_action_complete_chk = FLAP_STOP;

			time_1ms_stall_chk = 0;		 // test
			time_1ms_stall_chk_flag = 0; // test

			stall_chk_cnt = 0;		// stall reset
			stall_chk_time_1ms = 0; // stall reset

			motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset

			aaf_step = AAF_INITIALIZATION;		  // MCU is reset, AAF is initialized.
			aaf_init_step = START_INITIALIZATION; // MCU is reset, AAF is initialized.
			AAF_Tx_Position = UNKOWN_POSITION;	  //
			AAFx_Position_Status = Unknown_Status;
			lin_aaf_command = OPEN;
			softstart_complete = OFF;
			motor_step_value = STEP_TIME_1000RPM;

			init_move_step = START_INITIALIZATION_OPEN; // MCU is reset, AAF is initialized.

			fail_safety_step = 7;
			fail_safety_1_cycle_flag = ON;
			break;

		case 7:
			if (init_move_step == 19U)
			{
				init_move_step = 0;
				aaf_init_step = NORMAL_INITIALIZATION;
				fail_safety_step = 0;
				fail_safety_flag = OFF;
				fail_safety_1_cycle_flag = OFF;
				AAFx_ErrorStatus = No_ErrorStatus;
			}
			break;

		case 8:
			if (lin_aaf_command == OPEN) // lin open command chk
			{
				DRV8899_Wakeup();
				aaf_action = OPEN;
				motor_stall_flag = MOTOR_NORMAL;				  // stall reset
				stall_chk_time_1ms = 0;							  // stall reset
				motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset
				fail_safety_step = 9;
			}
			else if (lin_aaf_command == CLOSE) // lin close command chk
			{
				DRV8899_Wakeup();
				aaf_action = CLOSE;
				motor_stall_flag = MOTOR_NORMAL;				  // stall reset
				stall_chk_time_1ms = 0;							  // stall reset
				motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset
				fail_safety_step = 9;
			}
			else
			{
			}
			break;

		case 9:
			if (aaf_action == OPEN)
			{
				Motor_Open();				// dir OPEN
				DRV_On();					// drv on
				motor_start = ON;			// step start
				time_1ms_init_chk_flag = 1; // test
				fail_safety_step = 10;
			}
			else if (aaf_action == CLOSE)
			{
				Motor_Close();				// dir CLOSE
				DRV_On();					// drv on
				motor_start = ON;			// step start
				time_1ms_init_chk_flag = 1; // test
				fail_safety_step = 10;
			}
			else
			{
			}
			break;

		case 10:
			if ((motor_stall_flag == MOTOR_STALL) || (time_1ms_init_chk >= 4500U))
			{
				DRV_Off();
				motor_start = OFF;
				stall_chk_cnt = 0;
				stall_chk_time_1ms = 0; // stall reset
				fail_safety_step = 11;
				aaf_action = FLAP_STOP;
				time_1ms_init_chk_flag = 0;
				time_1ms_init_chk = 0;
				softstart_complete = OFF;
				motor_step_value = STEP_TIME_1000RPM;

				if (aaf_action == OPEN)
				{
					AAF_Tx_Position = OPEN;
				}
				else if (aaf_action == CLOSE)
				{
					AAF_Tx_Position = CLOSE;
				}
			}
			break;

		case 11:
			time_1ms_3minute_flag = 1;

			if (time_1s_3minute >= MINUTE_3)
			{
				time_1ms_3minute_flag = 0;
				time_1s_3minute = 0;
				time_1ms_3minute = 0;
				fail_safety_step = 12;
			}
			break;

		case 12:								  // 1st cycle start
			DRV_Off();							  // drv of
			motor_start = OFF;					  // step stop
			time_1ms_external_10s_chk_flag = OFF; // 10s chk timer off
			time_1ms_external_10s_chk = 0;
			aaf_action = FLAP_STOP;
			aaf_action_complete_chk = FLAP_STOP;

			time_1ms_stall_chk = 0;		 // test
			time_1ms_stall_chk_flag = 0; // test

			stall_chk_cnt = 0;		// stall reset
			stall_chk_time_1ms = 0; // stall reset

			motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset

			aaf_step = AAF_INITIALIZATION;		  // MCU is reset, AAF is initialized.
			aaf_init_step = START_INITIALIZATION; // MCU is reset, AAF is initialized.
			AAF_Tx_Position = UNKOWN_POSITION;
			AAFx_Position_Status = Unknown_Status;
			lin_aaf_command = OPEN;
			softstart_complete = OFF;
			motor_step_value = STEP_TIME_1000RPM;
			evrdy_on_flag = OFF;
			init_move_step = START_INITIALIZATION_OPEN; // MCU is reset, AAF is initialized.

			fail_safety_step = 13;
			fail_safety_1_cycle_flag = ON;
			break;

		case 13:
			if (init_move_step == 19U)
			{
				init_move_step = 0;
				aaf_init_step = NORMAL_INITIALIZATION;
				fail_safety_step = 0;
				fail_safety_flag = OFF;
				fail_safety_1_cycle_flag = OFF;
				AAFx_ErrorStatus = No_ErrorStatus;
			}
			break;

		case 14:

			DRV8899_Wakeup();
			aaf_action = OPEN;
			motor_stall_flag = MOTOR_NORMAL;				  // stall reset
			stall_chk_cnt = 0;								  // stall reset
			stall_chk_time_1ms = 0;							  // stall reset
			motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset
			Motor_Open();									  // dir OPEN
			DRV_On();										  // drv on
			motor_start = ON;								  // step start
			time_1ms_init_chk_flag = 1;						  // test
			fail_safety_step = 15;
			break;

		case 15:
			if ((motor_stall_flag == MOTOR_STALL) || (time_1ms_init_chk >= 4500U))
			{
				DRV_Off();
				motor_start = OFF;
				stall_chk_cnt = 0;
				stall_chk_time_1ms = 0; // stall reset
				fail_safety_step = 16;
				aaf_action = FLAP_STOP;
				time_1ms_init_chk_flag = 0;
				time_1ms_init_chk = 0;
				softstart_complete = OFF;
				motor_step_value = STEP_TIME_1000RPM;
			}
			break;

		case 16:
			AAF_Tx_Position = UNKOWN_POSITION;
			AAFx_Position_Status = Unknown_Status;
			AAFx_InitStatus = ABNORMAL_FINISHED_INITIALIZATION;
			motor_fault_chk = 1;
			break;

		default:
			break;
		}
	}
}

static void Initialize_variables(void)
{
	motor_start = OFF;
	time_1us_motor_flag = 0;
	time_1us_motor = 0;

	motor_stall_flag = MOTOR_NORMAL;
	motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE;
	time_1ms_spi = 0;
	time_1ms_init_chk_flag = 0;
	time_1ms_init_chk = 0;
	init_move_step = 0;
	time_1ms_init_move_flag = 0;
	time_1ms_init_move = 0;

	fail_safety_1_cycle_flag = OFF;
	time_1ms_external_10s_chk_flag = OFF;
	time_1ms_external_10s_chk = 0;

	step_start_flag = OFF;
	time_1ms_stall_chk_flag = OFF;

	spi_action_step = 0;
	adc_chk_ok_flag = 0;

	spi_receive_flag = 0;
	spi_send_flag = 0;

	time_1us_spi_flag = 0;
	time_1us_spi = 0;

	stall_chk_cnt = 0;
	stall_chk_time_1ms = 0; // stall reset
	fail_safety_1_cycle_flag = OFF;
	fail_safety_flag = OFF;
	fail_safety_step = 0;

	time_1ms_3minute_flag = 0;
	time_1s_3minute = 0;
	time_1ms_3minute = 0;

	timer_1ms_lin_bus_inactive = 0;

	lin_bus_inactive_flag = OFF;

	// wake_up_flag = 1;

	lin_rx_chk_flag = OFF;
}

static void Lin_bus_chk(void)
{
	if ((timer_1ms_lin_bus_inactive >= LIN_BUS_CHK_TIME_4_SEC) && (lin_bus_inactive_flag == OFF) && (LIN_Short_Ok == 0))
	{
		lin_bus_inactive_flag = ON;

		DRV_Off();							  // drv of
		motor_start = OFF;					  // step stop
		time_1ms_external_10s_chk_flag = OFF; // 10s chk timer off
		time_1ms_external_10s_chk = 0;
		aaf_action = FLAP_STOP;
		aaf_action_complete_chk = FLAP_STOP;
		softstart_complete = OFF;
		motor_step_value = STEP_TIME_1000RPM;

		time_1ms_stall_chk = 0;		 // test
		time_1ms_stall_chk_flag = 0; // test

		lin_sleep_step = 0;

		wake_up_motor_range_init_chk = 0;

		aaf_step = AAF_WAITING;
	}
}

static void Protection_Mode(void)
{
	if (stall_test_mode == 0U)
	{
		switch (protection_Mode_step)
		{
		case 0:
			DRV_Off();
			motor_start = OFF;
			stall_chk_cnt = 0;
			stall_chk_time_1ms = 0; // stall reset
			protection_Mode_step = 1;
			time_1ms_protection_mode_flag = 1;
			aaf_action = FLAP_STOP;
			softstart_complete = OFF;
			break;

		case 1:
			if (time_1ms_protection_mode >= 100U)
			{
				time_1ms_protection_mode_flag = 0;
				time_1ms_protection_mode = 0;
				protection_Mode_step = 2;
			}
			break;

		case 2:
			// if(((adc_avr >= ADC_UNDER_VOLTAGE_7V) && (adc_avr <= ADC_UNDER_VOLTAGE_9V)) || ((adc_avr >= ADC_OVER_VOLTAGE_16V) && (adc_avr <= ADC_OVER_VOLTAGE_18V)) || (AAF_ProtectionMode_Rx == ON))
			// if(((adc_avr >= ADC_OVER_VOLTAGE_16V) && (adc_avr <= ADC_OVER_VOLTAGE_18V)) || (AAF_ProtectionMode_Rx == ON) && (AAF_Tx_Position != OPEN))	// ???  ?   ?  ?   紐⑦꽣 ?  ?   紐삵븿
			if ((AAF_Tx_Position != OPEN))
			{
				Motor_Open();					 // dir OPEN
				DRV_On();						 // drv on
				motor_start = ON;				 // step start
				time_1ms_stall_chk_flag = ON;	 // test
				motor_stall_flag = MOTOR_NORMAL; // stall reset
				stall_chk_time_1ms = 0;			 // stall reset
				protection_Mode_step = 3;
				time_1ms_init_chk_flag = 1; // test
			}
			else
			{
				time_1ms_init_chk_flag = 0;
				protection_Mode_step = 4;
			}
			break;

		case 3:
			// if ((motor_stall_flag == MOTOR_STALL) || (time_1ms_init_chk >= 4500U))
			if ((step_position <= step_position_open + limit_step_position) || (motor_stall_flag == MOTOR_STALL))
			{
				DRV_Off();
				motor_start = OFF;
				stall_chk_cnt = 0;
				stall_chk_time_1ms = 0; // stall reset
				time_1ms_init_chk_flag = 0;
				time_1ms_init_chk = 0;
				AAF_Tx_Position = UNKOWN_POSITION;
				AAFx_InitStatus = DURING_INITIALIZATION;
				AAFx_Position_Status = Unknown_Status;
				softstart_complete = OFF;
				motor_step_value = STEP_TIME_1000RPM;
				protection_Mode_step = 4;
			}
			break;

		case 4:
			AAF_Tx_Position = UNKOWN_POSITION;
			AAFx_InitStatus = DURING_INITIALIZATION;
			AAFx_Position_Status = Unknown_Status;
			break;

		default:
			break;
		}
	}
}

static void Voltage_chk_current_limit_init(void)
{
	if (voltage_status_spi == 0U) //	6 ohm
	{
		if (adc_avr <= ADC_VOLTAGE_13_5V)
		{
			voltage_status_spi = LOW_VOLTAGE_1ST;
		}
		else if (adc_avr >= ADC_VOLTAGE_15V)
		{
			voltage_status_spi = HIGH_VOLTAGE_1ST;
		}
		else
		{
			voltage_status_spi = NORMAL_VOLTAGE;
		}

		voltage_status_change = ON;
	}
}

static void Current_limiting_select(void)
{
	if (voltage_status_spi == LOW_VOLTAGE_1ST)
	{
		if (adc_avr >= ADC_VOLTAGE_15V)
		{
			voltage_status_spi = HIGH_VOLTAGE_1ST;
			voltage_status_change = ON;
		}
		else if (adc_avr >= ADC_VOLTAGE_13_7V)
		{
			voltage_status_spi = NORMAL_VOLTAGE;
			voltage_status_change = ON;
		}
		else
		{
		}
	}
	else if (voltage_status_spi == NORMAL_VOLTAGE)
	{
		if (adc_avr <= ADC_VOLTAGE_13_5V)
		{
			voltage_status_spi = LOW_VOLTAGE_1ST;
			voltage_status_change = ON;
		}
		else if (adc_avr >= ADC_VOLTAGE_15V)
		{
			voltage_status_spi = HIGH_VOLTAGE_1ST;
			voltage_status_change = ON;
		}
		else
		{
		}
	}
	else if (voltage_status_spi == HIGH_VOLTAGE_1ST)
	{
		if (adc_avr <= ADC_VOLTAGE_13_7V)
		{
			voltage_status_spi = LOW_VOLTAGE_1ST;
			voltage_status_change = ON;
		}
		else if (adc_avr <= ADC_VOLTAGE_14_8V)
		{
			voltage_status_spi = NORMAL_VOLTAGE;
			voltage_status_change = ON;
		}
		else
		{
		}
	}
	else
	{
	}
}

static void Lin_sleep(void)
{
	switch (lin_sleep_step)
	{
	case 0:
		DRV_Off();
		motor_start = OFF;
		stall_chk_cnt = 0;
		stall_chk_time_1ms = 0;
		softstart_complete = OFF;
		motor_step_value = STEP_TIME_1000RPM;
		timer_1ms_init_fail_chk_flag = 0;
		timer_1ms_init_fail_chk = 0;

		lin_sleep_step = 1;
		break;

	case 1:
		timer_1ms_lin_sleep_flag = 1;

		if (timer_1ms_lin_sleep >= 50U)
		{
			timer_1ms_lin_sleep_flag = 0;
			timer_1ms_lin_sleep = 0;

			lin_sleep_step = 2;
		}
		break;

	case 2:
		if (AAF_LINOut == 0x00U)
		{
			if (lin_aaf_command == OPEN)
			{
				DRV8899_Wakeup();
				aaf_action = OPEN;
			}
			else if (lin_aaf_command == CLOSE)
			{
				DRV8899_Wakeup();
				aaf_action = CLOSE;
			}
			else
			{
			}
			lin_sleep_step = 3;
		}
		else if (AAF_LINOut == 0x01U)
		{
			DRV8899_Wakeup();
			aaf_action = OPEN;

			lin_sleep_step = 3;
		}
		else
		{
		}
		break;

	case 3:
		if (aaf_action == OPEN)
		{
			Motor_Open();	  // dir CLOSE
			DRV_On();		  // drv on
			motor_start = ON; // step start

			motor_stall_flag = MOTOR_NORMAL;				  // stall reset
			stall_chk_time_1ms = 0;							  // stall reset
			motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset
			time_1ms_spi = 0;

			// timer_1ms_lin_sleep_flag = 1; // test

			lin_sleep_step = 4;
		}
		else if (aaf_action == CLOSE)
		{
			Motor_Close();	  // dir CLOSE
			DRV_On();		  // drv on
			motor_start = ON; // step start

			motor_stall_flag = MOTOR_NORMAL;				  // stall reset
			stall_chk_time_1ms = 0;							  // stall reset
			motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset
			time_1ms_spi = 0;

			// timer_1ms_lin_sleep_flag = 1; // test

			lin_sleep_step = 4;
		}
		else
		{
			lin_sleep_step = 5;
		}
		break;

	case 4:
		if (((aaf_action == OPEN) && (step_position <= (step_position_open + limit_step_position)))) //
		{
			DRV_Off();
			motor_start = OFF;
			stall_chk_cnt = 0;
			stall_chk_time_1ms = 0; // stall reset
			softstart_complete = OFF;
			motor_step_value = STEP_TIME_1000RPM;
			AAF_Tx_Position = OPEN;
			AAFx_Position_Status = Open_Status;
			Tx_position_complete_chk();
			aaf_step = FINISHED_OPERATE;
			// timer_1ms_lin_sleep_flag = 0;
			// timer_1ms_lin_sleep = 0;
			lin_sleep_step = 5;
		}
		else if (((aaf_action == CLOSE) && (step_position >= (step_position_close - limit_step_position))))
		{
			DRV_Off();
			motor_start = OFF;
			stall_chk_cnt = 0;
			stall_chk_time_1ms = 0; // stall reset
			softstart_complete = OFF;
			motor_step_value = STEP_TIME_1000RPM;
			AAF_Tx_Position = CLOSE;
			AAFx_Position_Status = Close_Status;
			AAFx_InitStatus = NORMAL_INITIALIZATION;
			Tx_position_complete_chk();
			aaf_step = FINISHED_OPERATE;
			// timer_1ms_lin_sleep_flag = 0;
			// timer_1ms_lin_sleep = 0;
			lin_sleep_step = 5;
		}
		else if (motor_stall_flag == MOTOR_STALL)
		{
			DRV_Off();
			motor_start = OFF;
			stall_chk_cnt = 0;
			stall_chk_time_1ms = 0; // stall reset
			softstart_complete = OFF;
			motor_step_value = STEP_TIME_1000RPM;
			aaf_step = AAF_INITIALIZATION;
			aaf_init_step = WAIT_INITIALIZATION;
			AAF_Tx_Position = UNKOWN_POSITION;
			AAFx_Position_Status = Unknown_Status;
			AAFx_InitStatus = DURING_INITIALIZATION;
			motor_stall_flag = MOTOR_NORMAL; // stall reset
			lin_sleep_step = 5;
		}
		else
		{
		}
		break;
	case 5:
		if ((AAF_Tx_Position == UNKOWN_POSITION) || (AAFx_Position_Status == Unknown_Status) || (AAFx_InitStatus == DURING_INITIALIZATION))
		{
			aaf_init_step = WAIT_INITIALIZATION;
			AAF_Tx_Position = UNKOWN_POSITION;
			AAFx_Position_Status = Unknown_Status;
			AAFx_InitStatus = DURING_INITIALIZATION;
		}
		MCU_sleep();
		break;

	default:
		break;
	}
}

static void Antipinch_move(void)
{
	if (antipinch_action_on == ON)
	{
		AAFx_Position_Status = Unknown_Status;
		AAF_Tx_Position = UNKOWN_POSITION;
		AAFx_InitStatus = DURING_INITIALIZATION;
		if (antipinch_previous_action == OPEN)
		{
			switch (antipinch_step)
			{
			case 0:
				Motor_Close();				// dir CLOSE
				DRV_On();					// drv on
				motor_start = ON;			// step start
				time_1ms_init_chk_flag = 1; // test

				aaf_action = CLOSE;
				AAFx_ErrorStatus = Open_ErrorStatus;
				time_1ms_spi = 0;				 // stall reset
				motor_stall_flag = MOTOR_NORMAL; // stall reset
				antipinch_step = 1;
				break;

			case 1:
				if ((motor_stall_flag == MOTOR_STALL) || (time_1ms_init_chk >= 4500U)) //
				{
					DRV_Off();
					motor_start = OFF;
					stall_chk_cnt = 0;
					stall_chk_time_1ms = 0; // stall reset
					softstart_complete = OFF;
					motor_step_value = STEP_TIME_1000RPM;
					aaf_action = FLAP_STOP;
					time_1ms_init_chk_flag = 0;
					time_1ms_init_chk = 0;
					if (step_position >= step_position_close)
					{
						step_position = step_position_close;
					}

					antipinch_step = 2;
				}
				break;

			case 2:
				time_1ms_antipinch_flag = 1;

				if (time_1ms_antipinch >= 3000U)
				{
					time_1ms_antipinch_flag = 0;
					time_1ms_antipinch = 0;

					antipinch_step = 3;
				}
				break;

			case 3:
				Motor_Open();	  // dir OPEN
				DRV_On();		  // drv on
				motor_start = ON; // step start

				stall_chk_time_1ms = 0;							  // stall reset
				motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset
				motor_stall_flag = MOTOR_NORMAL;				  // stall reset
				time_1ms_spi = 0;
				aaf_action = OPEN;

				time_1ms_init_chk_flag = 1; // test

				antipinch_step = 4;

				break;

			case 4:
				if ((aaf_action == OPEN) && (step_position <= step_position_open + limit_step_position))
				{
					DRV_Off();
					motor_start = OFF;
					if (Diag_Mode != 0)
					{
						if (lin_aaf_command == DIAG_MODE_OPEN)
						{
							aaf_action = DIAG_MODE_OPEN;
							aaf_step = AAF_WAITING;
							AAFx_ErrorStatus = No_ErrorStatus;
							AAFx_InitStatus = NORMAL_FINISHED_INITIALIZATION;
							Tx_position_complete_chk();
						}
						else if (lin_aaf_command == DIAG_MODE_AUTO)
						{
							aaf_action = DIAG_MODE_AUTO;
							aaf_step = AAF_WAITING;
							AAFx_ErrorStatus = No_ErrorStatus;
							AAFx_InitStatus = NORMAL_FINISHED_INITIALIZATION;
							Tx_position_complete_chk();
						}
						else
						{
						}
					}
					else
					{
						AAF_Tx_Position = OPEN;
						AAFx_InitStatus = NORMAL_FINISHED_INITIALIZATION;
						AAFx_ErrorStatus = No_ErrorStatus;
						Tx_position_complete_chk();
					}

					fail_safety_flag = OFF;
					antipinch_action_on = OFF;

					stall_chk_cnt = 0;
					stall_chk_time_1ms = 0;
					softstart_complete = OFF;
					motor_step_value = STEP_TIME_1000RPM;

					antipinch_previous_action = ANTIWAIT;
					antipinch_step = 0;

					aaf_step = FINISHED_OPERATE;
				}
				else if ((motor_stall_flag == MOTOR_STALL) || (time_1ms_init_chk >= 4500U))
				{
					DRV_Off();
					motor_start = OFF;
					stall_chk_cnt = 0;
					stall_chk_time_1ms = 0;
					softstart_complete = OFF;
					motor_step_value = STEP_TIME_1000RPM;

					// antipinch_step = 5;//??

					time_1ms_init_chk_flag = 0;
					time_1ms_init_chk = 0;

					antipinch_previous_action = INITIALIZATION;
					antipinch_action_on = OFF;

					if (Diag_Mode != 0)
					{
						AAFx_InitStatus = ABNORMAL_FINISHED_INITIALIZATION;
						motor_fault_chk = 1;
						Diag_Mode = 0;
					}
					else
					{
						fail_safety_flag = ON;
						fail_safety_step = 5;
					}
				}
				break;

			default:
				break;
			}
		}
		else if (antipinch_previous_action == CLOSE)
		{
			AAFx_InitStatus = DURING_INITIALIZATION;
			switch (antipinch_step)
			{
			case 0:
				Motor_Open(); // dir CLOSE
				DRV_On();	  // drv on

				stall_chk_time_1ms = 0; // stall reset

				motor_start = ON;				 // step start
				time_1ms_init_chk_flag = 1;		 // test
				motor_stall_flag = MOTOR_NORMAL; // stall reset
				time_1ms_spi = 0;

				aaf_action = OPEN;
				AAFx_ErrorStatus = Close_ErrorStatus;
				antipinch_step = 1;
				break;

			case 1:
				if ((motor_stall_flag == MOTOR_STALL) || (time_1ms_init_chk >= 4500U))
				{
					DRV_Off();
					motor_start = OFF;
					stall_chk_cnt = 0;
					stall_chk_time_1ms = 0; // stall reset
					softstart_complete = OFF;
					motor_step_value = STEP_TIME_1000RPM;
					time_1ms_init_chk_flag = 0;
					time_1ms_init_chk = 0;
					if (step_position <= step_position_open)
					{
						step_position = step_position_open;
					}
					aaf_action = FLAP_STOP;

					antipinch_step = 2;
				}
				break;

			case 2:
				time_1ms_antipinch_flag = 1;

				if (time_1ms_antipinch >= 3000U)
				{
					time_1ms_antipinch_flag = 0;
					time_1ms_antipinch = 0;
					antipinch_step = 3;

					time_1ms_init_chk_flag = 0; // test
					time_1ms_init_chk = 0;		// test
				}
				break;

			case 3:
				Motor_Close();					 // dir OPEN
				DRV_On();						 // drv on
				motor_start = ON;				 // step start
				motor_stall_flag = MOTOR_NORMAL; // stall reset

				stall_chk_time_1ms = 0;							  // stall reset
				motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset
				time_1ms_spi = 0;

				aaf_action = CLOSE;

				time_1ms_init_chk_flag = 1; // test

				antipinch_step = 4;

				break;

			case 4:
				if ((aaf_action == CLOSE) && (step_position >= step_position_close - limit_step_position))
				{
					DRV_Off();
					motor_start = OFF;
					if (Diag_Mode != 0)
					{
						if (lin_aaf_command == DIAG_MODE_CLOSE)
						{
							aaf_action = DIAG_MODE_CLOSE;
							aaf_step = AAF_WAITING;
							AAFx_ErrorStatus = No_ErrorStatus;
							AAFx_InitStatus = NORMAL_FINISHED_INITIALIZATION;
							Tx_position_complete_chk();
						}
						else if (lin_aaf_command == DIAG_MODE_AUTO)
						{
							aaf_action = DIAG_MODE_AUTO;
							aaf_step = AAF_WAITING;
							AAFx_ErrorStatus = No_ErrorStatus;
							AAFx_InitStatus = NORMAL_FINISHED_INITIALIZATION;
							Tx_position_complete_chk();
						}
						else
						{
						}
					}
					else
					{
						AAF_Tx_Position = CLOSE;
						AAFx_InitStatus = NORMAL_FINISHED_INITIALIZATION;
						AAFx_ErrorStatus = No_ErrorStatus;
						Tx_position_complete_chk();
					}
					fail_safety_flag = OFF;
					antipinch_action_on = OFF;

					stall_chk_cnt = 0;
					stall_chk_time_1ms = 0;
					softstart_complete = OFF;
					motor_step_value = STEP_TIME_1000RPM;

					antipinch_previous_action = ANTIWAIT;
					antipinch_step = 0;

					aaf_step = FINISHED_OPERATE;
				}
				else if ((motor_stall_flag == MOTOR_STALL) || (time_1ms_init_chk >= 4500U))
				{
					DRV_Off();
					motor_start = OFF;
					stall_chk_cnt = 0;
					stall_chk_time_1ms = 0;
					softstart_complete = OFF;
					motor_step_value = STEP_TIME_1000RPM;

					time_1ms_init_chk_flag = 0;
					time_1ms_init_chk = 0;

					aaf_action = FLAP_STOP;

					antipinch_step = 5;
				}

				break;

			case 5:
				Motor_Open();				// dir CLOSE
				DRV_On();					// drv on
				motor_start = ON;			// step start
				time_1ms_init_chk_flag = 1; // test
				aaf_action = OPEN;
				motor_stall_flag = MOTOR_NORMAL; // stall reset
				time_1ms_spi = 0;

				stall_chk_time_1ms = 0;							  // stall reset
				motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset

				antipinch_step = 6;
				break;

			case 6:
				if ((motor_stall_flag == MOTOR_STALL) || (time_1ms_init_chk >= 4500U))
				{
					DRV_Off();
					motor_start = OFF;
					stall_chk_cnt = 0;
					stall_chk_time_1ms = 0; // stall reset
					softstart_complete = OFF;
					motor_step_value = STEP_TIME_1000RPM;

					aaf_action = FLAP_STOP;
					time_1ms_init_chk_flag = 0;
					time_1ms_init_chk = 0;

					antipinch_step = 7;

					// fail_safety_flag = ON;
					antipinch_action_on = OFF;

					antipinch_previous_action = INITIALIZATION;

					time_1ms_external_10s_chk_flag = OFF; // 10s chk timer off
					time_1ms_external_10s_chk = 0;
					aaf_action = FLAP_STOP;
					aaf_action_complete_chk = FLAP_STOP;

					time_1ms_stall_chk = 0;		 // test
					time_1ms_stall_chk_flag = 0; // test

					motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset

					/*aaf_step = AAF_INITIALIZATION;		  // MCU is reset, AAF is initialized.
					aaf_init_step = START_INITIALIZATION; // MCU is reset, AAF is initialized.
					AAF_Tx_Position = UNKOWN_POSITION;
					AAFx_Position_Status = Unknown_Status;
					lin_aaf_command = OPEN;*/
					// if ((aaf_action == DIAG_MODE_OPEN) || (aaf_action == DIAG_MODE_CLOSE) || (aaf_action == DIAG_MODE_AUTO))
					if (Diag_Mode != 0)
					{
						AAFx_InitStatus = ABNORMAL_FINISHED_INITIALIZATION;
						motor_fault_chk = 1;
						Diag_Mode = 0;
						// AAFx_Motor_Fault = 1;
						//  protection_function = ON;
					}
					else
					{
						fail_safety_flag = ON;
						fail_safety_step = 5;
					}
				}
				break;

			default:
				break;
			}
		}
	}
}

static void Torque_Test_Mode(void)
{
	if ((AAFx_Index == AAF_1) && (ReqRespAAFID == AAF_1))
	{
		torque_test_position = AAF1_TargetPosition;
	}
	else if ((AAFx_Index == AAF_2) && (ReqRespAAFID == AAF_2))
	{
		torque_test_position = AAF2_TargetPosition;
	}
	else if ((AAFx_Index == AAF_3) && (ReqRespAAFID == AAF_3))
	{
		torque_test_position = AAF3_TargetPosition;
	}
	else
	{
		torque_test_position = WAITING;
	}

	switch (torque_test_position)
	{
	case 0x7F: // STOP
		DRV_Off();
		motor_start = OFF;
		break;
	case 0x00: // CLOSE
		Motor_Close();
		DRV_On();
		motor_start = ON;
		break;
	case 0x64: // OPEN
		Motor_Open();
		DRV_On();
		motor_start = ON;
		break;
	default:
		break;
	}
}
//-----------------------------------------development START----------------------------------------
static void IGN_On_Memory_read(void)
{
	step_position_close = close_memory_read;
	step_position_open = open_memory_read;
	step_position = now_step_memory_read;
	AAF_Tx_Position = position_memory_read;
	AAFx_Position_Status = position_status_memory_read;
	AAFx_InitStatus = AAFx_InitStatus_memory_read;
	limit_step_position = limit_memory_read;
	evrdy_on_flag = Initial_memory_read;
	DTC_Status |= DTC_memory_read;
	power_chk = power_chk_memory_read;
	First_Powerchk = First_Powerchk_memory_read;
}
/*static void Init_direction(void)
{
	if ((AAFx_Type == EXTERNAL_TYPE) && ((init_move_step == 0) || (init_move_step == 9)))
	{
		Motor_Open();	  // dir CLOSE
	}
	else if ((AAFx_Type == EXTERNAL_TYPE) && ((init_move_step == 6) || (init_move_step == 13)))
	{
		Motor_Close();
	}
	else if ((AAFx_Type == INTERNAL_TYPE) && ((init_move_step == 0) || (init_move_step == 9)))
	{
		Motor_Close();
	}
	else if ((AAFx_Type == INTERNAL_TYPE) && ((init_move_step == 6) || (init_move_step == 13)))
	{
		Motor_Open();
	}
	else
	{

	}
}*/
static void LIMP_HOME(void)
{

	if ((LIMP_HOME_Count < 80) && (AAF_LIN_ChkSum_CHK == PASS) && (LIMP_HOME_step == 2))
	{
		LIMP_HOME_Count = 0;
		LIMP_HOME_step = 0;
		Re_Init();
	}
	switch (LIMP_HOME_step)
	{
	case 0:
		if ((LIMP_HOME_Count >= 80))
		{
			if ((AAF_Tx_Position != OPEN))
			{
				Motor_Open();					 // dir OPEN
				DRV_On();						 // drv on
				motor_start = ON;				 // step start
				time_1ms_stall_chk_flag = ON;	 // test
				motor_stall_flag = MOTOR_NORMAL; // stall reset
				stall_chk_time_1ms = 0;			 // stall reset
				LIMP_HOME_step = 1;
				time_1ms_init_chk_flag = 1; // test
			}
			else
			{
				LIMP_HOME_step = 1;
			}
		}
		break;
	case 1:
		if ((step_position <= (step_position_open + limit_step_position)) || (motor_stall_flag == MOTOR_STALL)) //
		{
			DRV_Off();
			motor_start = OFF;
			stall_chk_cnt = 0;
			stall_chk_time_1ms = 0; // stall reset
			time_1ms_init_chk_flag = 0;
			time_1ms_init_chk = 0;
			AAF_Tx_Position = UNKOWN_POSITION;
			AAFx_Position_Status = Unknown_Status;
			AAFx_InitStatus = DURING_INITIALIZATION;
			aaf_step = FINISHED_OPERATE;

			LIMP_HOME_step = 2;
		}
		else if ((AAF_Tx_Position == OPEN) && (AAFx_Position_Status == Open_Status))
		{
			AAF_Tx_Position = UNKOWN_POSITION;
			AAFx_Position_Status = Unknown_Status;
			AAFx_InitStatus = DURING_INITIALIZATION;
			aaf_step = FINISHED_OPERATE;
			LIMP_HOME_step = 2;
		}
		else
		{
		}
		break;
	case 2:
		if (AAFx_Position_Status == Open_Status)
		{
			Flash_memory_write();
		}

		break;

	default:
		break;
	}
}
static void step_check(void)
{
	if (((step_position_close - step_position_open) <= STEP_POSITION_MINIMUM_RANGE) || ((step_position_close - step_position_open) > STEP_POSITION_MAXIMUM_RANGE) || (step_position == REFERENCE_POSITION) || (step_position < (step_position_open + limit_step_position)) || (step_position > (step_position_close - limit_step_position)) || (step_position_close == 0) || (step_position_open == 0) || (limit_step_position == 0) || (evrdy_on_flag == OFF) || (step_position > POSITION_MAXIMUM_RANGE) || (step_position_open > POSITION_MAXIMUM_RANGE) || (step_position_close > POSITION_MAXIMUM_RANGE) || (limit_step_position > LIMITSTEP_MAXIMUM_RANGE) || (AAF_Tx_Position == UNKOWN_POSITION) || (AAFx_InitStatus == ABNORMAL_FINISHED_INITIALIZATION) || (AAFx_Position_Status == FlapMoving_Status) || (AAFx_Position_Status == Unknown_Status) || (power_chk == Shutdown_Check))
	{
		Re_Init();
	}
	else
	{
		if (AAFx_Position_Status == Open_Status)
		{
			aaf_step = AAF_WAITING;
			aaf_init_step = NORMAL_INITIALIZATION;
			AAF_Tx_Position = OPEN;
			// AAFx_Position_Status = Open_Status;
			AAFx_InitStatus = NORMAL_FINISHED_INITIALIZATION;
			evrdy_on_flag = ON;
			aaf_action_complete_chk = FLAP_STOP;
			antipinch_previous_action = ANTIWAIT;
		}
		else if (AAFx_Position_Status == Close_Status)
		{
			aaf_step = AAF_WAITING;
			aaf_init_step = NORMAL_INITIALIZATION;
			AAF_Tx_Position = CLOSE;
			// AAFx_Position_Status = Close_Status;
			AAFx_InitStatus = NORMAL_FINISHED_INITIALIZATION;
			evrdy_on_flag = ON;
			aaf_action_complete_chk = FLAP_STOP;
			antipinch_previous_action = ANTIWAIT;
		}
		else
		{
		}
	}
}

static void LIN_Short_Chk(void)
{
	Short_chk = PORT.PPR8 & _PORT_Pn1_OUTPUT_HIGH;

	if (Short_chk == 0)
	{
		if (time_1ms_LIN_GndShort_flag == 0U)
		{
			time_1ms_LIN_GndShort = 0U;
			time_1ms_LIN_GndShort_flag = 1U;
		}
	}
	else
	{
		time_1ms_LIN_GndShort = 0U;
		time_1ms_LIN_GndShort_flag = 0U;
	}

	if (time_1ms_LIN_GndShort >= 4000)
	{
		LIN_Short_Ok = 1U;
	}

	if (LIN_Short_Ok == 1U)
	{
		switch (LIN_Short_Sleep)
		{
		case 0:
			aaf_action = OPEN;
			LIN_Short_Sleep = 1;
			break;
		case 1:
			Motor_Open();	  // dir CLOSE
			DRV_On();		  // drv on
			motor_start = ON; // step start

			motor_stall_flag = MOTOR_NORMAL;				  // stall reset
			stall_chk_time_1ms = 0;							  // stall reset
			motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset
			time_1ms_spi = 0;
			LIN_Short_Sleep = 2;
			break;
		case 2:
			if (((aaf_action == OPEN) && (step_position <= (step_position_open + limit_step_position)))) //
			{
				DRV_Off();
				motor_start = OFF;
				stall_chk_cnt = 0;
				stall_chk_time_1ms = 0; // stall reset
				softstart_complete = OFF;
				motor_step_value = STEP_TIME_1000RPM;
				AAF_Tx_Position = OPEN;
				AAFx_Position_Status = Open_Status;
				Tx_position_complete_chk();
				aaf_step = FINISHED_OPERATE;
				LIN_Short_Sleep = 3;
			}
			else if ((motor_stall_flag == MOTOR_STALL))
			{
				DRV_Off();
				motor_start = OFF;
				stall_chk_cnt = 0;
				stall_chk_time_1ms = 0; // stall reset
				softstart_complete = OFF;
				motor_step_value = STEP_TIME_1000RPM;
				AAF_Tx_Position = UNKOWN_POSITION;
				AAFx_Position_Status = Unknown_Status;
				Tx_position_complete_chk();
				aaf_step = FINISHED_OPERATE;
				LIN_Short_Sleep = 3;
			}
			else
			{
			}
			break;
		case 3:
			MCU_sleep();
			LIN_Short_Sleep = 4;
			break;

		default:
			break;
		}
	}
}

static void Error_FaultClear(void)
{
	PORT.P8 &= ~_PORT_Pn0_OUTPUT_HIGH;

	R_Config_CSIH0_Send_Receive(&fault_clear[0], 1, &rx_16bit_spi[9], _CSIH_SELECT_CHIP_0);

	while (1)
	{
		if ((spi_receive_flag == 1U) && (spi_send_flag == 1U))
		{
			spi_receive_flag = 0;
			spi_send_flag = 0;
			time_1ms_spi_error_chk = 0;
			time_1ms_spi_error_chk_flag = 0;
			break;
		}
	}

	PORT.P8 |= _PORT_Pn0_OUTPUT_HIGH;

	time_1us_spi_flag = 1;

	while (1)
	{
		if (time_1us_spi >= 10U)
		{
			time_1us_spi_flag = 0;
			time_1us_spi = 0;
			break;
		}
	}

	PORT.P8 &= ~_PORT_Pn0_OUTPUT_HIGH;

	R_Config_CSIH0_Send_Receive(&rx_16bit_spi_id[9], 1, &rx_16bit_spi[9], _CSIH_SELECT_CHIP_0);

	while (1)
	{
		if ((spi_receive_flag == 1U) && (spi_send_flag == 1U))
		{
			spi_receive_flag = 0;
			spi_send_flag = 0;
			time_1ms_spi_error_chk = 0;
			time_1ms_spi_error_chk_flag = 0;
			break;
		}
	}

	PORT.P8 |= _PORT_Pn0_OUTPUT_HIGH;

	time_1us_spi_flag = 1;

	while (1)
	{
		if (time_1us_spi >= 10U)
		{
			time_1us_spi_flag = 0;
			time_1us_spi = 0;
			break;
		}
	}
}
static void Motor_SPI_Init(void)
{
	for (int i = 0; i < 11; i++)
	{
		PORT.P8 &= ~_PORT_Pn0_OUTPUT_HIGH;

		R_Config_CSIH0_Send_Receive(&tx_16bit_spi[i], 1, &rx_16bit_spi[i], _CSIH_SELECT_CHIP_0);

		time_1ms_spi_error_chk_flag = 1;

		while (1)
		{
			if ((spi_receive_flag == 1U) && (spi_send_flag == 1U))
			{
				spi_receive_flag = 0;
				spi_send_flag = 0;
				time_1ms_spi_error_chk = 0;
				time_1ms_spi_error_chk_flag = 0;
				break;
			}

			if (time_1ms_spi_error_chk >= 100U)
			{
				time_1ms_spi_error_chk = 0;
				time_1ms_spi_error_chk_flag = 0;
				spi_fail = 1;
				break;
			}
		}

		PORT.P8 |= _PORT_Pn0_OUTPUT_HIGH;

		time_1us_spi_flag = 1;

		while (1)
		{
			if (time_1us_spi >= 2U)
			{
				time_1us_spi_flag = 0;
				time_1us_spi = 0;
				break;
			}
		}
	}

	for (int i = 0; i < 11; i++)
	{
		PORT.P8 &= ~_PORT_Pn0_OUTPUT_HIGH;

		R_Config_CSIH0_Send_Receive(&rx_16bit_spi_id[i], 1, &rx_16bit_spi[i], _CSIH_SELECT_CHIP_0);

		time_1ms_spi_error_chk_flag = 1;

		while (1)
		{
			if ((spi_receive_flag == 1U) && (spi_send_flag == 1U))
			{
				spi_receive_flag = 0;
				spi_send_flag = 0;
				time_1ms_spi_error_chk = 0;
				time_1ms_spi_error_chk_flag = 0;
				break;
			}

			if (time_1ms_spi_error_chk >= 100U)
			{
				time_1ms_spi_error_chk = 0;
				time_1ms_spi_error_chk_flag = 0;
				spi_fail = 1;
				break;
			}
		}

		PORT.P8 |= _PORT_Pn0_OUTPUT_HIGH;

		time_1us_spi_flag = 1;

		while (1)
		{
			if (time_1us_spi >= 2U)
			{
				time_1us_spi_flag = 0;
				time_1us_spi = 0;
				break;
			}
		}
	}
}
static void LIN_Diag_Rx(void)
{
	if (Slave_RxSwData1[0] == 0x26)
	{
		if ((Slave_RxSwData1[1] == 0x06) && (Slave_RxSwData1[2] == 0xB2) && (Slave_RxSwData1[3] == 0x01) && (Slave_RxSwData1[4] == 0x0A) && (Slave_RxSwData1[5] == 0x00) && (Slave_RxSwData1[6] == 0x0B) && (Slave_RxSwData1[7] == 0x00))
		{
			SW_Chk = 1U; // SW VER
		}
		else
		{
			SW_Chk = 3U;
		}
	}
	else if ((Slave_RxSwData1[0] == 0x00) && (Slave_RxSwData1[1] == 0xFF) && (Slave_RxSwData1[2] == 0xFF) && (Slave_RxSwData1[3] == 0xFF) && (Slave_RxSwData1[4] == 0xFF) && (Slave_RxSwData1[5] == 0xFF) && (Slave_RxSwData1[6] == 0xFF) && (Slave_RxSwData1[7] == 0xFF))
	{
		SW_Chk = 2U; // GO TO SLEEP
	}
	else
	{
		SW_Chk = 0U;
	}
}
static void LIN_Diag_Action(void)
{
	if (SW_Chk == 1U)
	{
		Slave_SwData[0] = (uint8_t)(0x26);
		Slave_SwData[1] = (uint8_t)(0x05);
		Slave_SwData[2] = (uint8_t)(0xF2);
		Slave_SwData[3] = (uint8_t)(0x01); // VECHILE
		Slave_SwData[4] = (uint8_t)(0x11); // ENGINE, MODEL
		Slave_SwData[5] = (uint8_t)(0x13);
		Slave_SwData[6] = (uint8_t)(0x61);
		Slave_SwData[7] = (uint8_t)(0xFF);
	}
	else if (SW_Chk == 2U)
	{
		MCU_sleep();
	}
	else if (SW_Chk == 3U)
	{
		Slave_SwData[0] = (uint8_t)(0x26);
		Slave_SwData[1] = (uint8_t)(0x03);
		Slave_SwData[2] = (uint8_t)(0x7F);
		Slave_SwData[3] = (uint8_t)(0xB2);
		Slave_SwData[4] = (uint8_t)(0x12);
		Slave_SwData[5] = (uint8_t)(0xFF);
		Slave_SwData[6] = (uint8_t)(0xFF);
		Slave_SwData[7] = (uint8_t)(0xFF);
	}
	else
	{
		// invalid
	}
}
static void AAF_communicate_chk(void)
{
	Lin_rx_data_chk();

	Lin_tx_data_chk();

	SPI_chk();

	Lin_bus_chk();
}
//-----------------------------------------development END----------------------------------------
/* End user code. Do not edit comment generated here */
