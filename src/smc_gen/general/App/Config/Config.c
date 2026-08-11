#include "Config.h"

/* 2.1 Communication Buffers (LIN / SPI) */
uint8_t GetIDbuffer;
uint8_t Slave_RxData1[8]; /*reception data store array*/
uint8_t Slave_TxData[8] = {
	0,
}; /*Transmission data store array*/
uint8_t ID_chk_rxdata[11] = {
	0,
};
uint8_t w_buff[24] = {
	0,
};
uint32_t r_buff[8] = {
	0,
};

uint8_t Slave_SwData[8] = {
	0,
}; /* Transmission data store array  */

uint8_t Slave_RxSwData1[8] = {
	0,
};

uint16_t tx_16bit_spi[11] = {
	0x4000, // [0] (R) FAULT Status
	0x4200, // [1] (R) DIAG Status 1
	0x4400, // [2] (R) DIAG Status 2
	0x0690, // [3] ?  瑜섏젣?
	0x080F, // [4] (RW)CTRL 2
	0x0A05, // [5] (RW)CTRL 3
	0x0C3E, // [6] (RW)CTRL 4	3E (open load on) 0C36 0C3A
	0x0E00, // [7] (RW)CTRL 5
	0x1000, // [8] (RW)CTRL 6	stall threshold
	0x5200, // [9] (R) CTRL 7	stall count
	0x5400	// [10](R) CTRL 8
};

uint16_t tx_16bit_spi_current_limit[16] = {
	0x0601,
	0x0611,
	0x0621,
	0x0631,
	0x0641,
	0x0651,
	0x0661,
	0x0671,
	0x0681,
	0x0690,
	0x06A1,
	0x06B1,
	0x06C1,
	0x06D1,
	0x06E1,
	0x06F1};

uint16_t tx_16bit_spi_slew_change[2] = {
	0x0690,
	0x0691};

uint16_t rx_16bit_spi_id[11] = {
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

uint16_t rx_16bit_spi[11] = {
	0,
};

uint16_t fault_clear[1] = {
	0x0CBE};

/* 2.2 Motor Control Variables */
unsigned int motor_start = OFF;
unsigned int motor_wait_chk = 0U;
unsigned long long step_position = 0U;
unsigned int dir_state = 0U;
unsigned int init_move_step = 0U;
unsigned int motor_open_load = 0U;
volatile unsigned int softstart_complete = 0U;
unsigned int motor_stall_value = 255U;
unsigned int motor_stall_flag = 0U;
unsigned int motor_cw_stall_value = 0U;
unsigned int motor_ccw_stall_value = 0U;
unsigned int step_start_flag = 0U;

/* 2.3 AAF Application Variables */
unsigned int aaf_step = 0U;
unsigned int aaf_action = 0U;
unsigned int aaf_init_step = 0U;
unsigned int aaf_action_complete_chk = 0U;
unsigned int flap_move = FLAP_STOP;
unsigned int step_position_open = 0U;
unsigned int step_position_close = 0U;
unsigned int limit_step_position = 0U;
unsigned int limit_step_position_close = 0U;
unsigned int open_1st_step_position = 0U;
unsigned int open_2nd_step_position = 0U;
unsigned int protection_function = 0U;
unsigned int protection_Mode_step = 0U;
unsigned int AAF_location_type = 0U;
unsigned int AAF_OverCurrent = 0U;
unsigned int AAF_LINOut = 0U;
unsigned int AAFx_Type = 0U;
unsigned int AAFx_InitStatus = 0U;
unsigned int AAFx_Index = 0U;
unsigned int TotalNumOfAAF = 0U;
unsigned int AAFx_Circuit_Open = 0U;
unsigned int AAFx_Circuit_Short = 0U;
unsigned int AAFx_Motor_Fault = 0U;
unsigned int AAFx_Low_Volt = 0U;
unsigned int AAFx_Over_Volt = 0U;

unsigned int AAF1_TargetPosition = 0U;
unsigned int AAF2_TargetPosition = 0U;
unsigned int AAF3_TargetPosition = 0U;
unsigned int AAF1_TargetPosition_select = 0U;
unsigned int AAF2_TargetPosition_select = 0U;
unsigned int AAF3_TargetPosition_select = 0U;

unsigned int AAF_ProtectionMode_Rx = 0U;
volatile unsigned int AAF_ProtectionMode_Tx = 0U;
unsigned int AAF_Tx_Position = 0U;
unsigned int AAF_Tx_Position_LIN = 0U;

unsigned int ReqRespAAFID = 0U;
unsigned int ReqAAF1DiagMode = 0U;
unsigned int ReqAAF2DiagMode = 0U;
unsigned int ReqAAF3DiagMode = 0U;
unsigned int EngRunSta = 0U;
unsigned int HevRdy = 0U;
unsigned int Req_ChkSum_Rx = 0U;
unsigned int Req_Alive_Rx = 0U;
unsigned int AAFx_Mode = 0U;

unsigned int AAFx_SNSR_SCG = 0U;
unsigned int AAFx_SNSR_SCB = 0U;
unsigned int AAFx_SNSR_OC = 0U;
unsigned int AAFx_Position_Status = 0U;
unsigned int AAFx_ErrorStatus = 0U;
unsigned int TotalNumOfAAFSensor = 0U;
unsigned int AAFx_SNSR1_Position = 0U;
unsigned int AAFx_SNSR2_Position = 0U;
unsigned int AAFx_SNSR3_Position = 0U;
unsigned int AAFx_SNSR4_Position = 0U;
unsigned int Req_ChkSum_Tx = 0U;
unsigned int Req_Alive_Tx = 0U;
unsigned int AmbTempSta = 0U;

/* 2.4 Communication Flags & Status */
volatile uint8_t error_status = 0U;
unsigned int lin_aaf_command = 0U;
unsigned int lin_rx_pass_flag = 0U;
unsigned int lin_rx_chk_flag = 0U;
unsigned int AAF_LIN_ChkSum_CHK = 0U;
unsigned int AAF_LIN_ChkSum_CHK_value = 0U;
unsigned char spi_send_flag = 0U;
unsigned char spi_receive_flag = 0U;
unsigned char spi_error_flag = 0U;
unsigned int spi_action_step = 0U;
char ret = 0;
unsigned int lin_bus_inactive_flag = 0U;
unsigned int lin_sleep_step = 0U;
unsigned int LIN_Nrst = 0U;
unsigned int LIN_Nrst_Check = 0U;
unsigned int LIN_NRST_Recognition_Chk = 0U;
unsigned int LIN_En_Check = 0U;
unsigned int LIN_Tx_Check = 0U;

/* 2.5 ADC & Power Variables */
uint16_t bat_adc = 0U;
unsigned int adc_chk[10] = {
	0,
};
unsigned int adc_sum = 0U;
unsigned int adc_avr = 0U;
unsigned int adc_chk_ok_flag = 0U;
unsigned int adc_chk_ready = 0U;
uint16_t scan_results[6] = {
	0,
};

unsigned int voltage_status_spi = 0U;
unsigned int voltage_status_change = 0U;
unsigned int voltage_status_change_complete = 0U;
unsigned int voltage_chk_delay_complete = 0U;
unsigned int Under_Voltage_Deceted = 0U;
unsigned int Over_Voltage_Deceted = 0U;
unsigned int First_Powerchk = 0U;

/* 2.6 Fault & Diagnosis */
unsigned int fail_safety_flag = 0U;
unsigned int fail_safety_step = 0U;
unsigned int stall_cnt = STALL_CNT_DEFAULT;
unsigned int Last_aaf_action = 0U;
unsigned int Sleep_Stall = 0U;
unsigned int evrdy_on_flag = 0U;
unsigned int diag_mode_auto_dir = 0U;
unsigned int diag_mode_auto_action = 0U;
unsigned int fdl_fail = 0U;
volatile unsigned int wake_up_motor_range_init_chk = 0U;
unsigned int LIMP_HOME_Count = 0U;
unsigned int LIMP_HOME_step = 0U;
uint8_t DTC_Status = 0U;
unsigned int motor_fault_chk = 0U;
unsigned int power_chk = 0U;
unsigned int Diag_Mode = 0U;
unsigned int Diag_Mode_chk = 0U;
unsigned int Short_Detected = 0U;
unsigned int Open_Detected = 0U;
unsigned int Short_fault_check = 0U;
unsigned int Open_fault_check = 0U;
unsigned int motor_Short_chk_count = 0U;
unsigned int motor_Open_chk_count = 0U;
unsigned int Freeze_Hold_Mode = 0U;
unsigned int Freeze_Hold_Mode_Recognition_Chk = 0U;
unsigned int Freeze_Hold_Mode_Count = 0U;
unsigned int Init_Fail_Chk = 0U;

/* 2.7 Antipinch */
unsigned int antipinch_step = 0U;
unsigned int antipinch_previous_action = INITIALIZATION;
unsigned int antipinch_action_on = 0U;

/* 2.8 Flash Memory Variables (Shadow RAM) */
unsigned int close_memory_write = 0U; // close step
unsigned int close_memory_read = 0U;
unsigned int open_memory_write = 0U; // open step
unsigned int open_memory_read = 0U;
unsigned int now_step_memory_write = 0U; // now step
unsigned int now_step_memory_read = 0U;
unsigned int position_memory_write = 0U; // AAFx_Position
unsigned int position_memory_read = 0U;
unsigned int Initial_memory_write = 0U; // evrdy flag
unsigned int Initial_memory_read = 0U;
unsigned int position_Initial_combined_read = 0U; // AAFx_Position+evrdy flag
unsigned int limit_memory_write = 0U;			 // limit step(5%)
unsigned int limit_memory_read = 0U;
unsigned int position_status_memory_write = 0U; // AAFx_Position_Status
unsigned int position_status_memory_read = 0U;
unsigned int AAFx_InitStatus_memory_write = 0U; // AAFx_InitStatus
unsigned int AAFx_InitStatus_memory_read = 0U;
unsigned int position_Initstatus_combined_read = 0U; // AAFx_Position_Status+AAFx_InitStatus
unsigned int DTC_memory_write = 0U;					// DTC
unsigned int DTC_memory_read = 0U;
unsigned int power_chk_memory_write = 0U; // power chk
unsigned int power_chk_memory_read = 0;
unsigned int First_Powerchk_memory_write = 0U;
unsigned int First_Powerchk_memory_read = 0U;
unsigned int LIN_Sleep_FlashWrite = 0U;

/* 2.9 Timers (1us / 1ms Counters) */

/* 1us Timer Group */
Global_Timer1usType G_Timer1us = {0};
Global_Timer1usFlagType G_Timer1usFlag = {0};

/* 1ms Timer Group */
Global_Timer1msType G_Timer1ms = {0};
Global_Timer1msFlagType G_Timer1msFlag = {0};

unsigned int step_check_flag = 0U;
unsigned int IGN_Chk = 0U;
unsigned int IGN_Chk_On = 0U;
unsigned int SW_Chk = 0U;
unsigned int Operating_flag = 0U;

unsigned int AAF_Tx_Position_Temporary = 0U;
unsigned int AAFx_Position_Status_Temporary = 0U;
unsigned int AAFx_InitStatus_Temporary = 0U;
unsigned int AAFx_Last_Command = 0U;
unsigned int LIN_Recover = 0U;
unsigned int Power_Reset_Flag = 0U;
unsigned int Adc_Recovery_Detected = 0U;

unsigned int vol_ret = 0U;
