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

static uint16_t tx_16bit_spi_slew_change[2] = {
    0x0690,
    0x0691};

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
static unsigned int limit_memory_write = 0;             // limit step(5%)
static unsigned int limit_memory_read = 0;
static unsigned int position_status_memory_write = 0; // AAFx_Position_Status
static unsigned int position_status_memory_read = 0;
static unsigned int AAFx_InitStatus_memory_write = 0; // AAFx_InitStatus
static unsigned int AAFx_InitStatus_memory_read = 0;
static unsigned int position_Initstatus_combined_read = 0; // AAFx_Position_Status+AAFx_InitStatus
static unsigned int DTC_memory_write = 0;                  // DTC
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

// development
static unsigned int SW_Chk = 0U;
// static unsigned int Open_Min_Limit = 0U;
// static unsigned int Open_Max_Limit = 0U;
// static unsigned int Close_Min_Limit = 0U;
// static unsigned int Close_Max_Limit = 0U;
// static unsigned int test = 0U;
static uint8_t Slave_RxSwData1[8]; /*  */
// static unsigned int wakeup_chk = 1;      /*  */
// static unsigned int spi_fail = 0;        /*  */
static unsigned int LIN_Short_Ok = 0U; /*  */

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
static void DRV8899_On(void);
static void DRV8899_Off(void);
static void Motor_dir_open(void);
static void Motor_dir_close(void);
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
static void Lin_sleep(void); // V

/* 3.3 Application Logic Functions */
static void Operating_mode(void);
static void MCU_sleep(void); // V
static void Stall_chk(void);
static void CHK_external_factors(void);
static void Fail_safety_mode(void);
static void Protection_Mode(void);
static void Voltage_chk_current_limit_init(void);
// static void Current_limiting_select(void);
static void Antipinch_move(void); // V
static void Torque_Test_Mode(void);
static void LIMP_HOME(void); // V
static void Init_move(void);
static void step_check(void); // V
static void Re_Init(void);    // V
static void LIN_Short_Chk(void);
static void Error_FaultClear(void);
static void ERROR_chk(void);
static void Tx_position_complete_chk(void);

/* 3.4 AAF Application Functions */
static void AAF_Init(void);
static void AAF_communicate_chk(void);
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

    AAF_Init();

    while (1)
    {
        AAF_App();
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
    RLN30.LTRC = 0x02;         /*setting RTS=1,response reception is started*/
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
void RLIN_Slave_NoResponse(void)
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
 * Function Name: AAF_Init
 * Description  : This function is main user init
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
static void AAF_Init(void)
{

    protected_write(WPROTR.PROTCMD0, WPROTR.PROTS0, STBC_IOHOLD.IOHOLD, 0x00u);

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

    AAFx_Index = AAF_2;          //
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
    while (1)              // 1ms wait (spi error prevention)
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
}

/* =========================================================================================
 * Application Logic Functions (Main Loop Logic)
 * ========================================================================================= */

/***********************************************************************************************************************
 * Function Name: Process_motor_action
 * Description  : Handles protection timer and executes motor movement (Action Logic).
 * Called By    : AAF_App
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Process_motor_action(void)
{
    // 1. Protection Timer Clamp (Max 550ms)
    if (time_1ms_protection_chk >= 550)
    {
        time_1ms_protection_chk_flag = 0;
        time_1ms_protection_chk = 550;
    }

    // 2. Motor Execution (Start after 50ms)
    if (time_1ms_protection_chk >= 50)
    {
        Motor_Action();
    }
}

/***********************************************************************************************************************
 * Function Name: Process_IGN_Error_Check
 * Description  : IGN/HEV 신호를 모니터링하고 에러 체크 루틴을 수행함
 ***********************************************************************************************************************/
static void Process_IGN_Error_Check(void)
{
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
}

static void Process_monitoring_hw_signal(void)
{
    Process_IGN_Error_Check();

    VDC_adc();
}

/***********************************************************************************************************************
 * Function Name: Main_mode_chk
 * Description  : Checks and executes the primary operation modes (Normal, Test, Sleep).
 * Called By    : Process_monitoring_sw_logic
 ***********************************************************************************************************************/
static void Main_mode_chk(void)
{
    if ((protection_function == OFF) && (lin_bus_inactive_flag == OFF) && (AAF_Maximum_Torque_Test_Mode == OFF) && (time_1ms_protection_chk == 550))
    {
        Operating_mode();
    }
    else if (AAF_Maximum_Torque_Test_Mode == ON)
    {
        Torque_Test_Mode();
    }
    else if (lin_bus_inactive_flag == ON)
    {
        Lin_sleep();
    }
    else
    {
        // invalid
    }
}

/***********************************************************************************************************************
 * Function Name: AAF_communicate_chk
 * Description  : Handles all communication related checks (LIN Rx/Tx, SPI).
 * Called By    : Process_monitoring_sw_logic
 ***********************************************************************************************************************/
static void AAF_communicate_chk(void)
{
    Lin_rx_data_chk();
    Lin_tx_data_chk();
    SPI_chk();
}

/***********************************************************************************************************************
 * Function Name: Safety_protect_chk
 * Description  : Checks and executes safety (Fail-safe, Antipinch) and protection modes.
 * Called By    : Process_monitoring_sw_logic
 ***********************************************************************************************************************/
static void Safety_protect_chk(void)
{
    // 1. Fail Safety Mode
    if ((protection_function == OFF) && (lin_bus_inactive_flag == OFF) && (AAF_Maximum_Torque_Test_Mode == OFF))
    {
        Fail_safety_mode();
    }

    // 2. Anti-Pinch Logic
    if ((antipinch_previous_action == OPEN) || (antipinch_previous_action == CLOSE))
    {
        Antipinch_move();
    }

    // 3. LIN Bus Check
    Lin_bus_chk();

    // 4. Protection Mode
    if ((protection_function == ON) && (lin_bus_inactive_flag == OFF) && (AAF_Maximum_Torque_Test_Mode == OFF))
    {
        Protection_Mode();
    }
}

/***********************************************************************************************************************
 * Function Name: Step_init_chk
 * Description  : Handles step position initialization and flash memory sync upon Ignition ON.
 * Called By    : Process_monitoring_sw_logic
 ***********************************************************************************************************************/
static void Step_init_chk(void)
{
    if ((step_check_flag == 0) && ((EngRunSta == 0x01) || (HevRdy == 0x01)))
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
}

/***********************************************************************************************************************
 * Function Name: Process_monitoring_sw_logic
 * Description  : Monitors software-level states executing logic in the EXACT order of the original loop.
 * Called By    : AAF_App
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Process_monitoring_sw_logic(void)
{
    // [Sequence 1] Main Operation Mode Check
    Main_mode_chk();

    // [Sequence 2] Communication Check
    AAF_communicate_chk();

    // [Sequence 3] External Factors Check
    CHK_external_factors();

    // [Sequence 4] Safety & Protection Logic (Fail-safe, Antipinch, LIN Bus, Protection)
    Safety_protect_chk();

    // [Sequence 5] Step Initialization Check
    Step_init_chk();
}

/***********************************************************************************************************************
 * Function Name: AAF_App
 * Description  : Application Main Loop Function.
 * Called By    : Main Loop (while(1))
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void AAF_App(void)
{
    // 1. Motor Action & Timer
    Process_motor_action();

    // 2. Hardware Signal Monitoring (IGN, ADC)
    Process_monitoring_hw_signal();

    // 3. Software Logic Execution (Modes, Comms, Safety)
    Process_monitoring_sw_logic();

    // 4. Watchdog Refresh
    R_Config_WDT0_Restart();
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
    RLN30.LST = 0x00U;        // Clear Status
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

        if (LIMP_HOME_Count >= 4U)
            LIMP_HOME_Count -= 4U;
        else
            LIMP_HOME_Count = 0U;
    }
    else if ((is_response_received == 0x02U) && (AAF_LIN_ChkSum_CHK_value != Req_ChkSum_Rx))
    {
        AAF_LIN_ChkSum_CHK = FAIL;

        if (LIMP_HOME_Count <= 158U)
            LIMP_HOME_Count += 2U;
        else
            LIMP_HOME_Count = 160U;
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
    receive_header_flag = (uint8_t)(RLN30.LST & 0x80U);   /* 1: Header transmission completed */
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
    RLN30.LTRC = 0x01U;              // Set FTS bit (Ready for next frame)
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
    PORT.P10 |= _PORT_Pn0_OUTPUT_HIGH;  // MCU_DRVOFF
    PORT.P10 |= _PORT_Pn2_OUTPUT_HIGH;  // MCU_DRV_SLEEP_MODE
    PORT.P9 |= _PORT_Pn1_OUTPUT_HIGH;   // VREF
    PORT.P10 &= ~_PORT_Pn4_OUTPUT_HIGH; // MCU_DIR
    PORT.P9 &= ~_PORT_Pn0_OUTPUT_HIGH;  // MCU_STEP
}

static void DRV8899_Wakeup(void)
{
    PORT.P10 |= _PORT_Pn0_OUTPUT_HIGH; // MCU_DRVOFF
    PORT.P10 |= _PORT_Pn2_OUTPUT_HIGH; // MCU_DRV_SLEEP_MODE
    PORT.P9 |= _PORT_Pn1_OUTPUT_HIGH;  // VREF
}

static void DRV8899_Sleep(void)
{
    PORT.P10 |= _PORT_Pn0_OUTPUT_HIGH;  // MCU_DRVOFF
    PORT.P10 &= ~_PORT_Pn2_OUTPUT_HIGH; // MCU_DRV_SLEEP_MODE
    PORT.P9 &= ~_PORT_Pn1_OUTPUT_HIGH;  // VREF
    PORT.P10 &= ~_PORT_Pn4_OUTPUT_HIGH; // MCU_DIR
    PORT.P9 &= ~_PORT_Pn0_OUTPUT_HIGH;  // MCU_STEP
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
        // invalid
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
        // invalid
    }
}

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

    Execute_legacy_soft_start(); // not used

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
    if (dir == OPEN)
        Motor_dir_open();
    else
        Motor_dir_close();

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

        if (dir == CLOSE)
            step_position_close = step_position;
        else
            step_position_open = step_position;

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
        Motor_dir_close();                   // dir CLOSE
        DRV8899_On();                        // drv on
        motor_start = ON;                    // step start
        time_1ms_external_10s_chk_flag = ON; // 10s chk timer on

        motor_stall_flag = MOTOR_NORMAL; // stall reset
        // stall_chk_cnt = 0;			 stall reset
        stall_chk_time_1ms = 0;                           // stall reset
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
        R_Config_ADCA0_ScanGroup1_GetResult(scan_results, 1);
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

/***********************************************************************************************************************
 * Function Name: Check_voltage_error
 * Description  : 전압 상태(저전압/과전압)를 체크하고 임계값 이탈 시 모터를 정지하거나 보호 모드로 진입함
 * Arguments    : void
 * Return Value : 1 (즉시 정지 상황), 0 (정상 또는 대기 상황)
 ***********************************************************************************************************************/
static uint8_t Check_voltage_error(void)
{
    if (adc_chk_ok_flag != 10U)
        return 0;

    // 1. 저전압 (Under Voltage)
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
        return 1; // 원본의 return; 로직 반영
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
    else if (Under_Voltage_Deceted == 1)
    {
        Under_Voltage_Deceted = 0;
        time_1ms_adc_1s_chk = 0;
        time_1ms_adc_1s_chk_flag = 0;
    }

    // 2. 과전압 (Over Voltage)
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
        return 1; // 원본의 return; 로직 반영
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
    else if (adc_avr >= ADC_OVER_VOLTAGE_16_5V)
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
    }
    else if (Over_Voltage_Deceted == 1)
    {
        Over_Voltage_Deceted = 0;
        time_1ms_adc_1s_chk = 0;
        time_1ms_adc_1s_chk_flag = 0;
    }

    return 0;
}

/***********************************************************************************************************************
 * Function Name: Check_short_error
 * Description  : 모터 쇼트(과전류) 감지 시 재시도를 수행하고, 10회 실패 시 영구 고장 처리함
 ***********************************************************************************************************************/
static void Check_short_error(void)
{
    if (AAFx_Circuit_Short == AAF_CIRCUIT_SHORT)
        return;

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
    else if (Short_Detected == 1)
    {
        if ((time_1ms_motor_Short_chk >= 200) && (Short_fault_check == 0))
        {
            Error_FaultClear();
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
                Short_Detected = 0;
            }
        }
        if (motor_Short_chk_count >= 10)
        {
            DRV_Off();
            motor_start = OFF;
            AAFx_Circuit_Short = AAF_CIRCUIT_SHORT;
            DRV8899_Sleep();
            DTC_Status |= 0x04;
            time_1ms_motor_Short_chk_flag = 0;
            Short_Detected = 0;
            Flash_memory_write();
        }
    }
    else
    {
        // invalid
    }

    LIN_Short_Chk();
}

/***********************************************************************************************************************
 * Function Name: Check_open_error
 * Description  : 모터 단선(Open Load) 감지 시 재시도를 수행하고, 10회 실패 시 영구 고장으로 판단함
 ***********************************************************************************************************************/
static void Check_open_error(void)
{
    if (AAFx_Motor_Fault == 1)
        return;

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
                time_1ms_motor_Open_chk = 0;
                time_1ms_motor_Open_chk_flag = 0;
                Open_Detected = 0;
            }
            else if ((motor_open_load == MOTOR_FAULT) && (motor_Open_chk_count < 10))
            {
                Open_Detected = 0;
            }
        }
    }
    if (motor_Open_chk_count >= 10)
    {
        DRV_Off();
        motor_start = OFF;
        AAFx_Motor_Fault = 1;
        DRV8899_Sleep();
        DTC_Status |= 0x10;
        time_1ms_motor_Open_chk_flag = 0;
        Flash_memory_write();
    }
}

/***********************************************************************************************************************
 * Function Name: ERROR_chk
 * Description  : 외부 보호 요청, 전압, 드라이버 결함, 쇼트/오픈 상태를 순차적으로 체크함
 ***********************************************************************************************************************/
static void ERROR_chk(void)
{
    if (AAF_ProtectionMode_Rx == ON)
    {
        protection_function = ON;
        AAF_ProtectionMode_Tx = ON;
    }
    else if ((AAF_ProtectionMode_Rx == OFF) && (stall_test_mode == 0U))
    {
        // 1. 전압 체크 (내부 return 로직 포함)
        if (Check_voltage_error() == 1)
            return;

        // 2. 모터 초기화 결함 체크
        if (((AAFx_InitStatus == ABNORMAL_FINISHED_INITIALIZATION)) && (motor_fault_chk == 1))
        {
            DRV8899_Sleep();
            AAFx_Motor_Fault = 1;
            DTC_Status |= 0x10;
        }
        else
        {
            AAFx_Motor_Fault = NO_ERROR;
        }

        // 3. 쇼트 체크
        Check_short_error();

        // 4. 오픈 체크 (내부 AAFx_Motor_Fault 리턴 포함)
        Check_open_error();

        // 5. FDL/LIMP HOME 체크
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

    if (ret < (char)0) // error
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

    if (ret < (char)0) // error
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

    if (ret < (char)0) // error
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

    close_memory_read = (unsigned int)r_buff[0] & 0xFFFF;        // 4byte 源뚯?
    open_memory_read = (unsigned int)(r_buff[0] >> 16) & 0xFFFF; // 4byte 源뚯?

    now_step_memory_read = (unsigned int)r_buff[1] & 0xFFFF; // 4byte 源뚯?

    position_Initial_combined_read = (unsigned int)(r_buff[1] >> 16) & 0xFFFF;        // position_memory_read+Initial_memory_read
    position_memory_read = (unsigned int)position_Initial_combined_read & 0xFF;       // 4byte 源뚯?
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
    aaf_step = AAF_INITIALIZATION;                    // MCU is reset, AAF is initialized.
    aaf_init_step = START_INITIALIZATION;             // MCU is reset, AAF is initialized.
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

/***********************************************************************************************************************
 * Function Name: lin_command_to_action
 * Description  : LIN 명령을 받아 모터 드라이버를 깨우고 해당 동작 모드로 설정함
 * Arguments    : action - 실행할 동작 (OPEN, CLOSE, OPEN_1ST 등)
 * Return Value : void
 ***********************************************************************************************************************/
static void lin_command_to_action(unsigned int action)
{
    DRV8899_Wakeup();
    time_1ms_diag_auto = 0;
    time_1ms_diag_auto_flag = OFF;
    diag_mode_auto_action = OFF;

    aaf_action = action;
    aaf_step = AAF_OPERATE;
}

/***********************************************************************************************************************
 * Function Name: Setup_action
 * Description  : LIN 명령(lin_aaf_command)을 해석하여 적절한 Action 모드로 진입 준비를 수행함
 * Called By    : Process_operating_waiting
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Setup_action(void)
{

    if ((lin_aaf_command == OPEN) || (lin_aaf_command == CLOSE) ||
        (lin_aaf_command == OPEN_1ST) || (lin_aaf_command == OPEN_2ND))
    {
        lin_command_to_action(lin_aaf_command);
    }
    else if ((lin_aaf_command == DIAG_MODE_OPEN) && (AAF_Tx_Position != DIAG_MODE_OPEN)) // lin init command chk
    {
        lin_command_to_action(DIAG_MODE_OPEN);
    }
    else if ((lin_aaf_command == DIAG_MODE_CLOSE) && (AAF_Tx_Position != DIAG_MODE_CLOSE)) // lin init command chk
    {
        lin_command_to_action(DIAG_MODE_CLOSE);
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
                // invalid
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
        // invalid
    }

    aaf_action_complete_chk = FLAP_START;

    step_start_flag = OFF;
}

/***********************************************************************************************************************
 * Function Name: Setup_auto_action
 * Description  : 진단 자동(Auto) 모드 실행 중, 방향을 전환하여 연속 동작을 설정함
 * Called By    : Process_operating_waiting
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Setup_auto_action(void)
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
        // Invalid
    }

    aaf_action = DIAG_MODE_AUTO;
    aaf_step = AAF_OPERATE;

    aaf_action_complete_chk = FLAP_START;
    time_1ms_diag_auto_flag = ON;
    step_start_flag = OFF;
}

/***********************************************************************************************************************
 * Function Name: Process_operating_waiting
 * Description  : AAF_WAITING 상태에서 LIN 명령 수신 여부 또는 자동 모드 플래그를 확인하여 동작을 시작함
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Process_operating_waiting(void)
{
    if ((step_start_flag == ON) && (lin_bus_inactive_flag == OFF))
    {
        Setup_action();
    }
    else if ((diag_mode_auto_action == ON) && (lin_bus_inactive_flag == OFF))
    {
        Setup_auto_action();
    }
    else
    {
        // invalid
    }
}

/***********************************************************************************************************************
 * Function Name: Action_normal_operate
 * Description  : 일반 동작(Normal Mode)을 위해 모터 방향 설정, 드라이버 On, 각종 감지 타이머를 활성화함
 * Arguments    : direction - 모터 이동 방향 (OPEN / CLOSE)
 * Return Value : void
 ***********************************************************************************************************************/
static void Action_normal_operate(unsigned int direction)
{
    if (direction == OPEN)
        Motor_dir_open();
    else
        Motor_dir_close();

    DRV8899_On();
    motor_start = ON;
    time_1ms_external_10s_chk_flag = ON;
    time_1ms_stall_chk_flag = ON;
    stall_chk_time_1ms = 0;
    motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE;

    if (direction == OPEN)
        flap_move = OPEN;
    else
        flap_move = CLOSE;

    Diag_Mode = 0;
    aaf_step = TRAVEL_RANGE_COMPLETE_CHECK;
}

/***********************************************************************************************************************
 * Function Name: Action_diag_operate
 * Description  : 진단 동작(Diagnostic Mode)을 위해 모터를 구동하고 진단 모드 플래그(Diag_Mode)를 설정함
 * Arguments    : direction - 모터 이동 방향
 * is_auto   - 자동 반복 모드 여부 (ON/OFF)
 * Return Value : void
 ***********************************************************************************************************************/
static void Action_diag_operate(unsigned int direction, unsigned int is_auto)
{
    if (direction == OPEN)
        Motor_dir_open();
    else
        Motor_dir_close();

    DRV8899_On();
    motor_start = ON;
    time_1ms_stall_chk_flag = ON;
    stall_chk_time_1ms = 0;
    motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE;
    Diag_Mode = 1;

    if (direction == OPEN)
        flap_move = OPEN;
    else
        flap_move = CLOSE;

    // time_1ms_init_chk_flag = 1;

    if (is_auto == ON)
    {
        time_1ms_diag_auto_flag = 1;
    }

    aaf_step = TRAVEL_RANGE_COMPLETE_CHECK;
}

/***********************************************************************************************************************
 * Function Name: Process_normal_operate
 * Description  : 현재 스텝 위치(step_position)와 목표 위치를 비교하여 모터를 구동할지, 완료 상태로 넘길지 결정함
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Process_normal_operate(void)
{
    if (aaf_action == OPEN)
    {
        if (step_position >= (step_position_open + limit_step_position))
        {
            Action_normal_operate(OPEN);
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
            Action_normal_operate(CLOSE);
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
            Action_normal_operate(CLOSE);
        }
        else
        {
            Action_normal_operate(OPEN);
        }

        Diag_Mode = 0;
        aaf_step = TRAVEL_RANGE_COMPLETE_CHECK;
    }
    else if (aaf_action == OPEN_2ND)
    {
        if (step_position <= (step_position_open + open_2nd_step_position))
        {
            Action_normal_operate(CLOSE);
        }
        else
        {
            Action_normal_operate(OPEN);
        }

        Diag_Mode = 0;
        aaf_step = TRAVEL_RANGE_COMPLETE_CHECK;
    }
    else
    {
        // invalid
    }
}

/***********************************************************************************************************************
 * Function Name: Process_diag_operate
 * Description  : 진단 모드 요청에 따라 강제 열림/닫힘 또는 오토 사이클 동작을 수행함
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Process_diag_operate(void)
{
    if (aaf_action == DIAG_MODE_OPEN)
    {
        Action_diag_operate(OPEN, OFF);
    }
    else if (aaf_action == DIAG_MODE_CLOSE)
    {
        Action_diag_operate(CLOSE, OFF);
    }
    else if ((aaf_action == DIAG_MODE_AUTO) && (diag_mode_auto_dir == OPEN))
    {
        Action_diag_operate(OPEN, ON); // auto
    }
    else if ((aaf_action == DIAG_MODE_AUTO) && (diag_mode_auto_dir == CLOSE))
    {
        Action_diag_operate(CLOSE, ON); // auto
    }
    else
    {
        // invalid
    }
}

/***********************************************************************************************************************
 * Function Name: Process_operating_operate
 * Description  : AAF_OPERATE 상태에서 요청된 액션 타입(일반/진단)에 따라 적절한 처리 함수를 호출함
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Process_operating_operate(void)
{
    if ((aaf_action == OPEN) || (aaf_action == CLOSE) ||
        (aaf_action == OPEN_1ST) || (aaf_action == OPEN_2ND))
    {
        Process_normal_operate();
    }
    else
    {
        Process_diag_operate();
    }

    aaf_action_complete_chk = FLAP_MOVING;
}

/***********************************************************************************************************************
 * Function Name: Check_stall
 * Description  : 동작 중 스톨(모터 걸림) 발생 시 모터를 정지하고 상태를 업데이트함
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Check_stall(void)
{
    // if(((motor_stall_flag == MOTOR_STALL) && (time_1ms_stall_chk >= 100)) || (time_1ms_init_chk >= 4800))
    if ((motor_stall_flag == MOTOR_STALL) && (time_1ms_stall_chk >= 100U))
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
            // invalid
        }

        time_1ms_external_10s_chk_flag = OFF; // 10s chk timer on
        time_1ms_external_10s_chk = 0;
        aaf_step = CHECK_AAF_CONDITION;
        time_1ms_init_chk_flag = 0; // test
        time_1ms_init_chk = 0;      // test

        aaf_action = FLAP_STOP;
    }
}

/***********************************************************************************************************************
 * Function Name: Process_operating_Range_Chk
 * Description  : 목표 위치 도달 여부를 확인하고, 도달 시 TX 포지션을 업데이트한 후 종료 상태로 전환함
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Process_operating_Range_Chk(void)
{
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
    else
    {
        Check_stall();
    }
}

/***********************************************************************************************************************
 * Function Name: Process_operating_check_condition
 * Description  : 스톨 발생 후 정지 상태에서 끼임(Anti-Pinch) 감지를 위한 초기화 및 플래그 설정을 수행함
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Process_operating_check_condition(void)
{
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
    else
    {
        // invalid
    }

    time_1ms_stall_chk = 0;      // test
    time_1ms_stall_chk_flag = 0; // test
}

/***********************************************************************************************************************
 * Function Name: Process_operating_init
 * Description  : 시스템 초기화 시퀀스(학습 모드)를 단계별로 처리함
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Process_operating_init(void)
{
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
}

/***********************************************************************************************************************
 * Function Name: Process_operating_finish
 * Description  : 모든 동작이 완료된 후 드라이버를 끄고 변수들을 초기화하며 대기 상태(AAF_WAITING)로 전환함
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Process_operating_finish(void)
{
    DRV_Off();                            // drv of
    motor_start = OFF;                    // step stop
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
        // invalid
    }

    time_1ms_stall_chk = 0;      // test
    time_1ms_stall_chk_flag = 0; // test
    softstart_complete = OFF;
    motor_step_value = STEP_TIME_1000RPM;
    time_1ms_init_chk_flag = 0;                       // test
    time_1ms_init_chk = 0;                            // test
    stall_chk_cnt = 0;                                // stall reset
    stall_chk_time_1ms = 0;                           // stall reset
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
}

/***********************************************************************************************************************
 * Function Name: Operating_mode
 * Description  : AAF 시스템의 메인 상태 머신(State Machine). 현재 단계(aaf_step)에 따라 적절한 프로세스를 호출함
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Operating_mode(void)
{
    switch (aaf_step)
    {
    case AAF_WAITING:

        Process_operating_waiting();

        break;

    case AAF_OPERATE:

        Process_operating_operate();

        break;

    case TRAVEL_RANGE_COMPLETE_CHECK:

        Process_operating_Range_Chk();

        break;

    case CHECK_AAF_CONDITION:

        Process_operating_check_condition();

        break;

    case AAF_INITIALIZATION:

        // init action

        Process_operating_init();

        break;

    case FINISHED_OPERATE:

        Process_operating_finish();

        break;

    default:
        break;
    }
}

/***********************************************************************************************************************
 * Function Name: Update_lin_command
 * Description  : 파싱된 타겟 위치(target_select)에 따라 글로벌 제어 명령(lin_aaf_command)을 업데이트함
 * Arguments    : target_select - 결정된 목표 위치 또는 모드
 * Return Value : void
 ***********************************************************************************************************************/
static void Update_lin_command(unsigned int target_select)
{
    switch (target_select)
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

/***********************************************************************************************************************
 * Function Name: Translate_rxdata
 * Description  : 수신된 LIN 데이터(Slave_RxData1)를 내부 검증용 버퍼(ID_chk_rxdata)로 복사함
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Translate_rxdata(void)
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
}

/***********************************************************************************************************************
 * Function Name: AAF1_lin_rx_data_chk
 * Description  : AAF 1번 유닛에 대한 진단 모드 및 타겟 위치를 파싱하고 명령을 설정함
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void AAF1_lin_rx_data_chk(void)
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
        else
        {
            // invalid
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
        // invalid
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
        // invalid
    }

    Update_lin_command(AAF1_TargetPosition_select);
}

/***********************************************************************************************************************
 * Function Name: AAF2_lin_rx_data_chk
 * Description  : AAF 2번 유닛에 대한 진단 모드 및 타겟 위치를 파싱하고 명령을 설정함
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void AAF2_lin_rx_data_chk(void)
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
        else
        {
            // invalid
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
        // invalid
    }

    Update_lin_command(AAF2_TargetPosition_select);
}

/***********************************************************************************************************************
 * Function Name: AAF3_lin_rx_data_chk
 * Description  : AAF 3번 유닛에 대한 진단 모드 및 타겟 위치를 파싱하고 명령을 설정함
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void AAF3_lin_rx_data_chk(void)
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
        else
        {
            // invalid
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
        // invalid
    }

    Update_lin_command(AAF3_TargetPosition_select);
}

/***********************************************************************************************************************
 * Function Name: Protection_function_off_mode
 * Description  : 보호 기능이 꺼져 있을 때의 일반적인 데이터 파싱 및 AAF별 로직 분기를 수행함
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Protection_function_off_mode(void)
{

    Translate_rxdata();

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

            if (AAFx_Index == AAF_1)
            {
                AAF1_lin_rx_data_chk();
            }
            else if (AAFx_Index == AAF_2)
            {
                AAF2_lin_rx_data_chk();
            }
            else if (AAFx_Index == AAF_3)
            {
                AAF3_lin_rx_data_chk();
            }
            else
            {
                // invalid
            }
        }
        else
        {
            ReqRespAAFID = (unsigned int)((ID_chk_rxdata[0] & 0xC0U) >> 6U);
            EngRunSta = (unsigned int)((ID_chk_rxdata[4] & 0x30U) >> 4U);
            HevRdy = (unsigned int)((ID_chk_rxdata[4] & 0x0CU) >> 2U);
            AAF_LINOut = (unsigned int)(ID_chk_rxdata[4] & 0x03U);
        }
    }
}

/***********************************************************************************************************************
 * Function Name: Protection_function_on_mode
 * Description  : 보호 기능이 켜져 있을 때 보호 모드 해제 요청 비트만 모니터링함
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Protection_function_on_mode(void)
{
    Translate_rxdata();

    if (lin_rx_pass_flag == PASS)
    {
        AAF_ProtectionMode_Rx = (unsigned int)((ID_chk_rxdata[7] & 0x40U) >> 6U);
    }
}

/***********************************************************************************************************************
 * Function Name: Lin_rx_data_chk
 * Description  : LIN 수신 데이터를 검증하고 보호 기능 상태(ON/OFF)에 따라 처리 루틴을 호출하는 메인 함수
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Lin_rx_data_chk(void)
{
    LIN_Diag_Rx();

    ReqRespAAFID = (unsigned int)((Slave_RxData1[0] & 0xC0U) >> 6U);
    if (AAF_LIN_ChkSum_CHK == PASS)
    {
        // ReqRespAAFID = (unsigned int)((Slave_RxData1[0] & 0xC0U) >> 6U);

        if (protection_function == OFF) //
        {
            Protection_function_off_mode();
        }
        else if (protection_function == ON)
        {
            Protection_function_on_mode();
        }
        else
        {
            // invalid
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

/* =========================================================================================
 * MCU Sleep Mode Management Functions
 * ========================================================================================= */

/***********************************************************************************************************************
 * Function Name: External_device_off
 * Description  : MCU가 슬립 모드로 진입하기 전, 연결된 외부 하드웨어(모터 드라이버, 트랜시버 등)를 끔
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void External_device_off(void)
{
    DRV8899_Sleep();       // 모터 드라이버 슬립 전환
    Lin_Transceiver_Off(); // LIN 트랜시버 전원 차단
    SPI_select_pin_Off();  // SPI 통신 핀 비활성화
}

/***********************************************************************************************************************
 * Function Name: Sleep_port_config
 * Description  : 슬립 모드 중 누설 전류 방지 및 LIN Wake-up 대기를 위해 GPIO 포트 상태를 재설정함
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Sleep_port_config(void)
{
    // 포트 기능을 GPIO 입/출력으로 리셋
    R_PORT_ResetAltFunc(Port10, 10, Output);
    R_PORT_ResetAltFunc(Port10, 9, Input);

    // LIN TX 핀을 Low로 설정하여 슬립 상태 유지 (Leakage 방지)
    PORT.P10 &= ~_PORT_Pn10_OUTPUT_HIGH; // MCU_LIN_Tx_Low Sleep go
}

/***********************************************************************************************************************
 * Function Name: Sleep_internal_module_stop
 * Description  : 전력 소모를 줄이기 위해 MCU 내부 주변장치(ADC, 타이머, 통신 모듈)의 클럭을 정지함
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Sleep_internal_module_stop(void)
{
    R_Config_INTC_Create(); // 인터럽트 컨트롤러 재설정 (Wake-up 준비)
    // R_Config_INTC_INTP5_Start(); // (주석 유지)

    R_Config_CSIH0_Stop();    // SPI 모듈 정지
    R_Config_ADCA0_Halt();    // ADC 모듈 정지
    R_Config_TAUD0_13_Stop(); // 타이머 정지
    R_Config_TAUD0_3_Stop();  // 타이머 정지

    time_1ms_spi_flag = 0; // 관련 플래그 초기화
    time_1ms_spi = 0;
}

/***********************************************************************************************************************
 * Function Name: Sleep_enter_deep_stop
 * Description  : 클럭 생성기를 슬립 모드용으로 설정하고, 최종적으로 Deep Stop Mode로 진입함
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Sleep_enter_deep_stop(void)
{
    R_CGC_Create_sleepmode(); // 클럭 설정 변경

    R_Config_STBC_Prepare_Deep_Stop_Mode(); // 대기 모드 진입 준비 레지스터 설정
    R_Config_STBC_Start_Deep_Stop_Mode();   // [진입점] 여기서 MCU 동작 멈춤
}

/***********************************************************************************************************************
 * Function Name: MCU_sleep
 * Description  : 시스템 종료 절차를 수행하고 MCU를 저전력 모드(Deep Stop)로 전환하는 메인 함수
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void MCU_sleep(void)
{
    // 1. 종료 상태 플래그 설정
    power_chk = Normal_Shutdown;
    First_Powerchk = 1;

    // 2. 필요 시 플래시 메모리에 데이터 저장
    if (step_check_flag == 1)
    {
        Flash_memory_write();
    }

    // 3. 외부 하드웨어 전원 차단
    External_device_off();

    // 4. 슬립 대비 포트 설정 (누설 전류 방지)
    Sleep_port_config();

    // 5. 내부 주변장치 클럭 정지
    Sleep_internal_module_stop();

    // 6. Deep Stop 모드 진입 (Wake-up 이벤트 발생 전까지 정지)
    Sleep_enter_deep_stop();
}

/* =========================================================================================
 * SPI Communication & Stall Check Functions
 * ========================================================================================= */

/***********************************************************************************************************************
 * Function Name: Execute_voltage_change_spi
 * Description  : Sends SPI commands to update current limits when voltage status changes (Step 0 Condition 1).
 * Called By    : Process_spi_initiation
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Execute_voltage_change_spi(void)
{
    PORT.P8 &= ~_PORT_Pn0_OUTPUT_HIGH; // CS Low

    if (voltage_status_spi == HIGH_VOLTAGE_1ST)
    {
        R_Config_CSIH0_Send_Receive(&tx_16bit_spi_current_limit[9], 1, &rx_16bit_spi[3], _CSIH_SELECT_CHIP_0);
        motor_cw_stall_value = MOTOR_CW_STALL_CHK_VALUE_HIGH_VOLTAGE_1ST;
        motor_ccw_stall_value = MOTOR_CCW_STALL_CHK_VALUE_HIGH_VOLTAGE_1ST;
    }
    else if (voltage_status_spi == NORMAL_VOLTAGE)
    {
        R_Config_CSIH0_Send_Receive(&tx_16bit_spi_current_limit[9], 1, &rx_16bit_spi[3], _CSIH_SELECT_CHIP_0);
        motor_cw_stall_value = MOTOR_CW_STALL_CHK_VALUE_NORMAL_VOLTAGE;
        motor_ccw_stall_value = MOTOR_CCW_STALL_CHK_VALUE_NORMAL_VOLTAGE;
    }
    else if (voltage_status_spi == LOW_VOLTAGE_1ST)
    {
        R_Config_CSIH0_Send_Receive(&tx_16bit_spi_current_limit[9], 1, &rx_16bit_spi[3], _CSIH_SELECT_CHIP_0);
        motor_cw_stall_value = MOTOR_CW_STALL_CHK_VALUE_LOW_VOLTAGE_1ST;
        motor_ccw_stall_value = MOTOR_CCW_STALL_CHK_VALUE_LOW_VOLTAGE_1ST;
    }
    else
    {
        // Invalid
    }

    voltage_status_change_complete = WAIT;
    time_1ms_volt_stat_chg_wait_flag = 1;
    spi_action_step = 1;
    voltage_status_change = OFF;
}

/***********************************************************************************************************************
 * Function Name: Send_spi_command_id_9
 * Description  : Sends the standard SPI command (ID 9) for status monitoring (Step 0 Condition 2 & 3).
 * Called By    : Process_spi_initiation
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Send_spi_command_id_9(void)
{
    PORT.P8 &= ~_PORT_Pn0_OUTPUT_HIGH; // CS Low

    R_Config_CSIH0_Send_Receive(&rx_16bit_spi_id[9], 1, &rx_16bit_spi[9], _CSIH_SELECT_CHIP_0);

    spi_action_step = 1;
}

/***********************************************************************************************************************
 * Function Name: Handle_spi_step_wait
 * Description  : Waits for SPI transmission/reception completion or timeout (Step 1).
 * Called By    : SPI_chk
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Handle_spi_step_wait(void)
{
    time_1ms_spi_error_chk_flag = 1;

    // Check for successful completion
    if ((spi_receive_flag >= 1U) && (spi_send_flag >= 1U))
    {
        spi_receive_flag = 0;
        spi_send_flag = 0;

        PORT.P8 |= _PORT_Pn0_OUTPUT_HIGH; // CS High

        time_1ms_spi_error_chk = 0;
        time_1ms_spi_error_chk_flag = 0;

        spi_action_step = 2;
    }

    // Check for timeout
    if (time_1ms_spi_error_chk >= 100U)
    {
        spi_action_step = 2;
        // DRV_Off();

        // spi_fail = 1;

        time_1ms_spi_error_chk = 0;
        time_1ms_spi_error_chk_flag = 0;
    }
}

/***********************************************************************************************************************
 * Function Name: Handle_spi_step_delay
 * Description  : Provides a short delay (2us) between SPI transactions (Step 2).
 * Called By    : SPI_chk
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Handle_spi_step_delay(void)
{
    time_1us_spi_flag = 1;

    if (time_1us_spi >= 2U)
    {
        time_1us_spi_flag = 0;
        time_1us_spi = 0;

        spi_action_step = 3;
    }
}

/***********************************************************************************************************************
 * Function Name: Handle_spi_step_data
 * Description  : Processes received SPI data and performs stall checking (Step 3).
 * Called By    : SPI_chk
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Handle_spi_step_data(void)
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

/***********************************************************************************************************************
 * Function Name: Process_spi_initiation
 * Description  : Checks conditions to start a new SPI transaction (Step 0 Logic).
 * Called By    : SPI_chk
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Process_spi_initiation(void)
{
    // Condition 1: Voltage Status Change
    if ((voltage_status_change == ON) && (motor_start == OFF) && (time_1ms_spi >= 20U))
    {
        Execute_voltage_change_spi();
    }
    // Condition 2: Motor ON (Fast Polling)
    else if ((motor_start == ON) && (time_1ms_spi >= 2U) && (time_1us_motor <= (STEP_TIME_1250RPM / 2U)) && (spi_action_step == 0U))
    {
        Send_spi_command_id_9();
    }
    // Condition 3: Idle (Slow Polling)
    else if ((time_1ms_spi >= 50U) && (time_1us_motor == 0U) && (spi_action_step == 0U))
    {

        // Current_limiting_select();

        if (voltage_status_change == OFF)
        {
            Send_spi_command_id_9();
        }
    }
    else
    {
        // invalid
    }
}

/***********************************************************************************************************************
 * Function Name: SPI_chk
 * Description  : Main function for SPI communication control and stall checking.
 * Metric Info  : FUCYC = 2 (Pass), FUNDC = 2 (Pass)
 * Called By    : Main Loop
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void SPI_chk(void)
{
    switch (spi_action_step)
    {

    case 0:
        Process_spi_initiation();
        break;

    case 1:
        Handle_spi_step_wait();
        break;

    case 2:
        Handle_spi_step_delay();
        break;

    case 3:
        Handle_spi_step_data();
        break;

    default:
        // Invalid
        break;
    }
}

/***********************************************************************************************************************
 * Function Name: Update_stall_counter
 * Description  : Compares current motor value against thresholds and updates the stall counter/flag.
 * Called By    : Process_stall_close, Process_stall_open
 * Arguments    : low_limit  - The lower threshold for stall detection
 * high_limit - The upper threshold for stall detection
 * Return Value : void
 ***********************************************************************************************************************/
static void Update_stall_counter(unsigned int low_limit, unsigned int high_limit)
{
    // Check if the value is out of the normal range (Stall condition)
    if ((motor_stall_value <= low_limit) || (motor_stall_value >= high_limit))
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
    // Value is within normal range
    else
    {
        motor_stall_flag = MOTOR_NORMAL;
        stall_cnt = STALL_CNT_DEFAULT;

        // [Legacy Comment Preserved]
        // stall_cnt--;
        // if (stall_cnt < STALL_CNT_DEFAULT)
        // {
        //     stall_cnt = STALL_CNT_DEFAULT;
        // }
    }
}

/***********************************************************************************************************************
 * Function Name: Process_stall_close
 * Description  : Handles stall detection logic when the motor is in CLOSE direction.
 * Called By    : Stall_chk
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Process_stall_close(void)
{
    if (AAF_location_type == RH_TYPE)
    {
        // RH Type + CLOSE Direction -> Use CCW Thresholds
        Update_stall_counter(motor_ccw_stall_value, MOTOR_CCW_STALL_CHK_HIGH_VALUE);
    }
    else if (AAF_location_type == LH_TYPE)
    {
        // LH Type + CLOSE Direction -> Use CW Thresholds
        Update_stall_counter(motor_cw_stall_value, MOTOR_CW_STALL_CHK_HIGH_VALUE);
    }
    else
    {
        // Invalid
    }
}

/***********************************************************************************************************************
 * Function Name: Process_stall_open
 * Description  : Handles stall detection logic when the motor is in OPEN direction.
 * Called By    : Stall_chk
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Process_stall_open(void)
{
    if (AAF_location_type == RH_TYPE)
    {
        // RH Type + OPEN Direction -> Use CW Thresholds
        Update_stall_counter(motor_cw_stall_value, MOTOR_CW_STALL_CHK_HIGH_VALUE);
    }
    else if (AAF_location_type == LH_TYPE)
    {
        // LH Type + OPEN Direction -> Use CCW Thresholds
        Update_stall_counter(motor_ccw_stall_value, MOTOR_CCW_STALL_CHK_HIGH_VALUE);
    }
    else
    {
        // Invalid
    }
}

/***********************************************************************************************************************
 * Function Name: Stall_chk
 * Description  : Main function for motor stall detection.
 * Called By    : SPI_chk
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Stall_chk(void)
{
    if (motor_start == ON)
    {
        if (dir_state == CLOSE)
        {
            Process_stall_close();
        }
        else if (dir_state == OPEN)
        {
            Process_stall_open();
        }
        else
        {
            // Invalid
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

/***********************************************************************************************************************
 * Function Name: Set_AAF_ID
 * Description  : AAF 모듈 인덱스(1~3)에 따라 통신 ID 데이터(ID_chk_rxdata)를 설정함.
 * (원본 Case 0의 ID 설정 로직 분리)
 * Called By    : Fail_safety_init
 * Return Value : void
 ***********************************************************************************************************************/
static void Set_AAF_ID(void)
{
    if (AAFx_Index == AAF_1)
    {
        ID_chk_rxdata[4] = 0x03U;
    }
    else if (AAFx_Index == AAF_2)
    {
        ID_chk_rxdata[4] = 0x18U;
    }
    else if (AAFx_Index == AAF_3)
    {
        ID_chk_rxdata[5] = 0x03U;
    }
    else
    {
        // Invalid Index
    }
}

/***********************************************************************************************************************
 * Function Name: Fail_safety_init
 * Description  : Fail-Safety 각 사이클(1차, 2차, 최종) 시작 시 하드웨어 및 변수를 초기화함.
 * Called By    : Run_fail_safety_cycle_1 (Step 0), Run_fail_safety_cycle_2 (Step 6), Run_fail_safety_final (Step 12)
 * Arguments    : next_step - 초기화 후 이동할 다음 Fail-Safety Step
 * Return Value : void
 ***********************************************************************************************************************/
static void Fail_safety_init(unsigned int next_step)
{
    // 1. 공통 초기화
    DRV_Off();         // 모터 드라이버 Off
    motor_start = OFF; // 스텝 생성 중지
    time_1ms_external_10s_chk_flag = OFF;
    time_1ms_external_10s_chk = 0;
    aaf_action = FLAP_STOP;
    aaf_action_complete_chk = FLAP_STOP;

    time_1ms_stall_chk = 0;
    time_1ms_stall_chk_flag = 0;

    stall_chk_cnt = 0;
    stall_chk_time_1ms = 0;
    motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE;

    aaf_step = AAF_INITIALIZATION; // 메인 상태를 초기화 모드로 변경
    aaf_init_step = START_INITIALIZATION;
    AAF_Tx_Position = UNKOWN_POSITION;
    AAFx_Position_Status = Unknown_Status;
    lin_aaf_command = OPEN;
    softstart_complete = OFF;
    motor_step_value = STEP_TIME_1000RPM;

    init_move_step = START_INITIALIZATION_OPEN;

    // 2. [Case 0 전용] 1차 사이클 시작 시 ID 데이터 설정
    if (next_step == 1U)
    {
        Set_AAF_ID();
    }

    // 3. [Case 12 전용] Final 사이클 시작 시 Ready 플래그 해제
    if (next_step == 13U)
    {
        evrdy_on_flag = OFF;
    }

    // 4. 다음 단계 설정 및 사이클 플래그 On
    fail_safety_step = next_step;
    fail_safety_1_cycle_flag = ON;
}

/***********************************************************************************************************************
 * Function Name: Fail_safety_check_init_end
 * Description  : 초기화 이동(Init Move) 시퀀스가 완료되었는지 확인하고 정상 상태로 복귀함.
 * Called By    : Case 1, 7, 13
 * Return Value : void
 ***********************************************************************************************************************/
static void Fail_safety_check_init_end(void)
{
    if (init_move_step == 19U) // Init_move 함수 내 완료 조건
    {
        init_move_step = 0;
        aaf_init_step = NORMAL_INITIALIZATION;
        fail_safety_step = 0;
        fail_safety_flag = OFF;
        fail_safety_1_cycle_flag = OFF;
        AAFx_ErrorStatus = No_ErrorStatus;
    }
}

/***********************************************************************************************************************
 * Function Name: Command_to_action_next_step
 * Description  : LIN 명령(OPEN/CLOSE)을 수신하여 실제 동작(Action) 변수를 설정함.
 * Called By    : Case 2, 8
 * Arguments    : next_step - 다음 단계
 * Return Value : void
 ***********************************************************************************************************************/
static void Command_to_action_next_step(unsigned int next_step)
{
    if ((lin_aaf_command == OPEN) || (lin_aaf_command == CLOSE))
    {
        DRV8899_Wakeup();

        if (lin_aaf_command == OPEN)
        {
            aaf_action = OPEN;
        }
        else
        {
            aaf_action = CLOSE;
        }

        motor_stall_flag = MOTOR_NORMAL;
        stall_chk_time_1ms = 0;
        motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE;
        fail_safety_step = next_step;
    }
    else
    {
        // invalid
    }
}

/***********************************************************************************************************************
 * Function Name: Action_start_next_step
 * Description  : 설정된 Action에 따라 모터를 물리적으로 구동 시작함.
 * Called By    : Case 3, 9
 * Arguments    : next_step - 다음 단계
 * Return Value : void
 ***********************************************************************************************************************/
static void Action_start_next_step(unsigned int next_step)
{
    if ((aaf_action == OPEN) || (aaf_action == CLOSE))
    {
        if (aaf_action == OPEN)
        {
            Motor_dir_open();
        }
        else
        {
            Motor_dir_close();
        }

        DRV8899_On();
        motor_start = ON;
        time_1ms_init_chk_flag = 1;
        fail_safety_step = next_step;
    }
    else
    {
        // invalid
    }
}

/***********************************************************************************************************************
 * Function Name: Check_stall_stop_next_step
 * Description  : 동작 중 스톨(Stall) 또는 타임아웃 발생 시 모터를 정지하고 다음 단계로 이동함.
 * (Case 4, 10에서는 Tx Position을 업데이트하고, Case 15에서는 업데이트하지 않음)
 * Called By    : Case 4, 10, 15
 * Arguments    : next_step - 다음 단계
 * Return Value : void
 ***********************************************************************************************************************/
static void Check_stall_stop_next_step(unsigned int next_step)
{
    if ((motor_stall_flag == MOTOR_STALL) || (time_1ms_init_chk >= 4500U))
    {
        DRV_Off();
        motor_start = OFF;
        stall_chk_cnt = 0;
        stall_chk_time_1ms = 0;

        fail_safety_step = next_step;

        aaf_action = FLAP_STOP;
        time_1ms_init_chk_flag = 0;
        time_1ms_init_chk = 0;
        softstart_complete = OFF;
        motor_step_value = STEP_TIME_1000RPM;

        // [중요 수정] 원본 로직 복구: Step 16(Final Fail)으로 가는 경우가 아닐 때만 위치 업데이트
        if (next_step != 16U)
        {
            if (aaf_action == OPEN)
            {
                AAF_Tx_Position = OPEN;
            }
            else if (aaf_action == CLOSE)
            {
                AAF_Tx_Position = CLOSE;
            }
        }
    }
}

/***********************************************************************************************************************
 * Function Name: Execute_final_recovery_action
 * Description  : 마지막 복구 시도(Case 14)를 위해 강제 OPEN 동작을 수행함.
 * Called By    : Run_fail_safety_final
 * Return Value : void
 ***********************************************************************************************************************/
static void Execute_final_recovery_action(void)
{
    DRV8899_Wakeup();

    aaf_action = OPEN;
    motor_stall_flag = MOTOR_NORMAL;
    stall_chk_cnt = 0;
    stall_chk_time_1ms = 0;
    motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE;

    Motor_dir_open();
    DRV8899_On();
    motor_start = ON;
    time_1ms_init_chk_flag = 1;

    fail_safety_step = 15;
}

/* =========================================================================================
 * Fail Safety Cycle Execution Functions
 * ========================================================================================= */

/***********************************************************************************************************************
 * Function Name: Run_fail_safety_cycle_1
 * Description  : Fail-Safety 1차 사이클 (Step 0 ~ 5) 실행.
 * 초기화 -> 명령 확인 -> 구동 -> 스톨 감지 -> 3분 대기
 * Called By    : Fail_safety_mode
 * Return Value : void
 ***********************************************************************************************************************/
static void Run_fail_safety_cycle_1(void)
{
    switch (fail_safety_step)
    {
    case 0:
        Fail_safety_init(1);
        break;
    case 1:
        Fail_safety_check_init_end();
        break;
    case 2:
        Command_to_action_next_step(3);
        break;
    case 3:
        Action_start_next_step(4);
        break;
    case 4:
        Check_stall_stop_next_step(5);
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
    default:
        break;
    }
}

/***********************************************************************************************************************
 * Function Name: Run_fail_safety_cycle_2
 * Description  : Fail-Safety 2차 사이클 (Step 6 ~ 11) 실행.
 * 1차 사이클과 동일한 과정을 한 번 더 반복함.
 * Called By    : Fail_safety_mode
 * Return Value : void
 ***********************************************************************************************************************/
static void Run_fail_safety_cycle_2(void)
{
    switch (fail_safety_step)
    {
    case 6:
        Fail_safety_init(7);
        break;
    case 7:
        Fail_safety_check_init_end();
        break;
    case 8:
        Command_to_action_next_step(9);
        break;
    case 9:
        Action_start_next_step(10);
        break;
    case 10:
        Check_stall_stop_next_step(11);
        break;
    case 11:
        time_1ms_3minute_flag = 1;
        antipinch_step = 0;

        if (time_1s_3minute >= MINUTE_3)
        {
            time_1ms_3minute_flag = 0;
            time_1s_3minute = 0;
            time_1ms_3minute = 0;
            fail_safety_step = 12;
        }
        break;
    default:
        break;
    }
}

/***********************************************************************************************************************
 * Function Name: Run_fail_safety_final
 * Description  : Fail-Safety 최종 사이클 (Step 12 ~ 16) 실행.
 * 초기화 -> 강제 OPEN 시도 -> 실패 시 최종 에러 확정
 * Called By    : Fail_safety_mode
 * Return Value : void
 ***********************************************************************************************************************/
static void Run_fail_safety_final(void)
{
    switch (fail_safety_step)
    {
    case 12:
        Fail_safety_init(13);
        break;
    case 13:
        Fail_safety_check_init_end();
        break;
    case 14:
        Execute_final_recovery_action();
        break;
    case 15:
        Check_stall_stop_next_step(16);
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

/* =========================================================================================
 * Main Fail Safety Mode Dispatcher
 * ========================================================================================= */

/***********************************************************************************************************************
 * Function Name: Fail_safety_mode
 * Description  : Fail-Safety 모드의 진입점. 현재 단계(step)에 따라 1차/2차/최종 사이클 함수를 호출함.
 * Called By    : Safety_protect_chk
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Fail_safety_mode(void)
{
    if (fail_safety_flag == ON)
    {
        // [Cycle 1] Steps 0 ~ 5
        if (fail_safety_step <= 5U)
        {
            Run_fail_safety_cycle_1();
        }
        // [Cycle 2] Steps 6 ~ 11
        else if (fail_safety_step <= 11U)
        {
            Run_fail_safety_cycle_2();
        }
        // [Final Cycle] Steps 12 ~ 16
        else
        {
            Run_fail_safety_final();
        }
    }
}

// static void Initialize_variables(void)
// {
// 	motor_start = OFF;
// 	time_1us_motor_flag = 0;
// 	time_1us_motor = 0;

// 	motor_stall_flag = MOTOR_NORMAL;
// 	motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE;
// 	time_1ms_spi = 0;
// 	time_1ms_init_chk_flag = 0;
// 	time_1ms_init_chk = 0;
// 	init_move_step = 0;
// 	time_1ms_init_move_flag = 0;
// 	time_1ms_init_move = 0;

// 	fail_safety_1_cycle_flag = OFF;
// 	time_1ms_external_10s_chk_flag = OFF;
// 	time_1ms_external_10s_chk = 0;

// 	step_start_flag = OFF;
// 	time_1ms_stall_chk_flag = OFF;

// 	spi_action_step = 0;
// 	adc_chk_ok_flag = 0;

// 	spi_receive_flag = 0;
// 	spi_send_flag = 0;

// 	time_1us_spi_flag = 0;
// 	time_1us_spi = 0;

// 	stall_chk_cnt = 0;
// 	stall_chk_time_1ms = 0; // stall reset
// 	fail_safety_1_cycle_flag = OFF;
// 	fail_safety_flag = OFF;
// 	fail_safety_step = 0;

// 	time_1ms_3minute_flag = 0;
// 	time_1s_3minute = 0;
// 	time_1ms_3minute = 0;

// 	timer_1ms_lin_bus_inactive = 0;

// 	lin_bus_inactive_flag = OFF;

// 	// wake_up_flag = 1;

// 	lin_rx_chk_flag = OFF;
// }

static void Lin_bus_chk(void)
{
    if ((timer_1ms_lin_bus_inactive >= LIN_BUS_CHK_TIME_4_SEC) && (lin_bus_inactive_flag == OFF) && (LIN_Short_Ok == 0))
    {
        lin_bus_inactive_flag = ON;

        DRV_Off();                            // drv of
        motor_start = OFF;                    // step stop
        time_1ms_external_10s_chk_flag = OFF; // 10s chk timer off
        time_1ms_external_10s_chk = 0;
        aaf_action = FLAP_STOP;
        aaf_action_complete_chk = FLAP_STOP;
        softstart_complete = OFF;
        motor_step_value = STEP_TIME_1000RPM;

        time_1ms_stall_chk = 0;      // test
        time_1ms_stall_chk_flag = 0; // test

        lin_sleep_step = 0;

        wake_up_motor_range_init_chk = 0;

        aaf_step = AAF_WAITING;
    }
}

/***********************************************************************************************************************
 * Function Name: Reset_protection_mode_start
 * Description  : Protection Mode 진입 시 초기화 수행 (Case 0)
 * Called By    : Process_protection_mode_prep
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Reset_protection_mode_start(void)
{
    DRV_Off();
    motor_start = OFF;
    stall_chk_cnt = 0;
    stall_chk_time_1ms = 0; // stall reset
    protection_Mode_step = 1;
    time_1ms_protection_mode_flag = 1;
    aaf_action = FLAP_STOP;
    softstart_complete = OFF;
}

/***********************************************************************************************************************
 * Function Name: Wait_protection_mode_delay
 * Description  : 100ms 대기 후 다음 단계로 이동 (Case 1)
 * Called By    : Process_protection_mode_prep
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Wait_protection_mode_delay(void)
{
    if (time_1ms_protection_mode >= 100U)
    {
        time_1ms_protection_mode_flag = 0;
        time_1ms_protection_mode = 0;
        protection_Mode_step = 2;
    }
}

/***********************************************************************************************************************
 * Function Name: Start_protection_mode_move
 * Description  : 조건 확인 후 모터 Open 구동 시작 (Case 2)
 * Called By    : Process_protection_mode_action
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Start_protection_mode_move(void)
{
    // if(((adc_avr >= ADC_UNDER_VOLTAGE_7V) && (adc_avr <= ADC_UNDER_VOLTAGE_9V)) || ((adc_avr >= ADC_OVER_VOLTAGE_16V) && (adc_avr <= ADC_OVER_VOLTAGE_18V)) || (AAF_ProtectionMode_Rx == ON))
    // if(((adc_avr >= ADC_OVER_VOLTAGE_16V) && (adc_avr <= ADC_OVER_VOLTAGE_18V)) || (AAF_ProtectionMode_Rx == ON) && (AAF_Tx_Position != OPEN))  
    if ((AAF_Tx_Position != OPEN))
    {
        Motor_dir_open();                // dir OPEN
        DRV8899_On();                    // drv on
        motor_start = ON;                // step start
        time_1ms_stall_chk_flag = ON;    // test
        motor_stall_flag = MOTOR_NORMAL; // stall reset
        stall_chk_time_1ms = 0;          // stall reset
        protection_Mode_step = 3;
        time_1ms_init_chk_flag = 1; // test
    }
    else
    {
        time_1ms_init_chk_flag = 0;
        protection_Mode_step = 4;
    }
}

/***********************************************************************************************************************
 * Function Name: Check_protection_mode_stop
 * Description  : 목표 위치 도달 또는 스톨 발생 시 정지 (Case 3)
 * Called By    : Process_protection_mode_action
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Check_protection_mode_stop(void)
{
    // if ((motor_stall_flag == MOTOR_STALL) || (time_1ms_init_chk >= 4500U))
    if ((step_position <= (step_position_open + limit_step_position)) || (motor_stall_flag == MOTOR_STALL))
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
}

/***********************************************************************************************************************
 * Function Name: Finalize_protection_mode
 * Description  : Protection Mode 최종 상태 업데이트 (Case 4)
 * Called By    : Process_protection_mode_action
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Finalize_protection_mode(void)
{
    AAF_Tx_Position = UNKOWN_POSITION;
    AAFx_InitStatus = DURING_INITIALIZATION;
    AAFx_Position_Status = Unknown_Status;
    // Flash_memory_write();
}

static void Process_protection_mode_prep(void)
{
    switch (protection_Mode_step)
    {
    case 0:
        Reset_protection_mode_start();
        break;
    case 1:
        Wait_protection_mode_delay();
        break;
    default:
        break;
    }
}

static void Process_protection_mode_action(void)
{
    switch (protection_Mode_step)
    {
    case 2:
        Start_protection_mode_move();
        break;
    case 3:
        Check_protection_mode_stop();
        break;
    case 4:
        Finalize_protection_mode();
        break;
    default:
        break;
    }
}

/***********************************************************************************************************************
 * Function Name: Protection_Mode
 * Description  : Protection Mode 동작 시퀀스 제어
 * Metric Info  : FUCYC = 3 (Pass), FUNDC = 2 (Pass)
 * Called By    : Main Loop
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Protection_Mode(void)
{
    if (stall_test_mode == 0U)
    {
        // Step 0 ~ 1: 준비 및 대기
        if (protection_Mode_step <= 1)
        {
            Process_protection_mode_prep();
        }
        // Step 2 ~ 4: 동작 및 완료
        else
        {
            Process_protection_mode_action();
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

// static void Current_limiting_select(void)
// {
// 	if (voltage_status_spi == LOW_VOLTAGE_1ST)
// 	{
// 		if (adc_avr >= ADC_VOLTAGE_15V)
// 		{
// 			voltage_status_spi = HIGH_VOLTAGE_1ST;
// 			voltage_status_change = ON;
// 		}
// 		else if (adc_avr >= ADC_VOLTAGE_13_7V)
// 		{
// 			voltage_status_spi = NORMAL_VOLTAGE;
// 			voltage_status_change = ON;
// 		}
// 		else
// 		{
// 			//invalid
// 		}
// 	}
// 	else if (voltage_status_spi == NORMAL_VOLTAGE)
// 	{
// 		if (adc_avr <= ADC_VOLTAGE_13_5V)
// 		{
// 			voltage_status_spi = LOW_VOLTAGE_1ST;
// 			voltage_status_change = ON;
// 		}
// 		else if (adc_avr >= ADC_VOLTAGE_15V)
// 		{
// 			voltage_status_spi = HIGH_VOLTAGE_1ST;
// 			voltage_status_change = ON;
// 		}
// 		else
// 		{
// 			//invalid
// 		}
// 	}
// 	else if (voltage_status_spi == HIGH_VOLTAGE_1ST)
// 	{
// 		if (adc_avr <= ADC_VOLTAGE_13_7V)
// 		{
// 			voltage_status_spi = LOW_VOLTAGE_1ST;
// 			voltage_status_change = ON;
// 		}
// 		else if (adc_avr <= ADC_VOLTAGE_14_8V)
// 		{
// 			voltage_status_spi = NORMAL_VOLTAGE;
// 			voltage_status_change = ON;
// 		}
// 		else
// 		{
// 			//invalid
// 		}
// 	}
// 	else
// 	{
// 		//invalid
// 	}
// }

/* =========================================================================================
 * LIN Sleep Mode
 * ========================================================================================= */

/***********************************************************************************************************************
 * Function Name: Stop_motor_and_reset
 * Description  : 모터 구동 정지 및 제어 변수 리셋 (Case 0, 4 공통)
 * Called By    : Reset_lin_sleep_start, Check_lin_motor_completion
 ***********************************************************************************************************************/
static void Stop_motor_and_reset(void)
{
    DRV_Off();
    motor_start = OFF;
    stall_chk_cnt = 0;
    stall_chk_time_1ms = 0;
    softstart_complete = OFF;
    motor_step_value = STEP_TIME_1000RPM;
}

/***********************************************************************************************************************
 * Function Name: Reset_lin_sleep_start
 * Description  : Lin Sleep 모드 초기 진입 시 변수 초기화 수행 (Case 0)
 * Called By    : Process_lin_sleep_prep
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Reset_lin_sleep_start(void)
{
    Stop_motor_and_reset();

    // Case 0에만 있는 추가 초기화
    timer_1ms_init_fail_chk_flag = 0;
    timer_1ms_init_fail_chk = 0;

    lin_sleep_step = 1;
}

/***********************************************************************************************************************
 * Function Name: Wait_lin_sleep_delay
 * Description  : 50ms 동안 대기 후 다음 단계로 이동 (Case 1)
 * Called By    : Process_lin_sleep_prep
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Wait_lin_sleep_delay(void)
{
    timer_1ms_lin_sleep_flag = 1;

    if (timer_1ms_lin_sleep >= 50U)
    {
        timer_1ms_lin_sleep_flag = 0;
        timer_1ms_lin_sleep = 0;
        lin_sleep_step = 2;
    }
}

/***********************************************************************************************************************
 * Function Name: Parse_lin_wakeup_command
 * Description  : LIN 통신으로 수신된 Wakeup 명령을 해석하여 동작(Open/Close)을 결정함 (Case 2)
 * Called By    : Process_lin_sleep_prep
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Parse_lin_wakeup_command(void)
{
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
            // invalid
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
        // invalid
    }
}

/***********************************************************************************************************************
 * Function Name: Process_lin_sleep_prep
 * Description  : LIN Sleep 전반부(준비) 단계 처리 (초기화 -> 대기 -> 명령 해석)
 * Called By    : Lin_sleep (Step 0 ~ 2)
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Process_lin_sleep_prep(void)
{
    switch (lin_sleep_step)
    {
    case 0:
        Reset_lin_sleep_start();
        break;
    case 1:
        Wait_lin_sleep_delay();
        break;
    case 2:
        Parse_lin_wakeup_command();
        break;
    default:
        break;
    }
}

/***********************************************************************************************************************
 * Function Name: Start_lin_motor_move
 * Description  : 결정된 Action(OPEN/CLOSE)에 따라 모터 구동을 시작함 (Case 3)
 * Called By    : Process_lin_sleep_move
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Start_lin_motor_move(void)
{
    if (aaf_action == OPEN)
    {
        Motor_dir_open();
        DRV_On();
        motor_start = ON;
        motor_stall_flag = MOTOR_NORMAL;
        stall_chk_time_1ms = 0;
        motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE;
        time_1ms_spi = 0;

        lin_sleep_step = 4;
    }
    else if (aaf_action == CLOSE)
    {
        Motor_dir_close();
        DRV_On();
        motor_start = ON;
        motor_stall_flag = MOTOR_NORMAL;
        stall_chk_time_1ms = 0;
        motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE;
        time_1ms_spi = 0;

        lin_sleep_step = 4;
    }
    else
    {
        lin_sleep_step = 5;
    }
}

/***********************************************************************************************************************
 * Function Name: Check_lin_motor_completion
 * Description  : 모터 구동 중 목표 위치 도달 또는 스톨 발생 여부를 확인하고 정지함 (Case 4)
 * Called By    : Process_lin_sleep_move
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Check_lin_motor_completion(void)
{
    // 조건 1: OPEN 방향 목표 위치 도착
    if ((aaf_action == OPEN) && (step_position <= (step_position_open + limit_step_position)))
    {
        Stop_motor_and_reset(); // 공통 정지

        AAF_Tx_Position = OPEN;
        AAFx_Position_Status = Open_Status;
        Tx_position_complete_chk();
        aaf_step = FINISHED_OPERATE;

        lin_sleep_step = 5;
    }
    // 조건 2: CLOSE 방향 목표 위치 도착
    else if ((aaf_action == CLOSE) && (step_position >= (step_position_close - limit_step_position)))
    {
        Stop_motor_and_reset(); // 공통 정지

        AAF_Tx_Position = CLOSE;
        AAFx_Position_Status = Close_Status;
        AAFx_InitStatus = NORMAL_INITIALIZATION;
        Tx_position_complete_chk();
        aaf_step = FINISHED_OPERATE;

        lin_sleep_step = 5;
    }
    // 조건 3: 스톨 발생
    else if (motor_stall_flag == MOTOR_STALL)
    {
        Stop_motor_and_reset(); // 공통 정지

        aaf_step = AAF_INITIALIZATION;
        aaf_init_step = WAIT_INITIALIZATION;
        AAF_Tx_Position = UNKOWN_POSITION;
        AAFx_Position_Status = Unknown_Status;
        AAFx_InitStatus = DURING_INITIALIZATION;
        motor_stall_flag = MOTOR_NORMAL;

        lin_sleep_step = 5;
    }
    else
    {
        // invalid
    }
}

/***********************************************************************************************************************
 * Function Name: Process_final_sleep
 * Description  : 최종 Sleep 진입 전 상태를 정리하고 MCU Sleep을 호출함 (Case 5)
 * Called By    : Process_lin_sleep_move
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Process_final_sleep(void)
{
    if ((AAF_Tx_Position == UNKOWN_POSITION) || (AAFx_Position_Status == Unknown_Status) || (AAFx_InitStatus == DURING_INITIALIZATION))
    {
        aaf_init_step = WAIT_INITIALIZATION;
        AAF_Tx_Position = UNKOWN_POSITION;
        AAFx_Position_Status = Unknown_Status;
        AAFx_InitStatus = DURING_INITIALIZATION;
    }
    MCU_sleep();
}

/***********************************************************************************************************************
 * Function Name: Process_lin_sleep_move
 * Description  : LIN Sleep 후반부(동작) 단계 처리 (구동 시작 -> 완료 확인 -> Sleep)
 * Called By    : Lin_sleep (Step 3 ~ 5)
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Process_lin_sleep_move(void)
{
    switch (lin_sleep_step)
    {
    case 3:
        Start_lin_motor_move();
        break;
    case 4:
        Check_lin_motor_completion();
        break;
    case 5:
        Process_final_sleep();
        break;
    default:
        break;
    }
}

/***********************************************************************************************************************
 * Function Name: Lin_sleep
 * Description  : LIN Sleep 모드 진입 및 Wakeup 동작 시퀀스 전체 제어
 * Called By    : Main Loop
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Lin_sleep(void)
{
    // Step 0 ~ 2: 준비 및 명령 수신 단계
    if (lin_sleep_step <= 2)
    {
        Process_lin_sleep_prep();
    }
    // Step 3 ~ 5: 모터 구동 및 Sleep 단계
    else
    {
        Process_lin_sleep_move();
    }
}

/* =========================================================================================
 * Anti-Pinch Movement Helper Functions
 * ========================================================================================= */

/***********************************************************************************************************************
 * Function Name: Antipinch_prev_open
 * Description  : 이전에 OPEN 동작 중 끼임이 발생했을 때의 복구 시퀀스 (Close -> Wait -> Open)
 * Called By    : Antipinch_move
 ***********************************************************************************************************************/
static void Antipinch_prev_open(void)
{
    switch (antipinch_step)
    {
    case 0:
        Motor_Close();
        DRV_On();
        motor_start = ON;
        time_1ms_init_chk_flag = 1;

        aaf_action = CLOSE;
        AAFx_ErrorStatus = Open_ErrorStatus;
        time_1ms_spi = 0;
        motor_stall_flag = MOTOR_NORMAL;
        antipinch_step = 1;
        break;

    case 1:
        if ((motor_stall_flag == MOTOR_STALL) || (time_1ms_init_chk >= 4500U))
        {
            DRV_Off();
            motor_start = OFF;
            stall_chk_cnt = 0;
            stall_chk_time_1ms = 0;
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
        Motor_Open();
        DRV_On();
        motor_start = ON;

        stall_chk_time_1ms = 0;
        motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE;
        motor_stall_flag = MOTOR_NORMAL;
        time_1ms_spi = 0;
        aaf_action = OPEN;

        time_1ms_init_chk_flag = 1;

        antipinch_step = 4;
        break;

    case 4:
        if ((aaf_action == OPEN) && (step_position <= step_position_open + limit_step_position))
        {
            DRV_Off();
            motor_start = OFF;

            // 진단 모드 여부에 따른 분기
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
                    // invalid
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

/***********************************************************************************************************************
 * Function Name: Antipinch_prev_close
 * Description  : 이전에 CLOSE 동작 중 끼임이 발생했을 때의 복구 시퀀스 (Open -> Wait -> Close)
 * Called By    : Antipinch_move
 ***********************************************************************************************************************/
static void Antipinch_prev_close(void)
{
    AAFx_InitStatus = DURING_INITIALIZATION;

    switch (antipinch_step)
    {
    case 0:
        Motor_Open();
        DRV_On();

        stall_chk_time_1ms = 0;

        motor_start = ON;
        time_1ms_init_chk_flag = 1;
        motor_stall_flag = MOTOR_NORMAL;
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
            stall_chk_time_1ms = 0;
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

            time_1ms_init_chk_flag = 0;
            time_1ms_init_chk = 0;
        }
        break;

    case 3:
        Motor_Close();
        DRV_On();
        motor_start = ON;
        motor_stall_flag = MOTOR_NORMAL;

        stall_chk_time_1ms = 0;
        motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE;
        time_1ms_spi = 0;

        aaf_action = CLOSE;
        time_1ms_init_chk_flag = 1;
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
                    // invalid
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
        Motor_Open();
        DRV_On();
        motor_start = ON;
        time_1ms_init_chk_flag = 1;
        aaf_action = OPEN;
        motor_stall_flag = MOTOR_NORMAL;
        time_1ms_spi = 0;

        stall_chk_time_1ms = 0;
        motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE;

        antipinch_step = 6;
        break;

    case 6:
        if ((motor_stall_flag == MOTOR_STALL) || (time_1ms_init_chk >= 4500U))
        {
            DRV_Off();
            motor_start = OFF;
            stall_chk_cnt = 0;
            stall_chk_time_1ms = 0;
            softstart_complete = OFF;
            motor_step_value = STEP_TIME_1000RPM;

            aaf_action = FLAP_STOP;
            time_1ms_init_chk_flag = 0;
            time_1ms_init_chk = 0;

            antipinch_step = 7;

            // fail_safety_flag = ON;
            antipinch_action_on = OFF;

            antipinch_previous_action = INITIALIZATION;

            time_1ms_external_10s_chk_flag = OFF;
            time_1ms_external_10s_chk = 0;
            aaf_action = FLAP_STOP;
            aaf_action_complete_chk = FLAP_STOP;

            time_1ms_stall_chk = 0;
            time_1ms_stall_chk_flag = 0;

            motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE;

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

/* =========================================================================================
 * Main Anti-Pinch Function
 * ========================================================================================= */

/***********************************************************************************************************************
 * Function Name: Antipinch_move
 * Description  : Anti-Pinch(끼임 방지) 동작 시퀀스 제어. 이전 동작 방향에 따라 반대 방향으로 회피 구동함.
 * Called By    : Main Loop (Operating_mode)
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Antipinch_move(void)
{
    if (antipinch_action_on == ON)
    {
        AAFx_Position_Status = Unknown_Status;
        AAF_Tx_Position = UNKOWN_POSITION;
        AAFx_InitStatus = DURING_INITIALIZATION;

        if (antipinch_previous_action == OPEN)
        {
            Antipinch_prev_open();
        }
        else if (antipinch_previous_action == CLOSE)
        {
            Antipinch_prev_close();
        }
        else
        {
            // invalid
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
                Motor_Open();                    // dir OPEN
                DRV_On();                        // drv on
                motor_start = ON;                // step start
                time_1ms_stall_chk_flag = ON;    // test
                motor_stall_flag = MOTOR_NORMAL; // stall reset
                stall_chk_time_1ms = 0;          // stall reset
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
            Motor_Open();     // dir CLOSE
            DRV_On();         // drv on
            motor_start = ON; // step start

            motor_stall_flag = MOTOR_NORMAL;                  // stall reset
            stall_chk_time_1ms = 0;                           // stall reset
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
                // spi_fail = 1;
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
                // spi_fail = 1;
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

/* End user code. Do not edit comment generated here */
