#include "Lin_Sleep.h"
#include "Service.h"

/***********************************************************************************************************************
 * Function Name: LinSleep_StopMotorAndReset
 * Description  : 모터 구동 정지 및 제어 변수 리셋 (Case 0, 4 공통)
 * Called By    : LinSleep_Reset, LinSleep_CheckCompletion
 ***********************************************************************************************************************/
static void LinSleep_StopMotorAndReset(void)
{
    Drv8889_Off2();
    motor_start = OFF;
    G_Timer1msFlag.StallTimeFlag = 0U;
    G_Timer1ms.StallTime = 0U;
    softstart_complete = OFF;
    motor_step_value = STEP_TIME_1000RPM;
    fail_safety_flag = OFF;
    fail_safety_step = 0U;
}

/***********************************************************************************************************************
 * Function Name: LinSleep_Reset
 * Description  : Lin Sleep 모드 초기 진입 시 변수 초기화 수행 (Case 0)
 * Called By    : LinSleep_Cycle1
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void LinSleep_Reset(void)
{
    LinSleep_StopMotorAndReset();

    // Case 0에만 있는 추가 초기화
    G_Timer1msFlag.InitFailCheckFlag = 0U;
    G_Timer1ms.InitFailCheck = 0U;

    lin_sleep_step = 1U;
}

/***********************************************************************************************************************
 * Function Name: LinSleep_Delay
 * Description  : 50ms 동안 대기 후 다음 단계로 이동 (Case 1)
 * Called By    : LinSleep_Cycle1
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void LinSleep_Delay(void)
{
    G_Timer1msFlag.LinSleepModeFlag = 1U;

    if (G_Timer1ms.LinSleepMode >= 50U)
    {
        G_Timer1msFlag.LinSleepModeFlag = 0U;
        G_Timer1ms.LinSleepMode = 0U;
        lin_sleep_step = 2U;
    }
}

/***********************************************************************************************************************
 * Function Name: LinSleep_ParsingCommand
 * Description  : LIN 통신으로 수신된 Wakeup 명령을 해석하여 동작(Open/Close)을 결정함 (Case 2)
 * Called By    : LinSleep_Cycle1
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void LinSleep_ParsingCommand(void)
{
    if (AAF_LINOut == 0x00U)
    {
        if (lin_aaf_command == OPEN)
        {
            if (IGN_Chk == 0)
            {
                lin_sleep_step = 8U;
            }
            else
            {
                Drv8889_Wakeup();
                aaf_action = OPEN;
                Last_aaf_action = aaf_action;
                lin_sleep_step = 3U;
            }
        }
        else if (lin_aaf_command == CLOSE)
        {
            if (IGN_Chk == 0)
            {
                lin_sleep_step = 8U;
            }
            else
            {
                Drv8889_Wakeup();
                aaf_action = CLOSE;
                Last_aaf_action = aaf_action;
                lin_sleep_step = 3U;
            }
        }
        else if (lin_aaf_command == UNKOWN_POSITION)
        {
            if (IGN_Chk == 0)
            {
                lin_sleep_step = 8;
            }
            else if (AAFx_Last_Command == CLOSE)
            {
                Drv8889_Wakeup();
                aaf_action = CLOSE;
                Last_aaf_action = aaf_action;
                lin_sleep_step = 3;
            }
            else
            {
                Drv8889_Wakeup();
                aaf_action = OPEN;
                Last_aaf_action = aaf_action;
                lin_sleep_step = 3;
            }
        }
        else
        {
            lin_sleep_step = 8;
        }
    }
    else if (AAF_LINOut == 0x01U)
    {
        Drv8889_Wakeup();
        aaf_action = OPEN;
        Last_aaf_action = aaf_action;
        lin_sleep_step = 3U;
    }
    else
    {
        // invalid
    }
}

/***********************************************************************************************************************
 * Function Name: LinSleep_Cycle1
 * Description  : LIN Sleep 전반부(준비) 단계 처리 (초기화 -> 대기 -> 명령 해석)
 * Called By    : Lin_Sleep (Step 0 ~ 2)
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void LinSleep_Cycle1(void)
{
    switch (lin_sleep_step)
    {
    case 0:
        LinSleep_Reset();
        break;
    case 1:
        LinSleep_Delay();
        break;
    case 2:
        LinSleep_ParsingCommand();
        break;
    default:
        break;
    }
}

/***********************************************************************************************************************
 * Function Name: LinSleep_StartMotor
 * Description  : 결정된 Action(OPEN/CLOSE)에 따라 모터 구동을 시작함 (Case 3)
 * Called By    : LinSleep_Cycle2
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void LinSleep_StartMotor(void)
{
    if (Last_aaf_action == OPEN)
    {
        Motor_Open2();
        Drv8889_On2();
        motor_start = ON;
        motor_stall_flag = MOTOR_NORMAL;
        G_Timer1ms.StallTime = 0U;
        motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE;
        G_Timer1ms.Spi = 0U;

        lin_sleep_step = 4U;
    }
    else if (Last_aaf_action == CLOSE)
    {
        Motor_Close2();
        Drv8889_On2();
        motor_start = ON;
        motor_stall_flag = MOTOR_NORMAL;
        G_Timer1ms.StallTime = 0U;
        motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE;
        G_Timer1ms.Spi = 0U;

        lin_sleep_step = 4U;
    }
    else
    {
        lin_sleep_step = 8U;
    }
}

/***********************************************************************************************************************
 * Function Name: LinSleep_CheckCompletion
 * Description  : 모터 구동 중 목표 위치 도달 또는 스톨 발생 여부를 확인하고 정지함 (Case 4)
 * Called By    : LinSleep_Cycle2
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void LinSleep_CheckCompletion(void)
{
    // 조건 1: OPEN 방향 목표 위치 도착
    if ((Last_aaf_action == OPEN) && (step_position <= (step_position_open + limit_step_position)))
    {
        LinSleep_StopMotorAndReset(); // 공통 정지

        if ((EngRunSta == 0x00U) && (HevRdy == 0x00U))
        {
            AAF_Tx_Position = OPEN;
            AAFx_Position_Status = Open_Status;
            AAFx_InitStatus = NORMAL_FINISHED_INITIALIZATION;
            Position_Temporary_write();
        }
        else
        {
            AAF_Tx_Position = OPEN;
            AAFx_Position_Status = Open_Status;
            AAFx_InitStatus = NORMAL_FINISHED_INITIALIZATION;
        }
        Operate_SelectTxPostion();
        aaf_step = FINISHED_OPERATE;

        lin_sleep_step = 8U;
    }
    // 조건 2: 외장형 CLOSE 방향 Stopper 도착
    else if ((Last_aaf_action == CLOSE) && (motor_stall_flag == MOTOR_STALL) && (step_position >= (step_position_close - limit_step_position)) && (AAFx_Type == EXTERNAL_TYPE))
    {
        LinSleep_StopMotorAndReset(); // 공통 정지

        motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE;
        step_position = step_position_close;
        motor_stall_flag = MOTOR_NORMAL;
        lin_sleep_step = 5U;
    }
    // 조건 2: 내장형 CLOSE 방향 TBD 도착
    else if ((Last_aaf_action == CLOSE) && (step_position >= (step_position_close - limit_step_position)) && (AAFx_Type == INTERNAL_TYPE))
    {
        LinSleep_StopMotorAndReset(); // 공통 정지
        if ((EngRunSta == 0x00U) && (HevRdy == 0x00U))
        {
            AAF_Tx_Position = CLOSE;
            AAFx_Position_Status = Close_Status;
            AAFx_InitStatus = NORMAL_FINISHED_INITIALIZATION;
            Position_Temporary_write();
        }
        else
        {
            AAF_Tx_Position = CLOSE;
            AAFx_Position_Status = Close_Status;
            AAFx_InitStatus = NORMAL_FINISHED_INITIALIZATION;
        }
        motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE;
        motor_stall_flag = MOTOR_NORMAL;
        lin_sleep_step = 8U;
    }
    // 조건 4: 스톨 발생
    else if (motor_stall_flag == MOTOR_STALL)
    {
        LinSleep_StopMotorAndReset(); // 공통 정지
        Sleep_Stall = ON;
        lin_sleep_step = 5U;
    }
    else
    {
        // invalid
    }
}
/***********************************************************************************************************************
 * Function Name: LinSleep_Stall_Delay
 * Description  : 500ms 동안 대기 후 다음 단계로 이동 (Case 5)
 * Called By    : LinSleep_Cycle2
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void LinSleep_Stall_Delay(void)
{
    G_Timer1msFlag.LinSleepStallFlag = 1U;

    if (G_Timer1ms.LinSleepStall >= 800U)
    {
        G_Timer1msFlag.LinSleepStallFlag = 0U;
        G_Timer1ms.LinSleepStall = 0U;

        if (Last_aaf_action == OPEN)
        {
            lin_sleep_step = 8U;
        }
        else
        {
            lin_sleep_step = 6U;
        }
    }
}
/***********************************************************************************************************************
 * Function Name: LinSleep_Cycle2
 * Description  : LIN Sleep 후반부(동작) 단계 처리 (구동 시작 -> 완료 확인 -> Sleep)
 * Called By    : Lin_Sleep (Step 3 ~ 5)
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void LinSleep_Cycle2(void)
{
    switch (lin_sleep_step)
    {
    case 3:
        LinSleep_StartMotor();
        break;
    case 4:
        LinSleep_CheckCompletion();
        break;
    case 5:
        LinSleep_Stall_Delay();
        break;
    default:
        break;
    }
}
/***********************************************************************************************************************
 * Function Name: LinSleep_Stall_Open
 * Description  : Open방향 이동 (Case 6)
 * Called By    : LinSleep_Cycle2
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void LinSleep_Stall_Open(void)
{
    Motor_Open2();
    Drv8889_On2();
    motor_start = ON;
    motor_stall_flag = MOTOR_NORMAL;
    G_Timer1ms.StallTime = 0U;
    motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE;
    G_Timer1ms.Spi = 0U;

    lin_sleep_step = 7;
}
/***********************************************************************************************************************
 * Function Name: LinSleep_Stall_Stop
 * Description  : 마지막동작 멈추기위한 부분 (Case 7)
 * Called By    : LinSleep_Cycle2
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void LinSleep_Stall_Stop(void)
{
    if ((motor_stall_flag == MOTOR_STALL) && (Sleep_Stall == ON))
    {
        LinSleep_StopMotorAndReset();
        AAF_Tx_Position = UNKOWN_POSITION;
        AAFx_Position_Status = Unknown_Status;
        AAFx_InitStatus = DURING_INITIALIZATION;
        AAFx_SNSR1_Position = Initial_Value;
        AAFx_SNSR2_Position = Initial_Value;
        AAFx_SNSR3_Position = Initial_Value;
        AAFx_SNSR4_Position = Initial_Value;
        lin_sleep_step = 8;
    }
    else if ((step_position <= (step_position_close - limit_step_position)) && (Sleep_Stall == OFF))
    {
        LinSleep_StopMotorAndReset(); // 공통 정지
        if ((EngRunSta == 0x00U) && (HevRdy == 0x00U))
        {
            AAF_Tx_Position = CLOSE;
            AAFx_Position_Status = Close_Status;
            AAFx_InitStatus = NORMAL_FINISHED_INITIALIZATION;
            Position_Temporary_write();
        }
        else
        {
            AAF_Tx_Position = CLOSE;
            AAFx_Position_Status = Close_Status;
            AAFx_InitStatus = NORMAL_FINISHED_INITIALIZATION;
        }

        // Operate_SelectTxPostion();
        aaf_step = FINISHED_OPERATE;
        G_Timer1ms.Spi = 0;
        lin_sleep_step = 8;
    }
    else
    {
    }
}
/***********************************************************************************************************************
 * Function Name: LinSleep_Final
 * Description  : 최종 Sleep 진입 전 상태를 정리하고 MCU Sleep을 호출함 (Case 5)
 * Called By    : LinSleep_Cycle2
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void LinSleep_Final(void)
{
    // if ((AAF_Tx_Position == UNKOWN_POSITION) || (AAFx_Position_Status == Unknown_Status) || (AAFx_InitStatus == DURING_INITIALIZATION))
    // {
    //     aaf_init_step = WAIT_INITIALIZATION;
    //     AAF_Tx_Position = UNKOWN_POSITION;
    //     AAFx_Position_Status = Unknown_Status;
    //     AAFx_InitStatus = DURING_INITIALIZATION;
    // }
    if (G_Timer1ms.LinBusInactive >= LIN_BUS_CHK_TIME_4_SEC)
    {
        MCU_Sleep();
    }
}

/***********************************************************************************************************************
 * Function Name: LinSleep_Cycle3
 * Description  : LIN Sleep 후반부(동작) 단계 처리 (구동 시작 -> 완료 확인 -> Sleep)
 * Called By    : Lin_Sleep (Step 3 ~ 5)
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void LinSleep_Cycle3(void)
{
    switch (lin_sleep_step)
    {
    case 6:
        LinSleep_Stall_Open();
        break;
    case 7:
        LinSleep_Stall_Stop();
        break;
    case 8:
        LinSleep_Final();
        break;
    default:
        break;
    }
}

/***********************************************************************************************************************
 * Function Name: McuSleep_ExternalOff
 * Description  : MCU가 슬립 모드로 진입하기 전, 연결된 외부 하드웨어(모터 드라이버, 트랜시버 등)를 끔
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void McuSleep_ExternalOff(void)
{
    Drv8889_Sleep();     // 모터 드라이버 슬립 전환
    LinTrcv_Off();       // LIN 트랜시버 전원 차단
    Drv8889_ScsActive(); // SPI 통신 핀 활성화
}

/***********************************************************************************************************************
 * Function Name: McuSleep_PortConfig
 * Description  : 슬립 모드 중 누설 전류 방지 및 LIN Wake-up 대기를 위해 GPIO 포트 상태를 재설정함
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void McuSleep_PortConfig(void)
{
    // 포트 기능을 GPIO 입/출력으로 리셋
    R_PORT_ResetAltFunc(Port10, 10U, Output);
    R_PORT_ResetAltFunc(Port10, 9U, Input);

    // LIN TX 핀을 Low로 설정하여 슬립 상태 유지 (Leakage 방지)
    PORT.P10 &= ~_PORT_Pn10_OUTPUT_HIGH; // MCU_LIN_Tx_Low Sleep go
}

/***********************************************************************************************************************
 * Function Name: McuSleep_InternalModuleStop
 * Description  : 전력 소모를 줄이기 위해 MCU 내부 주변장치(ADC, 타이머, 통신 모듈)의 클럭을 정지함
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void McuSleep_InternalModuleStop(void)
{
    R_Config_INTC_Create(); // 인터럽트 컨트롤러 재설정 (Wake-up 준비)
    // R_Config_INTC_INTP5_Start(); // (주석 유지)

    R_Config_CSIH0_Stop();    // SPI 모듈 정지
    R_Config_ADCA0_Halt();    // ADC 모듈 정지
    R_Config_TAUD0_13_Stop(); // 타이머 정지
    R_Config_TAUD0_3_Stop();  // 타이머 정지

    G_Timer1msFlag.SpiFlag = 0U; // 관련 플래그 초기화
    G_Timer1ms.Spi = 0U;
}

/***********************************************************************************************************************
 * Function Name: McuSleep_DeepStop
 * Description  : 클럭 생성기를 슬립 모드용으로 설정하고, 최종적으로 Deep Stop Mode로 진입함
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void McuSleep_DeepStop(void)
{
    R_CGC_Create_sleepmode(); // 클럭 설정 변경

    R_Config_STBC_Prepare_Deep_Stop_Mode(); // 대기 모드 진입 준비 레지스터 설정
    R_Config_STBC_Start_Deep_Stop_Mode();   // [진입점] 여기서 MCU 동작 멈춤
}

/***********************************************************************************************************************
 * Function Name: Lin_Sleep
 * Description  : LIN Sleep 모드 진입 및 Wakeup 동작 시퀀스 전체 제어
 * Called By    : Main Loop
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
void Lin_Sleep(void)
{
    // Step 0 ~ 2: 준비 및 명령 수신 단계
    if (lin_sleep_step <= 2U)
    {
        LinSleep_Cycle1();
    }
    // Step 3 ~ 5: 마지막 구동 후 스톨에 따른 분기
    else if ((lin_sleep_step >= 3) && (lin_sleep_step <= 5))
    {
        LinSleep_Cycle2();
    }
    // Step 6 ~ 8: 명령에 따른 동작 후 Sleep 들어가는 단계
    else
    {
        LinSleep_Cycle3();
    }
}

/* =========================================================================================
 * MCU Sleep Mode Management Functions
 * ========================================================================================= */

/***********************************************************************************************************************
 * Function Name: MCU_Sleep
 * Description  : 시스템 종료 절차를 수행하고 MCU를 저전력 모드(Deep Stop)로 전환하는 메인 함수
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
void MCU_Sleep(void)
{
    // 1. 종료 상태 플래그 설정
    power_chk = Normal_Shutdown;
    First_Powerchk = 1U;

    // 2. 필요 시 플래시 메모리에 데이터 저장
    if (step_check_flag == 2U)
    {
        FDL_Write();
    }

    // 3. 외부 하드웨어 전원 차단
    McuSleep_ExternalOff();

    // 4. 슬립 대비 포트 설정 (누설 전류 방지)
    McuSleep_PortConfig();

    // 5. 내부 주변장치 클럭 정지
    McuSleep_InternalModuleStop();

    // 6. Deep Stop 모드 진입 (Wake-up 이벤트 발생 전까지 정지)
    McuSleep_DeepStop();
}
