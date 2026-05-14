#include "Obd_Check.h"
#include "Service.h"

static void Obd_CheckSensor1(void);
static void Obd_CheckSensor2(void);
static void Obd_CheckSensor3(void);
static void Obd_HandleErrorMove(void);
static void Obd_CheckRecovery(void);
static void Obd_CheckSensor1Stat(void);
static void Obd_CheckSensor2Stat(void);
static void Obd_CheckSensor3Stat(void);
static void Obd_ClearAllTimers(void);
static void Obd_ClearAllErrors(void);
static void Obd_RecoveryReset(void);

/*----------------------------------------------------------------------------*/
/* Static Functions                                                           */
/*----------------------------------------------------------------------------*/

static void Obd_CheckSensor1(void)
{
    if (OBD1_adc >= OBD_BAT_SHORT_LIMIT)
    {
        G_Timer1msFlag.ObdBatShortFlag = 1U;
        OBD_Return_flag = 1U;
        if (G_Timer1ms.ObdBatShort >= 10000U)
        {
            G_Timer1ms.ObdBatShort = 10000U;
            G_Timer1msFlag.ObdBatShortFlag = 0U;
            AAFx_SNSR_SCB = 1U;
            OBD_Short_Bat = 1U;
            OBD_Error_flag = 1U;
        }
    }
    else if ((OBD1_adc <= OBD_Open_LIMIT) && (AAFx_SNSR_SCG == NO_ERROR))
    {
        G_Timer1msFlag.ObdOpenCircuitFlag = 1U;
        OBD_Return_flag = 1U;
        if (G_Timer1ms.ObdOpenCircuit >= 10000U)
        {
            G_Timer1ms.ObdOpenCircuit = 10000U;
            G_Timer1msFlag.ObdOpenCircuitFlag = 0U;
            AAFx_SNSR_OC = 1U;
            OBD_Open_Circuit = 1U;
            OBD_Error_flag = 1U;
        }
    }
    else
    {
        if (OBD_Error_flag != 1U)
        {
            Obd_ClearAllTimers();
            Obd_ClearAllErrors();
        }
    }
}

static void Obd_CheckSensor2(void)
{
    if ((OBD1_adc >= OBD_BAT_SHORT_LIMIT) || (OBD2_adc >= OBD_BAT_SHORT_LIMIT))
    {
        G_Timer1msFlag.ObdBatShortFlag = 1U;
        OBD_Return_flag = 1U;
        if (G_Timer1ms.ObdBatShort >= 10000U)
        {
            G_Timer1ms.ObdBatShort = 10000U;
            G_Timer1msFlag.ObdBatShortFlag = 0U;
            AAFx_SNSR_SCB = 1U;
            OBD_Short_Bat = 1U;
            OBD_Error_flag = 1U;
        }
    }
    else if (((OBD1_adc <= OBD_Open_LIMIT) || (OBD2_adc <= OBD_Open_LIMIT)) && (AAFx_SNSR_SCG == NO_ERROR))
    {
        G_Timer1msFlag.ObdOpenCircuitFlag = 1U;
        OBD_Return_flag = 1U;
        if (G_Timer1ms.ObdOpenCircuit >= 10000U)
        {
            G_Timer1ms.ObdOpenCircuit = 10000U;
            G_Timer1msFlag.ObdOpenCircuitFlag = 0U;
            AAFx_SNSR_OC = 1U;
            OBD_Open_Circuit = 1U;
            OBD_Error_flag = 1U;
        }
    }
    else
    {
        if (OBD_Error_flag != 1U)
        {
            Obd_ClearAllTimers();
            Obd_ClearAllErrors();
        }
    }
}

static void Obd_CheckSensor3(void)
{
    if ((OBD1_adc >= OBD_BAT_SHORT_LIMIT) || (OBD2_adc >= OBD_BAT_SHORT_LIMIT) || (OBD3_adc >= OBD_BAT_SHORT_LIMIT))
    {
        G_Timer1msFlag.ObdBatShortFlag = 1U;
        OBD_Return_flag = 1U;
        if (G_Timer1ms.ObdBatShort >= 10000U)
        {
            G_Timer1ms.ObdBatShort = 10000U;
            G_Timer1msFlag.ObdBatShortFlag = 0U;
            AAFx_SNSR_SCB = 1U;
            OBD_Short_Bat = 1U;
            OBD_Error_flag = 1U;
        }
    }
    else if ((OBD1_adc <= OBD_Open_LIMIT) || (OBD2_adc <= OBD_Open_LIMIT) || (OBD3_adc <= OBD_Open_LIMIT))
    {
        G_Timer1msFlag.ObdOpenCircuitFlag = 1U;
        OBD_Return_flag = 1U;
        if (G_Timer1ms.ObdOpenCircuit >= 10000U)
        {
            G_Timer1ms.ObdOpenCircuit = 10000U;
            G_Timer1msFlag.ObdOpenCircuitFlag = 0U;
            AAFx_SNSR_OC = 1U;
            OBD_Open_Circuit = 1U;
            OBD_Error_flag = 1U;
        }
    }
    else
    {
        if (OBD_Error_flag != 1U)
        {
            Obd_ClearAllTimers();
            Obd_ClearAllErrors();
        }
    }
}

static void Obd_HandleErrorMove(void)
{
    switch (OBD_Error_move)
    {
    case 0U:
        Motor_Open();     // dir CLOSE
        Drv8889_On();     // drv on
        motor_start = ON; // step start
        aaf_action = OPEN;
        motor_stall_flag = MOTOR_NORMAL;                  // stall reset
        G_Timer1ms.StallTime = 0U;                        // stall reset
        motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; // stall reset
        G_Timer1ms.Spi = 0U;
        OBD_Error_move = 1U;
        break;
    case 1U:
        if (((aaf_action == OPEN) && (step_position <= (step_position_open + limit_step_position)))) //
        {
            Error_UnknownStatus();
            OBD_Error_move = 2U;
        }
        else if ((motor_stall_flag == MOTOR_STALL))
        {
            Error_UnknownStatus();
            OBD_Error_move = 2U;
        }
        else
        {
            // invalid
        }
        break;
    case 2U:
        OBD_Error_check_flag = 1U;
        break;

    default:
        break;
    }
}

static void Obd_CheckRecovery(void)
{
    if ((TotalNumOfAAFSensor == SENSOR_X1) && (SNSR1_Check == USE_SNSR1))
    {
        G_Timer1msFlag.ObdRecoveryCheckFlag = 1U;
        if (G_Timer1ms.ObdRecoveryCheck >= 1000U)
        {
            Obd_RecoveryReset();
        }
    }
    else if ((TotalNumOfAAFSensor == SENSOR_X2) && (SNSR1_Check == USE_SNSR1) && (SNSR2_Check == USE_SNSR2))
    {
        G_Timer1msFlag.ObdRecoveryCheckFlag = 1U;
        if (G_Timer1ms.ObdRecoveryCheck >= 1000U)
        {
            Obd_RecoveryReset();
        }
    }
    else if ((TotalNumOfAAFSensor == SENSOR_X3) && (SNSR1_Check == USE_SNSR1) && (SNSR2_Check == USE_SNSR2) && (SNSR3_Check == USE_SNSR3))
    {
        G_Timer1msFlag.ObdRecoveryCheckFlag = 1U;
        if (G_Timer1ms.ObdRecoveryCheck >= 1000U)
        {
            Obd_RecoveryReset();
        }
    }
    else
    {
        G_Timer1ms.ObdRecoveryCheck = 0U;
        G_Timer1msFlag.ObdRecoveryCheckFlag = 0U;
    }
}

static void Obd_CheckSensor1Stat(void)
{
    if ((AAFx_Position_Status == FlapMoving_Status) && (SNSR1_Check == USE_SNSR1) && (AAFx_InitStatus == NORMAL_FINISHED_INITIALIZATION) && (Diag_Mode == 0U))
    {
        if ((OBD1_adc <= OBD1_adc_threshold_close) && (AAFx_SNSR1_Position != SNSR1_Close))
        {
            AAFx_SNSR1_Position = SNSR1_Close;
            SNSR_Position_Ok = 1U;
        }
        else if ((OBD1_adc >= OBD1_adc_threshold_open) && (AAFx_SNSR1_Position != SNSR1_Open))
        {
            AAFx_SNSR1_Position = SNSR1_Open;
            SNSR_Position_Ok = 1U;
        }
        else
        {
            // invalid
        }
    }
    else if ((AAFx_Position_Status != FlapMoving_Status) && (SNSR1_Check == USE_SNSR1) && (AAFx_InitStatus == NORMAL_FINISHED_INITIALIZATION) && (Diag_Mode == 0U))
    {
        if ((OBD1_adc <= OBD1_adc_threshold_close) && (AAFx_SNSR1_Position != SNSR1_Close))
        {
            AAFx_SNSR1_Position = SNSR1_Close;
            SNSR_Position_Ok = 1U;
        }
        else if ((OBD1_adc >= OBD1_adc_threshold_open) && (AAFx_SNSR1_Position != SNSR1_Open))
        {
            AAFx_SNSR1_Position = SNSR1_Open;
            SNSR_Position_Ok = 1U;
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

static void Obd_CheckSensor2Stat(void)
{
    if ((AAFx_Position_Status == FlapMoving_Status) && (SNSR2_Check == USE_SNSR2) && (AAFx_InitStatus == NORMAL_FINISHED_INITIALIZATION) && (Diag_Mode == 0U))
    {
        if ((OBD2_adc <= OBD2_adc_threshold_close) && (AAFx_SNSR2_Position != SNSR2_Close))
        {
            AAFx_SNSR2_Position = SNSR2_Close;
            SNSR2_Position_Ok = 1U;
        }
        else if ((OBD2_adc >= OBD2_adc_threshold_open) && (AAFx_SNSR2_Position != SNSR2_Open))
        {
            AAFx_SNSR2_Position = SNSR2_Open;
            SNSR2_Position_Ok = 1U;
        }
        else
        {
            // invalid
        }
    }
    else if ((AAFx_Position_Status != FlapMoving_Status) && (SNSR2_Check == USE_SNSR2) && (AAFx_InitStatus == NORMAL_FINISHED_INITIALIZATION) && (Diag_Mode == 0U))
    {
        if (OBD2_adc <= OBD2_adc_threshold_close)
        {
            AAFx_SNSR2_Position = SNSR2_Close;
            SNSR2_Position_Ok = 1U;
        }
        else if (OBD2_adc >= OBD2_adc_threshold_open)
        {
            AAFx_SNSR2_Position = SNSR2_Open;
            SNSR2_Position_Ok = 1U;
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

static void Obd_CheckSensor3Stat(void)
{
    if ((AAFx_Position_Status == FlapMoving_Status) && (SNSR3_Check == USE_SNSR3))
    {
        if ((OBD3_adc <= OBD3_adc_threshold_close) && (AAFx_SNSR3_Position != SNSR3_Close))
        {
            AAFx_SNSR3_Position = SNSR3_Close;
            SNSR3_Position_Ok = 1U;
        }
        else if ((OBD3_adc >= OBD3_adc_threshold_open) && (AAFx_SNSR3_Position != SNSR3_Open))
        {
            AAFx_SNSR3_Position = SNSR3_Open;
            SNSR3_Position_Ok = 1U;
        }
        else
        {
            // invalid
        }
    }
    else if ((AAFx_Position_Status != FlapMoving_Status) && (SNSR3_Check == USE_SNSR3) && (AAFx_InitStatus == NORMAL_FINISHED_INITIALIZATION) && (Diag_Mode == 0U))
    {
        if ((OBD3_adc <= OBD3_adc_threshold_close))
        {
            AAFx_SNSR3_Position = SNSR3_Close;
            SNSR3_Position_Ok = 1U;
        }
        else if ((OBD3_adc >= OBD3_adc_threshold_open))
        {
            AAFx_SNSR3_Position = SNSR3_Open;
            SNSR3_Position_Ok = 1U;
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

static void Obd_ErrorOperation(void)
{
    if ((TotalNumOfAAFSensor == SENSOR_X1) || (TotalNumOfAAFSensor == SENSOR_X2) || (TotalNumOfAAFSensor == SENSOR_X3))
    {
        if (((OBD_Short_Gnd == ERROR) || (OBD_Short_Bat == ERROR) || (OBD_Open_Circuit == ERROR)) && (OBD_Error_check_flag == 0U))
        {
            Obd_HandleErrorMove();
        }
        else if ((OBD_Error_flag == 1U) && (OBD_Error_check_flag == 1U))
        {
            Obd_CheckRecovery();
        }
        else
        {
            G_Timer1ms.ObdRecoveryCheck = 0U;
            G_Timer1msFlag.ObdRecoveryCheckFlag = 0U;
        }
    }
    else
    {
        // invalid
    }
}

static void Obd_ClearAllTimers(void)
{
    G_Timer1ms.ObdBatShort = 0U;
    G_Timer1msFlag.ObdBatShortFlag = 0U;
    G_Timer1ms.ObdGndShort = 0U;
    G_Timer1msFlag.ObdGndShortFlag = 0U;
    G_Timer1ms.ObdOpenCircuit = 0U;
    G_Timer1msFlag.ObdOpenCircuitFlag = 0U;
}

static void Obd_ClearAllErrors(void)
{
    OBD_Short_Gnd = NO_ERROR;
    OBD_Short_Bat = NO_ERROR;
    OBD_Open_Circuit = NO_ERROR;
    AAFx_SNSR_SCB = NO_ERROR;
    AAFx_SNSR_SCG = NO_ERROR;
    AAFx_SNSR_OC = NO_ERROR;
}

static void Obd_RecoveryReset(void)
{
    Re_Init();
    OBD_Short_Gnd = 0U;
    OBD_Short_Bat = 0U;
    OBD_Open_Circuit = 0U;
    AAFx_SNSR_SCB = 0U;
    AAFx_SNSR_SCG = 0U;
    AAFx_SNSR_OC = 0U;
    OBD_Error_flag = 0U;
    OBD_Error_move = 0U;
    OBD_Error_check_flag = 0U;
    G_Timer1ms.ObdRecoveryCheck = 0U;
    G_Timer1msFlag.ObdRecoveryCheckFlag = 0U;
}

void Obd_PositionStat1(void)
{
    if ((TotalNumOfAAFSensor == SENSOR_X1) || (TotalNumOfAAFSensor == SENSOR_X2) || (TotalNumOfAAFSensor == SENSOR_X3))
    {
        if (((OBD1_adc) >= OBD_ADC_MAX_LIMIT) || ((OBD1_adc) <= OBD_ADC_MIN_LIMIT))
        {
            SNSR1_Check = NO_SNSR1;
            AAFx_SNSR1_Position = Initial_Value;
        }
        else
        {
            SNSR1_Check = USE_SNSR1;
        }

        if (AAFx_InitStatus != NORMAL_FINISHED_INITIALIZATION)
        {
            AAFx_SNSR1_Position = Initial_Value;
            step_check_ok = 0U;
        }
        else if ((AAFx_InitStatus == NORMAL_FINISHED_INITIALIZATION) && (Diag_Mode == 0U))
        {
            if (step_check_ok == 0U)
            {
                if ((SNSR1_Check == USE_SNSR1))
                {
                    if (OBD1_adc <= OBD1_adc_threshold_close)
                    {
                        AAFx_SNSR1_Position = SNSR1_Close;
                    }
                    else if (OBD1_adc >= OBD1_adc_threshold_open)
                    {
                        AAFx_SNSR1_Position = SNSR1_Open;
                    }
                    else
                    {
                        // invalid
                    }
                }
                step_check_ok = 1U;
            }
        }
        else
        {
            // invalid
        }

        Obd_CheckSensor1Stat();
    }
}

void Obd_PositionStat2(void)
{
    if ((TotalNumOfAAFSensor == SENSOR_X2) || (TotalNumOfAAFSensor == SENSOR_X3))
    {
        if (((OBD2_adc) >= OBD_ADC_MAX_LIMIT) || ((OBD2_adc) <= OBD_ADC_MIN_LIMIT))
        {
            AAFx_SNSR2_Position = Initial_Value;
            SNSR2_Check = NO_SNSR2;
        }
        else
        {
            SNSR2_Check = USE_SNSR2;
        }

        if (AAFx_InitStatus != NORMAL_FINISHED_INITIALIZATION)
        {
            AAFx_SNSR2_Position = Initial_Value;
            step2_check_ok = 0U;
        }
        else if ((AAFx_InitStatus == NORMAL_FINISHED_INITIALIZATION) && (Diag_Mode == 0U))
        {
            if (step2_check_ok == 0U)
            {
                if ((SNSR2_Check == USE_SNSR2))
                {
                    if (OBD2_adc <= OBD2_adc_threshold_close)
                    {
                        AAFx_SNSR2_Position = SNSR2_Close;
                    }
                    else if (OBD2_adc >= OBD2_adc_threshold_open)
                    {
                        AAFx_SNSR2_Position = SNSR2_Open;
                    }
                    else
                    {
                        // invalid
                    }
                }
                step2_check_ok = 1U;
            }
        }
        else
        {
            // invalid
        }

        Obd_CheckSensor2Stat();
    }
}

void Obd_PositionStat3(void) // 0.8v 1100 1.6v 1700
{
    if (TotalNumOfAAFSensor == SENSOR_X3)
    {
        if (((OBD3_adc) >= OBD_ADC_MAX_LIMIT) || ((OBD3_adc) <= OBD_ADC_MIN_LIMIT))
        {
            SNSR3_Check = NO_SNSR3;
            AAFx_SNSR3_Position = Initial_Value;
        }
        else
        {
            SNSR3_Check = USE_SNSR3;
        }

        if (AAFx_InitStatus != NORMAL_FINISHED_INITIALIZATION)
        {
            AAFx_SNSR3_Position = Initial_Value;
            step3_check_ok = 0U;
        }
        else if ((AAFx_InitStatus == NORMAL_FINISHED_INITIALIZATION) && (Diag_Mode == 0U))
        {
            if (step3_check_ok == 0U)
            {
                if ((SNSR3_Check == USE_SNSR3))
                {
                    if ((OBD3_adc <= OBD3_adc_threshold_close))
                    {
                        AAFx_SNSR3_Position = SNSR3_Close;
                    }
                    else if (OBD3_adc >= OBD3_adc_threshold_open)
                    {
                        AAFx_SNSR3_Position = SNSR3_Open;
                    }
                    else
                    {
                        // invalid
                    }
                }
                step3_check_ok = 1U;
            }
        }
        else
        {
            // invalid
        }

        Obd_CheckSensor3Stat();
    }
}

/*----------------------------------------------------------------------------*/
/* Global Functions                                                           */
/*----------------------------------------------------------------------------*/

void Obd_DiagStatCheck(void) // HW 0.2 NO PULL UP
{
    if (TotalNumOfAAFSensor == SENSOR_X1)
    {
        Obd_CheckSensor1();
    }
    else if (TotalNumOfAAFSensor == SENSOR_X2)
    {
        Obd_CheckSensor2();
    }
    else if (TotalNumOfAAFSensor == SENSOR_X3)
    {
        Obd_CheckSensor3();
    }
    else
    {
        // invalid
    }

    if (OBD_Error_flag == 1U)
    {
        Obd_ErrorOperation();
    }
}
