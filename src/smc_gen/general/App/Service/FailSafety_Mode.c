#include "FailSafety_Mode.h"
#include "Service.h"

/***********************************************************************************************************************
 * Fail-Safety_Mode
 ***********************************************************************************************************************/
/***********************************************************************************************************************
 * Function Name: FS_MotorOpenStart
 * Description  : 모터 OPEN 방향 구동을 시작하고 관련 변수를 초기화함.
 *                (원본 case 0, 5, 10의 공통 로직)
 * Called By    : Run_fs_cycle_1_open, Run_fs_cycle_2_open, Run_fs_final_open
 * Arguments    : next_step - 초기화 후 이동할 다음 Fail-Safety Step
 * Return Value : void
 ***********************************************************************************************************************/
static void FS_MotorOpenStart(unsigned int next_step)
{
    Motor_Open();  /* dir OPEN */
    Drv8889_On2(); /* drv on */
    motor_start = ON;
    G_Timer1ms.Spi = 0U;
    aaf_action = OPEN;
    aaf_step = AAF_WAITING;
    motor_stall_flag = MOTOR_NORMAL;                  /* stall reset */
    G_Timer1msFlag.StallTimeFlag = 0U;                /* stall reset */
    G_Timer1ms.StallTime = 0U;                        /* stall reset */
    motor_stall_value = MOTOR_STALL_CHK_NORMAL_VALUE; /* stall reset */
    G_Timer1ms.InitCheck = 0U;                        /* step start */
    G_Timer1msFlag.InitCheckFlag = 1U;
    fail_safety_step = next_step;
}

/***********************************************************************************************************************
 * Function Name: FS_CheckStallStop
 * Description  : 스톨(Stall) 또는 타임아웃 발생 시 모터를 정지하고 다음 단계로 이동함.
 *                (원본 case 1, 6, 11의 공통 로직)
 * Called By    : Run_fs_cycle_1_stall, Run_fs_cycle_2_stall, Run_fs_final_stall
 * Arguments    : next_step - 정지 후 이동할 다음 Fail-Safety Step
 * Return Value : void
 ***********************************************************************************************************************/
static void FS_CheckStallStop(unsigned int next_step)
{
    if ((motor_stall_flag == MOTOR_STALL) || (G_Timer1ms.InitCheck >= 4500U))
    {
        Drv8889_Off3();
        motor_start = OFF;
        G_Timer1msFlag.StallTimeFlag = 0U;
        G_Timer1ms.StallTime = 0U; /* stall reset */
        aaf_action = FLAP_STOP;
        G_Timer1msFlag.InitCheckFlag = 0U;
        G_Timer1ms.InitCheck = 0U;
        softstart_complete = OFF;
        fail_safety_step = next_step;
    }
}

/***********************************************************************************************************************
 * Function Name: FS_Delay3minute
 * Description  : 3분 대기 타이머를 구동하고, 완료 시 다음 단계로 이동함.
 *                (원본 case 2, 7의 공통 로직)
 * Called By    : Run_fs_cycle_1_wait, Run_fs_cycle_2_wait
 * Arguments    : next_step - 3분 대기 완료 후 이동할 다음 Fail-Safety Step
 * Return Value : void
 ***********************************************************************************************************************/
static void FS_Delay3minute(unsigned int next_step)
{
    G_Timer1msFlag.Timer3minuteFlag = 1U;
    antipinch_step = 0U;

    if (G_Timer1ms.Timer3minuteSec >= MINUTE_3)
    {
        G_Timer1msFlag.Timer3minuteFlag = 0U;
        G_Timer1ms.Timer3minuteSec = 0U;
        G_Timer1ms.Timer3minute = 0U;
        fail_safety_step = next_step;
    }
}

/***********************************************************************************************************************
 * Function Name: FS_ReInit
 * Description  : Re_Init()을 호출하고 다음 단계로 이동함.
 *                (원본 case 3, 8의 공통 로직)
 * Called By    : Run_fs_cycle_1_reinit, Run_fs_cycle_2_reinit
 * Arguments    : next_step - Re_Init 호출 후 이동할 다음 Fail-Safety Step
 * Return Value : void
 ***********************************************************************************************************************/
static void FS_ReInit(unsigned int next_step)
{
    Re_Init();
    fail_safety_step = next_step;
}

/***********************************************************************************************************************
 * Function Name: FS_CheckInitComplete
 * Description  : 초기화 이동(Init Move) 시퀀스 완료를 확인하고 정상 상태로 복귀함.
 *                (원본 case 4, 9의 공통 로직)
 * Called By    : Run_fs_cycle_1_complete, Run_fs_cycle_2_complete
 * Return Value : void
 ***********************************************************************************************************************/
static void FS_CheckInitComplete(void)
{
    if (init_move_step == 19U)
    {
        init_move_step = 0U;
        aaf_init_step = NORMAL_INITIALIZATION;
        fail_safety_step = 0U;
        fail_safety_flag = OFF;
        AAFx_ErrorStatus = No_ErrorStatus;
    }
}

/***********************************************************************************************************************
 * Function Name: FS_FinalErrorSet
 * Description  : 최종 에러 상태를 확정하고 관련 플래그를 설정함.
 *                (원본 case 12의 로직)
 * Called By    : FS_Cycle3
 * Return Value : void
 ***********************************************************************************************************************/
static void FS_FinalErrorSet(void)
{
    AAF_Tx_Position = UNKOWN_POSITION;
    AAFx_Position_Status = Unknown_Status;
    AAFx_InitStatus = ABNORMAL_FINISHED_INITIALIZATION;
    motor_fault_chk = 1U;
}
/* =========================================================================================
 * Freeze Hold 전용 함수
 * ========================================================================================= */

/***********************************************************************************************************************
 * Function Name: FS_CheckCountBranch
 * Description  : Freeze Hold 재진입 여부를 확인하고 횟수에 따라 분기함.
 *                - Init_Fail_Chk == 2 : 이전 Freeze Hold 대기 후 재시도 실패를 의미
 *                  → Freeze_Hold_Mode_Count 증가 후 횟수 판정
 *                - count >= 3 : 반복 시도 횟수 초과 → 바로 Step 13(판정)으로
 *                - count <  3 : 아직 시도 가능  → Step 11(Motor Open)으로
 *                (원본 case 10 로직)
 * Called By    : FS_Cycle3
 * Return Value : void
 ***********************************************************************************************************************/
static void FS_CheckCountBranch(void)
{
    /* Init_Fail_Chk == 2 : Step 13에서 재진입 경로로 돌아온 경우 */
    if (Init_Fail_Chk == 2U)
    {
        Freeze_Hold_Mode_Count++; /* Freeze Hold 후 재시도 실패 누적 */
        Init_Fail_Chk = 1U;       /* 재진입 플래그를 '1회 시도 완료' 상태로 되돌림 */
    }

    if (Freeze_Hold_Mode_Count >= 3U)
    {
        /* 이미 3회 이상 반복했으면 Motor Open 시도 없이 바로 판정으로 */
        fail_safety_step = 13U;
    }
    else
    {
        /* 아직 시도 횟수 남아있음 → Motor Open 재시도 */
        fail_safety_step = 11U;
    }
}

/***********************************************************************************************************************
 * Function Name: FS_CheckStallStopFreezeHold
 * Description  : Freeze Hold 사이클의 스톨/타임아웃 처리.
 *                정지 후 세 가지 경로로 분기:
 *                  1) count >= 10 : 최종 에러 확정 (무한 루프 방지 상한)
 *                  2) Init_Fail_Chk != 0 : 3분 대기 후 Step 13 이동
 *                     (Freeze Hold 해제 여부를 기다리는 구간)
 *                  3) Init_Fail_Chk == 0 : 즉시 Step 13 이동
 *                (원본 case 12 로직)
 * Called By    : FS_Cycle3
 * Return Value : void
 ***********************************************************************************************************************/
static void FS_CheckStallStopFreezeHold(void)
{
    if ((motor_stall_flag == MOTOR_STALL) || (G_Timer1ms.InitCheck >= 4500U))
    {
        /* 공통 정지 처리 */
        Drv8889_Off3();
        motor_start = OFF;
        G_Timer1msFlag.StallTimeFlag = 0U;
        G_Timer1ms.StallTime = 0U;
        aaf_action = FLAP_STOP;
        G_Timer1msFlag.InitCheckFlag = 0U;
        G_Timer1ms.InitCheck = 0U;
        softstart_complete = OFF;

        /* ── 분기 1 : count >= 10 ─────────────────────────────────────────── */
        /* Freeze Hold 반복 상한 초과. 더 이상 시도하지 않고 최종 에러 확정    */
        if (Freeze_Hold_Mode_Count >= 10U)
        {
            AAF_Tx_Position = UNKOWN_POSITION;
            AAFx_Position_Status = Unknown_Status;
            AAFx_InitStatus = ABNORMAL_FINISHED_INITIALIZATION;
            motor_fault_chk = 1U;
            G_Timer1msFlag.Freeze_Hold_ModeChkFlag = 0U;
            /* fail_safety_step 변경 없음 — Step 12에 머물러 에러 확정 유지  */
        }
        /* ── 분기 2 : Init_Fail_Chk != 0 ─────────────────────────────────── */
        /* 이전 사이클에서 한 번 이상 시도한 이력이 있음                       */
        /* 3분 대기를 통해 Freeze_Hold_Mode 해제(Off) 시점을 기다림            */
        else if (Init_Fail_Chk != 0U)
        {
            G_Timer1msFlag.Timer3minuteFlag = 1U;

            if (G_Timer1ms.Timer3minuteSec >= MINUTE_3)
            {
                G_Timer1msFlag.Timer3minuteFlag = 0U;
                G_Timer1ms.Timer3minuteSec = 0U;
                G_Timer1ms.Timer3minute = 0U;
                fail_safety_step = 13U;
            }
            /* 3분 미만이면 이 case를 매 루프마다 재실행하며 타이머 대기 */
        }
        /* ── 분기 3 : Init_Fail_Chk == 0 ─────────────────────────────────── */
        /* 첫 진입이거나 이전 이력 없음 → 대기 없이 즉시 판정                  */
        else
        {
            fail_safety_step = 13U;
        }
    }
}

/***********************************************************************************************************************
 * Function Name: FS_FreezeHoldBranch
 * Description  : Freeze Hold 상태(On/Off)와 횟수에 따라 최종 동작을 결정함.
 *
 *                분기 요약:
 *                  A) Freeze_Hold_On  + count <= 3 : 아직 동결 중
 *                     → Freeze_Hold_Start() 호출 후 대기 (Init_Fail_Chk = 1)
 *
 *                  B) Freeze_Hold_Off + Init_Fail_Chk==1 + count <= 2 : 동결 해제 확인
 *                     → Step 8(Re_Init)으로 복귀하여 초기화 재시도
 *                        Init_Fail_Chk = 2 로 마킹 (Step 10에서 count++ 트리거)
 *
 *                  C) Freeze_Hold_Off + count >= 3 : 해제됐지만 횟수 초과
 *                     → count = 10 강제 세팅 후 Step 11(Motor Open) 마지막 시도
 *                        이후 Step 12에서 count >= 10 조건으로 최종 에러 확정
 *
 *                  D) Freeze_Hold_Off + Init_Fail_Chk==0 + count < 1 : 이력 없음
 *                     → 즉시 최종 에러 확정
 *
 *                (원본 case 13 로직)
 * Called By    : FS_Cycle3
 * Return Value : void
 ***********************************************************************************************************************/
static void FS_FreezeHoldBranch(void)
{
    /* ── 분기 A : 현재 동결 상태(Freeze Hold On) ──────────────────────────── */
    if ((Freeze_Hold_Mode == Freeze_Hold_On) && (Freeze_Hold_Mode_Count <= 3U))
    {
        /* 동결 상태이므로 강제로 Open 위치 상태를 재설정하고 대기            */
        /* Freeze_Hold_Start() = Error_Unknown_status() : open stopper 재시도 */
        Freeze_Hold_Start();
        Init_Fail_Chk = 1U; /* '1회 시도 완료' 마킹 — Step 10 분기에 사용  */
        /* fail_safety_step 변경 없음 → Step 13을 반복 실행하며 Off 전환 대기 */
    }
    /* ── 분기 B : 동결 해제됨 + 재시도 횟수 남아있음 ──────────────────────── */
    else if ((Freeze_Hold_Mode == Freeze_Hold_Off) &&
             (Init_Fail_Chk == 1U) &&
             (Freeze_Hold_Mode_Count <= 2U))
    {
        /* Freeze Hold가 해제됐고 아직 재시도 기회 있음 → 2차 사이클 Re_Init으로 */
        fail_safety_flag = ON;
        fail_safety_step = 8U; /* Step 8 : Re_Init() 호출 지점으로 점프     */
        Init_Fail_Chk = 2U;    /* Step 10 진입 시 count++ 트리거용 마킹     */
    }
    /* ── 분기 C : 동결 해제됐지만 반복 횟수 초과 ──────────────────────────── */
    else if ((Freeze_Hold_Mode == Freeze_Hold_Off) && (Freeze_Hold_Mode_Count >= 3U))
    {
        /* 마지막 1회 강제 Motor Open 시도 후 에러 확정으로 유도               */
        Freeze_Hold_Mode_Count = 10U; /* Step 12에서 >= 10 조건으로 에러 확정  */
        fail_safety_step = 11U;
    }
    /* ── 분기 D : 동결 해제 + 이력 없음 ───────────────────────────────────── */
    else if ((Freeze_Hold_Mode == Freeze_Hold_Off) &&
             (Init_Fail_Chk == 0U) &&
             (Freeze_Hold_Mode_Count < 1U))
    {
        /* 단순 초기화 실패(동결 아님) → 즉시 에러 확정                        */
        FS_FinalErrorSet();
    }
    else
    {
        /* 그 외 조건 — 현재 상태 유지 (Step 13 반복 대기) */
        (void)0;
    }
}
/* =========================================================================================
 * Fail Safety Cycle Execution Functions
 * ========================================================================================= */

/***********************************************************************************************************************
 * Function Name: FS_Cycle1
 * Description  : Fail-Safety 1차 사이클 (Step 0 ~ 4) 실행.
 *                Motor OPEN 구동 → 스톨 감지 → 3분 대기 → Re_Init → 완료 확인
 * Called By    : FailSafety_Mode
 * Return Value : void
 ***********************************************************************************************************************/
static void FS_Cycle1(void)
{
    switch (fail_safety_step)
    {
    case 0: /* Motor OPEN 구동 시작 */
        FS_MotorOpenStart(1U);
        break;
    case 1: /* 스톨/타임아웃 감지 → 모터 정지 */
        FS_CheckStallStop(2U);
        break;
    case 2: /* 3분 대기 */
        FS_Delay3minute(3U);
        break;
    case 3: /* Re_Init 호출 */
        FS_ReInit(4U);
        break;
    case 4: /* 초기화 완료 확인 → 정상 복귀 */
        FS_CheckInitComplete();
        break;
    default:
        break;
    }
}

/***********************************************************************************************************************
 * Function Name: FS_Cycle2
 * Description  : Fail-Safety 2차 사이클 (Step 5 ~ 9) 실행.
 *                1차 사이클과 동일한 과정을 한 번 더 반복함.
 *                Motor OPEN 구동 → 스톨 감지 → 3분 대기 → Re_Init → 완료 확인
 * Called By    : FailSafety_Mode
 * Return Value : void
 ***********************************************************************************************************************/
static void FS_Cycle2(void)
{
    switch (fail_safety_step)
    {
    case 5: /* Motor OPEN 구동 시작 */
        FS_MotorOpenStart(6U);
        break;
    case 6: /* 스톨/타임아웃 감지 → 모터 정지 */
        FS_CheckStallStop(7U);
        break;
    case 7: /* 3분 대기 */
        FS_Delay3minute(8U);
        break;
    case 8: /* Re_Init 호출 */
        FS_ReInit(9U);
        break;
    case 9: /* 초기화 완료 확인 → 정상 복귀 */
        FS_CheckInitComplete();
        break;
    default:
        break;
    }
}

/***********************************************************************************************************************
 * Function Name: FS_Cycle3
 * Description  : Fail-Safety 최종 사이클 (Step 10 ~ 12) 실행.
 *                마지막 Motor OPEN 시도 후 실패 시 에러 확정 (원본 case 10~12).
 * Called By    : FailSafety_Mode
 * Return Value : void
 ***********************************************************************************************************************/
static void FS_Cycle3(void)
{
    switch (fail_safety_step)
    {

    case 10U:
        FS_CheckCountBranch(); /* Motor OPEN 구동 시작 (최후 시도) */
        break; 
    case 11U:
        FS_MotorOpenStart(12U); /* 스톨/타임아웃 감지 → 모터 정지 */
        break; 
    case 12U:
        FS_CheckStallStopFreezeHold(); /* 최종 에러 확정 */
        break; 
    case 13U:
        FS_FreezeHoldBranch();
        break;
    default:
        break;
    }
}

/* =========================================================================================
 * Main Fail Safety Mode Dispatcher
 * ========================================================================================= */

/***********************************************************************************************************************
 * Function Name: FailSafety_Mode
 * Description  : Fail-Safety 모드의 진입점. 현재 단계(step)에 따라 1차/2차/최종 사이클 함수를 호출함.
 * Called By    : Safety_Check
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
void FailSafety_Mode(void)
{
    if (fail_safety_flag == ON)
    {
        /* [Cycle 1] Steps 0 ~ 4 */
        if (fail_safety_step <= 4U)
        {
            FS_Cycle1();
        }
        /* [Cycle 2] Steps 5 ~ 9 */
        else if (fail_safety_step <= 9U)
        {
            FS_Cycle2();
        }
        /* [Final Cycle] Steps 10 ~ 12 */
        else
        {
            FS_Cycle3();
        }
    }
}
