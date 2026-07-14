#include "Lin_Service.h"
#include "Service.h"

#ifdef UDS
#include "cpu.h"
#include "..\..\uds\def_lin_uds.h"
#include "..\..\uds\lin_uds.h"
#include "..\..\uds\util.h"

void RLIN_Slave_uds_Transmit(uint8_t *databuf, uint8_t Data_length)
{
    uint8_t i;
    uint8_t *Databuf_adr;
    RLN30.LDFC = 0x18;                     /*b5=0: checksum mode classic; b4=1:transmission*/
    Databuf_adr = (uint8_t *)&RLN30.LDBR1; /* get the data buffer address*/
    for (i = 0; i < Data_length; i++)      /* setting tansmission data to date buffer*/
    {
        *((uint8_t *)(Databuf_adr + i)) = databuf[i];
    }
    RLN30.LTRC = 0x02; /*setting RTS=1;Response transmission start*/
}
#endif

/***********************************************************************************************************************
 * Function Name: Lin_Wakeup
 * Description  : Wake up in sleep mode when LIN communication is detected and initialize the associated variables
 * Called By    : Lin_HandleReceivedHeader
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
static void Lin_Wakeup(void)
{
    if (lin_bus_inactive_flag == ON)
    {
        if ((lin_sleep_step <= 2U) || (lin_sleep_step >= 8U))
        {
            lin_bus_inactive_flag = OFF;
            lin_sleep_step = 0U;
            G_Timer1msFlag.LinSleepModeFlag = 0U;
            G_Timer1ms.LinSleepMode = 0U;

            Drv8889_Off2();
            motor_start = OFF;
            G_Timer1msFlag.StallTimeFlag = 0U;
            G_Timer1ms.StallTime = 0U;
            G_Timer1msFlag.InitFailCheckFlag = 0U;
            G_Timer1ms.InitFailCheck = 0U;

            if (AAFx_Position_Status == FlapMoving_Status)
            {
                aaf_step = AAF_OPERATE;
            }
        }
    }
    if (((LIN_Recover == 1U) || (Sleep_Stall == ON)) && ((EngRunSta == 0x01U) || (HevRdy == 0x01U)))
    {
        // fail_safety_flag = ON;
        // fail_safety_step = 2U;
        Re_Init();
        LIN_Recover = 0U;
        Sleep_Stall = OFF;
    }
}

/***********************************************************************************************************************
 * Function Name: Lin_HandleReceivedHeader
 * Description  : Check ID on LIN header reception and set send/receive mode
 * Called By    : Lin_ReceiveComplete_Interrupt
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
void Lin_HandleReceivedHeader(void)
{
    RLN30.LST = 0x00U;        // Clear Status
    GetIDbuffer = RLN30.LIDB; // Get ID

    if (GetIDbuffer == 0x25U)
    {
        Lin_SlaveReceive(6U); // 0x25
    }
    else if ((GetIDbuffer == 0xA6U) && (AAFx_Index == ReqRespAAFID))
    {
        Lin_SlaveTransmit(Slave_TxData, 7U); // 0x26
    }
    else if (GetIDbuffer == 0x3CU)
    {
        Lin_SlaveReceive(8U); // 0x3C
    }
#ifdef UDS
    else if ((GetIDbuffer == 0x7DU) && (uds.state == EST_TX))
    {
        RLIN_Slave_uds_Transmit((uint8_t *)&uds.tx_data[1], 8);
        uds.state = EST_RX;
    }
#endif
    else if ((GetIDbuffer == 0x7DU) && ((SW_Chk == 1U) || (SW_Chk == 3U)))
    {
        Lin_SlaveTransmit(Slave_SwData, 8U); // 0x3D
    }
    else
    {
        Lin_SlaveNoResponse();
    }

    Lin_Wakeup();
}

/***********************************************************************************************************************
 * Function Name: Lin_HandleReceivedResponse
 * Description  : LIN data reception completion flag processing and receiving buffer read
 * Called By    : Lin_ReceiveComplete_Interrupt
 * Arguments    : void
 * Return Value : void
 ***********************************************************************************************************************/
void Lin_HandleReceivedResponse(void)
{
    RLN30.LST &= 0xFDu; // Clear successful response reception flag

    switch (GetIDbuffer)
    {
    case 0x25u:
        Lin_GetReponseRxData(Slave_RxData1);
        lin_rx_chk_flag = ON;
        break;
    case 0x3Cu:
        Lin_GetReponseRxData(Slave_RxSwData1);
#ifdef UDS
        unsigned int i;
        if (Slave_RxSwData1[0] == NAD_PHY)
        {
            for (i = 0; i < 7; i++)
            {
                uds.packet_data[i] = Slave_RxSwData1[i + 2];
                Slave_RxSwData1[i] = 0;
            }
            uds_frame_parser();
        }
#endif
        break;
    default:
        break;
    }
}

/***********************************************************************************************************************
 * Function Name: Lin_CalculateVerifyChecksum
 * Description  : Manage LIMP HOME counts by calculating and validating the checksum of received data
 * Called By    : Lin_ReceiveComplete_Interrupt
 * Arguments    : is_response_received - Response received flag (0: Not received, 2: Received)
 * Return Value : void
 ***********************************************************************************************************************/
void Lin_CalculateVerifyChecksum(uint8_t is_response_received)
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
        if ((G_Timer1ms.IgnCheck >= 500U) && ((EngRunSta == 0x01u) || (HevRdy == 0x01u)))
        {
            if (LIMP_HOME_Count >= 4U)
                LIMP_HOME_Count -= 4U;
            else
                LIMP_HOME_Count = 0U;
        }
    }
    else if ((is_response_received == 0x02U) && (AAF_LIN_ChkSum_CHK_value != Req_ChkSum_Rx))
    {
        AAF_LIN_ChkSum_CHK = FAIL;
        if ((G_Timer1ms.IgnCheck >= 500U) && ((EngRunSta == 0x01u) || (HevRdy == 0x01u)))
        {
            if (LIMP_HOME_Count <= 158U)
                LIMP_HOME_Count += 2U;
            else
                LIMP_HOME_Count = 160U;
        }
    }
    else
    {
        // Waiting
    }
}
