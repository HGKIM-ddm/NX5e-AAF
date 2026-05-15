#include "Fdl.h"
#include "Service.h"

void FDL_Init(void)
{
	FDL_Open();
	ret = function_FDL_init();

	G_Timer1msFlag.FdlErrorCheckFlag = 1U;
	if (ret < (char)0) // error
	{
		while (1)
		{
			if (G_Timer1ms.FdlErrorCheck >= 100U)
			{
				fdl_fail += 1U;
				break;
			}
		}
	}

	G_Timer1msFlag.FdlErrorCheckFlag = 0U;
	G_Timer1ms.FdlErrorCheck = 0U;
}

void FDL_Write(void)
{
	G_Timer1msFlag.FdlErrorCheckFlag = 0U;
	G_Timer1ms.FdlErrorCheck = 0U;
	close_memory_write = step_position_close;
	open_memory_write = step_position_open;
	now_step_memory_write = step_position;
	// position_memory_write = AAF_Tx_Position;
	Initial_memory_write = evrdy_on_flag;
	limit_memory_write = limit_step_position;
	if (AAF_LINOut == 0x00)
	{
		position_memory_write = AAF_Tx_Position_Temporary;
		position_status_memory_write = AAFx_Position_Status_Temporary;
		AAFx_InitStatus_memory_write = AAFx_InitStatus_Temporary;
	}
	else
	{
		position_memory_write = AAF_Tx_Position;
		position_status_memory_write = AAFx_Position_Status;
		AAFx_InitStatus_memory_write = AAFx_InitStatus;
	}
	// position_status_memory_write = AAFx_Position_Status;
	// AAFx_InitStatus_memory_write = AAFx_InitStatus;
	DTC_memory_write = DTC_Status;
	power_chk_memory_write = power_chk;
	First_Powerchk_memory_write = First_Powerchk;
	// only sx3k
	// OBD1_Close_Check_memory_write = OBD1_Close_Check;
	// OBD1_Open_Check_memory_write = OBD1_Open_Check;
	// OBD2_Close_Check_memory_write = OBD2_Close_Check;
	// OBD2_Open_Check_memory_write = OBD2_Open_Check;
	// OBD3_Close_Check_memory_write = OBD3_Close_Check;
	// OBD3_Open_Check_memory_write = OBD3_Open_Check;

	// w_buff[0] = close_memory_write & 0x00FFU; // write 2byte read 4byte ?븯?쐞
	// w_buff[1] = (close_memory_write & 0xFF00U) >> 8U;

	// w_buff[2] = open_memory_write & 0x00FFU; // write 2byte read 4byte ?긽?쐞
	// w_buff[3] = (open_memory_write & 0xFF00U) >> 8U;

	// w_buff[4] = now_step_memory_write & 0x00FFU; // write 2byte read 4byte
	// w_buff[5] = (now_step_memory_write & 0xFF00U) >> 8U;

	// w_buff[6] = position_memory_write & 0xFFU; // write 2byte read 4byte position+evrdy
	// w_buff[7] = (Initial_memory_write & 0xFFU);

	// w_buff[8] = limit_memory_write & 0x00FFU; // write 2byte read 4byte limitstep
	// w_buff[9] = (limit_memory_write & 0xFF00U) >> 8U;

	// w_buff[10] = position_status_memory_write & 0x0FU; // write 2byte read 4byte limitstep
	// w_buff[11] = (AAFx_InitStatus_memory_write & 0x0FU);

	// w_buff[12] = DTC_memory_write & 0x00FFU; // write 2byte read 4byte limitstep
	// w_buff[13] = (DTC_memory_write & 0xFF00U) >> 8U;

	// w_buff[14] = power_chk_memory_write & 0x00FFU; // write 2byte read 4byte limitstep
	// w_buff[15] = (power_chk_memory_write & 0xFF00U) >> 8U;

	// w_buff[16] = First_Powerchk_memory_write & 0x00FFU; // write 2byte read 4byte limitstep
	// w_buff[17] = (First_Powerchk_memory_write & 0xFF00U) >> 8U;

	// w_buff[18] = OBD1_Close_Check_memory_write & 0x00FFU; // write 2byte read 4byte limitstep
	// w_buff[19] = (OBD1_Close_Check_memory_write & 0xFF00U) >> 8U;

	// w_buff[20] = OBD1_Open_Check_memory_write & 0x00FFU; // write 2byte read 4byte limitstep
	// w_buff[21] = (OBD1_Open_Check_memory_write & 0xFF00U) >> 8U;

	// w_buff[22] = OBD2_Close_Check_memory_write & 0x00FFU; // write 2byte read 4byte limitstep
	// w_buff[23] = (OBD2_Close_Check_memory_write & 0xFF00U) >> 8U;

	// w_buff[24] = OBD2_Open_Check_memory_write & 0x00FFU; // write 2byte read 4byte limitstep
	// w_buff[25] = (OBD2_Open_Check_memory_write & 0xFF00U) >> 8U;

	// w_buff[26] = OBD3_Close_Check_memory_write & 0x00FFU; // write 2byte read 4byte limitstep
	// w_buff[27] = (OBD3_Close_Check_memory_write & 0xFF00U) >> 8U;

	// w_buff[28] = OBD3_Open_Check_memory_write & 0x00FFU; // write 2byte read 4byte limitstep
	// w_buff[29] = (OBD3_Open_Check_memory_write & 0xFF00U) >> 8U;

	/* ============================= */
	/* 4byte data                    */
	/* ============================= */

	w_buff[0] = (uint8_t)(close_memory_write & 0x000000FFUL);
	w_buff[1] = (uint8_t)((close_memory_write >> 8U) & 0x000000FFUL);
	w_buff[2] = (uint8_t)((close_memory_write >> 16U) & 0x000000FFUL);
	w_buff[3] = (uint8_t)((close_memory_write >> 24U) & 0x000000FFUL);

	w_buff[4] = (uint8_t)(open_memory_write & 0x000000FFUL);
	w_buff[5] = (uint8_t)((open_memory_write >> 8U) & 0x000000FFUL);
	w_buff[6] = (uint8_t)((open_memory_write >> 16U) & 0x000000FFUL);
	w_buff[7] = (uint8_t)((open_memory_write >> 24U) & 0x000000FFUL);

	w_buff[8] = (uint8_t)(now_step_memory_write & 0x000000FFUL);
	w_buff[9] = (uint8_t)((now_step_memory_write >> 8U) & 0x000000FFUL);
	w_buff[10] = (uint8_t)((now_step_memory_write >> 16U) & 0x000000FFUL);
	w_buff[11] = (uint8_t)((now_step_memory_write >> 24U) & 0x000000FFUL);

	/* ============================= */
	/* 2byte / 1byte data            */
	/* ============================= */

	w_buff[12] = position_memory_write & 0xFFU;
	w_buff[13] = Initial_memory_write & 0xFFU;

	w_buff[14] = limit_memory_write & 0x00FFU;
	w_buff[15] = (limit_memory_write & 0xFF00U) >> 8U;

	w_buff[16] = position_status_memory_write & 0x0FU;
	w_buff[17] = AAFx_InitStatus_memory_write & 0x0FU;

	w_buff[18] = DTC_memory_write & 0x00FFU;
	w_buff[19] = (DTC_memory_write & 0xFF00U) >> 8U;

	w_buff[20] = power_chk_memory_write & 0x00FFU;
	w_buff[21] = (power_chk_memory_write & 0xFF00U) >> 8U;

	w_buff[22] = First_Powerchk_memory_write & 0x00FFU;
	w_buff[23] = (First_Powerchk_memory_write & 0xFF00U) >> 8U;

	// w_buff[24] = OBD1_Close_Check_memory_write & 0x00FFU;
	// w_buff[25] = (OBD1_Close_Check_memory_write & 0xFF00U) >> 8U;

	// w_buff[26] = OBD1_Open_Check_memory_write & 0x00FFU;
	// w_buff[27] = (OBD1_Open_Check_memory_write & 0xFF00U) >> 8U;

	// w_buff[28] = OBD2_Close_Check_memory_write & 0x00FFU;
	// w_buff[29] = (OBD2_Close_Check_memory_write & 0xFF00U) >> 8U;

	// w_buff[30] = OBD2_Open_Check_memory_write & 0x00FFU;
	// w_buff[31] = (OBD2_Open_Check_memory_write & 0xFF00U) >> 8U;

	// w_buff[32] = OBD3_Close_Check_memory_write & 0x00FFU;
	// w_buff[33] = (OBD3_Close_Check_memory_write & 0xFF00U) >> 8U;

	// w_buff[34] = OBD3_Open_Check_memory_write & 0x00FFU;
	// w_buff[35] = (OBD3_Open_Check_memory_write & 0xFF00U) >> 8U;

	ret = function_FDL_erease(0U, 1U);

	G_Timer1msFlag.FdlErrorCheckFlag = 1U;

	if (ret < (char)0) // error
	{
		while (1)
		{
			if (G_Timer1ms.FdlErrorCheck >= 100U)
			{
				fdl_fail += 1U;
				break;
			}
		}
	}

	ret = function_FDL_write(w_buff, 0, 12); // size = word(2byte)

	G_Timer1msFlag.FdlErrorCheckFlag = 1;

	if (ret < 0) // error
	{
		while (1)
		{
			if (G_Timer1ms.FdlErrorCheck >= 100U)
			{
				fdl_fail += 1U;
				break;
			}
		}
	}

	G_Timer1msFlag.FdlErrorCheckFlag = 0U;
	G_Timer1ms.FdlErrorCheck = 0U;
}

void FDL_Read(void)
{

	ret = function_FDL_read(r_buff, 0U, 12U);

	G_Timer1msFlag.FdlErrorCheckFlag = 1U;

	if (ret < (char)0) // error
	{
		while (1)
		{
			if (G_Timer1ms.FdlErrorCheck >= 100U)
			{
				fdl_fail += 1U;
				break;
			}
		}
	}

	G_Timer1msFlag.FdlErrorCheckFlag = 0U;
	G_Timer1ms.FdlErrorCheck = 0U;

	// close_memory_read = (unsigned int)r_buff[0U] & 0xFFFFu;
	// open_memory_read = (unsigned int)(r_buff[0U] >> 16U) & 0xFFFFu;

	// now_step_memory_read = (unsigned int)r_buff[1U] & 0xFFFFu;

	// position_Initial_combined_read = (unsigned int)(r_buff[1U] >> 16U) & 0xFFFFu;
	// position_memory_read = (unsigned int)position_Initial_combined_read & 0xFFu;
	// Initial_memory_read = (unsigned int)(position_Initial_combined_read >> 8U) & 0xFFu;

	// limit_memory_read = (unsigned int)r_buff[2U] & 0xFFFFu;

	// position_Initstatus_combined_read = (unsigned int)(r_buff[2U] >> 16U) & 0xFFFFu;
	// position_status_memory_read = (unsigned int)position_Initstatus_combined_read & 0x0Fu;
	// AAFx_InitStatus_memory_read = (unsigned int)(position_Initstatus_combined_read >> 8U) & 0x0Fu;

	// DTC_memory_read = (unsigned int)(r_buff[3U]) & 0xFFFFu;

	// power_chk_memory_read = (unsigned int)(r_buff[3U] >> 16U) & 0xFu;
	// First_Powerchk_memory_read = (unsigned int)(r_buff[4U]) & 0xFu;
	// // only sx3k
	// OBD1_Close_Check_memory_read = (unsigned int)(r_buff[4U] >> 16U) & 0xFFFFu;
	// OBD1_Open_Check_memory_read = (unsigned int)(r_buff[5U]) & 0xFFFFu;
	// OBD2_Close_Check_memory_read = (unsigned int)(r_buff[5U] >> 16U) & 0xFFFFu;
	// OBD2_Open_Check_memory_read = (unsigned int)(r_buff[6U]) & 0xFFFFu;
	// OBD3_Close_Check_memory_read = (unsigned int)(r_buff[6U] >> 16U) & 0xFFFFu;
	// OBD3_Open_Check_memory_read = (unsigned int)(r_buff[7U]) & 0xFFFFu;
	/* ============================= */
	/* 4byte data                    */
	/* ============================= */

	close_memory_read = (uint32_t)r_buff[0U];

	open_memory_read = (uint32_t)r_buff[1U];

	now_step_memory_read = (uint32_t)r_buff[2U];

	/* ============================= */
	/* combined data                 */
	/* ============================= */

	position_Initial_combined_read = (uint32_t)r_buff[3U];
	position_memory_read = (uint32_t)position_Initial_combined_read & 0xFFU;
	Initial_memory_read = (uint32_t)(position_Initial_combined_read >> 8U) & 0xFFU;

	limit_memory_read = (uint32_t)(r_buff[3U] >> 16) & 0xFFFFU;

	// position_Initstatus_combined_read = (uint32_t)(r_buff[4U] >> 16U) & 0xFFFFU;
	position_Initstatus_combined_read = (uint32_t)(r_buff[4U]) & 0xFFFFU;
	position_status_memory_read = (uint32_t)position_Initstatus_combined_read & 0x0FU;
	AAFx_InitStatus_memory_read = (uint32_t)(position_Initstatus_combined_read >> 8U) & 0x0FU;

	DTC_memory_read = (uint32_t)(r_buff[4U] >> 16) & 0xFFFFU;

	power_chk_memory_read = (uint32_t)r_buff[5U] & 0xFFFFU;

	First_Powerchk_memory_read = (uint32_t)(r_buff[5U] >> 16) & 0xFFFFU;

	/* ============================= */
	/* OBD data                      */
	/* ============================= */

	// OBD1_Close_Check_memory_read = (uint32_t)r_buff[6U] & 0xFFFFU;

	// OBD1_Open_Check_memory_read = (uint32_t)(r_buff[6U] >> 16) & 0xFFFFU;

	// OBD2_Close_Check_memory_read = (uint32_t)r_buff[7U] & 0xFFFFU;

	// OBD2_Open_Check_memory_read = (uint32_t)(r_buff[7U] >> 16) & 0xFFFFU;

	// OBD3_Close_Check_memory_read = (uint32_t)r_buff[8U] & 0xFFFFU;

	// OBD3_Open_Check_memory_read = (uint32_t)(r_buff[8U] >> 16) & 0xFFFFU;
	if (position_status_memory_read >= Memory_Range_Break)
	{
		position_status_memory_read = Memory_Range_Init;
	}
	if (AAFx_InitStatus_memory_read >= Memory_Range_Break)
	{
		AAFx_InitStatus_memory_read = Memory_Range_Init;
	}
}
void Position_Temporary_write(void)
{
	AAF_Tx_Position_Temporary = AAF_Tx_Position;
	AAFx_Position_Status_Temporary = AAFx_Position_Status;
	AAFx_InitStatus_Temporary = AAFx_InitStatus;

	AAF_Tx_Position = UNKOWN_POSITION;
	AAFx_Position_Status = Unknown_Status;
	AAFx_InitStatus = DURING_INITIALIZATION;
}
void Position_Temporary_read(void)
{
	AAF_Tx_Position = AAF_Tx_Position_Temporary;
	AAFx_Position_Status = AAFx_Position_Status_Temporary;
	AAFx_InitStatus = AAFx_InitStatus_Temporary;
}
