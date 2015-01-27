/*
 * TC.c
 *
 *  Created on: 18 нояб. 2014 г.
 *      Author: maximus
 */
#include "mezonin.h"
#include "global.h"
//------------------------------------------------------------------------------
void Mez_TC_init(mezonin *MezStruct)
{
	uint32_t j;
	for (j = 0; j < 4; j++) {
		Mezonin_TC[MezStruct->Mez_ID - 1].Channel[j].Value = 0xFF;
	}
	//конфигурирование ввода-вывода для адресных входов
	AT91F_PIO_CfgOutput(MezStruct->LineA0.PIO_ctrl_ptr, MezStruct->LineA0.PIO_Line | MezStruct->LineA1.PIO_Line);
	//Установить в ноль
	AT91F_PIO_ClearOutput(MezStruct->LineA0.PIO_ctrl_ptr, MezStruct->LineA0.PIO_Line | MezStruct->LineA1.PIO_Line);
	//конфигурирование портов ввода-вывода для ввода сигналов
	AT91F_PIO_CfgInput(MezStruct->LineMDATA.PIO_ctrl_ptr, MezStruct->LineMDATA.PIO_Line);
	AT91F_PIO_CfgInput(MezStruct->LineBRK.PIO_ctrl_ptr, MezStruct->LineBRK.PIO_Line);

}
//------------------------------------------------------------------------------
void TCValueHandler (Mez_Value *Mez_V)
{
	uint32_t ChannelNumber;
	uint32_t ID;
	ChannelNumber = Mez_V->ID * 4 + Mez_V->Channel;
	ID = MAKE_MSG_ID(priority_N, identifier_TC, ChannelNumber, ParamTC);
	TC_Channel * tc_channel = &Mezonin_TC[Mez_V->ID].Channel[Mez_V->Channel];

	if ((tc_channel->Value != (Mez_V->ui32Value & 1)) || (tc_channel->State != (Mez_V->ui32Value >> 1))) { // если ТС изменился

		tc_channel->Value = Mez_V->ui32Value & 1;
		tc_channel->State = Mez_V->ui32Value >> 1; // доработать со сдвигами
		if(MSO.Mode==MSO_MODE_ON) SendCanMessage(ID, tc_channel->Value, tc_channel->State);
	}


}
//------------------------------------------------------------------------------
void Mez_TC_handler(mezonin *MezStruct)
{
//	  uint32_t ChannelNumber;
//	  uint32_t time;
	uint8_t BRK;
	uint8_t MDATA = TC_OFF;
	Mez_Value Real_TC;

	MezStruct->ActiveChannel++;
	if (MezStruct->ActiveChannel > 4)
		MezStruct->ActiveChannel = 1;

//	  ChannelNumber = Mez_TT_DefineInputChannelNumber (MezStruct->Mez_ID);

	if (Mezonin_TC[MezStruct->Mez_ID - 1].Channel[MezStruct->ActiveChannel - 1].Params.Mode
			== mode_on|| Mezonin_TC[MezStruct->Mez_ID - 1].Channel[MezStruct->ActiveChannel
			- 1].Params.Mode == mode_ok)
//	  Mez_TT_EnableChannel (ChannelNumber, MezStruct->LineA0.PIO_Line, MezStruct->LineA1.PIO_Line);
//	  if ( MezStruct->ChannelMode[MezStruct->ActiveChannel] != Channel_OFF )
			{
		Mez_EnableChannel(MezStruct);		// переделал под свою функцию

		//задержка для выполлнения переходных процессов в мультиплексоре и компараторе
//        time=50; while(time);

		//считывание входов BRK
		if (((MezStruct->LineBRK.PIO_ctrl_ptr->PIO_PDSR /*AT91C_BASE_PIOB->PIO_PDSR*/) & MezStruct->LineBRK.PIO_Line) == 0)	//На линии обрыв торчит ноль значит обрыв
				{

			BRK = TC_BRK;
//    	               TC_State[MezStruct->Mez_ID] = TC_BRK; // NNENENEJHGvf
		} else {
			BRK = TC_OK;
		}

		if (((MezStruct->LineMDATA.PIO_ctrl_ptr->PIO_PDSR /*AT91C_BASE_PIOB->PIO_PDSR*/) & MezStruct->LineMDATA.PIO_Line) == MezStruct->LineMDATA.PIO_Line)	//На линии MDATA торчит ноль значит ВКЛ
				{
			MDATA = TC_ON;
//       		 TC_State[MezStruct->Mez_ID] = TC_ON;
		} else if (((MezStruct->LineMDATA.PIO_ctrl_ptr->PIO_PDSR /*AT91C_BASE_PIOB->PIO_PDSR*/) & MezStruct->LineMDATA.PIO_Line) == 0)//На линии MDATA торчит единица значит ВЫКЛ
				{
			MDATA = TC_OFF;
//        	  TC_State[MezStruct->Mez_ID] = TC_OFF;
		}
		Real_TC.ui32Value = MDATA | BRK << 1;
		Real_TC.Channel = MezStruct->ActiveChannel - 1;
		Real_TC.ID = MezStruct->Mez_ID - 1;

		if (xMezQueue != 0) {
			// Send an unsigned long.  Wait for 10 ticks for space to become
			// available if necessary.
			//		Switch_State=Value;
			if (xQueueSend(xMezQueue, &Real_TC, /*(TickType_t ) 0*/portMAX_DELAY)) {
				//	       test++;     // Failed to post the message, even after 10 ticks.
			}
		}
	}
//        time=1000;while(time);

}

//------------------------------------------------------------------------------
uint32_t Get_TCParams(TC_Value *TC_ParamData)
{
	uint32_t i, a;
	uint8_t DataRecieve[24];
	uint16_t temp_CRC;
	for (i = 0; i < 4; i++) {
		TC_ParamData->Channel[i].Params.Mode = 4;
		a = MEZ_memory_Read(TC_ParamData->ID, i + 1, 0x00, sizeof(TC_Param), DataRecieve); // чтение начальных параметров из EEPROM
		temp_CRC = Crc16(DataRecieve, offsetof(TC_Param, CRC)); //TODO fix TCParams size possibly works
		if (temp_CRC == ((TC_Param *) DataRecieve)->CRC) {
			TC_ParamData->Channel[i].Params = *(TC_Param *) DataRecieve;
		} else {
			return 1;
		}
	}
	return 0;

}

//------------------------------------------------------------------------------
void Set_TCDefaultParams(uint8_t MezNum)
{
	TC_Param * Params;
	uint8_t i;
	for (i = 0; i < 4; i++) {
		Params = &Mezonin_TC[MezNum].Channel[i].Params;
		Params->Mode = 8;
		Params->CRC = Crc16((uint8_t *) Params, offsetof(TC_Param, CRC)); //TODO fix TCParams size possibly works
		if (xTWISemaphore != NULL) {
			if (xSemaphoreTake( xTWISemaphore, portMAX_DELAY ) == pdTRUE) {
				MEZ_memory_Write(MezNum, i + 1, 0x00, sizeof(TC_Param), (uint8_t *) Params);
				vTaskDelay(100);
				xSemaphoreGive(xTWISemaphore);

			}
		}

	}
}

