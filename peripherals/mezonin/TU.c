/*
 * TU.c
 *
 *  Created on: 25 сент. 2014 г.
 *      Author: maximus
 */
#include "mezonin.h"
#include "global.h"
#include "constant.h"
#include "../spi/spi.h"

int MakeTUValue(const Mez_Value* Real_TU)
{
	return Mezonin_TU[Real_TU->ID].Channel[0].Value << 4 | Mezonin_TU[Real_TU->ID].Channel[1].Value << 5 | Mezonin_TU[Real_TU->ID].Channel[2].Value << 6
			| Mezonin_TU[Real_TU->ID].Channel[3].Value << 7;
}

//------------------------------------------------------------------------------
void Mez_TU_handler(mezonin *MezStruct)
{

	uint8_t MDATA;
	Mez_Value Real_TU;

	if (Mezonin_TU[MezStruct->Mez_ID - 1].Channel[MezStruct->ActiveChannel - 1].Params.Mode == mode_on
			|| Mezonin_TU[MezStruct->Mez_ID - 1].Channel[MezStruct->ActiveChannel - 1].Params.Mode == mode_ok) {
		Mez_EnableChannel(MezStruct);		// переделал под свою функцию

		if (MezStruct->TUQueue != 0) {
			if (xQueueReceive(MezStruct->TUQueue, &Real_TU, portMAX_DELAY)) {
				uint32_t ID;
				ID = MAKE_MSG_ID(priority_N, identifier_TU, (Real_TU.ID*4 + Real_TU.Channel), ParamTU);
				switch (Real_TU.ui32Value) {
					case TUCommandOn:
						Mezonin_TU[Real_TU.ID].Channel[Real_TU.Channel].TickCount = Mezonin_TU[Real_TU.ID].Channel[Real_TU.Channel].Params.TimeTU;
						Mezonin_TU[Real_TU.ID].Channel[Real_TU.Channel].Value = TUCommandOn;
						break;
					case TUCommandOff:
						Mezonin_TU[Real_TU.ID].Channel[Real_TU.Channel].Value = TUCommandOff;
						break;
				}
				MDATA = MakeTUValue(&Real_TU);		// пишем сразу в 4 канала
				if (xSPISemaphore != NULL) {
					if (xSemaphoreTake( xSPISemaphore, portMAX_DELAY ) == pdTRUE) {
						SPI_Write(AT91C_BASE_SPI0, Real_TU.ID, MDATA);
						SPI_Read(AT91C_BASE_SPI0);
						xSemaphoreGive(xSPISemaphore);
						SendCanMessage(ID, Mezonin_TU[Real_TU.ID].Channel[Real_TU.Channel].Value, 0);
					}
				}

			}
		}
	}

}
//------------------------------------------------------------------------------
void TUTickHandler(void)
{
	static portBASE_TYPE xTaskWoken = pdFALSE;
	int32_t i, j;
	Mez_Value Real_TU;

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			if ((Mezonin_TU[i].Channel[j].Value == TUCommandOn) && (Mezonin_TU[i].Channel[j].Params.TimeTU) ) {
				if (Mezonin_TU[i].Channel[j].TickCount == 0) {
					Real_TU.ID = i;
					Real_TU.Channel = j;
					Real_TU.ui32Value = TUCommandOff;
					if (mezonin_my[j].TUQueue != 0) {
						xQueueSendFromISR(mezonin_my[i].TUQueue, &Real_TU, &xTaskWoken);
					}
				} else {
					Mezonin_TU[i].Channel[j].TickCount--;

				}
			}
		}
	}
}
//------------------------------------------------------------------------------
uint32_t Get_TUParams(TU_Value *TU_temp)
{
	uint32_t i;
	uint16_t temp_CRC;
	uint8_t DataRecieve[sizeof(TU_Param)];
	for (i = 0; i < 4; i++) {
		TU_temp->Channel[i].Params.Mode = 0;
		TU_temp->Channel[i].Value = 0;

		MEZ_memory_Read(TU_temp->ID, i + 1, 0x00, sizeof(TU_Param), DataRecieve); // чтение начальных параметров из EEPROM

		temp_CRC = Crc16(DataRecieve, offsetof(TU_Param, CRC)); //TODO fix TT_Param size possibly works
		if (temp_CRC == ((TU_Param *) DataRecieve)->CRC) {
			TU_temp->Channel[i].Params = *(TU_Param *) DataRecieve;
		} else {
			return 1;
		}
	}
	return 0;

}//------------------------------------------------------------------------------
void WriteTUParams(uint8_t MezNum, int ChannelNumber, TU_Param* Params)
{
	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, portMAX_DELAY) == pdTRUE) {
			int a;
			a = MEZ_memory_Write(MezNum, ChannelNumber + PageParam, 0x00, sizeof(TU_Param), (uint8_t *) Params);
			vTaskDelay(10);
			xSemaphoreGive(xTWISemaphore);
		}
	}
}
//------------------------------------------------------------------------------
void Mez_TU_init(mezonin *MezStruct)
{
	uint32_t j;
	uint32_t CS_configuration;
	for (j = 0; j < 4; j++) {
		Mezonin_TU[MezStruct->Mez_ID - 1].Channel[j].Value = 0;
	}
	CS_configuration = AT91C_SPI_BITS_16 |/*AT91C_SPI_CPOL |*/AT91C_SPI_NCPHA| 0x30 << 8;
	SPI_ConfigureNPCS(AT91C_BASE_SPI0, MezStruct->Mez_ID - 1, CS_configuration);

}
//------------------------------------------------------------------------------
void Set_TUDefaultParams(uint8_t MezNum)
{
	TU_Param * Params;
	uint8_t i;
	for (i = 0; i < 4; i++) {
		Params = &Mezonin_TU[MezNum].Channel[i].Params;
		Params->Mode = 0;
		Params->TimeTU = 0;
		Params->CRC = Crc16((uint8_t *) Params, offsetof(TU_Param, CRC)); //TODO fix TUParams size possibly works
		if (xTWISemaphore != NULL) {
			if (xSemaphoreTake( xTWISemaphore, portMAX_DELAY ) == pdTRUE) {
				MEZ_memory_Write(MezNum, i + 1, 0x00, sizeof(TU_Param), (uint8_t *) Params);
				vTaskDelay(10);
				xSemaphoreGive(xTWISemaphore);

			}
		}

	}
}
//------------------------------------------------------------------------------
