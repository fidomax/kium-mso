/*
 * TI.c
 *
 *  Created on: 22 сент. 2014 г.
 *      Author: maximus
 */
#include "mezonin.h"
#include "global.h"
#include "../spi/spi.h"
#include "constant.h"
#include "MSO_functions/MSO_functions.h"
//------------------------------------------------------------------------------
void Mez_TI_init(mezonin *MezStruct)
{
	uint32_t j;
	uint32_t CS_configuration;
	for (j = 0; j < 4; j++) {
		Mezonin_TI[MezStruct->Mez_ID - 1].Channel[j].CountTI = 0;
	}
	CS_configuration = AT91C_SPI_BITS_8 /*|AT91C_SPI_CPOL | AT91C_SPI_NCPHA*/ | 0x10 << 8 | 0x1F << 24 | 0x1F << 16;
	SPI_ConfigureNPCS(AT91C_BASE_SPI0, MezStruct->Mez_ID - 1, CS_configuration);
}
//------------------------------------------------------------------------------
void Mez_TI_handler(mezonin *MezStruct)
{
	Mez_Value Real_TI;
	Real_TI.ID = MezStruct->Mez_ID - 1;


	SPI_Write(AT91C_BASE_SPI0, MezStruct->Mez_ID-1, 0x10);
	SPI_Read(AT91C_BASE_SPI0);
	SPI_Write(AT91C_BASE_SPI0, MezStruct->Mez_ID-1, 0x11);
	Real_TI.ui32Value = SPI_Read(AT91C_BASE_SPI0);
	Real_TI.Channel = 0;
	if (xMezQueue != 0) {
		xQueueSend(xMezQueue, &Real_TI, (TickType_t ) 0);
	}
	SPI_Write(AT91C_BASE_SPI0, MezStruct->Mez_ID-1, 0x12);
	Real_TI.ui32Value = SPI_Read(AT91C_BASE_SPI0);
	Real_TI.Channel = 1;
	if (xMezQueue != 0) {
		xQueueSend(xMezQueue, &Real_TI, (TickType_t ) 0);
	}
	SPI_Write(AT91C_BASE_SPI0, MezStruct->Mez_ID-1, 0x13);
	Real_TI.ui32Value = SPI_Read(AT91C_BASE_SPI0);
	Real_TI.Channel = 2;
	if (xMezQueue != 0) {
		xQueueSend(xMezQueue, &Real_TI, (TickType_t ) 0);
	}
	SPI_Write(AT91C_BASE_SPI0, MezStruct->Mez_ID-1, 0x00);
	Real_TI.ui32Value = SPI_Read(AT91C_BASE_SPI0);
	Real_TI.Channel = 3;
	if (xMezQueue != 0) {
		xQueueSend(xMezQueue, &Real_TI, (TickType_t ) 0);
	}
}
//------------------------------------------------------------------------------
void TIValueHandler (Mez_Value *Mez_V)
{
	uint32_t ChannelNumber;
	uint32_t ID,ID1;
	ChannelNumber = Mez_V->ID * 4 + Mez_V->Channel;
	ID = MAKE_MSG_ID(priority_N, identifier_TI, ChannelNumber, ParamTI);
	ID1 = MAKE_MSG_ID(priority_N, identifier_TI, ChannelNumber, 1);
	TI_Channel * ti_channel = &Mezonin_TI[Mez_V->ID].Channel[Mez_V->Channel];

	if (Mez_V->ui32Value ) { // если есть импульсы

		ti_channel->CountTI += Mez_V->ui32Value;
		SendCanMessage(ID, ti_channel->CountTI,0);
		ti_channel->Value=ti_channel->Params.CoeFf*ti_channel->CountTI;
		SendCanMessage(ID1, *((uint32_t *) &ti_channel->Value),0);
	}


}
//------------------------------------------------------------------------------
void WriteTIParams(uint8_t MezNum, int ChannelNumber, TI_Param* Params)
{
	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, portMAX_DELAY) == pdTRUE) {
			int a;
			a = MEZ_memory_Write(MezNum, ChannelNumber + PageParam, 0x00, sizeof(TI_Param), (uint8_t *) Params);
			vTaskDelay(10);
			xSemaphoreGive(xTWISemaphore);
		}
	}
}
//------------------------------------------------------------------------------
uint32_t Get_TIParams(TI_Value *TI_temp)
{
	uint32_t i;
	uint16_t temp_CRC;
	uint8_t DataRecieve[sizeof(TI_Param)];
	for (i = 0; i < 4; i++) {
		TI_temp->Channel[i].Params.Mode = 0;
		TI_temp->Channel[i].Value = 0;

		MEZ_memory_Read(TI_temp->ID, i + 1, 0x00, sizeof(TI_Param), DataRecieve); // чтение начальных параметров из EEPROM

		temp_CRC = Crc16(DataRecieve, offsetof(TI_Param, CRC)); //TODO fix TI_Param size possibly works
		if (temp_CRC == ((TI_Param *) DataRecieve)->CRC) {
			TI_temp->Channel[i].Params = *(TI_Param *) DataRecieve;
		} else {
			return 1;
		}
	}
	return 0;
}
//------------------------------------------------------------------------------
void Set_TIDefaultParams(uint8_t MezNum)
{
	TI_Param * Params;
	uint8_t i;
	for (i = 0; i < 4; i++) {
		Params = &Mezonin_TI[MezNum].Channel[i].Params;
		Params->Mode = 0;
		Params->Sense = 0;
		Params->CoeFf = 1;
		Params->CRC = Crc16((uint8_t *) Params, offsetof(TI_Param, CRC)); //TODO fix TIParams size possibly works
		if (xTWISemaphore != NULL) {
			if (xSemaphoreTake( xTWISemaphore, portMAX_DELAY ) == pdTRUE) {
				MEZ_memory_Write(MezNum, i + 1, 0x00, sizeof(TI_Param), (uint8_t *) Params);
				vTaskDelay(10);
				xSemaphoreGive(xTWISemaphore);

			}
		}

	}
}
//------------------------------------------------------------------------------

