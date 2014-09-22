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
	uint32_t ID;
	ChannelNumber = Mez_V->ID * 4 + Mez_V->Channel;
	ID = MAKE_CAN_ID(priority_N, identifier_TI, MSO_Address, ChannelNumber, ParamTI);
	TI_Channel * ti_channel = &Mezonin_TI[Mez_V->ID].Channel[Mez_V->Channel];

	if (Mez_V->ui32Value ) { // если есть импульсы

		ti_channel->CountTI += Mez_V->ui32Value;
		SendCanMessage(ID, ti_channel->CountTI);
	}


}
//------------------------------------------------------------------------------
