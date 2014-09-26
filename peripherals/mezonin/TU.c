/*
 * TU.c
 *
 *  Created on: 25 сент. 2014 г.
 *      Author: maximus
 */
#include "mezonin.h"
#include "../spi/spi.h"

int MakeTUValue(const Mez_Value* Real_TU)
{
	return Mezonin_TU[Real_TU->ID].Channel[0].Value << 4 | Mezonin_TU[Real_TU->ID].Channel[1].Value << 5 | Mezonin_TU[Real_TU->ID].Channel[2].Value << 6
			| Mezonin_TU[Real_TU->ID].Channel[3].Value << 7;
}

//------------------------------------------------------------------------------
void Mez_TU_handler(mezonin *MezStruct)
{

	uint8_t MDATA[4];
	Mez_Value Real_TU;
	if (Mezonin_TU[MezStruct->Mez_ID - 1].Channel[MezStruct->ActiveChannel - 1].Params.Mode
			== mode_on|| Mezonin_TU[MezStruct->Mez_ID - 1].Channel[MezStruct->ActiveChannel
			- 1].Params.Mode == mode_ok) {
		Mez_EnableChannel(MezStruct);		// переделал под свою функцию

		if (MezStruct->TUQueue != 0) {
			if (xQueueReceive(MezStruct->TUQueue, &Real_TU, 1000)) {
				Mezonin_TU[Real_TU.ID].Channel[Real_TU.Channel].Value = Real_TU.ui32Value;
				MDATA[Real_TU.ID] = MakeTUValue(&Real_TU);		// пишем сразу в 4 канала
				if (xSPISemaphore != NULL) {
					if (xSemaphoreTake( xSPISemaphore, portMAX_DELAY ) == pdTRUE) {
						SPI_Write(AT91C_BASE_SPI0, Real_TU.ID, 0x00);
						SPI_Read(AT91C_BASE_SPI0);
						SPI_Write(AT91C_BASE_SPI0, Real_TU.ID, 0x00);
						SPI_Read(AT91C_BASE_SPI0);
						SPI_Write(AT91C_BASE_SPI0, Real_TU.ID, MDATA[Real_TU.ID]);
						SPI_Read(AT91C_BASE_SPI0);
						xSemaphoreGive(xSPISemaphore);
					}
				}

			} else {
				// время выдержки

			}
		}
	}

}
//------------------------------------------------------------------------------
uint32_t Get_TUParams(TU_Value *TU_temp)
{
	uint32_t i;
	for (i = 0; i < 4; i++) {
		TU_temp->Channel[i].Params.Mode = 0;
		TU_temp->Channel[i].Value = 0;

		//a = MEZ_memory_Read (TC_temp->ID, i+1, 0x00, sizeof (TC_Param), DataRecieve); // чтение начальных параметров из EEPROM
		//vTaskDelay(100);

		/*TC_temp->Channel[i].Params = * (TC_Param *) DataRecieve;

		 temp_CRC = Crc16(DataRecieve, sizeof (TC_Param) - sizeof (uint16_t)-2);

		 if (temp_CRC == TC_temp->Channel[i].Params.CRC)		//проверка СRC
		 {
		 TC_temp->Channel[i].Params.Mode = TC_temp->Channel[i].Params.Mode;
		 count = 0;
		 }
		 else
		 {
		 count++;
		 if (count < 3)
		 i = i-1;
		 else
		 count = 0;
		 }
		 */
	}
	return 0;

}
//------------------------------------------------------------------------------
void Mez_TU_init(mezonin *MezStruct)
{
	uint32_t j;
	uint32_t CS_configuration;
	for (j = 0; j < 4; j++) {
		Mezonin_TU[MezStruct->Mez_ID - 1].Channel[j].Value = 0xFF;
	}
	CS_configuration = AT91C_SPI_BITS_8 /*|AT91C_SPI_CPOL | AT91C_SPI_NCPHA*/ | 0x30 << 8;
	SPI_ConfigureNPCS(AT91C_BASE_SPI0, MezStruct->Mez_ID - 1, CS_configuration);

}
//------------------------------------------------------------------------------
