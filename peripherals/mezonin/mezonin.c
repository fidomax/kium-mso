#include "boards/MSO_board.h"
#include "MSO_functions/MSO_functions.h"
#include "mezonin.h"
#include "constant.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "../pio/pio_config.h"
#include "../spi/spi.h"


#include "global.h"












uint32_t TI[4];

//------------------------------------------//*//------------------------------------------------
void SendCanMessage(uint32_t id, uint32_t data_l, uint32_t data_h)
{
	Message Send_Message;
	Send_Message.real_Identifier = id;

	Send_Message.data_low_reg = data_l;
	Send_Message.data_high_reg = data_h;
	Send_Message.canID = 0;

	CAN_Write(&Send_Message);
	Send_Message.canID = 1;
	CAN_Write(&Send_Message);
}
//------------------------------------------------------------------------------
// выбор памяти одного из мезонинов
void Mez_Select(uint32_t MezMemoryLine)
{
	AT91F_PIO_SetOutput(AT91C_BASE_PIOA,
	I2CCSE_1 | I2CCSE_2 | I2CCSE_3 | I2CCSE_4);
	AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, MezMemoryLine);
}

//------------------------------------------------------------------------------
uint8_t Mez_Recognition(uint8_t MezNum)
{

	uint32_t error;
	uint8_t Mez_Type;
	error = MEZ_memory_Read(MezNum, 0x00, 0x00, 1, &Mez_Type);
	if (error != 1)
		Mez_Type = Mez_NOT;
	return Mez_Type;
}
//------------------------------------------------------------------------------
int32_t Mez_SetType(uint8_t MezNum, uint8_t MezType)
{
	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, portMAX_DELAY ) == pdTRUE) {
			MEZ_memory_Write(MezNum, 0x00, 0x00, 1, &MezType);
			vTaskDelay(100);
			xSemaphoreGive(xTWISemaphore);
		}
	}
	return 0;
}
//------------------------------------------------------------------------------
void Mez_PreInit(mezonin *Mez1, mezonin *Mez2, mezonin *Mez3, mezonin *Mez4)
{
//------------------------------------------------------------------------------
// мезонин 1
//------------------------------------------------------------------------------
	Mez1->Mez_ID = Mez_1;
	Mez1->Mez_Type = Mez_NOT;
	Mez1->Mez_Mem_address = Mez_MemAddr;
	Mez1->TC_ptr = AT91C_BASE_TC0;
	Mez1->TC_ID = AT91C_ID_TC0;
	Mez1->TC_mode = AT91C_TC_CLKS_XC0 | AT91C_TC_BURST_XC0;
	Mez1->TC_blk_ptr = AT91C_BASE_TCB0;
	Mez1->TC_blk_mode = AT91C_TCB_TC0XC0S_TCLK0;
	Mez1->TC_ISR_ptr = Mez_1_TT_ISR_TC;
	Mez1->PWM_ptr = AT91C_BASE_PWMC_CH0;
	Mez1->PWM_ID = AT91C_PWMC_CHID0;
	Mez1->PWM_Number = 0;
	Mez1->Periph_AB = PeriphA;
	Mez1->LineMDATA.PIO_Line = MDATA_1;
	Mez1->LineMDATA.PIO_ctrl_ptr = AT91C_BASE_PIOB;
	Mez1->LineFIN.PIO_Line = FIN_1;
	Mez1->LineFIN.PIO_ctrl_ptr = AT91C_BASE_PIOA;
	Mez1->LineA0.PIO_Line = A1_0;
	Mez1->LineA0.PIO_ctrl_ptr = AT91C_BASE_PIOB;
	Mez1->LineA1.PIO_Line = A1_1;
	Mez1->LineA1.PIO_ctrl_ptr = AT91C_BASE_PIOB;
	Mez1->LineBRK.PIO_Line = BRK_1;
	Mez1->LineBRK.PIO_ctrl_ptr = AT91C_BASE_PIOB;
	Mez1->LineOVF.PIO_Line = OVF_1;
	Mez1->LineOVF.PIO_ctrl_ptr = AT91C_BASE_PIOB;
	Mez1->I2CCSE = I2CCSE_1;
	Mez1->CS = CS_1;
	Mez1->Start = 0;
	Mez1->TickCount = 0;
	Mez1->ActiveChannel = 0;
	vSemaphoreCreateBinary(Mez1->xSemaphore);
	xSemaphoreTake(Mez1->xSemaphore, portMAX_DELAY);
	Mez1->TUQueue = xQueueCreate(8, sizeof(Mez_Value));
	Mez1->TPQueue = xQueueCreate(8, sizeof(Mez_Value));
//------------------------------------------------------------------------------
// мезонин 2
//------------------------------------------------------------------------------
	Mez2->Mez_ID = Mez_2;
	Mez2->Mez_Type = Mez_NOT;
	Mez2->Mez_Mem_address = Mez_MemAddr;
	Mez2->TC_ptr = AT91C_BASE_TC1;
	Mez2->TC_ID = AT91C_ID_TC1;
	Mez2->TC_mode = AT91C_TC_CLKS_XC1 | AT91C_TC_BURST_XC1;
	Mez2->TC_blk_ptr = AT91C_BASE_TCB0;
	Mez2->TC_blk_mode = AT91C_TCB_TC1XC1S_TCLK1;
	Mez2->TC_ISR_ptr = Mez_2_TT_ISR_TC;
	Mez2->PWM_ptr = AT91C_BASE_PWMC_CH1;
	Mez2->PWM_ID = AT91C_PWMC_CHID1;
	Mez2->PWM_Number = 1;
	Mez2->Periph_AB = PeriphA;
	Mez2->LineMDATA.PIO_Line = MDATA_2;
	Mez2->LineMDATA.PIO_ctrl_ptr = AT91C_BASE_PIOB;
	Mez2->LineFIN.PIO_Line = FIN_2;
	Mez2->LineFIN.PIO_ctrl_ptr = AT91C_BASE_PIOA;
	Mez2->LineA0.PIO_Line = A2_0;
	Mez2->LineA0.PIO_ctrl_ptr = AT91C_BASE_PIOB;
	Mez2->LineA1.PIO_Line = A2_1;
	Mez2->LineA1.PIO_ctrl_ptr = AT91C_BASE_PIOB;
	Mez2->LineBRK.PIO_Line = BRK_2;
	Mez2->LineBRK.PIO_ctrl_ptr = AT91C_BASE_PIOB;
	Mez2->LineOVF.PIO_Line = OVF_2;
	Mez2->LineOVF.PIO_ctrl_ptr = AT91C_BASE_PIOB;
	Mez2->I2CCSE = I2CCSE_2;
	Mez2->CS = CS_2;
	Mez2->Start = 0;
	Mez2->TickCount = 0;
	Mez2->ActiveChannel = 0;
	vSemaphoreCreateBinary(Mez2->xSemaphore);
	xSemaphoreTake(Mez2->xSemaphore, portMAX_DELAY);
	Mez2->TUQueue = xQueueCreate(8, sizeof(Mez_Value));
	Mez2->TPQueue = xQueueCreate(8, sizeof(Mez_Value));
//------------------------------------------------------------------------------
// мезонин 3
//------------------------------------------------------------------------------
	Mez3->Mez_ID = Mez_3;
	Mez3->Mez_Type = Mez_NOT;
	Mez3->Mez_Mem_address = Mez_MemAddr;
	Mez3->TC_ptr = AT91C_BASE_TC2;
	Mez3->TC_ID = AT91C_ID_TC2;
	Mez3->TC_mode = AT91C_TC_CLKS_XC2 | AT91C_TC_BURST_XC2;
	Mez3->TC_blk_ptr = AT91C_BASE_TCB0;
	Mez3->TC_blk_mode = AT91C_TCB_TC2XC2S_TCLK2;
	Mez3->TC_ISR_ptr = Mez_3_TT_ISR_TC;
	Mez3->PWM_ptr = AT91C_BASE_PWMC_CH2;
	Mez3->PWM_ID = AT91C_PWMC_CHID2;
	Mez3->PWM_Number = 2;
	Mez3->Periph_AB = PeriphA;
	Mez3->LineMDATA.PIO_Line = MDATA_3;
	Mez3->LineMDATA.PIO_ctrl_ptr = AT91C_BASE_PIOB;
	Mez3->LineFIN.PIO_Line = FIN_3;
	Mez3->LineFIN.PIO_ctrl_ptr = AT91C_BASE_PIOA;
	Mez3->LineA0.PIO_Line = A3_0;
	Mez3->LineA0.PIO_ctrl_ptr = AT91C_BASE_PIOB;
	Mez3->LineA1.PIO_Line = A3_1;
	Mez3->LineA1.PIO_ctrl_ptr = AT91C_BASE_PIOB;
	Mez3->LineBRK.PIO_Line = BRK_3;
	Mez3->LineBRK.PIO_ctrl_ptr = AT91C_BASE_PIOB;
	Mez3->LineOVF.PIO_Line = OVF_3;
	Mez3->LineOVF.PIO_ctrl_ptr = AT91C_BASE_PIOB;
	Mez3->I2CCSE = I2CCSE_3;
	Mez3->CS = CS_3;
	Mez3->Start = 0;
	Mez3->TickCount = 0;
	Mez3->ActiveChannel = 0;
	vSemaphoreCreateBinary(Mez3->xSemaphore);
	xSemaphoreTake(Mez3->xSemaphore, portMAX_DELAY);
	Mez3->TUQueue = xQueueCreate(8, sizeof(Mez_Value));
	Mez3->TPQueue = xQueueCreate(8, sizeof(Mez_Value));
//------------------------------------------------------------------------------
// мезонин 4
//------------------------------------------------------------------------------
	Mez4->Mez_ID = Mez_4;
	Mez4->Mez_Type = Mez_NOT;
	Mez4->Mez_Mem_address = Mez_MemAddr;
	Mez4->TC_ptr = AT91C_BASE_TC5;
	Mez4->TC_ID = AT91C_ID_TC5;
	Mez4->TC_mode = AT91C_TC_CLKS_XC2 | AT91C_TC_BURST_XC2;
	Mez4->TC_blk_ptr = AT91C_BASE_TCB1;
	Mez4->TC_blk_mode = AT91C_TCB_TC2XC2S_TCLK2;
	Mez4->TC_ISR_ptr = Mez_4_TT_ISR_TC;
	Mez4->PWM_ptr = AT91C_BASE_PWMC_CH3;
	Mez4->PWM_ID = AT91C_PWMC_CHID3;
	Mez4->PWM_Number = 3;
	Mez4->Periph_AB = PeriphB;
	Mez4->LineMDATA.PIO_Line = MDATA_4;
	Mez4->LineMDATA.PIO_ctrl_ptr = AT91C_BASE_PIOA;
	Mez4->LineFIN.PIO_Line = FIN_4;
	Mez4->LineFIN.PIO_ctrl_ptr = AT91C_BASE_PIOA;
	Mez4->LineA0.PIO_Line = A4_0;
	Mez4->LineA0.PIO_ctrl_ptr = AT91C_BASE_PIOB;
	Mez4->LineA1.PIO_Line = A4_1;
	Mez4->LineA1.PIO_ctrl_ptr = AT91C_BASE_PIOB;
	Mez4->LineBRK.PIO_Line = BRK_4;
	Mez4->LineBRK.PIO_ctrl_ptr = AT91C_BASE_PIOB;
	Mez4->LineOVF.PIO_Line = OVF_4;
	Mez4->LineOVF.PIO_ctrl_ptr = AT91C_BASE_PIOB;
	Mez4->I2CCSE = I2CCSE_4;
	Mez4->CS = CS_4;
	Mez4->Start = 0;
	Mez4->TickCount = 0;
	Mez4->ActiveChannel = 0;
	vSemaphoreCreateBinary(Mez4->xSemaphore);
	xSemaphoreTake(Mez4->xSemaphore, portMAX_DELAY);
	Mez4->TUQueue = xQueueCreate(8, sizeof(Mez_Value));
	Mez4->TPQueue = xQueueCreate(8, sizeof(Mez_Value));
}
//------------------------------------------------------------------------------
void Mez_init(uint32_t Mezonin_Type, mezonin *MezStruct)
{

	switch (Mezonin_Type) {
		case Mez_TC:
			Mez_TC_init(MezStruct);
			break;

		case Mez_TU:
			Mez_TU_init(MezStruct);
			break;

		case Mez_TT:
			Mez_TT_init(MezStruct);
			break;

		case Mez_TR:
			Mez_TR_init(MezStruct);
			break;

		case Mez_TI:
			Mez_TI_init(MezStruct);
			break;

		case Mez_NOT:
			Mez_NOT_init();
			break;
	}

}
//------------------------------------------------------------------------------
void Mez_TR_init(mezonin *MezStruct)
{
	uint32_t CS_configuration;
	Mezonin_TR[MezStruct->Mez_ID - 1].Channel.flDAC = 0;
	CS_configuration = AT91C_SPI_BITS_8  | 0x30 << 8  |  0<<24| 0<<16  | AT91C_SPI_NCPHA;
	SPI_ConfigureNPCS(AT91C_BASE_SPI0, MezStruct->Mez_ID-1, CS_configuration);
	if (xSPISemaphore != NULL) {
		if (xSemaphoreTake( xSPISemaphore, portMAX_DELAY ) == pdTRUE) {
			SPI_Write(AT91C_BASE_SPI0, MezStruct->Mez_ID-1, 0b01010101);//control register
			SPI_Read(AT91C_BASE_SPI0);
			SPI_Write(AT91C_BASE_SPI0, MezStruct->Mez_ID-1, 0b10000);//output enable
			SPI_Read(AT91C_BASE_SPI0);
			SPI_Write(AT91C_BASE_SPI0, MezStruct->Mez_ID-1, 0b110);// от 0 до 20 mA
			SPI_Read(AT91C_BASE_SPI0);
			SPI_Write(AT91C_BASE_SPI0, MezStruct->Mez_ID-1, 0b1);//data register
			SPI_Read(AT91C_BASE_SPI0);
			SPI_Write(AT91C_BASE_SPI0, MezStruct->Mez_ID-1, 0);
			SPI_Read(AT91C_BASE_SPI0);
			SPI_Write(AT91C_BASE_SPI0, MezStruct->Mez_ID-1, 0);
			SPI_Read(AT91C_BASE_SPI0);
			xSemaphoreGive(xSPISemaphore);
		}
	}

}
//------------------------------------------------------------------------------
// EnableChannel (Channel) активирует выбранный канал
// Входной параметр Channel - номер канала 1, 2, 3, 4.
//------------------------------------------------------------------------------
void Mez_EnableChannel(mezonin *MezStruct)
{
	switch (MezStruct->ActiveChannel) {
		case 1:
			AT91F_PIO_ClearOutput(MezStruct->LineA0.PIO_ctrl_ptr, MezStruct->LineA0.PIO_Line | MezStruct->LineA1.PIO_Line);
			//Конфигурация канала:   0      0
			//                       А1    А0
			break;

		case 2:
			AT91F_PIO_ClearOutput(MezStruct->LineA1.PIO_ctrl_ptr, MezStruct->LineA1.PIO_Line);
			AT91F_PIO_SetOutput(MezStruct->LineA0.PIO_ctrl_ptr, MezStruct->LineA0.PIO_Line);
			//Конфигурация канала:   0      1
			//                       А1    А0
			break;

		case 3:
			AT91F_PIO_ClearOutput(MezStruct->LineA0.PIO_ctrl_ptr, MezStruct->LineA0.PIO_Line);
			AT91F_PIO_SetOutput(MezStruct->LineA1.PIO_ctrl_ptr, MezStruct->LineA1.PIO_Line);
			//Конфигурация канала:   1      0
			//                       А1    А0
			break;

		case 4:
			AT91F_PIO_SetOutput(MezStruct->LineA0.PIO_ctrl_ptr, MezStruct->LineA0.PIO_Line | MezStruct->LineA1.PIO_Line);
			//Конфигурация канала:   1      1
			//                       А1     А0
			break;
	}
	vTaskDelay(2);

}
//------------------------------------------------------------------------------

void Mez_NOT_init(void)
{

}

//------------------------------------------------------------------------------
void Mez_NOT_handler(void)
{
	// пустая функция
}

//------------------------------------------------------------------------------
void Mez_TP_handler(mezonin *MezStruct)
{
	unsigned short usDAC;
	Mez_Value Real_TR;
	if (MezStruct->TPQueue != 0) {
		if (xQueueReceive(MezStruct->TPQueue, &Real_TR, 1000)) {
			Mezonin_TR[Real_TR.ID].Channel.flDAC = Real_TR.fValue;
			if(Mezonin_TR[Real_TR.ID].Channel.flDAC>20.0){
				usDAC=65535;
			} else{
				usDAC=Mezonin_TR[Real_TR.ID].Channel.flDAC/20*65535;
			}
			if (xSPISemaphore != NULL) {
				if (xSemaphoreTake( xSPISemaphore, portMAX_DELAY ) == pdTRUE) {
					SPI_Write(AT91C_BASE_SPI0, MezStruct->Mez_ID-1, 0b1);//data register
					SPI_Read(AT91C_BASE_SPI0);
					SPI_Write(AT91C_BASE_SPI0, MezStruct->Mez_ID-1, ((usDAC >>8) & 0xFF));
					SPI_Read(AT91C_BASE_SPI0);
					SPI_Write(AT91C_BASE_SPI0, MezStruct->Mez_ID-1, (usDAC & 0xFF));
					SPI_Read(AT91C_BASE_SPI0);
					xSemaphoreGive(xSPISemaphore);
				}
			}
		}
	}
}
//------------------------------------------------------------------------------


