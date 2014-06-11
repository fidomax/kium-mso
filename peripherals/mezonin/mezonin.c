#include "boards/MSO_board.h"
#include "MSO_functions/MSO_functions.h"
#include "mezonin.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "../pio/pio_config.h"
#include "../spi/spi.h"
#include "../pwmc/pwmc.h"
#include "../tc/tc.h"
#include "../aic/aic.h"
uint32_t VVV[4][4];

#define TC_OFF      0
#define TC_ON       1
#define TC_OK		1
#define TC_BRK		0
#define Channel_OFF	4

//------------------------------------------------------------------------------
//         For Page Mezonin
//------------------------------------------------------------------------------
#define PageType	((uint32_t) 0)
#define PageParam	((uint32_t) 1)
#define PageCoeff	((uint32_t) 5)
#define PageLevel	((uint32_t) 9)

uint32_t overflow[4];		// массив значений перполнений счетчика TT

extern TT_Value Mezonin_TT[4];
extern TC_Value Mezonin_TC[4];
extern TU_Value Mezonin_TU[4];

extern QueueHandle_t xMezQueue;
extern QueueHandle_t xMezTUQueue;
extern SemaphoreHandle_t xTWISemaphore;

uint32_t Min20, Min100, Max20, Max100;
int16_t flag_calib;

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
		if (xSemaphoreTake( xTWISemaphore, ( TickType_t ) 10 ) == pdTRUE) {
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

		case Mez_TP:
			Mez_TP_init(MezStruct);
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
void Mez_TU_init(mezonin *MezStruct)
{
	uint32_t SPI0_configuration, CS_configuration;
	uint32_t j;
	for (j = 0; j < 4; j++) {
		Mezonin_TU[MezStruct->Mez_ID - 1].Channel[j].Value = 0xFF;
	}

	AT91F_PIO_CfgOutput(AT91C_BASE_PIOB, LED_0);

	SPI_Pin_config();

	CS_configuration = AT91C_SPI_BITS_8 | AT91C_SPI_NCPHA | 0x30 << 8;

	SPI0_configuration = AT91C_SPI_MSTR | AT91C_SPI_MODFDIS | AT91C_SPI_PS_VARIABLE | 0x00 << 16;
	SPI_Configure(AT91C_BASE_SPI0, AT91C_ID_SPI0, SPI0_configuration);
	SPI_ConfigureNPCS(AT91C_BASE_SPI0, 0, CS_configuration);
	SPI_ConfigureNPCS(AT91C_BASE_SPI0, 1, CS_configuration);
	SPI_ConfigureNPCS(AT91C_BASE_SPI0, 2, CS_configuration);
	SPI_ConfigureNPCS(AT91C_BASE_SPI0, 3, CS_configuration);

	SPI_Enable(AT91C_BASE_SPI0);
}
//------------------------------------------------------------------------------
void Mez_TT_init(mezonin *MezStruct)
{
	uint32_t PWM_period, PWM_dutyCycle;

	uint32_t j;
	for (j = 0; j < 4; j++) {
		Mezonin_TT[MezStruct->Mez_ID - 1].Channel[j].OldValue = 0xFF;
	}

	AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PWMC; //Enable PMC для PWM
	AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PIOB; //Enable PMC для PIOB
	AT91C_BASE_PMC->PMC_PCER = 1 << MezStruct->TC_ID;

	AT91C_BASE_PWMC->PWMC_DIS = MezStruct->PWM_ID;
	while ((AT91C_BASE_PWMC->PWMC_SR & MezStruct->PWM_ID) == MezStruct->PWM_ID)
		;

	AT91F_PIO_CfgOutput(MezStruct->LineA0.PIO_ctrl_ptr, MezStruct->LineA0.PIO_Line | MezStruct->LineA1.PIO_Line);

	AT91F_PIO_CfgPeriph(MezStruct->LineFIN.PIO_ctrl_ptr, MezStruct->LineFIN.PIO_Line, 0);
	//Конфигурация PWM канала 1FIN, частота 4 МГц
	PWM_period = 12;
	PWM_dutyCycle = PWM_period / 2;

	PWMC_ConfigureClocks(0, 0, MCK); // запись 0 в регистр PWM_MR
	PWMC_ConfigureChannel(MezStruct->PWM_Number, 0, 0, 0); // запись 0 в регистр PWM_CMR 1-го канала
	PWMC_SetPeriod(MezStruct->PWM_Number, PWM_period); // Установка периода для 1-го канала
	PWMC_SetDutyCycle(MezStruct->PWM_Number, PWM_dutyCycle); // Установка duty ) для 1-го канала

	if (MezStruct->Periph_AB == PeriphA)
		AT91F_PIO_CfgPeriph(MezStruct->LineMDATA.PIO_ctrl_ptr, MezStruct->LineMDATA.PIO_Line, 0);

	else if (MezStruct->Periph_AB == PeriphB)
		AT91F_PIO_CfgPeriph(MezStruct->LineMDATA.PIO_ctrl_ptr, 0, MezStruct->LineMDATA.PIO_Line);

	AIC_ConfigureIT(MezStruct->TC_ID, 0, MezStruct->TC_ISR_ptr);
	AIC_EnableIT(MezStruct->TC_ID);

	MezStruct->TC_blk_ptr->TCB_BMR |= MezStruct->TC_blk_mode;
	// здесь, наверно, конфигурацию добавлять нужно будет (|=), т.к. блок на три счетчика надо конфигурировать
	// а вообще-то, чего ее добавлять, если для всех счетчиков конфигурация блока нулевая
	// но, конечно, добавлять будет правильнее, хотя в данном случае ни на что это не влияет

	//конфигурация счетчика TC1: внешний сигнал подается на TCLK1
	TC_Configure(MezStruct->TC_ptr, MezStruct->TC_mode);

	MezStruct->TC_ptr->TC_IER = AT91C_TC_COVFS;
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
//------------------------------------------------------------------------------
// EnableChannel (Channel) активирует выбранный канал
// Входной параметр Channel - номер канала 1, 2, 3, 4.
//------------------------------------------------------------------------------
/*void Mez_TT_EnableChannel (uint32_t Channel, uint32_t A0, uint32_t A1)
 {
 switch (Channel)
 {
 case 1:
 AT91F_PIO_ClearOutput(AT91C_BASE_PIOB, A0 | A1);
 //Конфигурация канала:   0      0
 //                       А1    А0
 break;

 case 2:
 AT91F_PIO_ClearOutput(AT91C_BASE_PIOB, A1);
 AT91F_PIO_SetOutput(AT91C_BASE_PIOB, A0);
 //Конфигурация канала:   0      1
 //                       А1    А0
 break;

 case 3:
 AT91F_PIO_ClearOutput(AT91C_BASE_PIOB, A0);
 AT91F_PIO_SetOutput(AT91C_BASE_PIOB, A1);
 //Конфигурация канала:   1      0
 //                       А1    А0
 break;

 case 4:
 AT91F_PIO_SetOutput(AT91C_BASE_PIOB, A0 | A1);
 //Конфигурация канала:   1      1
 //                       А1     А0
 break;
 }
 }*/
//------------------------------------------------------------------------------
float Mez_TT_Frequency(uint32_t measured_value, uint32_t ChannelNumber, uint32_t MEZ_ID) // нахождение физической величины
{
// TT_Value mez_freq;
// mez_freq[0].Channel[0].Min_Value = 4701;
	float MIN_measured_value = 0, MAX_measured_value = 0, MAX_value = 20; // Y0, Ym, Xm
	float corrected_value, real_value = 0, step; // Xi

	MIN_measured_value = Mezonin_TT[MEZ_ID].Channel[ChannelNumber].Min_Value;
	MAX_measured_value = Mezonin_TT[MEZ_ID].Channel[ChannelNumber].Max_Value;

	corrected_value = ((measured_value - MIN_measured_value) / (MAX_measured_value - MIN_measured_value)) * MAX_value;

	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, ( TickType_t ) 10 ) == pdTRUE) {
			step = (Mezonin_TT[MEZ_ID].Channel[ChannelNumber].Params.MaxF - Mezonin_TT[MEZ_ID].Channel[ChannelNumber].Params.MinF)
					/ (Mezonin_TT[MEZ_ID].Channel[ChannelNumber].Params.MaxD - Mezonin_TT[MEZ_ID].Channel[ChannelNumber].Params.MinD);
			real_value = Mezonin_TT[MEZ_ID].Channel[ChannelNumber].Params.MinF
					+ (corrected_value - Mezonin_TT[MEZ_ID].Channel[ChannelNumber].Params.MinD) * step;

			//  						vTaskDelay(100);
			xSemaphoreGive(xTWISemaphore);
		}
	}

	return real_value;
// return corrected_value;
}
//------------------------------------------------------------------------------
void Mez_TP_init(mezonin *MezStruct)
{

}
//------------------------------------------------------------------------------
void Mez_TI_init(mezonin *MezStruct)
{

}
//------------------------------------------------------------------------------
void Mez_NOT_init(void)
{

}
//------------------------------------------------------------------------------
/*void Mez_handler_select (uint32_t Mezonin_Type, mezonin *MezStruct)
 {
 switch (Mezonin_Type)
 {
 case Mez_TC:
 Mez_TC_handler (MezStruct);
 break;

 case Mez_TY:
 Mez_TY_handler (MezStruct);
 break;

 case Mez_TT:
 Mez_TT_handler (MezStruct);
 break;

 case Mez_TP:
 Mez_TP_handler (MezStruct);
 break;

 case Mez_TI:
 Mez_TI_handler (MezStruct);
 break;

 case Mez_NOT:
 //    default:
 Mez_NOT_handler ();
 break;
 }
 }*/
//------------------------------------------------------------------------------
void Mez_NOT_handler(void)
{
	// пустая функция
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
		Real_TC.Value = MDATA | BRK << 1;
		Real_TC.Channel = MezStruct->ActiveChannel - 1;
		Real_TC.ID = MezStruct->Mez_ID - 1;

		if (xMezQueue != 0) {
			// Send an unsigned long.  Wait for 10 ticks for space to become
			// available if necessary.
			//		Switch_State=Value;
			if (xQueueSend(xMezQueue, &Real_TC, (TickType_t ) 0)) {
				//	       test++;     // Failed to post the message, even after 10 ticks.
			}
		}
	}
//        time=1000;while(time);

}
//------------------------------------------------------------------------------
void Mez_TI_handler(mezonin *MezStruct)
{

}
//------------------------------------------------------------------------------
void Mez_TP_handler(mezonin *MezStruct)
{

}
//------------------------------------------------------------------------------
void Mez_TT_handler(mezonin *MezStruct/*, TT_Value *Mez_TT_temp*/)
{
	uint32_t Channel_V;
	uint32_t test_count = 0;

	Mez_Value Real_Value;

	if (flag_calib != 1) {

		MezStruct->ActiveChannel++;
		if (MezStruct->ActiveChannel == 5) {
			MezStruct->ActiveChannel = 1;
		}

		if (Mezonin_TT[MezStruct->Mez_ID - 1].Channel[MezStruct->ActiveChannel - 1].Params.Mode == mode_ok) {

			Mez_EnableChannel(MezStruct);	// переделал под свою функцию
			overflow[MezStruct->Mez_ID - 1] = 0;

			TC_Start(MezStruct->TC_ptr); //запуск счетчика TC1 (2MDATA)

			while ((MezStruct->TC_ptr->TC_SR & AT91C_TC_CLKSTA) != AT91C_TC_CLKSTA)
				;
			MezStruct->Start = 1;
			MezStruct->TickCount = Mezonin_TT[MezStruct->Mez_ID - 1].Channel[MezStruct->ActiveChannel - 1].Params.MeasTime;

			xSemaphoreTake(MezStruct->xSemaphore, portMAX_DELAY);
			TC_Stop(MezStruct->TC_ptr); //остановка счетчика TC1 (2MDATA)

			Channel_V = MezStruct->TC_ptr->TC_CV + 65536 * overflow[MezStruct->Mez_ID - 1]; //сохранение значения счетчика
			VVV[MezStruct->Mez_ID - 1][MezStruct->ActiveChannel - 1] = Channel_V;

			Real_Value.Value = Channel_V;
			Real_Value.Channel = MezStruct->ActiveChannel - 1;
			Real_Value.ID = MezStruct->Mez_ID - 1;

			if (xMezQueue != 0) {

				if (xQueueSend(xMezQueue, &Real_Value, (TickType_t ) 0)) {
					test_count++; // Failed to post the message, even after 10 ticks.
				}
			}
		}
	}
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void Mez_TT_Calib(mezonin *MezStruct, uint32_t Channel_Num, uint32_t flag/*, TT_Value *Mez_TT_temp*/)
{
//  uint32_t Signal_value [4]; //массив подсчитанных значений физических величин всех каналов ввода
//  uint32_t test = 0;
//	uint32_t Channel_V;
//  uint32_t Channel_value [4];   //массив значений счетчика для всех каналов ввода
//	uint32_t MEZ_ID;
//	uint32_t test_count = 0;
	TickType_t xLastWakeTime;
	uint32_t MeasTime1, MeasTime2, a;

//  TickType_t d2,mt,ct;

//  uint32_t ChannelNumber;
//  uint32_t ID;
//  Mez_Value Real_Value;
	flag_calib = 1;
	MeasTime1 = 20;
	MeasTime2 = 100;
	/*  MezStruct->ActiveChannel++;
	 if (MezStruct->ActiveChannel == 5)
	 MezStruct->ActiveChannel = 1;*/
	MezStruct->ActiveChannel = Channel_Num + 1;

//  if (Mezonin_TT[MezStruct->Mez_ID-1].Channel[MezStruct->ActiveChannel-1].Params.Mode == mode_ok)
//  {
	switch (flag)
//	  if (flag == 1)
	{
		case 1:
			Mez_EnableChannel(MezStruct);		// переделал под свою функцию
			// калибруем минимум для 20
			overflow[MezStruct->Mez_ID - 1] = 0;

			TC_Start(MezStruct->TC_ptr); //запуск счетчика TC1 (2MDATA)

			while ((MezStruct->TC_ptr->TC_SR & AT91C_TC_CLKSTA) != AT91C_TC_CLKSTA)
				;
			MezStruct->Start = 1;
//	  MezStruct->TickCount = Mezonin_TT[MezStruct->Mez_ID-1].Channel[MezStruct->ActiveChannel - 1].Params.MeasTime;
			MezStruct->TickCount = MeasTime1;

			xSemaphoreTake(MezStruct->xSemaphore, portMAX_DELAY);
			TC_Stop(MezStruct->TC_ptr); //остановка счетчика TC1 (2MDATA)

			Min20 = MezStruct->TC_ptr->TC_CV + 65536 * overflow[MezStruct->Mez_ID - 1]; //сохранение значения счетчика

			// калибруем минимум для 100
			overflow[MezStruct->Mez_ID - 1] = 0;

			TC_Start(MezStruct->TC_ptr); //запуск счетчика TC1 (2MDATA)

			while ((MezStruct->TC_ptr->TC_SR & AT91C_TC_CLKSTA) != AT91C_TC_CLKSTA)
				;
			MezStruct->Start = 1;
//	  MezStruct->TickCount = Mezonin_TT[MezStruct->Mez_ID-1].Channel[MezStruct->ActiveChannel - 1].Params.MeasTime;
			MezStruct->TickCount = MeasTime2;

			xSemaphoreTake(MezStruct->xSemaphore, portMAX_DELAY);
			TC_Stop(MezStruct->TC_ptr); //остановка счетчика TC1 (2MDATA)

			Min100 = MezStruct->TC_ptr->TC_CV + 65536 * overflow[MezStruct->Mez_ID - 1]; //сохранение значения счетчика
			break;
		case 2:
			Mez_EnableChannel(MezStruct);		// переделал под свою функцию
			// калибруем максимум для 20
			overflow[MezStruct->Mez_ID - 1] = 0;

			TC_Start(MezStruct->TC_ptr); //запуск счетчика TC1 (2MDATA)

			while ((MezStruct->TC_ptr->TC_SR & AT91C_TC_CLKSTA) != AT91C_TC_CLKSTA)
				;
			MezStruct->Start = 1;
//	  MezStruct->TickCount = Mezonin_TT[MezStruct->Mez_ID-1].Channel[MezStruct->ActiveChannel - 1].Params.MeasTime;
			MezStruct->TickCount = MeasTime1;

			xSemaphoreTake(MezStruct->xSemaphore, portMAX_DELAY);
			TC_Stop(MezStruct->TC_ptr); //остановка счетчика TC1 (2MDATA)

			Max20 = MezStruct->TC_ptr->TC_CV + 65536 * overflow[MezStruct->Mez_ID - 1]; //сохранение значения счетчика
//		  vTaskDelay(1000);
					// калибруем максимум для 100
			overflow[MezStruct->Mez_ID - 1] = 0;

			TC_Start(MezStruct->TC_ptr); //запуск счетчика TC1 (2MDATA)

			while ((MezStruct->TC_ptr->TC_SR & AT91C_TC_CLKSTA) != AT91C_TC_CLKSTA)
				;
			MezStruct->Start = 1;
//	  MezStruct->TickCount = Mezonin_TT[MezStruct->Mez_ID-1].Channel[MezStruct->ActiveChannel - 1].Params.MeasTime;
			MezStruct->TickCount = MeasTime2;

			xSemaphoreTake(MezStruct->xSemaphore, portMAX_DELAY);
			TC_Stop(MezStruct->TC_ptr); //остановка счетчика TC1 (2MDATA)

			Max100 = MezStruct->TC_ptr->TC_CV + 65536 * overflow[MezStruct->Mez_ID - 1]; //сохранение значения счетчика
			break;

		case 3:
			Mezonin_TT[MezStruct->Mez_ID - 1].Channel[Channel_Num].Coeffs.k_min = ((float) (Min100 - Min20)) / (MeasTime2 - MeasTime1);
			Mezonin_TT[MezStruct->Mez_ID - 1].Channel[Channel_Num].Coeffs.k_max = ((float) (Max100 - Max20)) / (MeasTime2 - MeasTime1);
			Mezonin_TT[MezStruct->Mez_ID - 1].Channel[Channel_Num].Coeffs.p_min = Min100
					- Mezonin_TT[MezStruct->Mez_ID - 1].Channel[Channel_Num].Coeffs.k_min * MeasTime2;
			Mezonin_TT[MezStruct->Mez_ID - 1].Channel[Channel_Num].Coeffs.p_max = Max100
					- Mezonin_TT[MezStruct->Mez_ID - 1].Channel[Channel_Num].Coeffs.k_max * MeasTime2;

			Mezonin_TT[MezStruct->Mez_ID - 1].Channel[Channel_Num].Min_Value = Mezonin_TT[MezStruct->Mez_ID - 1].Channel[Channel_Num].Coeffs.k_min
					* Mezonin_TT[MezStruct->Mez_ID - 1].Channel[Channel_Num].Params.MeasTime
					+ Mezonin_TT[MezStruct->Mez_ID - 1].Channel[Channel_Num].Coeffs.p_min;
			Mezonin_TT[MezStruct->Mez_ID - 1].Channel[Channel_Num].Max_Value = Mezonin_TT[MezStruct->Mez_ID - 1].Channel[Channel_Num].Coeffs.k_max
					* Mezonin_TT[MezStruct->Mez_ID - 1].Channel[Channel_Num].Params.MeasTime
					+ Mezonin_TT[MezStruct->Mez_ID - 1].Channel[Channel_Num].Coeffs.p_max;

			Mezonin_TT[MezStruct->Mez_ID - 1].Channel[Channel_Num].Coeffs.CRC = Crc16(
					(uint8_t *) &(Mezonin_TT[MezStruct->Mez_ID - 1].Channel[Channel_Num].Coeffs), sizeof(TT_Coeff) - sizeof(uint16_t) - 2);

			if (xTWISemaphore != NULL) {
				if (xSemaphoreTake( xTWISemaphore, ( TickType_t ) 10 ) == pdTRUE) {
					a = MEZ_memory_Write(MezStruct->Mez_ID - 1, Channel_Num + 5, 0x00, sizeof(TT_Coeff),
							(uint8_t *) &Mezonin_TT[MezStruct->Mez_ID - 1].Channel[Channel_Num].Coeffs);
					vTaskDelayUntil(&xLastWakeTime, 10);
					if (a == -6) {
						a = MEZ_memory_Write(MezStruct->Mez_ID - 1, Channel_Num + 5, 0x00, sizeof(TT_Coeff),
								(uint8_t *) &Mezonin_TT[MezStruct->Mez_ID - 1].Channel[Channel_Num].Coeffs);
						vTaskDelayUntil(&xLastWakeTime, 10);
					}
					xSemaphoreGive(xTWISemaphore);
				}
			}
			flag_calib = 0;

			break;
		case 4:
			Min20 = Min100 = Max20 = Max100 = 0;
			flag_calib = 0;
			break;

	}

//	  Channel_V = MezStruct->TC_ptr->TC_CV + 65536*overflow[MezStruct->Mez_ID - 1]; //сохранение значения счетчика
//	  VVV[MezStruct->Mez_ID-1][MezStruct->ActiveChannel - 1]=Channel_V;

//	  MEZ_ID = MezStruct->Mez_ID - 1;

//	  Real_Value.Value = Channel_V;
//	  Real_Value.Channel = MezStruct->ActiveChannel - 1;
//	  Real_Value.ID = MezStruct->Mez_ID - 1;

//	  if( xMezQueue != 0 )
//		  {
	// Send an unsigned long.  Wait for 10 ticks for space to become
	// available if necessary.
//		Switch_State=Value;
//		  if( xQueueSend( xMezQueue, &Real_Value, ( TickType_t ) 0 ))
//			  {
//	       test_count++;     // Failed to post the message, even after 10 ticks.
//			  }
//		  }
//	  }
//  flag_calib = 0;
}
//------------------------------------------------------------------------------
void Mez_TU_handler(mezonin *MezStruct)
{

	uint8_t MDATA[4];
	Mez_Value Real_TU;

	MezStruct->ActiveChannel++;
	if (MezStruct->ActiveChannel > 4) {
		MezStruct->ActiveChannel = 1;
	}

	if (Mezonin_TU[MezStruct->Mez_ID - 1].Channel[MezStruct->ActiveChannel - 1].Params.Mode
			== mode_on|| Mezonin_TU[MezStruct->Mez_ID - 1].Channel[MezStruct->ActiveChannel
			- 1].Params.Mode == mode_ok) {
		Mez_EnableChannel(MezStruct);		// переделал под свою функцию

		if (xMezTUQueue != 0) {
			if (xQueueReceive(xMezTUQueue, &Real_TU, portMAX_DELAY)) {
				Mezonin_TU[Real_TU.ID].Channel[Real_TU.Channel].Value = Real_TU.Value;
				MDATA[Real_TU.ID] = Mezonin_TU[Real_TU.ID].Channel[0].Value << 4 | Mezonin_TU[Real_TU.ID].Channel[1].Value << 5
						| Mezonin_TU[Real_TU.ID].Channel[2].Value << 6 | Mezonin_TU[Real_TU.ID].Channel[3].Value << 7;		// пишем сразу в 4 канала

				SPI_Write(AT91C_BASE_SPI0, Real_TU.ID, 0x00);
				SPI_Write(AT91C_BASE_SPI0, Real_TU.ID, 0x00);

				SPI_Write(AT91C_BASE_SPI0, Real_TU.ID, MDATA[Real_TU.ID]);

				// время выдержки

			}
		}
	}

}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// присвоение порогов
//------------------------------------------------------------------------------
uint32_t Get_TTLevels(TT_Value *TT_temp)
{
	uint32_t i, a;
	uint8_t DataRecieve[40];	// ИСПРАВИТЬ!!!!!!!
	uint16_t temp_CRC;

	for (i = 0; i < 4; i++) {

		TT_temp->Channel[i].Levels.Min_P_Level = 0;
		TT_temp->Channel[i].Levels.Max_P_Level = 0;
		TT_temp->Channel[i].Levels.Min_A_Level = 0;
		TT_temp->Channel[i].Levels.Max_A_Level = 0;
		TT_temp->Channel[i].Levels.Sense = 0;

		a = MEZ_memory_Read(TT_temp->ID, i + 9, 0x00, sizeof(TT_Level), DataRecieve); // чтение начальных параметров из EEPROM

		temp_CRC = Crc16(DataRecieve, sizeof(TT_Level) - sizeof(uint16_t) - 2);
		if (temp_CRC == ((TT_Level *) DataRecieve)->CRC) {
			TT_temp->Channel[i].Levels = *(TT_Level *) DataRecieve;
		} else {
			return 1;
		}
	}
	return 0;
}
//------------------------------------------------------------------------------
// присвоение коэффициентов
//------------------------------------------------------------------------------

uint32_t Get_TTCoeffs(TT_Value *TT_temp)
{
	uint32_t i, a/*,test*/;
	uint8_t DataRecieve[36];	// ИСПРАВИТЬ!!!!!!!
	uint16_t temp_CRC;
	for (i = 0; i < 4; i++) {
		TT_temp->Channel[i].Coeffs.k_max = 0;
		TT_temp->Channel[i].Coeffs.k_min = 0;
		TT_temp->Channel[i].Coeffs.p_max = 0;
		TT_temp->Channel[i].Coeffs.p_min = 0;
		a = MEZ_memory_Read(TT_temp->ID, i + 5, 0x00, sizeof(TT_Coeff), DataRecieve); // чтение начальных параметров из EEPROM
		temp_CRC = Crc16(DataRecieve, sizeof(TT_Coeff) - sizeof(uint16_t) - 2);
		if (temp_CRC == ((TT_Coeff *) DataRecieve)->CRC) {
			TT_temp->Channel[i].Coeffs = *(TT_Coeff *) DataRecieve;
			TT_temp->Channel[i].Min_Value = TT_temp->Channel[i].Coeffs.k_min * TT_temp->Channel[i].Params.MeasTime + TT_temp->Channel[i].Coeffs.p_min;
			TT_temp->Channel[i].Max_Value = TT_temp->Channel[i].Coeffs.k_max * TT_temp->Channel[i].Params.MeasTime + TT_temp->Channel[i].Coeffs.p_max;
		} else {
			return 1;
		}
	}
	return 0;
}
//------------------------------------------------------------------------------
// присвоение минимума и максимума всем физическим величинам
//------------------------------------------------------------------------------
uint32_t Get_TTParams(TT_Value *TT_temp)
{
	uint32_t i, a;
	uint8_t DataRecieve[44];	// ИСПРАВИТЬ!!!!!!!
	uint16_t temp_CRC;
	for (i = 0; i < 4; i++) {
		TT_temp->Channel[i].Levels.Sense = 1.0;
		a = MEZ_memory_Read(TT_temp->ID, i + 1, 0x00, sizeof(TT_Param), DataRecieve); // чтение начальных параметров из EEPROM

		temp_CRC = Crc16(DataRecieve, sizeof(TT_Param) - sizeof(uint16_t) - 2);
		if (temp_CRC == ((TT_Param *) DataRecieve)->CRC) {
			TT_temp->Channel[i].Params = *(TT_Param *) DataRecieve;
		} else {
			return 1;
		}
	}
	return 0;
}
//------------------------------------------------------------------------------
// определение режима работы каналов
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
// определение режима работы каналов TU
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
void Set_TCDefaultParams(uint8_t MezNum)
{
	TC_Param * Params;
	uint8_t i;
	for (i = 0; i < 4; i++) {
		Params = &Mezonin_TC[MezNum].Channel[i].Params;
		Params->Mode = 8;
		Params->CRC = Crc16((uint8_t *) Params, offsetof(TC_Param, CRC)); //TODO fix TCParams size possibly works
		if (xTWISemaphore != NULL) {
			if (xSemaphoreTake( xTWISemaphore, ( TickType_t ) 10 ) == pdTRUE) {
				MEZ_memory_Write(MezNum, i + 1, 0x00, sizeof(TC_Param), (uint8_t *) Params);
				vTaskDelay(100);
				xSemaphoreGive(xTWISemaphore);

			}
		}

	}
}
//------------------------------------------------------------------------------

void WriteTTCoeffs(uint8_t MezNum, int ChannelNumber, TT_Coeff* Coeffs)
{
	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, ( TickType_t ) 10) == pdTRUE) {
			int a;
			a = MEZ_memory_Write(MezNum, ChannelNumber + PageCoeff, 0x00, sizeof(TT_Coeff), (uint8_t *) Coeffs);
			vTaskDelay(100);
			/*				if (a == -6) {
			 a = MEZ_memory_Write(0, Channel_Num + PageCoeff, 0x00, sizeof(TT_Coeff),
			 (unsigned char *) &Mezonin_TT[0].Channel[Channel_Num].Coeffs);
			 vTaskDelayUntil(&xLastWakeTime, 10);
			 }*/
			xSemaphoreGive(xTWISemaphore);
		}
	}
}

void WriteTTLevels(uint8_t MezNum, int ChannelNumber, TT_Level* Levels)
{
	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, ( TickType_t ) 10) == pdTRUE) {
			int a;
			a = MEZ_memory_Write(MezNum, ChannelNumber + PageLevel, 0x00, sizeof(TT_Level), (uint8_t *) Levels);
			vTaskDelay(100);
			xSemaphoreGive(xTWISemaphore);
		}
	}
}

void WriteTTParams(uint8_t MezNum, int ChannelNumber, TT_Param* Params)
{
	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, ( TickType_t ) 10) == pdTRUE) {
			int a;
			a = MEZ_memory_Write(MezNum, ChannelNumber + PageParam, 0x00, sizeof(TT_Param), (uint8_t *) Params);
			vTaskDelay(100);
			xSemaphoreGive(xTWISemaphore);
		}
	}
}


void Set_TTDefaultParams(uint8_t MezNum)
{
	unsigned char * DataToWrite;
	int i;
	TT_Coeff * Coeffs;
	TT_Level * Levels;
	TT_Param * Params;
	for (i = 0; i < 4; i++) {
		Coeffs = &Mezonin_TT[MezNum].Channel[i].Coeffs;
		Levels = &Mezonin_TT[MezNum].Channel[i].Levels;
		Params = &Mezonin_TT[MezNum].Channel[i].Params;

		Coeffs->k_max = 1500;
		Coeffs->k_min = 233;
		Coeffs->p_max = -1500;
		Coeffs->p_min = -233;

		Coeffs->CRC = Crc16((uint8_t *) &(Coeffs), offsetof(TT_Coeff, CRC)); //TODO fix TT_Coeff size  possibly works

		Levels->Min_P_Level = 20;
		Levels->Max_P_Level = 40;
		Levels->Min_A_Level = 15;
		Levels->Max_A_Level = 45;
		Levels->Sense = 0.2;

		Levels->CRC = Crc16((uint8_t *) &(Levels), offsetof(TT_Level, CRC)); //TODO fix TT_Level size  possibly works

	    Params->MeasTime = 20;
	    Params->Mode = 0;
	    Params->MinD = 0;
	    Params->MaxD = 20;
	    Params->MinF = 0;
	    Params->MaxF = 100;

		WriteTTCoeffs(MezNum, i, Coeffs);
		WriteTTLevels(MezNum, i, Levels);
		WriteTTParams(MezNum, i, Params);
	}
}
