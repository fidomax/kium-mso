/*
 * TT.c
 *
 *  Created on: 29 сент. 2014 г.
 *      Author: maximus
 */
#include "mezonin.h"
#include "global.h"
#include "constant.h"
#include "../pio/pio_config.h"
#include "../pwmc/pwmc.h"
#include "../tc/tc.h"
#include "../aic/aic.h"
#include <math.h>
#include "MSO_functions/MSO_functions.h"
#include "FreeRTOS.h"
#include "task.h"
uint32_t overflow[4];		// массив значений перполнений счетчика TT
uint32_t Min20[4][4], Min100[4], Max20[4], Max100[4];
//uint32_t VVV[4][4];
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
float Mez_TT_Frequency(uint32_t measured_value, uint32_t ChannelNumber, uint32_t MEZ_ID) // нахождение физической величины
{
// TT_Value mez_freq;
// mez_freq[0].Channel[0].Min_Value = 4701;
	float MIN_measured_value = 0, MAX_measured_value = 0, MAX_value = 20; // Y0, Ym, Xm
	float corrected_value, real_value = 0, step; // Xi

	MIN_measured_value = Mezonin_TT[MEZ_ID].Channel[ChannelNumber].Min_Value;
	MAX_measured_value = Mezonin_TT[MEZ_ID].Channel[ChannelNumber].Max_Value;

	corrected_value = ((measured_value - MIN_measured_value) / (MAX_measured_value - MIN_measured_value)) * MAX_value;

//	if (xTWISemaphore != NULL) {
//		if (xSemaphoreTake( xTWISemaphore, ( TickType_t ) 10 ) == pdTRUE) {
	step = (Mezonin_TT[MEZ_ID].Channel[ChannelNumber].Params.MaxF - Mezonin_TT[MEZ_ID].Channel[ChannelNumber].Params.MinF)
			/ (Mezonin_TT[MEZ_ID].Channel[ChannelNumber].Params.MaxD - Mezonin_TT[MEZ_ID].Channel[ChannelNumber].Params.MinD);
	real_value = Mezonin_TT[MEZ_ID].Channel[ChannelNumber].Params.MinF + (corrected_value - Mezonin_TT[MEZ_ID].Channel[ChannelNumber].Params.MinD) * step;

	//  						vTaskDelay(100);
//			xSemaphoreGive(xTWISemaphore);
//		}
//	}

	return real_value;
// return corrected_value;
}
//------------------------------------------------------------------------------
void TTValueHandler(Mez_Value *Mez_V)
{
	uint32_t ChannelNumber;
	uint32_t ID;
	uint32_t Count;
	ChannelNumber = Mez_V->ID * 4 + Mez_V->Channel;
	ID = MAKE_CAN_ID(priority_N, identifier_TT, MSO_Address, ChannelNumber, ParamFV);
	Count = Mez_V->ui32Value; // сколько намотал счетчик
	TT_Channel * tt_channel = &Mezonin_TT[Mez_V->ID].Channel[Mez_V->Channel];
	tt_channel->Value = Mez_TT_Frequency(Count, Mez_V->Channel, Mez_V->ID);

	// анализировать состояние
	if ((tt_channel->Value > tt_channel->Levels.Max_W_Level) || (tt_channel->Value < tt_channel->Levels.Min_W_Level)) {
		if ((tt_channel->Value > tt_channel->Levels.Max_A_Level) || (tt_channel->Value < tt_channel->Levels.Min_A_Level)) {
			if (tt_channel->State != STATE_ALARM) {
				tt_channel->State = STATE_ALARM; // аварийный порог
				tt_channel->OldValue = tt_channel->Value;
				SendCanMessage(ID, *((uint32_t *) &tt_channel->Value), tt_channel->State);
			}
		} else {
			if (tt_channel->State != STATE_WARNING) {
				tt_channel->State = STATE_WARNING; // предупредительный порог
				tt_channel->OldValue = tt_channel->Value;
				SendCanMessage(ID, *((uint32_t *) &tt_channel->Value), tt_channel->State);
			}
		}
	} else {
		if (tt_channel->State != STATE_OK) {
			tt_channel->State = STATE_OK;
			tt_channel->OldValue = tt_channel->Value;
			SendCanMessage(ID, *((uint32_t *) &tt_channel->Value), tt_channel->State);
		}
	}
	if (tt_channel->Params.Mode == 0x04) {
		if (tt_channel->State != STATE_MASK) {
			tt_channel->State = STATE_MASK; // выключен
			SendCanMessage(ID, *((uint32_t *) &tt_channel->Value), tt_channel->State);
		}
	}

	if (fabs(tt_channel->Value - tt_channel->OldValue) > tt_channel->Levels.Sense)

	{
		tt_channel->OldValue = tt_channel->Value;
		SendCanMessage(ID, *((uint32_t *) &tt_channel->Value), tt_channel->State);
	}
}
//------------------------------------------------------------------------------
void MeasureTT(mezonin *MezStruct)
{
	uint32_t Channel_V;
	//uint32_t test_count = 0;

	Mez_Value Real_Value;
	Mez_EnableChannel(MezStruct);	// переделал под свою функцию
	overflow[MezStruct->Mez_ID - 1] = 0;

	TC_Start(MezStruct->TC_ptr); //запуск счетчика TC1 (2MDATA)

	while ((MezStruct->TC_ptr->TC_SR & AT91C_TC_CLKSTA) != AT91C_TC_CLKSTA)
		;
	MezStruct->Start = 1;
	MezStruct->TickCount = Mezonin_TT[MezStruct->Mez_ID - 1].Channel[MezStruct->ActiveChannel - 1].Params.MeasTime + 1;

	SaveTTConfig(MezStruct->Mez_ID - 1,MezStruct->ActiveChannel - 1);

	xSemaphoreTake(MezStruct->xSemaphore, portMAX_DELAY);
	TC_Stop(MezStruct->TC_ptr); //остановка счетчика TC1 (2MDATA)

	Channel_V = MezStruct->TC_ptr->TC_CV + 65536 * overflow[MezStruct->Mez_ID - 1]; //сохранение значения счетчика
//			VVV[MezStruct->Mez_ID - 1][MezStruct->ActiveChannel - 1] = Channel_V;

	Real_Value.ui32Value = Channel_V;
	Real_Value.Channel = MezStruct->ActiveChannel - 1;
	Real_Value.ID = MezStruct->Mez_ID - 1;

	if (xMezQueue != 0) {

		if (xQueueSend(xMezQueue, &Real_Value, (TickType_t ) 0)) {
//			test_count++; // Failed to post the message, even after 10 ticks.
		}
	}
}
//------------------------------------------------------------------------------
void SetTTTime(TT_Channel *Channel)
{
	Channel->Min_Value = Channel->Coeffs.k_min * Channel->Params.MeasTime + Channel->Coeffs.p_min;
	Channel->Max_Value = Channel->Coeffs.k_max * Channel->Params.MeasTime + Channel->Coeffs.p_max;
}
//------------------------------------------------------------------------------
void CalibTTMin(mezonin *MezStruct)
{
	TT_Channel * Channel;
	Mez_EnableChannel(MezStruct);		// переделал под свою функцию
	Channel = &(Mezonin_TT[MezStruct->Mez_ID - 1].Channel[MezStruct->ActiveChannel - 1]);
	// калибруем минимум для 20
	overflow[MezStruct->Mez_ID - 1] = 0;
	TC_Start(MezStruct->TC_ptr); //запуск счетчика TC1 (2MDATA)
	while ((MezStruct->TC_ptr->TC_SR & AT91C_TC_CLKSTA) != AT91C_TC_CLKSTA)
		;
	MezStruct->Start = 1;
	MezStruct->TickCount = 20 + 1;
	xSemaphoreTake(MezStruct->xSemaphore, portMAX_DELAY);
	TC_Stop(MezStruct->TC_ptr); //остановка счетчика TC1 (2MDATA)
	Channel->CalibMin20 = MezStruct->TC_ptr->TC_CV + 65536 * overflow[MezStruct->Mez_ID - 1]; //сохранение значения счетчика

	// калибруем минимум для 100
	overflow[MezStruct->Mez_ID - 1] = 0;
	TC_Start(MezStruct->TC_ptr); //запуск счетчика TC1 (2MDATA)
	while ((MezStruct->TC_ptr->TC_SR & AT91C_TC_CLKSTA) != AT91C_TC_CLKSTA)
		;
	MezStruct->Start = 1;
	MezStruct->TickCount = 100 + 1;
	xSemaphoreTake(MezStruct->xSemaphore, portMAX_DELAY);
	TC_Stop(MezStruct->TC_ptr); //остановка счетчика TC1 (2MDATA)
	Channel->CalibMin100 = MezStruct->TC_ptr->TC_CV + 65536 * overflow[MezStruct->Mez_ID - 1]; //сохранение значения счетчика

}
//------------------------------------------------------------------------------
void CalibTTMax(mezonin * MezStruct)
{
	TT_Channel * Channel;
	Mez_EnableChannel(MezStruct);		// переделал под свою функцию
	Channel = &(Mezonin_TT[MezStruct->Mez_ID - 1].Channel[MezStruct->ActiveChannel - 1]);
	// калибруем максимум для 20
	overflow[MezStruct->Mez_ID - 1] = 0;
	TC_Start(MezStruct->TC_ptr); //запуск счетчика TC1 (2MDATA)
	while ((MezStruct->TC_ptr->TC_SR & AT91C_TC_CLKSTA) != AT91C_TC_CLKSTA)
		;
	MezStruct->Start = 1;
	MezStruct->TickCount = 20 + 1;
	xSemaphoreTake(MezStruct->xSemaphore, portMAX_DELAY);
	TC_Stop(MezStruct->TC_ptr); //остановка счетчика TC1 (2MDATA)
	Channel->CalibMax20 = MezStruct->TC_ptr->TC_CV + 65536 * overflow[MezStruct->Mez_ID - 1]; //сохранение значения счетчика

	// калибруем максимум для 100
	overflow[MezStruct->Mez_ID - 1] = 0;
	TC_Start(MezStruct->TC_ptr); //запуск счетчика TC1 (2MDATA)
	while ((MezStruct->TC_ptr->TC_SR & AT91C_TC_CLKSTA) != AT91C_TC_CLKSTA)
		;
	MezStruct->Start = 1;
	MezStruct->TickCount = 100 + 1;
	xSemaphoreTake(MezStruct->xSemaphore, portMAX_DELAY);
	TC_Stop(MezStruct->TC_ptr); //остановка счетчика TC1 (2MDATA)
	Channel->CalibMax100 = MezStruct->TC_ptr->TC_CV + 65536 * overflow[MezStruct->Mez_ID - 1]; //сохранение значения счетчика
}
//------------------------------------------------------------------------------
void CalibTTSave(mezonin *MezStruct)
{
	TT_Channel * Channel;
	Channel = &(Mezonin_TT[MezStruct->Mez_ID - 1].Channel[MezStruct->ActiveChannel - 1]);

	Channel->Coeffs.k_min = ((float) (Channel->CalibMin100 - Channel->CalibMin20)) / (100 - 20);
	Channel->Coeffs.k_max = ((float) (Channel->CalibMax100 - Channel->CalibMax20)) / (100 - 20);
	Channel->Coeffs.p_min = Channel->CalibMin100 - Channel->Coeffs.k_min * 100;
	Channel->Coeffs.p_max = Channel->CalibMax100 - Channel->Coeffs.k_max * 100;

	SetTTTime(Channel);

	Channel->Coeffs.CRC = Crc16((uint8_t *) &(Channel->Coeffs), sizeof(TT_Coeff) - sizeof(uint16_t) - 2);

	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, portMAX_DELAY ) == pdTRUE) {
			int32_t a = MEZ_memory_Write(MezStruct->Mez_ID - 1, MezStruct->ActiveChannel - 1 + PageCoeff, 0x00, sizeof(TT_Coeff),
					(uint8_t *) &(Channel->Coeffs));
			vTaskDelay(10);
			if (a == -6) {
				a = MEZ_memory_Write(MezStruct->Mez_ID - 1, MezStruct->ActiveChannel - 1 + PageCoeff, 0x00, sizeof(TT_Coeff), (uint8_t *) &(Channel->Coeffs));
				vTaskDelay(10);
			}
			xSemaphoreGive(xTWISemaphore);
		}
	}
}
//------------------------------------------------------------------------------
void Mez_TT_handler(mezonin *MezStruct/*, TT_Value *Mez_TT_temp*/)
{

	MezStruct->ActiveChannel++;
	if (MezStruct->ActiveChannel == 5) {
		MezStruct->ActiveChannel = 1;
	}
	TT_Channel * tt_channel = &Mezonin_TT[MezStruct->Mez_ID - 1].Channel[MezStruct->ActiveChannel - 1];
	switch (tt_channel->Params.Mode) {
		case TT_mode_ok:
			MeasureTT(MezStruct);
			break;
		case TT_mode_calib0:
			CalibTTMin(MezStruct);
			tt_channel->Params.Mode = TT_mode_idle;
			break;
		case TT_mode_calib20:
			CalibTTMax(MezStruct);
			tt_channel->Params.Mode = TT_mode_idle;
			break;
		case TT_mode_calibSave:
			CalibTTSave(MezStruct);
			tt_channel->Params.Mode = TT_mode_ok;
			break;
		case TT_mode_mask:
			if (tt_channel->State != STATE_MASK) {
				tt_channel->State = STATE_MASK; // выключен
				SendCanMessage(MAKE_CAN_ID(priority_N, identifier_TT, MSO_Address, (MezStruct->Mez_ID - 1)*4 + MezStruct->ActiveChannel -1 , ParamFV), *((uint32_t *) &tt_channel->Value), tt_channel->State);
			}
			break;
	}
}
//------------------------------------------------------------------------------
void TTTickHandler(void)
{
	static portBASE_TYPE xTaskWoken = pdFALSE;
	int32_t i;
	uint32_t PWM = 0;
	for (i = 0; i < 4; i++) {
		if (mezonin_my[i].Start) {
			mezonin_my[i].Start = 0;
			PWM |= mezonin_my[i].PWM_ID; //подача синхросигнала 2FIN
		}

	}
	if (PWM) {
		AT91C_BASE_PWMC->PWMC_ENA = PWM;
	}
	PWM = 0;
	for (i = 0; i < 4; i++) {
		if (mezonin_my[i].TickCount) {
			mezonin_my[i].TickCount--;
			if ((mezonin_my[i].TickCount) == 0) {
				PWM |= mezonin_my[i].PWM_ID;
			}

		}

	}
	if (PWM) {
		AT91C_BASE_PWMC->PWMC_DIS = PWM;
	}
	for (i = 0; i < 4; i++) {
		if (mezonin_my[i].PWM_ID & PWM)
			xSemaphoreGiveFromISR(mezonin_my[i].xSemaphore, &xTaskWoken);
	}
}
//------------------------------------------------------------------------------
void Mez_TT_Calib(mezonin *MezStruct, uint32_t Channel_Num, uint32_t flag/*, TT_Value *Mez_TT_temp*/)
{
	MezStruct->ActiveChannel = Channel_Num + 1;
	switch (flag) {
		case 1:
			Mezonin_TT[MezStruct->Mez_ID - 1].Channel[MezStruct->ActiveChannel - 1].Params.Mode = TT_mode_calib0;
			break;
		case 2:
			Mezonin_TT[MezStruct->Mez_ID - 1].Channel[MezStruct->ActiveChannel - 1].Params.Mode = TT_mode_calib20;
			break;

		case 3:
			Mezonin_TT[MezStruct->Mez_ID - 1].Channel[MezStruct->ActiveChannel - 1].Params.Mode = TT_mode_calibSave;
			break;
		case 4:
			Mezonin_TT[MezStruct->Mez_ID - 1].Channel[MezStruct->ActiveChannel - 1].CalibMax100 = 0;
			Mezonin_TT[MezStruct->Mez_ID - 1].Channel[MezStruct->ActiveChannel - 1].CalibMax20 = 0;
			Mezonin_TT[MezStruct->Mez_ID - 1].Channel[MezStruct->ActiveChannel - 1].CalibMin100 = 0;
			Mezonin_TT[MezStruct->Mez_ID - 1].Channel[MezStruct->ActiveChannel - 1].CalibMax20 = 0;
			Mezonin_TT[MezStruct->Mez_ID - 1].Channel[MezStruct->ActiveChannel - 1].Params.Mode = TT_mode_ok;
			break;

	}

}

//------------------------------------------------------------------------------
// присвоение порогов
//------------------------------------------------------------------------------
uint32_t Get_TTLevels(TT_Value *TT_temp)
{
	uint32_t i, a;
	uint8_t DataRecieve[ sizeof(TT_Level)];
	uint16_t temp_CRC;

	for (i = 0; i < 4; i++) {

		TT_temp->Channel[i].Levels.Min_W_Level = 0;
		TT_temp->Channel[i].Levels.Max_W_Level = 0;
		TT_temp->Channel[i].Levels.Min_A_Level = 0;
		TT_temp->Channel[i].Levels.Max_A_Level = 0;
		TT_temp->Channel[i].Levels.Sense = 0;

		a = MEZ_memory_Read(TT_temp->ID, i + 9, 0x00, sizeof(TT_Level), DataRecieve); // чтение начальных параметров из EEPROM

		temp_CRC = Crc16(DataRecieve, offsetof(TT_Level, CRC)); //TODO fix TT_Level size possibly works
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
	uint8_t DataRecieve[sizeof(TT_Coeff)];
	uint16_t temp_CRC;
	for (i = 0; i < 4; i++) {
		TT_temp->Channel[i].Coeffs.k_max = 0;
		TT_temp->Channel[i].Coeffs.k_min = 0;
		TT_temp->Channel[i].Coeffs.p_max = 0;
		TT_temp->Channel[i].Coeffs.p_min = 0;
		a = MEZ_memory_Read(TT_temp->ID, i + 5, 0x00, sizeof(TT_Coeff), DataRecieve); // чтение начальных параметров из EEPROM
		temp_CRC = Crc16(DataRecieve, offsetof(TT_Coeff, CRC)); //TODO fix TT_Coeff size possibly works
		if (temp_CRC == ((TT_Coeff *) DataRecieve)->CRC) {
			TT_temp->Channel[i].Coeffs = *(TT_Coeff *) DataRecieve;
			SetTTTime(&TT_temp->Channel[i]);
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
	uint8_t DataRecieve[sizeof(TT_Param)];
	uint16_t temp_CRC;
	for (i = 0; i < 4; i++) {
		TT_temp->Channel[i].Levels.Sense = 1.0;
		a = MEZ_memory_Read(TT_temp->ID, i + 1, 0x00, sizeof(TT_Param), DataRecieve); // чтение начальных параметров из EEPROM

		temp_CRC = Crc16(DataRecieve, offsetof(TT_Param, CRC)); //TODO fix TT_Param size possibly works
		if (temp_CRC == ((TT_Param *) DataRecieve)->CRC) {
			TT_temp->Channel[i].Params = *(TT_Param *) DataRecieve;
		} else {
			return 1;
		}
	}
	return 0;
}
//------------------------------------------------------------------------------
void WriteTTCoeffs(uint8_t MezNum, int ChannelNumber, TT_Coeff* Coeffs)
{
	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, portMAX_DELAY) == pdTRUE) {
			int a;
			a = MEZ_memory_Write(MezNum, ChannelNumber + PageCoeff, 0x00, sizeof(TT_Coeff), (uint8_t *) Coeffs);
			vTaskDelay(10);
			/*				if (a == -6) {
			 a = MEZ_memory_Write(0, Channel_Num + PageCoeff, 0x00, sizeof(TT_Coeff),
			 (unsigned char *) &Mezonin_TT[0].Channel[Channel_Num].Coeffs);
			 vTaskDelayUntil(&xLastWakeTime, 10);
			 }*/
			xSemaphoreGive(xTWISemaphore);
		}
	}
}
//------------------------------------------------------------------------------
void WriteTTLevels(uint8_t MezNum, int ChannelNumber, TT_Level* Levels)
{
	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, portMAX_DELAY) == pdTRUE) {
			int a;
			a = MEZ_memory_Write(MezNum, ChannelNumber + PageLevel, 0x00, sizeof(TT_Level), (uint8_t *) Levels);
			vTaskDelay(10);
			xSemaphoreGive(xTWISemaphore);
		}
	}
}
//------------------------------------------------------------------------------
void WriteTTParams(uint8_t MezNum, int ChannelNumber, TT_Param* Params)
{
	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, portMAX_DELAY) == pdTRUE) {
			int a;
			a = MEZ_memory_Write(MezNum, ChannelNumber + PageParam, 0x00, sizeof(TT_Param), (uint8_t *) Params);
			vTaskDelay(10);
			xSemaphoreGive(xTWISemaphore);
		}
	}
}
//------------------------------------------------------------------------------
void SaveTTConfig(uint8_t MezNum, int ChannelNumber)
{
//	TT_Coeff * Coeffs;
	TT_Level * Levels;
	TT_Param * Params;
	uint16_t CRC;
//	Coeffs = &Mezonin_TT[MezNum].Channel[ChannelNumber].Coeffs;
	Levels = &Mezonin_TT[MezNum].Channel[ChannelNumber].Levels;
	Params = &Mezonin_TT[MezNum].Channel[ChannelNumber].Params;
/*	CRC = Crc16((uint8_t *) (Coeffs), offsetof(TT_Coeff, CRC));
	if (Coeffs->CRC != CRC){
		Coeffs->CRC = CRC;
		WriteTTCoeffs(MezNum, ChannelNumber, Coeffs);
	}*/
	CRC = Crc16((uint8_t *) (Levels), offsetof(TT_Level, CRC));
	if (Levels->CRC != CRC){
		Levels->CRC = CRC;
		WriteTTLevels(MezNum, ChannelNumber, Levels);
	}
	CRC = Crc16((uint8_t *) (Params), offsetof(TT_Param, CRC));
	if (Params->CRC != CRC){
		Params->CRC = CRC;
		WriteTTParams(MezNum, ChannelNumber, Params);
	}
}
//------------------------------------------------------------------------------
void Set_TTDefaultParams(uint8_t MezNum)
{
	int i;
	TT_Coeff * Coeffs;
	TT_Level * Levels;
	TT_Param * Params;
	for (i = 0; i < 4; i++) {
		Coeffs = &Mezonin_TT[MezNum].Channel[i].Coeffs;
		Levels = &Mezonin_TT[MezNum].Channel[i].Levels;
		Params = &Mezonin_TT[MezNum].Channel[i].Params;

		Coeffs->k_max = 1511;
		Coeffs->k_min = 249;
		Coeffs->p_max = 5.75;
		Coeffs->p_min = 0.25;

		Coeffs->CRC = Crc16((uint8_t *) (Coeffs), offsetof(TT_Coeff, CRC));

		Levels->Min_W_Level = 20;
		Levels->Max_W_Level = 40;
		Levels->Min_A_Level = 15;
		Levels->Max_A_Level = 45;
		Levels->Sense = 1;

		Levels->CRC = Crc16((uint8_t *) (Levels), offsetof(TT_Level, CRC));

		Params->MeasTime = 20;
		Params->Mode = 0;
		Params->MinD = 0;
		Params->MaxD = 20;
		Params->MinF = 0;
		Params->MaxF = 100;
		Params->ExtK = 0;

		Params->CRC = Crc16((uint8_t *) (Params), offsetof(TT_Param, CRC));

		WriteTTCoeffs(MezNum, i, Coeffs);
		WriteTTLevels(MezNum, i, Levels);
		WriteTTParams(MezNum, i, Params);
	}
}
//------------------------------------------------------------------------------
