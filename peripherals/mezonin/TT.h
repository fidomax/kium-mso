/*
 * TT.h
 *
 *  Created on: 29 сент. 2014 г.
 *      Author: maximus
 */

#ifndef TT_H_
#define TT_H_
#include "mezcommon.h"
//==========================коэффициенты для вычисления ФВ =======================
typedef struct _TT_Coeff
{
	float k_min;		// коэффициент для вычисления минимального значения
	float p_min;		// смещение для вычисления минимального значения
	float k_max;		// коэффициент для вычисления максимального значения
	float p_max;		// смещение для вычисления максимального значения
	uint16_t CRC;		// контрольная сумма
} TT_Coeff;

//==========================пороги канала, хранящиеся в EEPROM для ТТ=======================
typedef struct _TT_Level
{
	float Min_W_Level;	// минимальный предупредительный порог
	float Max_W_Level;	// максимальный предупредительный порог
	float Min_A_Level;	// минимальный аварийный порог
	float Max_A_Level;	// максимальный аварийный порог
	float Sense;			// чувствительность
	uint16_t CRC;			// контрольная сумма
} TT_Level;

//==========================парамеры канала, хранящиеся в EEPROM для ТТ=======================
typedef struct _TT_Param
{
	uint32_t MeasTime;	// время измерения
	uint32_t Mode;		// режим работы канала
	/*	float 					k_min;		// коэффициент для вычисления минимального значения
	 float 					p_min;		// смещение для вычисления минимального значения
	 float 					k_max;		// коэффициент для вычисления максимального значения
	 float 					p_max;		// смещение для вычисления максимального значения
	 float					Sense;		// чувствительность
	 */
	float MinD;		// значение минимума (например 0..20 это 0, в 4..20 это 4)
	float MaxD;	// значение максимума (например 0..20 это 20, в 4..20 это 20)
	float MinF;		// значение минимума физической величины
	float MaxF;		// значение максимума физической величины
	uint16_t CRC;		// контрольная сумма
} TT_Param;
//---------------структура канала ТТ (тип mezonin)----------------------
typedef struct _TT_Channel
{
	float Value; 		// значение физической величины
	float OldValue;	// старое значение физической величины
	uint8_t State;		// состояние ТТ
	uint32_t CalibMin20;
	uint32_t CalibMin100;
	uint32_t CalibMax20;
	uint32_t CalibMax100;
	TT_Param Params;		// параметры из EEPROM
	TT_Coeff Coeffs;		// коэффициенты из EEPROM
	TT_Level Levels;		// пороги из EEPROM
	uint32_t Min_Value;  // значение минимума для расчета
	uint32_t Max_Value;	// значение максимума для расчета
//  uint32_t			CRC;		// контрольная сумма
// возможно будет дополняться
} TT_Channel;
//---------------определение типа для ТТ (тип mezonin)------------------
typedef struct _TT_Value
{
	TT_Channel Channel[4]; // номер канала
	uint8_t PerTime;
	int32_t ID; // номер мезонина
} TT_Value;
//----------------------------------------------------------------------

void Mez_TT_init(mezonin *MezStruct);
void TTValueHandler (Mez_Value *Mez_V);
uint32_t Get_TTParams(TT_Value *TT_temp);
uint32_t Get_TTCoeffs(TT_Value *TT_temp);
uint32_t Get_TTLevels(TT_Value *TT_temp);
void Set_TTDefaultParams(uint8_t MezNum);

void WriteTTCoeffs(uint8_t MezNum, int ChannelNumber, TT_Coeff* Coeffs);
void WriteTTLevels(uint8_t MezNum, int ChannelNumber, TT_Level* Levels);
void WriteTTParams(uint8_t MezNum, int ChannelNumber, TT_Param* Params);
void Mez_TT_handler(mezonin *MezStruct/*, TT_Value *Mez_TT_temp*/);
void TTTickHandler(void);

void Mez_TT_Calib(mezonin *MezStruct, uint32_t Channel_Num, uint32_t flag);
void Mez_1_TT_ISR_TC(void);
void Mez_2_TT_ISR_TC(void);
void Mez_3_TT_ISR_TC(void);
void Mez_4_TT_ISR_TC(void);

float Mez_TT_Frequency(uint32_t measured_value, uint32_t ChannelNumber, uint32_t MEZ_ID);

#endif /* TT_H_ */
