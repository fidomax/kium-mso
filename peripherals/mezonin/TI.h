/*
 * TI.h
 *
 *  Created on: 25 сент. 2014 г.
 *      Author: maximus
 */

#ifndef TI_H_
#define TI_H_
#include "mezcommon.h"
//==========================парамеры канала, хранящиеся в EEPROM для ТI=======================
typedef struct _TI_Param
{
	uint32_t Mode;	// режим работы
	float Sense;  // Чувствительность
	float CoeFf;	// Коэффициент
	uint16_t CRC;	// контрольная сумма
} TI_Param;
//---------------структура канала ТI (тип mezonin)----------------------
typedef struct _TI_Channel
{
	uint32_t Value; 		// счетчик ФВ
	uint8_t State;  	// состояние
	uint32_t CountTI;	// счетчик импульсов
	TI_Param Params;		// параметры из EEPROM
//  uint8_t			Mode;		// режим работы канала (On/Off)
	// возможно будет дополняться
} TI_Channel;
//---------------определение типа для ТI (тип mezonin)------------------
typedef struct _TI_Value
{
	TI_Channel Channel[4]; // номер канала
	uint8_t PerTime;
	int32_t ID; // номер мезонина
} TI_Value;

void Mez_TI_init(mezonin *MezStruct);
void TIValueHandler (Mez_Value *Mez_V);
void Mez_TI_handler(mezonin *MezStruct);
extern TI_Value Mezonin_TI[4];
#endif /* TI_H_ */
