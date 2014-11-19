/*
 * TC.h
 *
 *  Created on: 18 нояб. 2014 г.
 *      Author: maximus
 */

#ifndef PERIPHERALS_MEZONIN_TC_H_
#define PERIPHERALS_MEZONIN_TC_H_
//==========================парамеры канала, хранящиеся в EEPROM для ТС=======================
typedef struct _TC_Param
{
	uint32_t Mode;	// режим работы
	uint16_t CRC;	// контрольная сумма
} TC_Param;
//---------------структура канала ТC (тип mezonin)----------------------
typedef struct _TC_Channel
{
	uint8_t Value; 		// значение канала (On или Off)
	uint8_t State;  	// состояние канала (Break или не Break)
	TC_Param Params;		// параметры из EEPROM
//  uint8_t			Mode;		// режим работы канала (On/Off)
	// возможно будет дополняться
} TC_Channel;
//---------------определение типа для ТC (тип mezonin)------------------
typedef struct _TC_Value
{
	TC_Channel Channel[4]; // номер канала
	int32_t ID; // номер мезонина
} TC_Value;

void Mez_TC_init(mezonin *MezStruct);
void TCValueHandler (Mez_Value *Mez_V);
void Mez_TC_handler(mezonin *MezStruct);
uint32_t Get_TCParams(TC_Value *TC_ParamData);
void Set_TCDefaultParams(uint8_t MezNum);



#endif /* PERIPHERALS_MEZONIN_TC_H_ */
