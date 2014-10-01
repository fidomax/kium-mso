#ifndef MEZONIN_H
#define MEZONIN_H
#include "boards/MSO_board.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "mezcommon.h"
#include "TU.h"
#include "TI.h"
#include "TT.h"
//=====================количество мезонинов======================
#define	  Mez_Count		 3 // Мезонины считаются с 0-го по 3-ий
//=====================номера мезонинов======================
#define   Mez_1          1
#define   Mez_2          2
#define   Mez_3          3
#define   Mez_4          4
//==========================типы мезонинов=============================
#define   Mez_NOT        0x00
#define   Mez_TC         0x01
#define   Mez_TU         0x02
#define   Mez_TT         0x03
#define   Mez_TR         0x04
#define   Mez_TI         0x05
//=====================================================================
#define   Mez_MemAddr       0x50 //адрес памяти
#define   Mez_Type_Address    0x00 
//----------------------------------------------------------------------
#define PeriphA   1
#define PeriphB   2
//----------------------------------------------------------------------
//         For Mode (режимы работы)
//------------------------------------------------------------------------------
#define mode_ok		((uint32_t)	0x00)	// режим работы "Норма"
#define mode_on		((uint32_t)	0x08)	// ежим работы "Включено"
#define mode_off	((uint32_t)	0x04)	// режим работы "Выключено"
#define mode_calib	((uint32_t)	0x06)	// режим работы калибровка

#define TT_mode_ok				((uint32_t)	0x00)	// режим работы "Норма"
#define TT_mode_calib0			((uint32_t)	0x10)	// режим работы "Калибровка 0"
#define TT_mode_calib20			((uint32_t)	0x20)	// режим работы "Калибровка 20"
#define TT_mode_calibCancel		((uint32_t)	0x30)	// режим работы "Отмена калибровки"
#define TT_mode_calibSave		((uint32_t)	0x40)	// режим работы "Сохранение калибровки"
#define TT_mode_idle			((uint32_t)	0xFF)	// режим работы "Откл"
//------------------------------------------------------------------------------
//---------------определение типа для LinePIO------------------

//------------------------------------------------------------------------------
//         For Page Mezonin
//------------------------------------------------------------------------------
#define PageType	((uint32_t) 0)
#define PageParam	((uint32_t) 1)
#define PageCoeff	((uint32_t) 5)
#define PageLevel	((uint32_t) 9)


#define TC_OFF      0
#define TC_ON       1
#define TC_OK		1
#define TC_BRK		0
#define Channel_OFF	4




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


//---------------структура канала ТP (тип mezonin)----------------------

typedef struct _TR_Channel
{
	float flDAC; 			//ФВ
} TR_Channel;
//---------------определение типа для ТP (тип mezonin)------------------
typedef struct _TR_Value
{
	TR_Channel Channel; // номер канала
	int32_t ID; // номер мезонина
} TR_Value;
//---------------структура канала ТI (тип mezonin)----------------------
//uint8_t Mez_Recognition_old (uint32_t MezMemoryLine, int8_t address);
//==========================init functions===========================

int32_t Mez_SetType(uint8_t MezNum, uint8_t MezType);

uint8_t Mez_Recognition(uint8_t MezNum);

void Mez_Select(uint32_t MezMemoryLine);

void Mez_PreInit(mezonin *Mez1, mezonin *Mez2, mezonin *Mez3, mezonin *Mez4);

void Mez_init(uint32_t Mezonin_Type, mezonin *MezStruct);

void Mez_TC_init(mezonin *MezStruct);



void Mez_TR_init(mezonin *MezStruct);

void Mez_NOT_init(void);

void TCValueHandler (Mez_Value *Mez_V);




uint32_t Get_TCParams(TC_Value *TC_temp);
uint32_t Get_TUParams(TU_Value *TU_temp);

void Set_TCDefaultParams(uint8_t MezNum);

void SendCanMessage(uint32_t id, uint32_t data_l, uint32_t data_h);
//========================handler functions===============================
void Mez_handler_select(int32_t Mezonin_Type, mezonin *MezStruct);

void Mez_TC_handler(mezonin *MezStruct);

void Mez_TP_handler(mezonin *MezStruct);

//------------------------------------------------------------------------------
// функции обработки ТТ
//------------------------------------------------------------------------------


//     uint32_t Mez_TT_DefineInputChannelNumber (int32_t Mez_IDent);

//     void Mez_TT_EnableChannel (uint32_t Channel, uint32_t A0, uint32_t A1);

void Mez_EnableChannel(mezonin *MezStruct);



//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Mez_TP_handler(mezonin *MezStruct);

void Mez_NOT_handler(void);

extern TT_Value Mezonin_TT[4];
extern TC_Value Mezonin_TC[4];
extern TR_Value Mezonin_TR[4];
extern QueueHandle_t xMezQueue;
extern QueueHandle_t xMezTUQueue;
extern SemaphoreHandle_t xSPISemaphore;


#endif

