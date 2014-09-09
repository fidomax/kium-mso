#ifndef MEZONIN_H
#define MEZONIN_H
#include "boards/MSO_board.h"
#include "FreeRTOS.h"
#include "semphr.h"

//=====================количество мезонинов======================
#define	  Mez_Count		 3 // Мезонины считаются с 0-го по 3-ий//=====================номера мезонинов======================#define   Mez_1          1#define   Mez_2          2
#define   Mez_3          3
#define   Mez_4          4
//==========================типы мезонинов=============================
#define   Mez_NOT        0x00
#define   Mez_TC         0x01
#define   Mez_TU         0x02
#define   Mez_TT         0x03
#define   Mez_TP         0x04
#define   Mez_TI         0x05
//=====================================================================
#define   Mez_MemAddr       0x50 //адрес памяти#define   Mez_Type_Address    0x00 //----------------------------------------------------------------------#define PeriphA   1
#define PeriphB   2
//----------------------------------------------------------------------
//         For Mode (режимы работы)
//------------------------------------------------------------------------------
#define mode_ok		((uint32_t)	0x00)	// режим работы "Норма"#define mode_on		((uint32_t)	0x08)	// ежим работы "Включено"#define mode_off	((uint32_t)	0x04)	// режим работы "Выключено"#define mode_calib	((uint32_t)	0x06)	// режим работы калибровка//------------------------------------------------------------------------------//---------------определение типа для LinePIO------------------typedef struct _LinePIO
{
	AT91PS_PIO PIO_ctrl_ptr; //указатель на PIO controller (PIOA или PIOB); нужен для конфигурации линии MDATA
	uint32_t PIO_Line;

} LinePIO;
//---------------определение типа для мезонинов (тип mezonin)------------------
typedef struct _mezonin
{
	int32_t Mez_ID; // идентификатор мезонина: 1, 2, 3, или 4
	uint8_t Mez_Type;	// тип мезонина
	int8_t Mez_Mem_address; // адрес памяти мезонина
	AT91PS_TC TC_ptr; // указатель на TC
	uint32_t TC_ID; // идентификатор TC
	uint32_t TC_mode; // конфигурация режима TC
	AT91PS_TCB TC_blk_ptr; // указатель на блок, к которому относится TC
	uint32_t TC_blk_mode; // конфигурация режима блока TC
	void (*TC_ISR_ptr)(void); //указатель на функцию обработки прерывания TC
	AT91PS_PWMC_CH PWM_ptr; // указатель на PWM
	uint32_t PWM_ID; // идентификатор PWM
	int32_t PWM_Number; // номер PWM: 0, 1, 2 или 3 (нужен для конфигурации PWM)
	int32_t Periph_AB; // периферия А или В; нужно для конфигурации линии MDATA (все А, кроме MDATA_4)
	LinePIO LineMDATA; // линия MDATA
	LinePIO LineFIN; // линия FIN
	LinePIO LineA0; // линия A0
	LinePIO LineA1; // линия A1
	LinePIO LineBRK; // линия BRK
	LinePIO LineOVF; // линия OVF
	uint32_t I2CCSE; // линия выбора памяти мезонина
	uint32_t CS; // линия выбора по SPI
	uint32_t Start;
	uint32_t TickCount;
	uint32_t ActiveChannel;
	SemaphoreHandle_t xSemaphore;
	QueueHandle_t TUQueue;
	QueueHandle_t TPQueue;
} mezonin;

//---------------определение типа для физической величины ТТ ------------------
typedef struct _Mez_Value
{
	union{
		uint32_t ui32Value; // значение физической величины
		float fValue; // значение физической величины
	};
	uint32_t Channel; // номер канала
	int32_t ID; // номер мезонина

} Mez_Value;

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
//	uint8_t PerTime;
	int32_t ID; // номер мезонина
} TC_Value;

//==========================парамеры канала, хранящиеся в EEPROM для ТI=======================
typedef struct _TI_Param
{
	uint32_t Mode;	// режим работы
	float Sense;  // Чувствительность
	float CoeFf;	// Коэффициент
	uint16_t CRC;	// контрольная сумма
} TI_Param;
//---------------структура канала ТU (тип mezonin)----------------------
typedef struct _TI_Channel
{
	uint8_t Value; 		// счетчик ФВ
	uint8_t State;  	// состояние
	unsigned long CountTI;	// счетчик импульсов
	TI_Param Params;		// параметры из EEPROM
//  uint8_t			Mode;		// режим работы канала (On/Off)
	// возможно будет дополняться
} TI_Channel;
//---------------определение типа для ТU (тип mezonin)------------------
typedef struct _TI_Value
{
	TI_Channel Channel[4]; // номер канала
	uint8_t PerTime;
	int32_t ID; // номер мезонина
} TI_Value;

//==========================парамеры канала, хранящиеся в EEPROM для ТU=======================
typedef struct _TU_Param
{
	uint32_t Mode;			// режим работы
	uint8_t TimeTU;			// время выдержки ТУ
	uint8_t NumberTC;		// ТС концевиков
	uint8_t ExtraTimeTU;	// Дополнительное время выдержки ТУ
	uint16_t CRC;			// контрольная сумма
} TU_Param;
//---------------структура канала ТC (тип mezonin)----------------------
typedef struct _TU_Channel
{
	uint8_t State;  	// состояние
	uint8_t Value; 		// значение канала (On или Off)
	TU_Param Params;		// параметры из EEPROM
//  uint8_t			Mode;		// режим работы канала (On/Off)
	// возможно будет дополняться
} TU_Channel;
//---------------определение типа для ТU (тип mezonin)------------------
typedef struct _TU_Value
{
	TU_Channel Channel[4]; // номер канала
	uint8_t PerTime;
	int32_t ID; // номер мезонина
} TU_Value;
//---------------структура канала ТP (тип mezonin)----------------------
typedef struct _TP_Channel
{
	float flDAC; 			//ФВ
} TP_Channel;
//---------------определение типа для ТP (тип mezonin)------------------
typedef struct _TP_Value
{
	TP_Channel Channel; // номер канала
	int32_t ID; // номер мезонина
} TP_Value;
//uint8_t Mez_Recognition_old (uint32_t MezMemoryLine, int8_t address);
//==========================init functions===========================

int32_t Mez_SetType(uint8_t MezNum, uint8_t MezType);

uint8_t Mez_Recognition(uint8_t MezNum);

void Mez_Select(uint32_t MezMemoryLine);

void Mez_PreInit(mezonin *Mez1, mezonin *Mez2, mezonin *Mez3, mezonin *Mez4);

void Mez_init(uint32_t Mezonin_Type, mezonin *MezStruct);

void Mez_TC_init(mezonin *MezStruct);

void Mez_TU_init(mezonin *MezStruct);

void Mez_TT_init(mezonin *MezStruct);

void Mez_TP_init(mezonin *MezStruct);

void Mez_TI_init(mezonin *MezStruct);

void Mez_NOT_init(void);

void TCValueHandler (Mez_Value *Mez_V);
void TTValueHandler (Mez_Value *Mez_V);

uint32_t Get_TTParams(TT_Value *TT_temp);
uint32_t Get_TTCoeffs(TT_Value *TT_temp);

uint32_t Get_TCParams(TC_Value *TC_temp);
uint32_t Get_TTLevels(TT_Value *TT_temp);
uint32_t Get_TUParams(TU_Value *TU_temp);

void Set_TCDefaultParams(uint8_t MezNum);
void Set_TTDefaultParams(uint8_t MezNum);

void WriteTTCoeffs(uint8_t MezNum, int ChannelNumber, TT_Coeff* Coeffs);
void WriteTTLevels(uint8_t MezNum, int ChannelNumber, TT_Level* Levels);
void WriteTTParams(uint8_t MezNum, int ChannelNumber, TT_Param* Params);

//========================handler functions===============================
void Mez_handler_select(int32_t Mezonin_Type, mezonin *MezStruct);

void Mez_TC_handler(mezonin *MezStruct);

void Mez_TU_handler(mezonin *MezStruct);

void Mez_TP_handler(mezonin *MezStruct);

//------------------------------------------------------------------------------
// функции обработки ТТ
//------------------------------------------------------------------------------
void Mez_TT_handler(mezonin *MezStruct/*, TT_Value *Mez_TT_temp*/);
void TTTickHandler(void);

void Mez_TT_Calib(mezonin *MezStruct, uint32_t Channel_Num, uint32_t flag);

//     uint32_t Mez_TT_DefineInputChannelNumber (int32_t Mez_IDent);

//     void Mez_TT_EnableChannel (uint32_t Channel, uint32_t A0, uint32_t A1);

void Mez_EnableChannel(mezonin *MezStruct);

void Mez_1_TT_ISR_TC(void);
void Mez_2_TT_ISR_TC(void);
void Mez_3_TT_ISR_TC(void);
void Mez_4_TT_ISR_TC(void);

float Mez_TT_Frequency(uint32_t measured_value, uint32_t ChannelNumber, uint32_t MEZ_ID);

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Mez_TP_handler(mezonin *MezStruct);

void Mez_TI_handler(mezonin *MezStruct);

void Mez_NOT_handler(void);
#endif

