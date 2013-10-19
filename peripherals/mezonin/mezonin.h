#ifndef MEZONIN_H
#define MEZONIN_H
#include "boards/MSO_board.h"
#include "FreeRTOS.h"
#include "semphr.h"

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
#define   Mez_TP         0x04
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
#define mode_ok		((unsigned int)	0x00)	// режим работы "Норма"
#define mode_on		((unsigned int)	0x08)	// ежим работы "Включено"
#define mode_off	((unsigned int)	0x04)	// режим работы "Выключено"
#define mode_calib	((unsigned int)	0x06)	// режим работы калибровка
//------------------------------------------------------------------------------
//---------------определение типа для LinePIO------------------
typedef struct _LinePIO
{
	AT91PS_PIO PIO_ctrl_ptr; //указатель на PIO controller (PIOA или PIOB); нужен для конфигурации линии MDATA
	unsigned int PIO_Line;

} LinePIO;
//---------------определение типа для мезонинов (тип mezonin)------------------
typedef struct _mezonin
{
	int Mez_ID; // идентификатор мезонина: 1, 2, 3, или 4
	unsigned char Mez_Type;	// тип мезонина
	char Mez_Mem_address; // адрес памяти мезонина
	AT91PS_TC TC_ptr; // указатель на TC
	unsigned int TC_ID; // идентификатор TC
	unsigned int TC_mode; // конфигурация режима TC
	AT91PS_TCB TC_blk_ptr; // указатель на блок, к которому относится TC
	unsigned int TC_blk_mode; // конфигурация режима блока TC
	void (*TC_ISR_ptr)(void); //указатель на функцию обработки прерывания TC
	AT91PS_PWMC_CH PWM_ptr; // указатель на PWM
	unsigned int PWM_ID; // идентификатор PWM
	int PWM_Number; // номер PWM: 0, 1, 2 или 3 (нужен для конфигурации PWM)
	int Periph_AB; // периферия А или В; нужно для конфигурации линии MDATA (все А, кроме MDATA_4)
	LinePIO LineMDATA; // линия MDATA
	LinePIO LineFIN; // линия FIN
	LinePIO LineA0; // линия A0
	LinePIO LineA1; // линия A1
	LinePIO LineBRK; // линия BRK
	LinePIO LineOVF; // линия OVF
	unsigned int I2CCSE; // линия выбора памяти мезонина
	unsigned int CS; // линия выбора по SPI
	unsigned int Start;
	unsigned int TickCount;
	unsigned int ActiveChannel;
	xSemaphoreHandle xSemaphore;

} mezonin;

//---------------определение типа для физической величины ТТ ------------------
typedef struct _Mez_Value
{
	unsigned int Value; // значение физической величины
	unsigned int Channel; // номер канала
	int ID; // номер мезонина
	
} Mez_Value;

//==========================коэффициенты для вычисления ФВ =======================
typedef struct _TT_Coeff
{
	float k_min;		// коэффициент для вычисления минимального значения
	float p_min;		// смещение для вычисления минимального значения
	float k_max;		// коэффициент для вычисления максимального значения
	float p_max;		// смещение для вычисления максимального значения
	unsigned short CRC;		// контрольная сумма
} TT_Coeff;

//==========================пороги канала, хранящиеся в EEPROM для ТТ=======================
typedef struct _TT_Level
{
	float Min_P_Level;	// минимальный предупредительный порог
	float Max_P_Level;	// максимальный предупредительный порог
	float Min_A_Level;	// минимальный аварийный порог
	float Max_A_Level;	// максимальный аварийный порог
	float Sense;			// чувствительность
	unsigned short CRC;			// контрольная сумма
} TT_Level;

//==========================парамеры канала, хранящиеся в EEPROM для ТТ=======================
typedef struct _TT_Param
{
	unsigned int MeasTime;	// время измерения
	unsigned int Mode;		// режим работы канала
	/*	float 					k_min;		// коэффициент для вычисления минимального значения
	 float 					p_min;		// смещение для вычисления минимального значения
	 float 					k_max;		// коэффициент для вычисления максимального значения
	 float 					p_max;		// смещение для вычисления максимального значения
	 float					Sense;		// чувствительность
	 */
	float MinD;		// значение минимума (например 0..20 это 0, в 4..20 это 4)
	float MaxD;		// значение максимума (например 0..20 это 20, в 4..20 это 20)
	float MinF;		// значение минимума физической величины
	float MaxF;		// значение максимума физической величины
	unsigned short CRC;		// контрольная сумма
} TT_Param;
//---------------структура канала ТТ (тип mezonin)----------------------
typedef struct _TT_Channel
{
	float Value; 		// значение физической величины
	float OldValue;	// старое значение физической величины
	unsigned char State;		// состояние ТТ
	TT_Param Params;		// параметры из EEPROM
	TT_Coeff Coeffs;		// коэффициенты из EEPROM
	TT_Level Levels;		// пороги из EEPROM
	unsigned int Min_Value;  // значение минимума для расчета
	unsigned int Max_Value;	// значение максимума для расчета
//  unsigned int			CRC;		// контрольная сумма
// возможно будет дополняться
} TT_Channel;
//---------------определение типа для ТТ (тип mezonin)------------------
typedef struct _TT_Value
{
	TT_Channel Channel[4]; // номер канала
	unsigned char PerTime;
	int ID; // номер мезонина
} TT_Value;

//==========================парамеры канала, хранящиеся в EEPROM для ТС=======================
typedef struct _TC_Param
{
	unsigned int Mode;	// режим работы
	unsigned short CRC;	// контрольная сумма
} TC_Param;
//---------------структура канала ТC (тип mezonin)----------------------
typedef struct _TC_Channel
{
	unsigned char Value; 		// значение канала (On или Off)
	unsigned char State;  	// состояние канала (Break или не Break)
	TC_Param Params;		// параметры из EEPROM
//  unsigned char			Mode;		// режим работы канала (On/Off)
	// возможно будет дополняться
} TC_Channel;
//---------------определение типа для ТC (тип mezonin)------------------
typedef struct _TC_Value
{
	TC_Channel Channel[4]; // номер канала
	unsigned char PerTime;
	int ID; // номер мезонина
} TC_Value;

//==========================парамеры канала, хранящиеся в EEPROM для ТI=======================
typedef struct _TI_Param
{
	unsigned int Mode;	// режим работы
	float Sense;  // Чувствительность
	float CoeFf;	// Коэффициент
	unsigned short CRC;	// контрольная сумма
} TI_Param;
//---------------структура канала ТU (тип mezonin)----------------------
typedef struct _TI_Channel
{
	unsigned char Value; 		// счетчик ФВ
	unsigned char State;  	// состояние
	unsigned long CountTI;	// счетчик импульсов
	TI_Param Params;		// параметры из EEPROM
//  unsigned char			Mode;		// режим работы канала (On/Off)
	// возможно будет дополняться
} TI_Channel;
//---------------определение типа для ТU (тип mezonin)------------------
typedef struct _TI_Value
{
	TI_Channel Channel[4]; // номер канала
	unsigned char PerTime;
	int ID; // номер мезонина
} TI_Value;

//==========================парамеры канала, хранящиеся в EEPROM для ТU=======================
typedef struct _TU_Param
{
	unsigned int Mode;			// режим работы
	unsigned char TimeTU;			// время выдержки ТУ
	unsigned char NumberTC;		// ТС концевиков
	unsigned char ExtraTimeTU;	// Дополнительное время выдержки ТУ
	unsigned short CRC;			// контрольная сумма
} TU_Param;
//---------------структура канала ТC (тип mezonin)----------------------
typedef struct _TU_Channel
{
	unsigned char State;  	// состояние
	unsigned char Value; 		// значение канала (On или Off)
	TU_Param Params;		// параметры из EEPROM
//  unsigned char			Mode;		// режим работы канала (On/Off)
	// возможно будет дополняться
} TU_Channel;
//---------------определение типа для ТU (тип mezonin)------------------
typedef struct _TU_Value
{
	TU_Channel Channel[4]; // номер канала
	unsigned char PerTime;
	int ID; // номер мезонина
} TU_Value;

//unsigned char Mez_Recognition_old (unsigned int MezMemoryLine, char address);
//==========================init functions===========================

unsigned char Mez_Recognition(unsigned char MezNum);

void Mez_Select(unsigned int MezMemoryLine);

void Mez_PreInit(mezonin *Mez1, mezonin *Mez2, mezonin *Mez3, mezonin *Mez4);

void Mez_init(int Mezonin_Type, mezonin *MezStruct);

void Mez_TC_init(mezonin *MezStruct);

void Mez_TU_init(mezonin *MezStruct);

void Mez_TT_init(mezonin *MezStruct);

void Mez_TP_init(mezonin *MezStruct);

void Mez_TI_init(mezonin *MezStruct);

void Mez_NOT_init(void);

unsigned int Get_TTParams(TT_Value *TT_temp);
unsigned int Get_TTCoeffs(TT_Value *TT_temp);

unsigned int Get_TCParams(TC_Value *TC_temp);
unsigned int Get_TTLevels(TT_Value *TT_temp);
unsigned int Get_TUParams(TU_Value *TU_temp);

//========================handler functions===============================
void Mez_handler_select(int Mezonin_Type, mezonin *MezStruct);

void Mez_TC_handler(mezonin *MezStruct);

void Mez_TU_handler(mezonin *MezStruct);


//------------------------------------------------------------------------------
// функции обработки ТТ
//------------------------------------------------------------------------------
void Mez_TT_handler(mezonin *MezStruct/*, TT_Value *Mez_TT_temp*/);

void Mez_TT_Calib(mezonin *MezStruct, int Channel_Num, int flag);

//     unsigned int Mez_TT_DefineInputChannelNumber (int Mez_IDent);

//     void Mez_TT_EnableChannel (unsigned int Channel, unsigned int A0, unsigned int A1);

void Mez_EnableChannel(mezonin *MezStruct);

void Mez_1_TT_ISR_TC(void);
void Mez_2_TT_ISR_TC(void);
void Mez_3_TT_ISR_TC(void);
void Mez_4_TT_ISR_TC(void);

float Mez_TT_Frequency(unsigned int measured_value, unsigned int ChannelNumber,
		unsigned int MEZ_ID);

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Mez_TP_handler(mezonin *MezStruct);

void Mez_TI_handler(mezonin *MezStruct);

void Mez_NOT_handler(void);
#endif

