/*
 * mezcommon.h
 *
 *  Created on: 25 сент. 2014 г.
 *      Author: maximus
 */

#ifndef MEZCOMMON_H_
#define MEZCOMMON_H_
#include "boards/MSO_board.h"
#include "protocol.h"
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
//-----------------------------------------------------------------------------
typedef struct _LinePIO
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

#endif /* MEZCOMMON_H_ */
