#ifndef MSO_FUNCTIONS_H
#define MSO_FUNCTIONS_H

#include "can/can.h"

//------------------------------------------------------------------------------
// определения для переключателей
//------------------------------------------------------------------------------
#define    Switch2_MASK         ((unsigned int)     0x3)
#define    Switch8_MASK         ((unsigned int)    0xFF)
//------------------------------------------------------------------------------
// функции для работы с переключателями
//------------------------------------------------------------------------------
int Switch8_Read(void);

int Switch2_Read(void);

//------------------------------------------------------------------------------
// функции для заполнения пакетов
//------------------------------------------------------------------------------
void FillTWIPacket(unsigned char*, unsigned int, unsigned char, unsigned char,
		int);
void FillCanPacket(CanTransfer *, unsigned char, unsigned char, unsigned int,
		unsigned int, unsigned int);
//------------------------------------------------------------------------------
// функции чтения и записи TWI
//------------------------------------------------------------------------------
int TWI_Read(unsigned char, unsigned int, unsigned char*, unsigned int, int); // добавлен флаг и все работает!!!!!

int TWI_Write(unsigned char, unsigned int, unsigned char*, unsigned int, int);
//------------------------------------------------------------------------------

/******************************************************************************/

//------------------------------------------------------------------------------
// определения для таймеров
//------------------------------------------------------------------------------
#define    TIMER_ON             ((unsigned char)      1)
#define    TIMER_OFF            ((unsigned char)      0)
#define    TIMER_NUM            ((unsigned  int)     16)
//------------------------------------------------------------------------------
// определение типа для таймеров
//------------------------------------------------------------------------------

typedef struct _timertype
{
	
	unsigned char Timer_State;
	unsigned int Timer_Value;

} TIMER_TYPE;
//------------------------------------------------------------------------------
// функции для работы с таймерами
//------------------------------------------------------------------------------

int Timer_Start(unsigned char channel, unsigned int count);

int Timer_Stop(unsigned char channel);

int Timer_Read(unsigned char channel, unsigned int *count);

void Timer_Handler(void);

/******************************************************************************/

//------------------------------------------------------------------------------
// определения для диодов
//------------------------------------------------------------------------------
#define    LED2_ON              ((unsigned char)      1)
#define    LED2_OFF             ((unsigned char)      0)
#define    LED2_HL5_MASK        ((unsigned  int) 1 << 0)
#define    LED2_HL6_MASK        ((unsigned  int) 1 << 1)
//------------------------------------------------------------------------------
// функции для работы с диоды
//------------------------------------------------------------------------------

int Led2_Write(unsigned char HL6_5);

int Led2_Read(unsigned char *HL6_5);

/******************************************************************************/

//------------------------------------------------------------------------------
// определения для TWI
//------------------------------------------------------------------------------
// коды ошибок
//------------------------------------------------------------------------------
#define    TWI_ERROR_NO_ERRORS             ((int)     1) // операция прошла успешно, ошибок нет
#define    TWI_ERROR_NO_CODE               ((int)     0) // операция не завершена
#define    TWI_ERROR_MEZ_NUM               ((int)    -1) // неверно указан номер мезонина
#define    TWI_ERROR_START_ADDRESS         ((int)    -2) // неверно указан начальный адрес памяти
#define    TWI_ERROR_NUMBER_OF_BYTES       ((int)    -3) // неверно указано количество байт для операции
#define    TWI_ERROR_PAGE_NUMBER           ((int)    -4) // неверно указан номер страницы памяти
#define    TWI_ERROR_CELL_NUMBER           ((int)    -5) // неверно указан номер ячейки памяти в странице
#define    TWI_ERROR_NO_DEVICE_ACK         ((int)    -6) // от устройства нет подтверждения: либо контакт пропадает,
// либо устройство не активировано, либо устройства вообще нет

#define    TWI_WRITE            ((unsigned char)      0) // операция записи
#define    TWI_READ             ((unsigned char)      1) // операция чтения
#define    TWI_LED				((int)	1)				 // Определение для работы с лампочками на передней панели
#define    TWI_MEZ				((int)	2)				 // Определение для работы с мезонинином
#define    TWI_SWITCH8			((int)	3)				 // Определение для работы с Switch 8
#define    TWI_MSO				((int)	4)				 // Определение для работы с памятью MSO
#define    TWI_MULTI_BYTE_TRANSFER  ((unsigned char)      0)
#define    TWI_ONE_BYTE_TRANSFER    ((unsigned char)      1)
//------------------------------------------------------------------------------
// определение типа для передачи данных по TWI
//------------------------------------------------------------------------------

typedef struct _twitransfer
{
	
	unsigned char* PointerToData;       // указатель на данные
	unsigned int DataBytes;           // количество байт данных
	unsigned char TransferStatus;      // состояние передачи
	unsigned char TWI_MODE;
	int TWIErrorCode;        // код ошибки передачи
	unsigned char OneByteTransferFlag; // флаг однобайтовой передачи
	
} TWI_TRANSFER;
//------------------------------------------------------------------------------
#define    TWI_TRANSFER_PROCESS ((unsigned char)      0)
#define    TWI_TRANSFER_FINISH  ((unsigned char)      1)
//------------------------------------------------------------------------------

void TWI_Transfer_Handler(void);

void CAN0_Handler(void);
void CAN1_Handler(void);
//------------------------------------------------------------------------------
// определения для памяти МСО
//------------------------------------------------------------------------------
#define    MSO_MEM_ADR          ((unsigned char)   0x54) // адрес памяти МСО
#define    MSO_MEM_IN_ADR_BYTES ((unsigned char)      2) // количество байт внутреннего адреса памяти МСО
#define    MSO_MEM_BYTES        ((unsigned  int) 0x2000) // количество ячеек памяти
//------------------------------------------------------------------------------
// функции для работы с памятью МСО
//------------------------------------------------------------------------------

int MSO_memory_Write(unsigned int StartAddr, unsigned int NumberOfBytes,
		unsigned char *DataBuffer);

int MSO_memory_Read(unsigned int StartAddr, unsigned int NumberOfBytes,
		unsigned char *DataBuffer);

/******************************************************************************/

//------------------------------------------------------------------------------
// определения для памяти мезонинов
//------------------------------------------------------------------------------
#define    COUNT_MEZ			((unsigned char)   0x04) // количество мезонинов
#define    MEZ_MEM_ADR_0        ((unsigned char)   0x50) // адрес первого блока памяти мезонина
#define    MEZ_MEM_ADR_1        ((unsigned char)   0x51) // адрес второго блока памяти мезонина
#define    MEZ_MEM_PAGES        ((unsigned  int)  0x1FF) // количество страниц памяти мезонина
#define    MEZ_MEM_BLOCK_PAGES  ((unsigned  int)   0xFF) // количество страниц блока памяти мезонина
#define    MEZ_MEM_PAGE_BYTES   ((unsigned  int)   0xFF) // количество байт в странице блока памяти мезонина
#define    MEZ_MEM_IN_ADR_BYTES ((unsigned char)      2) // количество байт внутреннего адреса памяти мезонина
//------------------------------------------------------------------------------
// функции для работы с памятью мезонинов
//------------------------------------------------------------------------------

int MEZ_memory_Write(unsigned char Mezonin, unsigned int PageNum,
		unsigned int ByteNum, unsigned char BytesToWrite, unsigned char* Data);

int MEZ_memory_Read(unsigned char Mezonin, unsigned int PageNum,
		unsigned int ByteNum, unsigned char BytesToRead, unsigned char* Data);

int MEZ_memory_Read_Page(unsigned char Mezonin, unsigned int PageNum,
		unsigned char *Data);

unsigned short Crc16(unsigned char *Data, unsigned short len);

/******************************************************************************/

//------------------------------------------------------------------------------
// определения для сторожевого таймера
//------------------------------------------------------------------------------
#define    WDT_KEYWORD          ((unsigned int)0xA5 << 24) // пароль для записи в регистр WDT_CR
//------------------------------------------------------------------------------
// функции работы со сторожевым таймером
//------------------------------------------------------------------------------

void WDT_Enable(void);

void WDT_Disable(void);

void WDT_Refresh(void);

/******************************************************************************/
#endif
