#include "FreeRTOS.h"
#include "MSO_functions.h"
#include "PCA9532/PCA9532.h"
#include "twi/twi.h"
#include "can/can.h"
#include "boards/MSO_board.h"
#include "pio/pio_config.h"

TIMER_TYPE timer_0;
TIMER_TYPE timer_1;
TIMER_TYPE timer_2;
TIMER_TYPE timer_3;
TIMER_TYPE timer_4;
TIMER_TYPE timer_5;
TIMER_TYPE timer_6;
TIMER_TYPE timer_7;
TIMER_TYPE timer_8;
TIMER_TYPE timer_9;
TIMER_TYPE timer_10;
TIMER_TYPE timer_11;
TIMER_TYPE timer_12;
TIMER_TYPE timer_13;
TIMER_TYPE timer_14;
TIMER_TYPE timer_15;

TIMER_TYPE* TIMERS[TIMER_NUM] = { &timer_0, &timer_1, &timer_2, &timer_3, &timer_4, &timer_5, &timer_6, &timer_7, &timer_8, &timer_9,
		&timer_10, &timer_11, &timer_12, &timer_13, &timer_14, &timer_15 };

volatile TWI_TRANSFER TWI_packet;
//------------------------------------------------------------------------------
// Функции заполнения packet
//------------------------------------------------------------------------------
void FillTWIPacket(unsigned char* Data, unsigned int Bytes, unsigned char Status, unsigned char Mode, int Error)
{
	TWI_packet.DataBytes = Bytes;
	TWI_packet.PointerToData = Data;
	TWI_packet.TransferStatus = Status;
	TWI_packet.TWIErrorCode = Error;
	TWI_packet.TWI_MODE = Mode;
}
//------------------------------------------------------------------------------
void FillCanPacket(CanTransfer *canTransfer, unsigned char number, unsigned char mailbox, unsigned int mode, unsigned int mask,
		unsigned int identifier)
{
	canTransfer->can_number = number;
	canTransfer->mailbox_number = mailbox;
	canTransfer->mode_reg = mode;
	canTransfer->acceptance_mask_reg = mask;
	canTransfer->identifier = identifier;
}
//------------------------------------------------------------------------------
// Функции чтения и записи TWI
//------------------------------------------------------------------------------
// Чтение TWI
//------------------------------------------------------------------------------
int TWI_Read(unsigned char address, unsigned int iadress, unsigned char* Data, unsigned int AmountBytes, int flag)
{
	int isize; // размер для StartRead

	if (AmountBytes == 1)
		TWI_packet.OneByteTransferFlag = TWI_ONE_BYTE_TRANSFER;

	else
		TWI_packet.OneByteTransferFlag = TWI_MULTI_BYTE_TRANSFER;

	if (flag == TWI_SWITCH8 || flag == TWI_LED)
		isize = 1;
	if (flag == TWI_MEZ || flag == TWI_MSO)
		isize = 2;

	FillTWIPacket(Data, AmountBytes, TWI_TRANSFER_PROCESS, TWI_READ, TWI_ERROR_NO_CODE);
	TWI_EnableIt(AT91C_BASE_TWI, AT91C_TWI_RXRDY);
	TWI_StartRead(AT91C_BASE_TWI, address, iadress, isize);
	*Data = TWI_ReadByte(AT91C_BASE_TWI);
	if (TWI_packet.OneByteTransferFlag == TWI_ONE_BYTE_TRANSFER)
		TWI_Stop(AT91C_BASE_TWI);
	TWI_EnableIt(AT91C_BASE_TWI, AT91C_TWI_TXCOMP);
	while (TWI_packet.TWIErrorCode == TWI_ERROR_NO_CODE)
		;
	return TWI_packet.TWIErrorCode;

}

//------------------------------------------------------------------------------
// Запись TWI
//------------------------------------------------------------------------------
int TWI_Write(unsigned char address, unsigned int iadress, unsigned char* Data, unsigned int AmountBytes, int flag)
{
	int isize; // размер для StartWrite

	if (AmountBytes == 1)
		TWI_packet.OneByteTransferFlag = TWI_ONE_BYTE_TRANSFER;

	else
		TWI_packet.OneByteTransferFlag = TWI_MULTI_BYTE_TRANSFER;

	if (flag == TWI_SWITCH8 || flag == TWI_LED)
		isize = 1;
	if (flag == TWI_MEZ || flag == TWI_MSO)
		isize = 2;

	FillTWIPacket(Data, AmountBytes, TWI_TRANSFER_PROCESS, TWI_WRITE, TWI_ERROR_NO_CODE);
	TWI_StartWrite(AT91C_BASE_TWI, address, iadress, isize);
	TWI_WriteByte(AT91C_BASE_TWI, *Data);
	if (TWI_packet.OneByteTransferFlag == TWI_ONE_BYTE_TRANSFER)
		TWI_Stop(AT91C_BASE_TWI);
	TWI_EnableIt(AT91C_BASE_TWI, AT91C_TWI_TXCOMP | AT91C_TWI_TXRDY);
	while (TWI_packet.TWIErrorCode == TWI_ERROR_NO_CODE)
		;
	return TWI_packet.TWIErrorCode;

}
//------------------------------------------------------------------------------
// Функции работы с диодами
//------------------------------------------------------------------------------
// Запись состояния диодов
//------------------------------------------------------------------------------
int Led2_Write(unsigned char HL6_5)
{

	if (HL6_5 <= 0x3) {
		if ((HL6_5 & LED2_HL5_MASK) == LED2_ON)
			AT91F_PIO_ClearOutput(AT91C_BASE_PIOB, LED_0);
		else
			AT91F_PIO_SetOutput(AT91C_BASE_PIOB, LED_0);

		if (((HL6_5 & LED2_HL6_MASK) >> 1) == LED2_ON)
			AT91F_PIO_ClearOutput(AT91C_BASE_PIOB, LED_1);
		else
			AT91F_PIO_SetOutput(AT91C_BASE_PIOB, LED_1);

		return 1;
	}

	else
		return -1;
}
//------------------------------------------------------------------------------
// Чтение состояния диодов
//------------------------------------------------------------------------------
int Led2_Read(unsigned char *HL6_5)
{

	*HL6_5 = (~(AT91C_BASE_PIOB->PIO_PDSR >> 12)) & (LED2_HL5_MASK | LED2_HL6_MASK);

	return 1;
}
//------------------------------------------------------------------------------

/******************************************************************************/

//------------------------------------------------------------------------------
// Функции работы с переключателями
//------------------------------------------------------------------------------
// Чтение состояния переключателей SA1
//------------------------------------------------------------------------------
int Switch8_Read(void)
{
	unsigned char Switch8;

	TWI_Read(PCA9532_address, PCA9532_INPUT1, &Switch8, 1, TWI_SWITCH8);

	return ((~Switch8) & Switch8_MASK);
}
//------------------------------------------------------------------------------
// Чтение состояния переключателей SA2
//------------------------------------------------------------------------------
int Switch2_Read(void)
{
	return ((~(AT91C_BASE_PIOB->PIO_PDSR >> 2)) & Switch2_MASK);
}
//------------------------------------------------------------------------------

/******************************************************************************/

//------------------------------------------------------------------------------
// Функции работы с таймерами
//------------------------------------------------------------------------------
// включение таймера с заданным значением
// unsigned char channel - номер таймера
// unsigned int count    - значение таймера для отсчета, мс
//------------------------------------------------------------------------------
int Timer_Start(unsigned char channel, unsigned int count)
{
	if (channel < TIMER_NUM) {
		TIMERS[channel]->Timer_Value = count;
		TIMERS[channel]->Timer_State = TIMER_ON;

		return 1;
	}

	else
		return -1;
}
//------------------------------------------------------------------------------
// выключение таймера
//------------------------------------------------------------------------------
int Timer_Stop(unsigned char channel)
{
	if (channel < TIMER_NUM) {
		TIMERS[channel]->Timer_State = TIMER_OFF;

		return 1;
	}

	else
		return -1;
}
//------------------------------------------------------------------------------
// чтение текущего значения таймера
//------------------------------------------------------------------------------
int Timer_Read(unsigned char channel, unsigned int *count)
{
	if (channel < TIMER_NUM) {
		*count = TIMERS[channel]->Timer_Value;

		return 1;
	}

	else
		return -1;
}
//------------------------------------------------------------------------------
// обработка таймеров
//--------------------
// Нужно сконфигурировать PIT (periodic interval timer) контроллера так, чтобы
// событие (PICNT: Periodic Interval Counter) возникало с периодом 1 мс,
// ( т.е. AT91C_BASE_PITC->PITC_PIMR = 0xBB2; ) и разрешить прерывания
// (AT91C_BASE_PITC->PITC_PIMR |= AT91C_PITC_PITIEN; или функция из библиотеки
//  PIT_EnableIT(); ). В функции обработки прерываний нужно прочитывать регистр
// PIT_PIVR (Periodic Interval Timer Value Register), чтобы стирать обработанное
// прерывание. В функцию обработки прерываний от PIT нужно включить эту функцию
// обработки таймеров Timer_Handler. Функция обработки прерываний от PIT будет
// выглядеть примерно так:
//                            void PIT_ISR (void)
//                             {  
//                               if (PIT_GetStatus() == AT91C_PITC_PITS)
//                                {
//                                  PIT_GetPIVR();
//                                  Timer_Handler ();
//                                }
//                              }
//------------------------------------------------------------------------------
void Timer_Handler(void)
{
	unsigned int i;

	for (i = 0; i < TIMER_NUM; i++) {
		if (TIMERS[i]->Timer_State == TIMER_ON)
			TIMERS[i]->Timer_Value--;
	}
}
//------------------------------------------------------------------------------

/******************************************************************************/

//------------------------------------------------------------------------------
// Функции работы с памятью МСО
// память организована как одна строка с 8192 ячейками
// при адресации указывается номер ячейки памяти
//------------------------------------------------------------------------------
//
// запись
//
// unsigned int StartAddr - начальный адрес для записи
//                          может принимать значения от 0х00 до 0х1FFF
//
// unsigned int NumberOfBytes - количество байт для записи
//                              может принимать значения от 0х01 до (8192 - StartAddr)
//
// unsigned char *DataBuffer - указатель на массив записываемых данных
//------------------------------------------------------------------------------
int MSO_memory_Write(unsigned int StartAddr, unsigned int NumberOfBytes, unsigned char *DataBuffer)
{

	int ReturnValue = 0;

	if ((NumberOfBytes != 0) && ((StartAddr + NumberOfBytes) <= MSO_MEM_BYTES)) {
		ReturnValue = TWI_Write(MSO_MEM_ADR, StartAddr, DataBuffer, NumberOfBytes, TWI_MSO);
	}

	else if (StartAddr > (MSO_MEM_BYTES - 1))
		ReturnValue = TWI_ERROR_START_ADDRESS;

	else if ((NumberOfBytes == 0) || ((StartAddr + NumberOfBytes) > MSO_MEM_BYTES))
		ReturnValue = TWI_ERROR_NUMBER_OF_BYTES;

	return ReturnValue;
}
//------------------------------------------------------------------------------
// чтение
//
// unsigned int StartAddr - начальный адрес для чтения
//                          может принимать значения от 0х00 до 0х1FFF
//
// unsigned int NumberOfBytes - количество байт для чтения
//                              может принимать значения от 0х01 до (8192 - StartAddr)
//
// unsigned char *DataBuffer - указатель на массив для сохранения считываемых данных
//------------------------------------------------------------------------------
int MSO_memory_Read(unsigned int StartAddr, unsigned int NumberOfBytes, unsigned char *DataBuffer)
{

	int ReturnValue = 0;

	if ((NumberOfBytes != 0) && ((StartAddr + NumberOfBytes) <= MSO_MEM_BYTES)) {
		ReturnValue = TWI_Read(MSO_MEM_ADR, StartAddr, DataBuffer, NumberOfBytes, TWI_MSO);
	}

	else if (StartAddr > (MSO_MEM_BYTES - 1))
		ReturnValue = TWI_ERROR_START_ADDRESS;

	else if ((NumberOfBytes == 0) || ((StartAddr + NumberOfBytes) > MSO_MEM_BYTES))
		ReturnValue = TWI_ERROR_NUMBER_OF_BYTES;

	return ReturnValue;
}
//------------------------------------------------------------------------------

/******************************************************************************/

//------------------------------------------------------------------------------
// Функции работы с памятью мезонинов
// память организована как 512 страниц по 256 ячеек
// при адресации указывается номер страницы и номер ячейки
//------------------------------------------------------------------------------
//
// запись байта
// после операции записи нужно подождать 10 мс перед следующим обращением
// (чтением / записью) к памяти
//
// unsigned char mezonin - номер мезонина
//                         может принимать значения 0, 1, 2 или 3
//
// unsigned int PageNum - номер страницы памяти
//                        может принимать значения от 0х00 до 0х1FF
//
// unsigned int ByteNum - номер ячейки
//                        может принимать значения от 0х00 до 0хFF
//
// unsigned char BytesToWrite - количество записываемых байтов
//                              может принимать значения от 0х00 до 0хFF
//
// unsigned char* Data - указатель на массив для сохранения считываемых данных
//------------------------------------------------------------------------------
int MEZ_memory_Write(unsigned char Mezonin, unsigned int PageNum, unsigned int ByteNum, unsigned char BytesToWrite, unsigned char* Data)
{
	unsigned char MezMemAddress;
	int ReturnValue;

	if ((Mezonin < COUNT_MEZ) && (PageNum <= MEZ_MEM_PAGES) && (ByteNum <= MEZ_MEM_PAGE_BYTES)
			&& ((ByteNum + BytesToWrite) <= (MEZ_MEM_PAGE_BYTES + 1))) {

		if (PageNum <= MEZ_MEM_BLOCK_PAGES)
			MezMemAddress = MEZ_MEM_ADR_0;
		else {
			MezMemAddress = MEZ_MEM_ADR_1;
			PageNum -= 0x100;
		}

		AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, (1 << (4 + Mezonin))); // выбор микросхемы памяти для конкретного мезонина
		AT91F_PIO_ClearOutput(AT91C_BASE_PIOB, WP);

		ReturnValue = TWI_Write(MezMemAddress, ((PageNum << 8) | ByteNum), Data, BytesToWrite, TWI_MEZ);

		AT91F_PIO_SetOutput(AT91C_BASE_PIOB, WP);
		AT91F_PIO_SetOutput(AT91C_BASE_PIOA, (1 << (4 + Mezonin)));
	}

	else if (Mezonin >= COUNT_MEZ)
		ReturnValue = TWI_ERROR_MEZ_NUM;

	else if (PageNum > MEZ_MEM_PAGES)
		ReturnValue = TWI_ERROR_PAGE_NUMBER;

	else if (ByteNum > MEZ_MEM_PAGE_BYTES)
		ReturnValue = TWI_ERROR_CELL_NUMBER;

	else
		ReturnValue = TWI_ERROR_NUMBER_OF_BYTES;

	return ReturnValue;

}
//------------------------------------------------------------------------------
// чтение байта
//
// unsigned char mezonin - номер мезонина
//                         может принимать значения 0, 1, 2 или 3
//
// unsigned int PageNum - номер страницы памяти
//                        может принимать значения от 0х00 до 0х1FF
//
// unsigned int ByteNum - номер ячейки
//                        может принимать значения от 0х00 до 0хFF
//
// unsigned char BytesToRead - количество считываемых байтов
//                              может принимать значения от 0х00 до 0хFF
//
// unsigned char* Data - указатель на массив для сохранения считываемых данных
//------------------------------------------------------------------------------
int MEZ_memory_Read(unsigned char Mezonin, unsigned int PageNum, unsigned int ByteNum, unsigned char BytesToRead, unsigned char* Data)
{

	unsigned char MezMemAddress;
	int ReturnValue;

	if ((Mezonin < COUNT_MEZ) && (PageNum <= MEZ_MEM_PAGES) && (ByteNum <= MEZ_MEM_PAGE_BYTES)
			&& ((ByteNum + BytesToRead) <= (MEZ_MEM_PAGE_BYTES + 1))) {

		if (PageNum <= MEZ_MEM_BLOCK_PAGES)
			MezMemAddress = MEZ_MEM_ADR_0;
		else {
			MezMemAddress = MEZ_MEM_ADR_1;
			PageNum -= 0x100;
		}

		AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, (1 << (4 + Mezonin))); // выбор микросхемы памяти для конкретного мезонина

		ReturnValue = TWI_Read(MezMemAddress, ((PageNum << 8) | ByteNum), Data, BytesToRead, TWI_MEZ);

		AT91F_PIO_SetOutput(AT91C_BASE_PIOA, (1 << (4 + Mezonin)));

	}

	else if (Mezonin >= COUNT_MEZ)
		ReturnValue = TWI_ERROR_MEZ_NUM;

	else if (PageNum > MEZ_MEM_PAGES)
		ReturnValue = TWI_ERROR_PAGE_NUMBER;

	else if (ByteNum > MEZ_MEM_PAGE_BYTES)
		ReturnValue = TWI_ERROR_CELL_NUMBER;

	else
		ReturnValue = TWI_ERROR_NUMBER_OF_BYTES;

	return ReturnValue;
}
//------------------------------------------------------------------------------
// чтение страницы - 256 байт
//
// unsigned char mezonin - номер мезонина
//                         может принимать значения 0, 1, 2 или 3
//
// unsigned int PageNum - номер страницы памяти
//                        может принимать значения от 0х00 до 0х1FF
//
// unsigned char *DataBuffer - указатель на массив для сохранения считываемых данных
//------------------------------------------------------------------------------
int MEZ_memory_Read_Page(unsigned char Mezonin, unsigned int PageNum, unsigned char *DataBuffer)
{
	unsigned char MezMemAddress;
	unsigned int NumberOfBytes = (MEZ_MEM_PAGE_BYTES + 1);
	int ReturnValue = 0;

	if ((Mezonin < COUNT_MEZ) && (PageNum <= MEZ_MEM_PAGES)) {
		if (PageNum <= MEZ_MEM_BLOCK_PAGES) {
			MezMemAddress = MEZ_MEM_ADR_0;
		} else {
			MezMemAddress = MEZ_MEM_ADR_1;
			PageNum -= 0x100;
		}

		AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, (1 << (4 + Mezonin))); // выбор микросхемы памяти для конкретного мезонина

		ReturnValue = TWI_Read(MezMemAddress, (PageNum << 8), DataBuffer, NumberOfBytes, TWI_MEZ);

		AT91F_PIO_SetOutput(AT91C_BASE_PIOA, (1 << (4 + Mezonin)));
	}

	else if (Mezonin >= COUNT_MEZ)
		ReturnValue = TWI_ERROR_MEZ_NUM;

	else if (PageNum > MEZ_MEM_PAGES)
		ReturnValue = TWI_ERROR_PAGE_NUMBER;

	return ReturnValue;
}
//------------------------------------------------------------------------------
// обработка передачи данных по TWI
// эта функция должна вызываться функцией обтаботки прерываний для TWI
//------------------------------------------------------------------------------
void TWI_Transfer_Handler(void)
{
	unsigned int TWI_status;

	TWI_status = AT91C_BASE_TWI->TWI_SR;

	if ((TWI_status & AT91C_TWI_RXRDY) == AT91C_TWI_RXRDY) {

		if (TWI_packet.TWI_MODE == TWI_READ) {
			if (TWI_packet.OneByteTransferFlag == TWI_ONE_BYTE_TRANSFER) {
				*(TWI_packet.PointerToData) = TWI_ReadByte(AT91C_BASE_TWI);
				TWI_DisableIt(AT91C_BASE_TWI, AT91C_TWI_RXRDY);
				TWI_ReadByte(AT91C_BASE_TWI);
				TWI_packet.TransferStatus = TWI_TRANSFER_FINISH;
			}

			else {
				*(TWI_packet.PointerToData) = TWI_ReadByte(AT91C_BASE_TWI);

				TWI_packet.PointerToData++;
				TWI_packet.DataBytes--;

				if ((TWI_packet.DataBytes - 1) == 0) {
					TWI_Stop(AT91C_BASE_TWI);
				}

				else if (TWI_packet.DataBytes == 0) {
					TWI_DisableIt(AT91C_BASE_TWI, AT91C_TWI_RXRDY);
					TWI_ReadByte(AT91C_BASE_TWI);
					TWI_packet.TransferStatus = TWI_TRANSFER_FINISH;
				}

			}

		}

	}

	else if ((TWI_status & AT91C_TWI_TXRDY) == AT91C_TWI_TXRDY) { // Ранее был код из 2/Mso_Functions
		if (TWI_packet.TWI_MODE == TWI_WRITE) {
			if (TWI_packet.DataBytes--) {
				TWI_packet.PointerToData++;

				if (TWI_packet.DataBytes >= 1) {
					TWI_WriteByte(AT91C_BASE_TWI, *(TWI_packet.PointerToData));
				} else {
					TWI_DisableIt(AT91C_BASE_TWI, AT91C_TWI_TXRDY);
					TWI_Stop(AT91C_BASE_TWI);
					TWI_packet.TransferStatus = TWI_TRANSFER_FINISH;
				}
			}

		}
	}

	if ((TWI_status & AT91C_TWI_TXCOMP) == AT91C_TWI_TXCOMP) {
		if (((TWI_status & AT91C_TWI_NACK) == AT91C_TWI_NACK)) {
			TWI_DisableIt(AT91C_BASE_TWI, AT91C_TWI_TXRDY | AT91C_TWI_RXRDY | AT91C_TWI_TXCOMP);
			TWI_packet.TWIErrorCode = TWI_ERROR_NO_DEVICE_ACK;
		}

		else if (TWI_packet.TransferStatus == TWI_TRANSFER_FINISH) {
			TWI_DisableIt(AT91C_BASE_TWI, AT91C_TWI_TXCOMP);
			TWI_packet.TWIErrorCode = TWI_ERROR_NO_ERRORS;
		}
	}

}
//------------------------------------------------------------------------------

/******************************************************************************/
//CAN
//------------------------------------------------------------------------------
/// CAN 0 Interrupt handler
//------------------------------------------------------------------------------
void CAN0_Handler(void)
{
	CAN_Handler(0);
}

//------------------------------------------------------------------------------
/// CAN 1 Interrupt handler
//------------------------------------------------------------------------------
#if defined AT91C_BASE_CAN1
void CAN1_Handler(void)
{
	CAN_Handler(1);
}
#endif

/******************************************************************************/
//------------------------------------------------------------------------------
// Функции работы со сторожевым таймером
//------------------------------------------------------------------------------
// Включение сторожевого таймера
//------------------------------------------------------------------------------
void WDT_Enable(void)
{
	AT91C_BASE_WDTC->WDTC_WDMR &= ~AT91C_WDTC_WDDIS;
}
//------------------------------------------------------------------------------
// Выключение сторожевого таймера
//------------------------------------------------------------------------------
void WDT_Disable(void)
{
	AT91C_BASE_WDTC->WDTC_WDMR |= AT91C_WDTC_WDDIS;
}
//------------------------------------------------------------------------------
// Обновление сторожевого таймера
//------------------------------------------------------------------------------
void WDT_Refresh(void)
{
	AT91C_BASE_WDTC->WDTC_WDCR = AT91C_WDTC_WDRSTT | WDT_KEYWORD;
}
//------------------------------------------------------------------------------
// Вычисление CRC
//------------------------------------------------------------------------------
/*
 Name  : CRC-16 CCITT
 Poly  : 0x1021    x^16 + x^12 + x^5 + 1
 Init  : 0xFFFF
 Revert: false
 XorOut: 0x0000
 Check : 0x29B1 ("123456789")
 MaxLen: 4095 байт (32767 бит) - обнаружение
 одинарных, двойных, тройных и всех нечетных ошибок
 */
uint16_t Crc16(uint8_t *Data, uint16_t len)
/*{
 unsigned char *p = data;
 unsigned short CRC = 0, buf;
 int i, j;

 for (i = 0; i < length; i++) {
 CRC ^= ((unsigned short)*p++ << 8);
 for (j = 0; j < 8; j++)   // полином: X16+X13+X12+X11+X10+X8+X6+X5+X2+1
 {
 buf = CRC; CRC <<= 1; if (buf & 0x8000) CRC ^= 0x3D65; }
 }
 return ~CRC;
 }*/

{
	uint32_t crc = *Data;
	len--;
//	unsigned char i;

	while (len--) {
		crc += *(++Data);

//        for (i = 0; i < 8; i++)
//            crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
	}

	return crc;
}

//------------------------------------------------------------------------------

/******************************************************************************/
