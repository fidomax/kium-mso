/*
 FreeRTOS V6.0.1 - Copyright (C) 2009 Real Time Engineers Ltd.

 ***************************************************************************
 *                                                                         *
 * If you are:                                                             *
 *                                                                         *
 *    + New to FreeRTOS,                                                   *
 *    + Wanting to learn FreeRTOS or multitasking in general quickly       *
 *    + Looking for basic training,                                        *
 *    + Wanting to improve your FreeRTOS skills and productivity           *
 *                                                                         *
 * then take a look at the FreeRTOS eBook                                  *
 *                                                                         *
 *        "Using the FreeRTOS Real Time Kernel - a Practical Guide"        *
 *                  http://www.FreeRTOS.org/Documentation                  *
 *                                                                         *
 * A pdf reference manual is also available.  Both are usually delivered   *
 * to your inbox within 20 minutes to two hours when purchased between 8am *
 * and 8pm GMT (although please allow up to 24 hours in case of            *
 * exceptional circumstances).  Thank you for your support!                *
 *                                                                         *
 ***************************************************************************

 This file is part of the FreeRTOS distribution.

 FreeRTOS is free software; you can redistribute it and/or modify it under
 the terms of the GNU General Public License (version 2) as published by the
 Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
 ***NOTE*** The exception to the GPL is included to allow you to distribute
 a combined work that includes FreeRTOS without being obliged to provide the
 source code for proprietary components outside of the FreeRTOS kernel.
 FreeRTOS is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 more details. You should have received a copy of the GNU General Public
 License and the FreeRTOS license exception along with FreeRTOS; if not it
 can be viewed here: http://www.freertos.org/a00114.html and also obtained
 by writing to Richard Barry, contact details for whom are available on the
 FreeRTOS WEB site.

 1 tab == 4 spaces!

 http://www.FreeRTOS.org - Documentation, latest information, license and
 contact details.

 http://www.SafeRTOS.com - A version that is certified for use in safety
 critical systems.

 http://www.OpenRTOS.com - Commercial support, development, porting,
 licensing and training services.
 */

/*
 NOTE : Tasks run in System mode and the scheduler runs in Supervisor mode.
 The processor MUST be in supervisor mode when vTaskStartScheduler is
 called.  The demo applications included in the FreeRTOS.org download switch
 to supervisor mode prior to main being called.  If you are not using one of
 these demo application projects then ensure Supervisor mode is used.
 */

/*
 * This demo includes a (basic) USB mouse driver and a WEB server.  It is
 * targeted for the AT91SAM7X EK prototyping board which includes a small
 * joystick to provide the mouse inputs.  The WEB interface provides some basic
 * interactivity through the use of a check box to turn on and off an LED.
 *
 * main() creates the WEB server, USB, and a set of the standard demo tasks
 * before starting the scheduler.  See the online FreeRTOS.org documentation
 * for more information on the standard demo tasks.
 *
 * LEDs D1 to D3 are controlled by the standard 'flash' tasks - each will
 * toggle at a different fixed frequency.
 *
 * A tick hook function is used to monitor the standard demo tasks - with LED
 * D4 being used to indicate the system status.  D4 toggling every 5 seconds
 * indicates that all the standard demo tasks are executing without error.  The
 * toggle rate increasing to 500ms is indicative of an error having been found
 * in at least one demo task.
 *
 * See the online documentation page that accompanies this demo for full setup
 * and usage information.
 */

/* Standard includes. */

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
/* Demo application includes. */
#include "partest.h"
#include "constant.h"

#include "BlockQ.h"
#include "blocktim.h"
#include "flash.h"
#include "QPeek.h"
#include "dynamic.h"
#include "peripherals/twi/twi.h"
#include "peripherals/PCA9532/PCA9532.h"
#include "peripherals/MSO_functions/MSO_functions.h"
#include "peripherals/can/can.h"
#include "peripherals/mezonin/mezonin.h"
#include "peripherals/spi/spi.h"
#include "peripherals/pio/pio_config.h"
#include "peripherals/aic/aic.h"

extern void TWI_ISR(void) __attribute__((naked));
extern void CAN0_ISR(void) __attribute__((naked));
extern void CAN1_ISR(void) __attribute__((naked));
//extern void PIT_ISR (void) __attribute__((naked));
//extern volatile TWI_TRANSFER  TWI_packet;

/* Priorities for the demo application tasks. */
#define mainUIP_PRIORITY					( tskIDLE_PRIORITY + 2 )
#define mainUSB_PRIORITY					( tskIDLE_PRIORITY + 2 )
#define mainBLOCK_Q_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainFLASH_PRIORITY                  ( tskIDLE_PRIORITY + 2 )
#define mainGEN_QUEUE_TASK_PRIORITY			( tskIDLE_PRIORITY )

#define mainUIP_TASK_STACK_SIZE_MAX		( configMINIMAL_STACK_SIZE * 10 )
#define mainUIP_TASK_STACK_SIZE_MED		( configMINIMAL_STACK_SIZE * 5 )
#define mainUIP_TASK_STACK_SIZE_MIN		( configMINIMAL_STACK_SIZE * 3 )

/* The LED toggle by the tick hook should an error have been found in a task. */
//#define mainERROR_LED						( 3 )
//------------------------------------------------------------------------------
//         For Page Mezonin
//------------------------------------------------------------------------------
#define PageParam	((unsigned int) 1)
#define PageCoeff	((unsigned int) 5)
#define PageLevel	((unsigned int) 9)

volatile char MSO_Addres; /* Состояние Switch8*/

xSemaphoreHandle xTWISemaphore;
xQueueHandle xCanQueue, xMezQueue, xMezTUQueue;

CanTransfer canTransfer0;
CanTransfer canTransfer1;
CanTransfer canTransfer_new1;
CanTransfer canTransfer_new0;

mezonin mezonin_my[4];

xTaskHandle xMezHandle;
TT_Value Mezonin_TT[4]; // массив для работы с ТТ

TC_Value Mezonin_TC[4]; // массив для работы с ТС

TU_Value Mezonin_TU[4]; // массив для работы с ТУ

int tt[100];
int c = 0;

unsigned int TY_state = 0x00;
unsigned int Prev_SPI_RDR_value = 0x00;
unsigned int Cur_SPI_RDR_value = 0x00;
/*-----------------------------------------------------------*/

static void prvSetupHardware(void);

void PCA9532_init(void)
{
	TWI_StartWrite(AT91C_BASE_TWI, (unsigned char)PCA9532_address, (PCA9532_PSC0 | PCA9532_AUTO_INCR), 1);
	//PSC0 с периодом 1 секунда
	TWI_WriteByte(AT91C_BASE_TWI, 0x97);
	Wait_THR_Ready();
	//PWM0 duty cycle 50%
	TWI_WriteByte(AT91C_BASE_TWI, 0x80);
	Wait_THR_Ready();
	//PSC1 с частотой 2 Гц
	TWI_WriteByte(AT91C_BASE_TWI, 0x4B);
	Wait_THR_Ready();
	//PWM1 duty cycle 50%
	TWI_WriteByte(AT91C_BASE_TWI, 0x80);
	Wait_THR_Ready();
	//Switch off red leds
	TWI_WriteByte(AT91C_BASE_TWI, (LED_OFF(0) | LED_OFF(1) | LED_OFF(2) | LED_OFF(3)));
	Wait_THR_Ready();
	//Switch off green leds
	TWI_WriteByte(AT91C_BASE_TWI, (LED_OFF(0) | LED_OFF(1) | LED_OFF(2) | LED_OFF(3)));
	Wait_THR_Ready();
	TWI_Stop(AT91C_BASE_TWI );
	Wait_TXCOMP();
	AT91C_BASE_TWI ->TWI_RHR;
	
}
//------------------------------------------//*//------------------------------------------------
void CanHandler(void *p)
{
	portTickType xLastWakeTime;
	unsigned int identifier;
	unsigned int Type, Priority, Channel_Num, Param;
	int a, canID;
	Mez_Value test23;
	Message Recieve_Message;
	
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	CAN_Init(1000, &canTransfer_new0, &canTransfer_new1);
	
	//Настройка приемного MailBox'а на прием по адресу платы
	identifier = AT91C_CAN_MIDE | /*priority_R | identifier_TC |*/MSO_Addres << PosAdress /*| 0 << 3*/;
	FillCanPacket(&canTransfer_new0, 1, 1, AT91C_CAN_MOT_RX, MaskAdress << PosAdress, identifier);
	FillCanPacket(&canTransfer_new1, 0, 1, AT91C_CAN_MOT_RX, MaskAdress << PosAdress, identifier);
	
	canTransfer_new0.data_low_reg = 0x00000000;
	canTransfer_new0.data_high_reg = 0x00000000;
	canTransfer_new0.control_reg = 0x00000000; // Mailbox Data Length Code
	canTransfer_new1.data_low_reg = 0x00000000;
	canTransfer_new1.data_high_reg = 0x00000000;
	canTransfer_new1.control_reg = 0x00000000; // Mailbox Data Length Code
	
	CAN_InitMailboxRegisters(&canTransfer_new0);
	CAN_InitMailboxRegisters(&canTransfer_new1);
	
	CAN_Read(&canTransfer_new0);
	CAN_Read(&canTransfer_new1);
	
	for (;;) {
		// Wait for the next cycle.
		if (xQueueReceive( xCanQueue, &Recieve_Message, portMAX_DELAY)) { // пришло CAN сообщение
		
			// разбор идентификатора
			identifier = Recieve_Message.real_Identifier;
			Type = identifier >> PosType & MaskType;
			Priority = identifier >> PosPriority & MaskPriority;
			Channel_Num = identifier >> PosChannel & MaskChannel;
			Param = identifier & MaskParam;
			
			switch (Priority) {
				case priority_V :

					Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Params.Mode = mode_calib;
					switch (Param) {
						case ParamCalib0 :
							Mez_TT_Calib(&mezonin_my[Channel_Num / 4], Channel_Num % 4, 1);
							break;
						case ParamCalib20 :
							Mez_TT_Calib(&mezonin_my[Channel_Num / 4], Channel_Num % 4, 2);
							break;
						case ParamWrite :
							Mez_TT_Calib(&mezonin_my[Channel_Num / 4], Channel_Num % 4, 3);
							Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Params.Mode = mode_ok;
							break;
						case ParamCalibEnd :
							Mez_TT_Calib(&mezonin_my[Channel_Num / 4], Channel_Num % 4, 4);
							Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Params.Mode = mode_ok;
							break;
					}
					
					break;
					
				case priority_W :
					switch (Type) {
						case identifier_TU :
							switch (Param) {
								case 0:
									//Mez_TT_Calib (&mezonin_my[Channel_Num/4], Channel_Num%4, 1);
									break;
								case 1:
									test23.ID = Channel_Num / 4;
									test23.Channel = Channel_Num % 4;
									test23.Value = Recieve_Message.data_low_reg;
									//Mezonin_TU[Channel_Num/4].Channel[Channel_Num%4].Value = Recieve_Message.data_low_reg;
									if (xMezTUQueue != 0) {
										if (xQueueSend( xMezTUQueue, &test23, ( portTickType ) 0 )) {
										}
									}
									break;
							}
							break;
							
						case identifier_Level :
							switch (Param) {
								case ParamWrite : {
									Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Levels.CRC = Crc16((unsigned char *)&(Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Levels), sizeof(TT_Level) - sizeof(unsigned short) - 2);
									
									if (xTWISemaphore != NULL) {
										if (xSemaphoreTake( xTWISemaphore, ( portTickType ) 10 ) == pdTRUE) {
											a = MEZ_memory_Write(Channel_Num / 4, Channel_Num % 4 + PageLevel, 0x00, sizeof(TT_Level), (unsigned char *)&Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Levels);
											vTaskDelayUntil(&xLastWakeTime, 10);
											xSemaphoreGive( xTWISemaphore);
										}
									}
								}
									break;
								case ParamMinPred :
									Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Levels.Min_P_Level = *((float *)&(Recieve_Message.data_low_reg));
									break;
								case ParamMaxPred :
									Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Levels.Max_P_Level = *((float *)&(Recieve_Message.data_low_reg));
									break;
								case ParamMinAvar :
									Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Levels.Min_A_Level = *((float *)&(Recieve_Message.data_low_reg));
									break;
								case ParamMaxAvar :
									Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Levels.Max_A_Level = *((float *)&(Recieve_Message.data_low_reg));
									break;
								case ParamSense :
									Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Levels.Sense = *((float *)&(Recieve_Message.data_low_reg));
									break;
							}
							break;
							
						case identifier_Coeff :
							switch (Param) {
								case ParamWrite : {
									Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Coeffs.CRC = Crc16((unsigned char *)&(Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Coeffs), sizeof(TT_Coeff) - sizeof(unsigned short) - 2);
									
									if (xTWISemaphore != NULL) {
										if (xSemaphoreTake( xTWISemaphore, ( portTickType ) 10 ) == pdTRUE) {
											a = MEZ_memory_Write(Channel_Num / 4, Channel_Num % 4 + PageCoeff, 0x00, sizeof(TT_Coeff), (unsigned char *)&Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Coeffs);
											vTaskDelayUntil(&xLastWakeTime, 10);
											xSemaphoreGive( xTWISemaphore);
										}
									}
								}
									break;
								case ParamKmin :
									Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Coeffs.k_min = *((float *)&(Recieve_Message.data_low_reg));
									break;
								case ParamKmax :
									Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Coeffs.k_max = *((float *)&(Recieve_Message.data_low_reg));
									break;
								case ParamPmin :
									Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Coeffs.p_min = *((float *)&(Recieve_Message.data_low_reg));
									break;
								case ParamPmax :
									Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Coeffs.p_max = *((float *)&(Recieve_Message.data_low_reg));
									break;
							}
							break;
							
						case identifier_ParamTT :
							switch (Param) {
								case ParamWrite : {
									Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Params.CRC = Crc16((unsigned char *)&(Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Params), sizeof(TT_Param) - sizeof(unsigned short) - 2);
									
									if (xTWISemaphore != NULL) {
										if (xSemaphoreTake( xTWISemaphore, ( portTickType ) 10 ) == pdTRUE) {
											a = MEZ_memory_Write(Channel_Num / 4, Channel_Num % 4 + PageParam, 0x00, sizeof(TT_Param), (unsigned char *)&Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Params);
											vTaskDelayUntil(&xLastWakeTime, 10);
											xSemaphoreGive( xTWISemaphore);
										}
									}
								}
									break;
								case ParamMode :
									Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Params.Mode = Recieve_Message.data_low_reg;
									break;
								case ParamTime :
									Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Params.MeasTime = Recieve_Message.data_low_reg;
									break;
								case ParamMinD :
									Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Params.MinD = *((float *)&(Recieve_Message.data_low_reg));
									break;
								case ParamMaxD :
									Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Params.MaxD = *((float *)&(Recieve_Message.data_low_reg));
									break;
								case ParamMinFV :
									Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Params.MinF = *((float *)&(Recieve_Message.data_low_reg));
									break;
								case ParamMaxFV :
									Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Params.MaxF = *((float *)&(Recieve_Message.data_low_reg));
									break;
							}
							break;
						case identifier_ParamTC :
							switch (Param) {
								case ParamWrite : // запись в ЕЕПРОМ!!!!!!!
									Mezonin_TC[Channel_Num / 4].Channel[Channel_Num % 4].Params.CRC = Crc16((unsigned char *)&(Mezonin_TC[Channel_Num / 4].Channel[Channel_Num % 4].Params), 4/*sizeof (TC_Param) - sizeof(unsigned short)*/);
									if (xTWISemaphore != NULL) {
										if (xSemaphoreTake( xTWISemaphore, ( portTickType ) 10 ) == pdTRUE) {
											a = MEZ_memory_Write(Channel_Num / 4, Channel_Num % 4 + 1, 0x00, sizeof(TC_Param), (unsigned char *)&Mezonin_TC[Channel_Num / 4].Channel[Channel_Num % 4].Params);
											vTaskDelayUntil(&xLastWakeTime, 10);
											xSemaphoreGive( xTWISemaphore);
										}
									}
									break;
								case ParamMode :
									Mezonin_TC[Channel_Num / 4].Channel[Channel_Num % 4].Params.Mode = Recieve_Message.data_low_reg;
									break;
							}
							break;
					}
					break;
					
				case priority_R : {
					identifier = AT91C_CAN_MIDE | priority_N << PosPriority | Type << PosType | MSO_Addres << PosAdress | Channel_Num << PosChannel | Param; // составление идентификатора для посылки
					canID = Recieve_Message.canID;	// номер CAN
					FillCanPacket(&canTransfer1, canID, 3, AT91C_CAN_MOT_TX | AT91C_CAN_PRIOR, 0x00000000, identifier); // Заполнение структуры CanTransfer с расширенным идентификатором
					
					switch (Type) {
						case identifier_TT :
							switch (Param) {
								case ParamFV :
									//Channel_Num = 14;
									*((float *)&(canTransfer1.data_low_reg)) = Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Value;
									//* ((float *) &(canTransfer0.data_low_reg)) = Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Value;
									canTransfer1.data_high_reg = Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].State;
									break;
									//case 1:
									//* ((int *) &(canTransfer0.data_low_reg)) = Mezonin_TT[Channel_Num/4].Channel[Channel_Num%4].Params.MeasTime;
									//* ((float *) &(canTransfer0.data_low_reg)) = Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Value;
									//canTransfer0.data_high_reg = 0;
									//break;
							}
							
							break;
						case identifier_TC :
							switch (Param) {
								case ParamTC :
									canTransfer1.data_low_reg = Mezonin_TC[Channel_Num / 4].Channel[Channel_Num % 4].Value;
									canTransfer1.data_high_reg = Mezonin_TC[Channel_Num / 4].Channel[Channel_Num % 4].State;
									break;
							}
							
							break;
							
						case identifier_ParamTT :
							switch (Param) {
								case ParamMode :
									*((int *)&(canTransfer1.data_low_reg)) = Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Params.Mode;
									canTransfer1.data_high_reg = 0;
									break;
								case ParamTime :
									*((int *)&(canTransfer1.data_low_reg)) = Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Params.MeasTime;
									canTransfer1.data_high_reg = 0;
									break;
								case ParamMinD :
									*((float *)&(canTransfer1.data_low_reg)) = Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Params.MinD;
									canTransfer1.data_high_reg = 0;
									break;
								case ParamMaxD :
									*((float *)&(canTransfer1.data_low_reg)) = Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Params.MaxD;
									canTransfer1.data_high_reg = 0;
									break;
								case ParamMinFV :
									*((float *)&(canTransfer1.data_low_reg)) = Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Params.MinF;
									canTransfer1.data_high_reg = 0;
									break;
								case ParamMaxFV :
									*((float *)&(canTransfer1.data_low_reg)) = Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Params.MaxF;
									canTransfer1.data_high_reg = 0;
									break;
							}
							break;
							
						case identifier_Level :
							switch (Param) {
								case ParamMinPred :
									*((float *)&(canTransfer1.data_low_reg)) = Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Levels.Min_P_Level;
									canTransfer1.data_high_reg = 0;
									break;
								case ParamMaxPred :
									*((float *)&(canTransfer1.data_low_reg)) = Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Levels.Max_P_Level;
									canTransfer1.data_high_reg = 0;
									break;
								case ParamMinAvar :
									*((float *)&(canTransfer1.data_low_reg)) = Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Levels.Min_A_Level;
									canTransfer1.data_high_reg = 0;
									break;
								case ParamMaxAvar :
									*((float *)&(canTransfer1.data_low_reg)) = Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Levels.Max_A_Level;
									canTransfer1.data_high_reg = 0;
									break;
								case ParamSense :
									*((float *)&(canTransfer1.data_low_reg)) = Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Levels.Sense;
									canTransfer1.data_high_reg = 0;
									break;
							}
							break;
							
						case identifier_Coeff :
							switch (Param) {
								case ParamKmin :
									*((float *)&(canTransfer1.data_low_reg)) = Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Coeffs.k_min;
									canTransfer1.data_high_reg = 0;
									break;
								case ParamKmax :
									*((float *)&(canTransfer1.data_low_reg)) = Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Coeffs.k_max;
									canTransfer1.data_high_reg = 0;
									break;
								case ParamPmin :
									*((float *)&(canTransfer1.data_low_reg)) = Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Coeffs.p_min;
									canTransfer1.data_high_reg = 0;
									break;
								case ParamPmax :
									*((float *)&(canTransfer1.data_low_reg)) = Mezonin_TT[Channel_Num / 4].Channel[Channel_Num % 4].Coeffs.p_max;
									canTransfer1.data_high_reg = 0;
									break;
							}
							break;
							
					}
					canTransfer1.control_reg = (AT91C_CAN_MDLC & (0x8 << 16)); // Mailbox Data Length Code
					CAN_InitMailboxRegisters(&canTransfer1);
					CAN_Write(&canTransfer1);
					CAN_ResetTransfer(&canTransfer1);
				}
					break;
				case priority_N :

					break;
			}
			
			vParTestToggleLED(0);
			CAN_ResetTransfer(&canTransfer_new1);
			CAN_ResetTransfer(&canTransfer_new0);
		}
	}
}
//------------------------------------------//*//------------------------------------------------
void LedBlinkTask(void *p)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 200;
	
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	
	for (;;) {
		// Wait for the next cycle.
		vParTestToggleLED(1);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		
		// Perform action here.
	}
	
}
//------------------------------------------//*//------------------------------------------------
void get_TY_state(void)
{
	
	Cur_SPI_RDR_value = (AT91C_BASE_SPI0 ->SPI_RDR & 0xFFFF);
	
	TY_state = Cur_SPI_RDR_value >> 1;
	
	TY_state |= Prev_SPI_RDR_value << 7;
	
	TY_state &= 0xFF;
	
	Prev_SPI_RDR_value = Cur_SPI_RDR_value;
}
//------------------------------------------//*//------------------------------------------------
void MyTask4(void *p)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 1000;
	xLastWakeTime = xTaskGetTickCount();
//	char rxValue;
	/*unsigned int SPI0_configuration, CS_configuration;

	 AT91F_PIO_CfgOutput(AT91C_BASE_PIOB, LED_0);

	 SPI_Pin_config ();

	 CS_configuration = AT91C_SPI_BITS_8 | AT91C_SPI_NCPHA | 0x30 << 8;

	 SPI0_configuration = AT91C_SPI_MSTR | AT91C_SPI_MODFDIS | AT91C_SPI_PS_FIXED | 0x00<<16;
	 SPI_Configure (AT91C_BASE_SPI0, AT91C_ID_SPI0, SPI0_configuration);
	 SPI_ConfigureNPCS (AT91C_BASE_SPI0, 0, CS_configuration);

	 SPI_Enable(AT91C_BASE_SPI0);
	 */
	//Mez_init(mezonin_my[i].Mez_Type,&mezonin_my[i]);
	Mez_TU_init(&mezonin_my[0]);
	
	SPI_Write(AT91C_BASE_SPI0, 0, 0x00);
	SPI_Write(AT91C_BASE_SPI0, 0, 0x00);
	
//    TWI_Write(mezonin_my[2].Mez_Mem_address, 0x00, &DataToWrite, 1, TWI_LED);
	for (;;) {
		SPI_Write(AT91C_BASE_SPI0, 0, 0xF0);
		//get_TY_state ();
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		
		SPI_Write(AT91C_BASE_SPI0, 0, 0x00);
		//get_TY_state ();
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		
		SPI_Write(AT91C_BASE_SPI0, 0, 0x10);
		//get_TY_state ();
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		
		SPI_Write(AT91C_BASE_SPI0, 0, 0x20);
		//get_TY_state ();
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		
		SPI_Write(AT91C_BASE_SPI0, 0, 0x40);
		//get_TY_state ();
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		
		SPI_Write(AT91C_BASE_SPI0, 0, 0x80);
		//get_TY_state ();
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}
//------------------------------------------//*//------------------------------------------------
void MezTT_WriteParam(void *p)
{
//	int Mez_1_type;
	portTickType xLastWakeTime;
	const portTickType xFrequency = 200;
	unsigned char Mez_Type;
//	unsigned char Value2;
//	unsigned char Period;
//	unsigned char * DataToWrite;
//	unsigned char    DataToWrite1 [28];
	int a, Channel_Num;
	
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	
//	    Mez_Num = p;
	
	Mez_Type = 0x03;
	
	Channel_Num = 0;  // номер канала, с которым идет работа в данный момент
	
	Mezonin_TT[0].Channel[Channel_Num].Params.MeasTime = 20;	//1-ый канал
	Mezonin_TT[0].Channel[Channel_Num].Params.Mode = 0;
	/*		Mezonin_TT[0].Channel[Channel_Num].Params.k_max = 1501.15;
	 Mezonin_TT[0].Channel[Channel_Num].Params.k_min = 233.75;
	 Mezonin_TT[0].Channel[Channel_Num].Params.p_max = -1497;
	 Mezonin_TT[0].Channel[Channel_Num].Params.p_min = -232;
	 Mezonin_TT[0].Channel[Channel_Num].Params.Sense = 1;
	 */Mezonin_TT[0].Channel[Channel_Num].Params.MinD = 0;
	Mezonin_TT[0].Channel[Channel_Num].Params.MaxD = 20;
	Mezonin_TT[0].Channel[Channel_Num].Params.MinF = 0;
	Mezonin_TT[0].Channel[Channel_Num].Params.MaxF = 20;
	
	/*		Mezonin_TT[0].PerTime = 200;

	 Period = Mezonin_TT[0].PerTime;*/

//	    a = MEZ_memory_Write (Mezonin_TT[0].ID, 0, 0, 1, &Period);
//	    for (i = 0; i < 4; i++)
//			{
	/*				Channel_Num = 0;  // номер канала, с которым идет работа в данный момент

	 Mezonin_TT[0].Channel[Channel_Num].Params.MeasTime = 20;
	 Mezonin_TT[0].Channel[Channel_Num].Params.PerTime = 200;
	 Mezonin_TT[0].Channel[Channel_Num].Params.k_max = 1499.275;
	 Mezonin_TT[0].Channel[Channel_Num].Params.k_min = 236.2625;
	 Mezonin_TT[0].Channel[Channel_Num].Params.p_max = -3.5;
	 Mezonin_TT[0].Channel[Channel_Num].Params.p_min = -1.25;*/
	/*				if (xSemaphore != NULL)
	 {
	 if (xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE)
	 {
	 a = MEZ_memory_Write (Mezonin_TT[0].ID, 0, 1, 1, &Period);
	 vTaskDelayUntil( &xLastWakeTime, 10);
	 xSemaphoreGive( xSemaphore );
	 }
	 }*/
// 			DataToWrite = (unsigned char *) &(Mezonin_TT[0].Channel[Channel_Num].Params); // формировка структуры для первого канала для записи в EEPROM
	Mezonin_TT[0].Channel[Channel_Num].Params.CRC = Crc16((unsigned char *)&(Mezonin_TT[0].Channel[Channel_Num].Params), sizeof(TT_Param) - sizeof(unsigned short));
	
	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, ( portTickType ) 10 ) == pdTRUE) {
			a = MEZ_memory_Write(0, Channel_Num + 1, 0x00, sizeof(TT_Param), (unsigned char *)&Mezonin_TT[0].Channel[Channel_Num].Params);
			vTaskDelayUntil(&xLastWakeTime, 10);
			if (a == -6) {
				a = MEZ_memory_Write(0, Channel_Num + 1, 0x00, sizeof(TT_Param), (unsigned char *)&Mezonin_TT[0].Channel[Channel_Num].Params);
				vTaskDelayUntil(&xLastWakeTime, 10);
			}
			xSemaphoreGive( xTWISemaphore);
		}
	}
	
	Channel_Num = 1;  // номер канала, с которым идет работа в данный момент
	
	Mezonin_TT[0].Channel[Channel_Num].Params.MeasTime = 20;
	Mezonin_TT[0].Channel[Channel_Num].Params.Mode = 0;
	/*    		    Mezonin_TT[0].Channel[Channel_Num].Params.k_max = 1502.325;
	 Mezonin_TT[0].Channel[Channel_Num].Params.k_min = 235.325;
	 Mezonin_TT[0].Channel[Channel_Num].Params.p_max = -1496.5;
	 Mezonin_TT[0].Channel[Channel_Num].Params.p_min = -233.5;
	 Mezonin_TT[0].Channel[Channel_Num].Params.Sense = 1;
	 */Mezonin_TT[0].Channel[Channel_Num].Params.MinD = 0;
	Mezonin_TT[0].Channel[Channel_Num].Params.MaxD = 20;
	Mezonin_TT[0].Channel[Channel_Num].Params.MinF = 0;
	Mezonin_TT[0].Channel[Channel_Num].Params.MaxF = 20;
	//Mezonin_TT[0].Channel[Channel_Num].Params.CRC = Crc16((unsigned char *) &Mezonin_TT[0].Channel[Channel_Num].Params, sizeof (TT_Param)-4);
	
//    			DataToWrite = (unsigned char *) &(Mezonin_TT[0].Channel[Channel_Num].Params); // формровка структуры для второго канала записи в EEPROM
	//memcpy(DataToWrite1, DataToWrite,28);
//    			DataToWrite1 = *DataToWrite;
	Mezonin_TT[0].Channel[Channel_Num].Params.CRC = Crc16((unsigned char *)&(Mezonin_TT[0].Channel[Channel_Num].Params), sizeof(TT_Param) - sizeof(unsigned short));
	
	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, ( portTickType ) 10 ) == pdTRUE) {
			a = MEZ_memory_Write(0, Channel_Num + 1, 0x00, sizeof(TT_Param), (unsigned char *)&Mezonin_TT[0].Channel[Channel_Num].Params);
			vTaskDelayUntil(&xLastWakeTime, 10);
			xSemaphoreGive( xTWISemaphore);
		}
	}
	
	Channel_Num = 2;  // номер канала, с которым идет работа в данный момент
	
	Mezonin_TT[0].Channel[Channel_Num].Params.MeasTime = 20;
	Mezonin_TT[0].Channel[Channel_Num].Params.Mode = 0;
	/*    		    Mezonin_TT[0].Channel[Channel_Num].Params.k_max = 1502.5375;
	 Mezonin_TT[0].Channel[Channel_Num].Params.k_min = 235.5125;
	 Mezonin_TT[0].Channel[Channel_Num].Params.p_max = -1494.75;
	 Mezonin_TT[0].Channel[Channel_Num].Params.p_min = -233.25;
	 Mezonin_TT[0].Channel[Channel_Num].Params.Sense = 1;
	 */Mezonin_TT[0].Channel[Channel_Num].Params.MinD = 0;
	Mezonin_TT[0].Channel[Channel_Num].Params.MaxD = 20;
	Mezonin_TT[0].Channel[Channel_Num].Params.MinF = 0;
	Mezonin_TT[0].Channel[Channel_Num].Params.MaxF = 20;
	//Mezonin_TT[0].Channel[Channel_Num].Params.CRC = Crc16((unsigned char *) &Mezonin_TT[0].Channel[Channel_Num].Params, sizeof (TT_Param)-4);
	
//    			DataToWrite = (unsigned char *) &(Mezonin_TT[0].Channel[Channel_Num].Params); // формровка структуры для третьего канала записи в EEPROM
	Mezonin_TT[0].Channel[Channel_Num].Params.CRC = Crc16((unsigned char *)&(Mezonin_TT[0].Channel[Channel_Num].Params), sizeof(TT_Param) - sizeof(unsigned short));
	
	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, ( portTickType ) 10 ) == pdTRUE) {
			a = MEZ_memory_Write(0, Channel_Num + 1, 0x00, sizeof(TT_Param), (unsigned char *)&Mezonin_TT[0].Channel[Channel_Num].Params);
			vTaskDelayUntil(&xLastWakeTime, 10);
			xSemaphoreGive( xTWISemaphore);
		}
	}
	
	Channel_Num = 3;  // номер канала, с которым идет работа в данный момент
	
	Mezonin_TT[0].Channel[Channel_Num].Params.MeasTime = 20;
	Mezonin_TT[0].Channel[Channel_Num].Params.Mode = 0;
	/*    		    Mezonin_TT[0].Channel[Channel_Num].Params.k_max = 1503.375;
	 Mezonin_TT[0].Channel[Channel_Num].Params.k_min = 237.25;
	 Mezonin_TT[0].Channel[Channel_Num].Params.p_max = -1492.5;
	 Mezonin_TT[0].Channel[Channel_Num].Params.p_min = -238;
	 Mezonin_TT[0].Channel[Channel_Num].Params.Sense = 1;
	 */Mezonin_TT[0].Channel[Channel_Num].Params.MinD = 0;
	Mezonin_TT[0].Channel[Channel_Num].Params.MaxD = 20;
	Mezonin_TT[0].Channel[Channel_Num].Params.MinF = 0;
	Mezonin_TT[0].Channel[Channel_Num].Params.MaxF = 20;
	//Mezonin_TT[0].Channel[Channel_Num].Params.CRC = Crc16((unsigned char *) &Mezonin_TT[0].Channel[Channel_Num].Params, sizeof (TT_Param)-4);
	
//    			DataToWrite = (unsigned char *) &(Mezonin_TT[0].Channel[Channel_Num].Params); // формровка структуры для третьего канала записи в EEPROM
	Mezonin_TT[0].Channel[Channel_Num].Params.CRC = Crc16((unsigned char *)&(Mezonin_TT[0].Channel[Channel_Num].Params), sizeof(TT_Param) - sizeof(unsigned short));
	
	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, ( portTickType ) 10 ) == pdTRUE) {
			a = MEZ_memory_Write(0, Channel_Num + 1, 0x00, sizeof(TT_Param), (unsigned char *)&Mezonin_TT[0].Channel[Channel_Num].Params);
			vTaskDelayUntil(&xLastWakeTime, 10);
			xSemaphoreGive( xTWISemaphore);
		}
	}
	
	/*    		    Channel_Num = 0;  // номер канала, с которым идет работа в данный момент*/ // было
	/*    			Mezonin_TT[1].PerTime = 200;

	 Period = Mezonin_TT[1].PerTime;*/

	/*    		    Mezonin_TT[1].Channel[Channel_Num].Params.MeasTime = 20;
	 Mezonin_TT[1].Channel[Channel_Num].Params.Mode = 0;
	 Mezonin_TT[1].Channel[Channel_Num].Params.k_max = 1498.1625;
	 Mezonin_TT[1].Channel[Channel_Num].Params.k_min = 235.6;
	 Mezonin_TT[1].Channel[Channel_Num].Params.p_max = 49.75;
	 Mezonin_TT[1].Channel[Channel_Num].Params.p_min = 8;
	 Mezonin_TT[1].Channel[Channel_Num].Params.Sense = 1;
	 Mezonin_TT[1].Channel[Channel_Num].Params.CRC = Crc16((unsigned char *) &Mezonin_TT[1].Channel[Channel_Num].Params, sizeof (TT_Param) - sizeof (unsigned short));
	 */// было
	/*				if (xSemaphore != NULL)
	 {
	 if (xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE)
	 {
	 a = MEZ_memory_Write (Mezonin_TT[1].ID, 0, 1, 1, &Period);
	 vTaskDelayUntil( &xLastWakeTime, 10);
	 xSemaphoreGive( xSemaphore );
	 }
	 }*/

//    			DataToWrite = (unsigned char *) &Mezonin_TT[1].Channel[Channel_Num].Params; // формровка структуры для третьего канала записи в EEPROM
	/*    			if (xSemaphore != NULL)
	 {
	 if (xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE)
	 {
	 a = MEZ_memory_Write(1 , Channel_Num + 1, 0x00, sizeof (TT_Param), (unsigned char *) &Mezonin_TT[0].Channel[Channel_Num].Params);
	 vTaskDelayUntil( &xLastWakeTime, 10);
	 xSemaphoreGive( xSemaphore );
	 }
	 }
	 Channel_Num = 1;  // номер канала, с которым идет работа в данный момент


	 Mezonin_TT[1].Channel[Channel_Num].Params.MeasTime = 20;
	 Mezonin_TT[1].Channel[Channel_Num].Params.Mode = 0;
	 Mezonin_TT[1].Channel[Channel_Num].Params.k_max = 1501.1;
	 Mezonin_TT[1].Channel[Channel_Num].Params.k_min = 237.25;
	 Mezonin_TT[1].Channel[Channel_Num].Params.p_max = -2;
	 Mezonin_TT[1].Channel[Channel_Num].Params.p_min = 5;
	 Mezonin_TT[1].Channel[Channel_Num].Params.Sense = 1;
	 Mezonin_TT[1].Channel[Channel_Num].Params.CRC = Crc16((unsigned char *) &Mezonin_TT[1].Channel[Channel_Num].Params, sizeof (TT_Param) - sizeof (unsigned short));
	 //    			DataToWrite = (unsigned char *) &Mezonin_TT[1].Channel[Channel_Num].Params; // формровка структуры для третьего канала записи в EEPROM

	 if (xSemaphore != NULL)
	 {
	 if (xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE)
	 {
	 a = MEZ_memory_Write(1 , Channel_Num + 1, 0x00, sizeof (TT_Param), (unsigned char *) &Mezonin_TT[0].Channel[Channel_Num].Params);
	 vTaskDelayUntil( &xLastWakeTime, 10);
	 xSemaphoreGive( xSemaphore );
	 }
	 }

	 Channel_Num = 2;  // номер канала, с которым идет работа в данный момент

	 Mezonin_TT[1].Channel[Channel_Num].Params.MeasTime = 25;
	 Mezonin_TT[1].Channel[Channel_Num].Params.Mode = 0;
	 Mezonin_TT[1].Channel[Channel_Num].Params.k_max = 1500.9625;
	 Mezonin_TT[1].Channel[Channel_Num].Params.k_min = 237.375;
	 Mezonin_TT[1].Channel[Channel_Num].Params.p_max = 49.75;
	 Mezonin_TT[1].Channel[Channel_Num].Params.p_min = 7.5;
	 Mezonin_TT[1].Channel[Channel_Num].Params.Sense = 1;
	 Mezonin_TT[1].Channel[Channel_Num].Params.CRC = Crc16((unsigned char *) &Mezonin_TT[1].Channel[Channel_Num].Params, sizeof (TT_Param) - sizeof (unsigned short));
	 //    			DataToWrite = (unsigned char *) &Mezonin_TT[1].Channel[Channel_Num].Params; // формровка структуры для третьего канала записи в EEPROM

	 if (xSemaphore != NULL)
	 {
	 if (xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE)
	 {
	 a = MEZ_memory_Write(1 , Channel_Num + 1, 0x00, sizeof (TT_Param), (unsigned char *) &Mezonin_TT[0].Channel[Channel_Num].Params);
	 vTaskDelayUntil( &xLastWakeTime, 10);
	 xSemaphoreGive( xSemaphore );
	 }
	 }

	 Channel_Num = 3;  // номер канала, с которым идет работа в данный момент

	 Mezonin_TT[1].Channel[Channel_Num].Params.MeasTime = 20;
	 Mezonin_TT[1].Channel[Channel_Num].Params.Mode = 0;
	 Mezonin_TT[1].Channel[Channel_Num].Params.k_max = 1502.9375;
	 Mezonin_TT[1].Channel[Channel_Num].Params.k_min = 239.2;
	 Mezonin_TT[1].Channel[Channel_Num].Params.p_max = 49.25;
	 Mezonin_TT[1].Channel[Channel_Num].Params.p_min = 8;
	 Mezonin_TT[1].Channel[Channel_Num].Params.Sense = 1;
	 Mezonin_TT[1].Channel[Channel_Num].Params.CRC = Crc16((unsigned char *) &Mezonin_TT[1].Channel[Channel_Num].Params, sizeof (TT_Param) - sizeof (unsigned short));
	 //    			DataToWrite = (unsigned char *) &Mezonin_TT[1].Channel[Channel_Num].Params; // формровка структуры для третьего канала записи в EEPROM

	 if (xSemaphore != NULL)
	 {
	 if (xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE)
	 {
	 a = MEZ_memory_Write(1 , Channel_Num + 1, 0x00, sizeof (TT_Param), (unsigned char *) &Mezonin_TT[0].Channel[Channel_Num].Params);
	 vTaskDelayUntil( &xLastWakeTime, 10);
	 xSemaphoreGive( xSemaphore );
	 }
	 }

	 Channel_Num = 0;  // номер канала, с которым идет работа в данный момент
	 */// было
	/*    			Mezonin_TT[2].PerTime = 200;

	 Period = Mezonin_TT[2].PerTime;*/

	/*    		    Mezonin_TT[2].Channel[Channel_Num].Params.MeasTime = 25;
	 Mezonin_TT[2].Channel[Channel_Num].Params.Mode = 0;
	 Mezonin_TT[2].Channel[Channel_Num].Params.k_max = 1501.7125;
	 Mezonin_TT[2].Channel[Channel_Num].Params.k_min = 235.8875;
	 Mezonin_TT[2].Channel[Channel_Num].Params.p_max = 87.75;
	 Mezonin_TT[2].Channel[Channel_Num].Params.p_min = 13.25;
	 Mezonin_TT[2].Channel[Channel_Num].Params.Sense = 1;
	 Mezonin_TT[2].Channel[Channel_Num].Params.CRC = Crc16((unsigned char *) &Mezonin_TT[2].Channel[Channel_Num].Params, sizeof (TT_Param) - sizeof (unsigned short));
	 */// было
	/*				if (xSemaphore != NULL)
	 {
	 if (xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE)
	 {
	 a = MEZ_memory_Write (Mezonin_TT[2].ID, 0, 1, 1, &Period);
	 vTaskDelayUntil( &xLastWakeTime, 10);
	 xSemaphoreGive( xSemaphore );
	 }
	 }*/

//    			DataToWrite = (unsigned char *) &Mezonin_TT[2].Channel[Channel_Num].Params; // формровка структуры для третьего канала записи в EEPROM
	/*    			if (xSemaphore != NULL)
	 {
	 if (xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE)
	 {
	 a = MEZ_memory_Write(2 , Channel_Num + 1, 0x00, sizeof (TT_Param), (unsigned char *) &Mezonin_TT[0].Channel[Channel_Num].Params);
	 vTaskDelayUntil( &xLastWakeTime, 10);
	 xSemaphoreGive( xSemaphore );
	 }
	 }

	 Channel_Num = 1;  // номер канала, с которым идет работа в данный момент

	 Mezonin_TT[2].Channel[Channel_Num].Params.MeasTime = 20;
	 Mezonin_TT[2].Channel[Channel_Num].Params.Mode = 0;
	 Mezonin_TT[2].Channel[Channel_Num].Params.k_max = 1502.3125;
	 Mezonin_TT[2].Channel[Channel_Num].Params.k_min = 237.3125;
	 Mezonin_TT[2].Channel[Channel_Num].Params.p_max = 92.75;
	 Mezonin_TT[2].Channel[Channel_Num].Params.p_min = 14.75;
	 Mezonin_TT[2].Channel[Channel_Num].Params.Sense = 1;
	 Mezonin_TT[2].Channel[Channel_Num].Params.CRC = Crc16((unsigned char *) &Mezonin_TT[2].Channel[Channel_Num].Params, sizeof (TT_Param) - sizeof (unsigned short));
	 //    			DataToWrite = (unsigned char *) &Mezonin_TT[2].Channel[Channel_Num].Params; // формровка структуры для третьего канала записи в EEPROM

	 if (xSemaphore != NULL)
	 {
	 if (xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE)
	 {
	 a = MEZ_memory_Write(2 , Channel_Num + 1, 0x00, sizeof (TT_Param), (unsigned char *) &Mezonin_TT[0].Channel[Channel_Num].Params);
	 vTaskDelayUntil( &xLastWakeTime, 10);
	 xSemaphoreGive( xSemaphore );
	 }
	 }

	 Channel_Num = 2;  // номер канала, с которым идет работа в данный момент

	 Mezonin_TT[2].Channel[Channel_Num].Params.MeasTime = 20;
	 Mezonin_TT[2].Channel[Channel_Num].Params.Mode = 0;
	 Mezonin_TT[2].Channel[Channel_Num].Params.k_max = 1502.3125;
	 Mezonin_TT[2].Channel[Channel_Num].Params.k_min = 237.3125;
	 Mezonin_TT[2].Channel[Channel_Num].Params.p_max = 92.75;
	 Mezonin_TT[2].Channel[Channel_Num].Params.p_min = 14.75;
	 Mezonin_TT[2].Channel[Channel_Num].Params.Sense = 1;
	 Mezonin_TT[2].Channel[Channel_Num].Params.CRC = Crc16((unsigned char *) &Mezonin_TT[2].Channel[Channel_Num].Params, sizeof (TT_Param) - sizeof (unsigned short));
	 //  			DataToWrite = (unsigned char *) &Mezonin_TT[2].Channel[Channel_Num].Params; // формровка структуры для третьего канала записи в EEPROM

	 if (xSemaphore != NULL)
	 {
	 if (xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE)
	 {
	 a = MEZ_memory_Write(2 , Channel_Num + 1, 0x00, sizeof (TT_Param), (unsigned char *) &Mezonin_TT[0].Channel[Channel_Num].Params);
	 vTaskDelayUntil( &xLastWakeTime, 10);
	 xSemaphoreGive( xSemaphore );
	 }
	 }

	 Channel_Num = 3;  // номер канала, с которым идет работа в данный момент

	 Mezonin_TT[2].Channel[Channel_Num].Params.MeasTime = 20;
	 Mezonin_TT[2].Channel[Channel_Num].Params.Mode = 0;
	 Mezonin_TT[2].Channel[Channel_Num].Params.k_max = 1502.3125;
	 Mezonin_TT[2].Channel[Channel_Num].Params.k_min = 237.3125;
	 Mezonin_TT[2].Channel[Channel_Num].Params.p_max = 92.75;
	 Mezonin_TT[2].Channel[Channel_Num].Params.p_min = 14.75;
	 Mezonin_TT[2].Channel[Channel_Num].Params.Sense = 1;
	 Mezonin_TT[2].Channel[Channel_Num].Params.CRC = Crc16((unsigned char *) &Mezonin_TT[2].Channel[Channel_Num].Params, sizeof (TT_Param) - sizeof (unsigned short));
	 //  			DataToWrite = (unsigned char *) &Mezonin_TT[2].Channel[Channel_Num].Params; // формровка структуры для третьего канала записи в EEPROM

	 if (xSemaphore != NULL)
	 {
	 if (xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE)
	 {
	 a = MEZ_memory_Write(2 , Channel_Num + 1, 0x00, sizeof (TT_Param), (unsigned char *) &Mezonin_TT[0].Channel[Channel_Num].Params);
	 vTaskDelayUntil( &xLastWakeTime, 10);
	 xSemaphoreGive( xSemaphore );
	 }
	 }

	 Channel_Num = 0;  // номер канала, с которым идет работа в данный момент


	 Mezonin_TT[3].Channel[Channel_Num].Params.MeasTime = 20;	//1-ый канал
	 Mezonin_TT[3].Channel[Channel_Num].Params.Mode = 0;
	 Mezonin_TT[3].Channel[Channel_Num].Params.k_max = 1499.6125;
	 Mezonin_TT[3].Channel[Channel_Num].Params.k_min = 236.25;
	 Mezonin_TT[3].Channel[Channel_Num].Params.p_max = -1492.25;
	 Mezonin_TT[3].Channel[Channel_Num].Params.p_min = -235;
	 Mezonin_TT[3].Channel[Channel_Num].Params.Sense = 1;
	 Mezonin_TT[3].Channel[Channel_Num].Params.CRC = Crc16((unsigned char *) &Mezonin_TT[3].Channel[Channel_Num].Params, sizeof (TT_Param) - sizeof (unsigned short));
	 */// было
	/*    			Mezonin_TT[3].PerTime = 200;

	 Period = Mezonin_TT[3].PerTime;*/

	/*    					if (xSemaphore != NULL)
	 {
	 if (xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE)
	 {
	 a = MEZ_memory_Write (Mezonin_TT[3].ID, 0, 1, 1, &Period);
	 vTaskDelayUntil( &xLastWakeTime, 10);
	 xSemaphoreGive( xSemaphore );
	 }
	 }*/
	//  					DataToWrite = (unsigned char *) &Mezonin_TT[3].Channel[Channel_Num].Params; // формировка структуры для первого канала для записи в EEPROM
	/*    					if (xSemaphore != NULL)
	 {
	 if (xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE)
	 {
	 a = MEZ_memory_Write(3 , Channel_Num + 1, 0x00, sizeof (TT_Param), (unsigned char *) &Mezonin_TT[0].Channel[Channel_Num].Params);
	 vTaskDelayUntil( &xLastWakeTime, 10);
	 xSemaphoreGive( xSemaphore );
	 }
	 }

	 Channel_Num = 1;  // номер канала, с которым идет работа в данный момент

	 Mezonin_TT[3].Channel[Channel_Num].Params.MeasTime = 20;
	 Mezonin_TT[3].Channel[Channel_Num].Params.Mode = 0;
	 Mezonin_TT[3].Channel[Channel_Num].Params.k_max = 1501.7625;
	 Mezonin_TT[3].Channel[Channel_Num].Params.k_min = 237.85;
	 Mezonin_TT[3].Channel[Channel_Num].Params.p_max = -1494.25;
	 Mezonin_TT[3].Channel[Channel_Num].Params.p_min = -238;
	 Mezonin_TT[3].Channel[Channel_Num].Params.Sense = 1;
	 Mezonin_TT[3].Channel[Channel_Num].Params.CRC = Crc16((unsigned char *) &Mezonin_TT[3].Channel[Channel_Num].Params, sizeof (TT_Param) - sizeof (unsigned short));
	 //  	    			DataToWrite = (unsigned char *) &Mezonin_TT[3].Channel[Channel_Num].Params; // формровка структуры для второго канала записи в EEPROM

	 if (xSemaphore != NULL)
	 {
	 if (xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE)
	 {
	 a = MEZ_memory_Write(3 , Channel_Num +1, 0x00, sizeof (TT_Param), (unsigned char *) &Mezonin_TT[0].Channel[Channel_Num].Params);
	 vTaskDelayUntil( &xLastWakeTime, 10);
	 xSemaphoreGive( xSemaphore );
	 }
	 }

	 Channel_Num = 2;  // номер канала, с которым идет работа в данный момент

	 Mezonin_TT[3].Channel[Channel_Num].Params.MeasTime = 20;
	 Mezonin_TT[3].Channel[Channel_Num].Params.Mode = 0;
	 Mezonin_TT[3].Channel[Channel_Num].Params.k_max = 1507.725;
	 Mezonin_TT[3].Channel[Channel_Num].Params.k_min = 247.5875;
	 Mezonin_TT[3].Channel[Channel_Num].Params.p_max = -1477.5;
	 Mezonin_TT[3].Channel[Channel_Num].Params.p_min = -242.75;
	 Mezonin_TT[3].Channel[Channel_Num].Params.Sense = 1;
	 Mezonin_TT[3].Channel[Channel_Num].Params.CRC = Crc16((unsigned char *) &Mezonin_TT[3].Channel[Channel_Num].Params, sizeof (TT_Param) - sizeof (unsigned short));
	 //  	    			DataToWrite = (unsigned char *) &Mezonin_TT[3].Channel[Channel_Num].Params; // формровка структуры для третьего канала записи в EEPROM

	 if (xSemaphore != NULL)
	 {
	 if (xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE)
	 {
	 a = MEZ_memory_Write(3 , Channel_Num +1, 0x00, sizeof (TT_Param), (unsigned char *) &Mezonin_TT[0].Channel[Channel_Num].Params);
	 vTaskDelayUntil( &xLastWakeTime, 10);
	 xSemaphoreGive( xSemaphore );
	 }
	 }

	 Channel_Num = 3;  // номер канала, с которым идет работа в данный момент

	 Mezonin_TT[3].Channel[Channel_Num].Params.MeasTime = 20;
	 Mezonin_TT[3].Channel[Channel_Num].Params.Mode = 0;
	 Mezonin_TT[3].Channel[Channel_Num].Params.k_max = 1507.925;
	 Mezonin_TT[3].Channel[Channel_Num].Params.k_min = 248.325;
	 Mezonin_TT[3].Channel[Channel_Num].Params.p_max = -1469.5;
	 Mezonin_TT[3].Channel[Channel_Num].Params.p_min = -242.5;
	 Mezonin_TT[3].Channel[Channel_Num].Params.Sense = 1;
	 Mezonin_TT[3].Channel[Channel_Num].Params.CRC = Crc16((unsigned char *) &Mezonin_TT[3].Channel[Channel_Num].Params, sizeof (TT_Param) - sizeof (unsigned short));
	 //  	    			DataToWrite = (unsigned char *) &Mezonin_TT[3].Channel[Channel_Num].Params; // формровка структуры для третьего канала записи в EEPROM

	 if (xSemaphore != NULL)
	 {
	 if (xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE)
	 {
	 a = MEZ_memory_Write(3 , Channel_Num + 1, 0x00, sizeof (TT_Param), (unsigned char *) &Mezonin_TT[0].Channel[Channel_Num].Params);
	 vTaskDelayUntil( &xLastWakeTime, 10);
	 xSemaphoreGive( xSemaphore );
	 }
	 }

	 //	    }

	 */
	for (;;) {
		/*			if (xSemaphore != NULL)
		 {
		 if (xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE)
		 {
		 for (i=0;i<3;i++)
		 //								DataToWrite1[i] = i+10;
		 a = MEZ_memory_Write(i , 0, 0x00, 1, &Mez_Type);
		 vTaskDelayUntil( &xLastWakeTime, 10);
		 xSemaphoreGive( xSemaphore );
		 }
		 }*/

		// Wait for the next cycle.
		/*            Value2 = Switch2_Read();
		 if (Value2 == 2)
		 {
		 //            	for (i=0;i<2;i++)
		 //					{
		 if (xSemaphore != NULL )
		 {
		 if( xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE )
		 {
		 //      			for (i=0;i<20;i++)
		 //      				DataToWrite[i] = i+10;
		 //            				i = 1;
		 for (i=0;i<4;i++)
		 {
		 //            				a = MEZ_memory_Write (Mezonin_TT[0].ID, 0, 0, 1, &Mez_Type);
		 a = MEZ_memory_Write(0 , i+1, 0x00, sizeof (TT_Param), DataToWrite);
		 vTaskDelayUntil( &xLastWakeTime, 10);

		 }
		 xSemaphoreGive( xSemaphore );
		 }
		 }
		 //					}
		 }
		 if (Value2 == 1)
		 {
		 //    		    Mez_handler_select (Mez_1_type, &mezonin_my[0]);
		 }*/
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		// Perform action here.
	}
	
}
//------------------------------------------//*//------------------------------------------------
//Задача записи порогов в EEPROM
//------------------------------------------//*//------------------------------------------------
void MezTT_WriteLevel(void *p)
{
//	int  Mez_1_type;
	portTickType xLastWakeTime;
	const portTickType xFrequency = 200;
//	unsigned char Mez_Type;
//	unsigned char * DataToWrite;
	int a, Channel_Num;
	
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	
//	    Mez_Num = p;
	
//	    Mez_Type = 0x03;
	
	Channel_Num = 0;  // номер канала, с которым идет работа в данный момент
	
	Mezonin_TT[0].Channel[Channel_Num].Levels.Min_P_Level = 20;	//1-ый канал
	Mezonin_TT[0].Channel[Channel_Num].Levels.Max_P_Level = 40;
	Mezonin_TT[0].Channel[Channel_Num].Levels.Min_A_Level = 15;
	Mezonin_TT[0].Channel[Channel_Num].Levels.Max_A_Level = 45;
	Mezonin_TT[0].Channel[Channel_Num].Levels.Sense = 1.0;
	
	Mezonin_TT[0].Channel[Channel_Num].Levels.CRC = Crc16((unsigned char *)&(Mezonin_TT[0].Channel[Channel_Num].Levels), sizeof(TT_Level) - sizeof(unsigned short));
	
	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, ( portTickType ) 10 ) == pdTRUE) {
			a = MEZ_memory_Write(0, Channel_Num + 9, 0x00, sizeof(TT_Level), (unsigned char *)&Mezonin_TT[0].Channel[Channel_Num].Levels);
			vTaskDelayUntil(&xLastWakeTime, 10);
			if (a == -6) {
				a = MEZ_memory_Write(0, Channel_Num + 9, 0x00, sizeof(TT_Level), (unsigned char *)&Mezonin_TT[0].Channel[Channel_Num].Levels);
				vTaskDelayUntil(&xLastWakeTime, 10);
			}
			xSemaphoreGive( xTWISemaphore);
		}
	}
	
	Channel_Num = 1;  // номер канала, с которым идет работа в данный момент
	
	Mezonin_TT[0].Channel[Channel_Num].Levels.Min_P_Level = 20;	//2-ый канал
	Mezonin_TT[0].Channel[Channel_Num].Levels.Max_P_Level = 40;
	Mezonin_TT[0].Channel[Channel_Num].Levels.Min_A_Level = 15;
	Mezonin_TT[0].Channel[Channel_Num].Levels.Max_A_Level = 45;
	Mezonin_TT[0].Channel[Channel_Num].Levels.Sense = 1.0;
	
	Mezonin_TT[0].Channel[Channel_Num].Levels.CRC = Crc16((unsigned char *)&(Mezonin_TT[0].Channel[Channel_Num].Levels), sizeof(TT_Level) - sizeof(unsigned short));
	
	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, ( portTickType ) 10 ) == pdTRUE) {
			a = MEZ_memory_Write(0, Channel_Num + 9, 0x00, sizeof(TT_Level), (unsigned char *)&Mezonin_TT[0].Channel[Channel_Num].Levels);
			vTaskDelayUntil(&xLastWakeTime, 10);
			xSemaphoreGive( xTWISemaphore);
		}
	}
	
	Channel_Num = 2;  // номер канала, с которым идет работа в данный момент
	
	Mezonin_TT[0].Channel[Channel_Num].Levels.Min_P_Level = 20;	//3-ый канал
	Mezonin_TT[0].Channel[Channel_Num].Levels.Max_P_Level = 40;
	Mezonin_TT[0].Channel[Channel_Num].Levels.Min_A_Level = 15;
	Mezonin_TT[0].Channel[Channel_Num].Levels.Max_A_Level = 45;
	Mezonin_TT[0].Channel[Channel_Num].Levels.Sense = 1.0;
	
	Mezonin_TT[0].Channel[Channel_Num].Levels.CRC = Crc16((unsigned char *)&(Mezonin_TT[0].Channel[Channel_Num].Levels), sizeof(TT_Level) - sizeof(unsigned short));
	
	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, ( portTickType ) 10 ) == pdTRUE) {
			a = MEZ_memory_Write(0, Channel_Num + 9, 0x00, sizeof(TT_Level), (unsigned char *)&Mezonin_TT[0].Channel[Channel_Num].Levels);
			vTaskDelayUntil(&xLastWakeTime, 10);
			xSemaphoreGive( xTWISemaphore);
		}
	}
	
	Channel_Num = 3;  // номер канала, с которым идет работа в данный момент
	
	Mezonin_TT[0].Channel[Channel_Num].Levels.Min_P_Level = 20;	//2-ый канал
	Mezonin_TT[0].Channel[Channel_Num].Levels.Max_P_Level = 40;
	Mezonin_TT[0].Channel[Channel_Num].Levels.Min_A_Level = 15;
	Mezonin_TT[0].Channel[Channel_Num].Levels.Max_A_Level = 45;
	Mezonin_TT[0].Channel[Channel_Num].Levels.Sense = 1.0;
	
	Mezonin_TT[0].Channel[Channel_Num].Levels.CRC = Crc16((unsigned char *)&(Mezonin_TT[0].Channel[Channel_Num].Levels), sizeof(TT_Level) - sizeof(unsigned short));
	
	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, ( portTickType ) 10 ) == pdTRUE) {
			a = MEZ_memory_Write(0, Channel_Num + 9, 0x00, sizeof(TT_Level), (unsigned char *)&Mezonin_TT[0].Channel[Channel_Num].Levels);
			vTaskDelayUntil(&xLastWakeTime, 10);
			xSemaphoreGive( xTWISemaphore);
		}
	}
	
	for (;;) {
		
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		// Perform action here.
	}
	
}

//------------------------------------------//*//------------------------------------------------
//Задача записи коэффициентов в EEPROM
//------------------------------------------//*//------------------------------------------------
void MezTT_WriteCoeff(void *p)
{
//	int  Mez_1_type;
	portTickType xLastWakeTime;
	const portTickType xFrequency = 200;
//	unsigned char Mez_Type;
	
//	unsigned char * DataToWrite;
	int a, Channel_Num;
	
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	
//	    Mez_Num = p;
	
//	    Mez_Type = 0x03;
	
	Channel_Num = 0;  // номер канала, с которым идет работа в данный момент
	
	Mezonin_TT[0].Channel[Channel_Num].Coeffs.k_max = 1501.15;	//1-ый канал
	Mezonin_TT[0].Channel[Channel_Num].Coeffs.k_min = 233.75;
	Mezonin_TT[0].Channel[Channel_Num].Coeffs.p_max = -1497;
	Mezonin_TT[0].Channel[Channel_Num].Coeffs.p_min = -232;
	
	Mezonin_TT[0].Channel[Channel_Num].Coeffs.CRC = Crc16((unsigned char *)&(Mezonin_TT[0].Channel[Channel_Num].Coeffs), sizeof(TT_Coeff) - sizeof(unsigned short));
	
	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, ( portTickType ) 10 ) == pdTRUE) {
			a = MEZ_memory_Write(0, Channel_Num + 5, 0x00, sizeof(TT_Coeff), (unsigned char *)&Mezonin_TT[0].Channel[Channel_Num].Coeffs);
			vTaskDelayUntil(&xLastWakeTime, 10);
			if (a == -6) {
				a = MEZ_memory_Write(0, Channel_Num + 5, 0x00, sizeof(TT_Coeff), (unsigned char *)&Mezonin_TT[0].Channel[Channel_Num].Coeffs);
				vTaskDelayUntil(&xLastWakeTime, 10);
			}
			xSemaphoreGive( xTWISemaphore);
		}
	}
	
	Channel_Num = 1;  // номер канала, с которым идет работа в данный момент
	
	Mezonin_TT[0].Channel[Channel_Num].Coeffs.k_max = 1502.325;
	Mezonin_TT[0].Channel[Channel_Num].Coeffs.k_min = 235.325;
	Mezonin_TT[0].Channel[Channel_Num].Coeffs.p_max = -1496.5;
	Mezonin_TT[0].Channel[Channel_Num].Coeffs.p_min = -233.5;
	
	Mezonin_TT[0].Channel[Channel_Num].Coeffs.CRC = Crc16((unsigned char *)&(Mezonin_TT[0].Channel[Channel_Num].Coeffs), sizeof(TT_Coeff) - sizeof(unsigned short));
	
	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, ( portTickType ) 10 ) == pdTRUE) {
			a = MEZ_memory_Write(0, Channel_Num + 5, 0x00, sizeof(TT_Coeff), (unsigned char *)&Mezonin_TT[0].Channel[Channel_Num].Coeffs);
			vTaskDelayUntil(&xLastWakeTime, 10);
			xSemaphoreGive( xTWISemaphore);
		}
	}
	
	Channel_Num = 2;  // номер канала, с которым идет работа в данный момент
	
	Mezonin_TT[0].Channel[Channel_Num].Coeffs.k_max = 1502.5375;
	Mezonin_TT[0].Channel[Channel_Num].Coeffs.k_min = 235.325;
	Mezonin_TT[0].Channel[Channel_Num].Coeffs.p_max = -1494.75;
	Mezonin_TT[0].Channel[Channel_Num].Coeffs.p_min = -233.25;
	
	Mezonin_TT[0].Channel[Channel_Num].Coeffs.CRC = Crc16((unsigned char *)&(Mezonin_TT[0].Channel[Channel_Num].Coeffs), sizeof(TT_Coeff) - sizeof(unsigned short));
	
	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, ( portTickType ) 10 ) == pdTRUE) {
			a = MEZ_memory_Write(0, Channel_Num + 5, 0x00, sizeof(TT_Coeff), (unsigned char *)&Mezonin_TT[0].Channel[Channel_Num].Coeffs);
			vTaskDelayUntil(&xLastWakeTime, 10);
			xSemaphoreGive( xTWISemaphore);
		}
	}
	
	Channel_Num = 3;  // номер канала, с которым идет работа в данный момент
	
	Mezonin_TT[0].Channel[Channel_Num].Coeffs.k_max = 1503.375;
	Mezonin_TT[0].Channel[Channel_Num].Coeffs.k_min = 235.325;
	Mezonin_TT[0].Channel[Channel_Num].Coeffs.p_max = -1492.5;
	Mezonin_TT[0].Channel[Channel_Num].Coeffs.p_min = -233.25;
	
	Mezonin_TT[0].Channel[Channel_Num].Coeffs.CRC = Crc16((unsigned char *)&(Mezonin_TT[0].Channel[Channel_Num].Coeffs), sizeof(TT_Coeff) - sizeof(unsigned short));
	
	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, ( portTickType ) 10 ) == pdTRUE) {
			a = MEZ_memory_Write(0, Channel_Num + 5, 0x00, sizeof(TT_Coeff), (unsigned char *)&Mezonin_TT[0].Channel[Channel_Num].Coeffs);
			vTaskDelayUntil(&xLastWakeTime, 10);
			xSemaphoreGive( xTWISemaphore);
		}
	}
	
	for (;;) {
		
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		// Perform action here.
	}
	
}
//------------------------------------------//*//------------------------------------------------
void Mez_TC_Task(void *p)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 200;
	int Mez_num;
	
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	
	Mez_num = (int)p;
	for (;;) {
		Mez_TC_handler(&mezonin_my[Mez_num]);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}
//------------------------------------------//*//------------------------------------------------
void Mez_TU_Task(void *p)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 200;
	int Mez_num;
	
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	
	Mez_num = (int)p;
	for (;;) {
		Mez_TU_handler(&mezonin_my[Mez_num]);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		// Perform action here.
	}
	
}
//------------------------------------------//*//------------------------------------------------
void Mez_TT_Task(void *p)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 200;
	int Mez_Num;
	xLastWakeTime = xTaskGetTickCount();
	Mez_Num = (int)p;
	for (;;) {
		Mez_TT_handler(&mezonin_my[Mez_Num]/*, &Mezonin_TT[Mez_num]*/);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	
}
//------------------------------------------//*//-----------------------------------------------
void Mez_TP_Task(void *p)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 200;
	int Mez_num;
	
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	
	Mez_num = (int)p;
	for (;;) {
		Mez_TP_Task(&mezonin_my[Mez_num]);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		// Perform action here.
	}
	
}
//------------------------------------------//*//------------------------------------------------
void Mez_TI_Task(void *p)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 200;
	int Mez_num;
	
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	Mez_num = (int)p;
	for (;;) {
		Mez_TI_Task(&mezonin_my[Mez_num]);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		// Perform action here.
	}
}
//------------------------------------------//*//------------------------------------------------
void MezValue(void *p)
{
//	portTickType xLastWakeTime;
//	const portTickType xFrequency = 200;
	
	unsigned int Count;
//    struct _Mez_Value	*rxValue;
//	int Dev_Num = 1;
	unsigned int identifier;
	Mez_Value Mez_V;
	int ChannelNumber;
//	int status;
	float tempTT, temp1TT;
	unsigned int ValueTC_temp, StateTC_temp/*,temp1,temp2*/;
	int n, i;
//    CANMessage m;
	
//    vTaskDelay(1000);
	
//    xLastWakeTime = xTaskGetTickCount();
	
	CAN_Init(1000, &canTransfer0, &canTransfer1);
	
	for (;;) {
		if (xMezQueue != 0) {
			if (xQueueReceive( xMezQueue, &Mez_V, portMAX_DELAY )) {
				switch (mezonin_my[Mez_V.ID].Mez_Type) {
					case Mez_TC:
						ValueTC_temp = Mezonin_TC[Mez_V.ID].Channel[Mez_V.Channel].Value;
						StateTC_temp = Mezonin_TC[Mez_V.ID].Channel[Mez_V.Channel].State;
//    			    	temp1 = Mez_V.Value & 1;
//    			    	temp2 = Mez_V.Value >> 1;
						if ((ValueTC_temp != (Mez_V.Value & 1)) || (StateTC_temp != (Mez_V.Value >> 1))) // если ТС изменился
						{
							Mezonin_TC[Mez_V.ID].Channel[Mez_V.Channel].Value = Mez_V.Value & 1;
							Mezonin_TC[Mez_V.ID].Channel[Mez_V.Channel].State = Mez_V.Value >> 1; // доработать со сдвигами
							
							ChannelNumber = Mez_V.ID * 4 + Mez_V.Channel;
							identifier = AT91C_CAN_MIDE | priority_N << 26 | identifier_TC << 18 | MSO_Addres << 10 | ChannelNumber << 4 | ParamFV; // составление идентификатора для посылки
							
							n = 0;
							while (!n) {
								for (i = 12; i < 16; i++) {
									AT91PS_CAN_MB CAN_Mailbox = (AT91PS_CAN_MB)((unsigned int)AT91C_BASE_CAN0_MB0 + (unsigned int)(0x20 * i));
									if ((CAN_Mailbox->CAN_MB_MSR & AT91C_CAN_MRDY)|| ((CAN_Mailbox->CAN_MB_MMR & AT91C_CAN_MOT)==0))n=i;
									break;
								}
							}
							//tt[c]=n;
							//c++;
							// для поиска свободного мэйлбокса
							
							//FillCanPacket(&canTransfer1, 0, n, AT91C_CAN_MOT_TX | AT91C_CAN_PRIOR, 0x00000000, identifier); // Заполнение структуры CanTransfer с расширенным идентификатором
							FillCanPacket(&canTransfer1, 1, n, AT91C_CAN_MOT_TX | AT91C_CAN_PRIOR, 0x00000000, identifier); // Заполнение структуры CanTransfer с расширенным идентификатором
							
							canTransfer1.data_low_reg = Mezonin_TC[Mez_V.ID].Channel[Mez_V.Channel].Value;
							canTransfer1.data_high_reg = Mezonin_TC[Mez_V.ID].Channel[Mez_V.Channel].State;
							canTransfer1.control_reg = (AT91C_CAN_MDLC & (0x8 << 16)); // Mailbox Data Length Code
							CAN_InitMailboxRegisters(&canTransfer1);
							CAN_Write(&canTransfer1);
							/*    						m.cob_id=identifier;
							 m.len=8;
							 m.data_low_reg=canTransfer1.data_low_reg;
							 m.data_high_reg=canTransfer1.data_high_reg;
							 canSend(&m);*/

							/*    						status = CAN_Write (&canTransfer0);
							 if (status!=0)
							 {
							 vTaskDelay(20);
							 CAN_Write (&canTransfer0);
							 }*/

							CAN_ResetTransfer(&canTransfer1);
							
							FillCanPacket(&canTransfer0, 0, n, AT91C_CAN_MOT_TX | AT91C_CAN_PRIOR, 0x00000000, identifier); // Заполнение структуры CanTransfer с расширенным идентификатором
							
							canTransfer0.data_low_reg = Mezonin_TC[Mez_V.ID].Channel[Mez_V.Channel].Value;
							canTransfer0.data_high_reg = Mezonin_TC[Mez_V.ID].Channel[Mez_V.Channel].State;
							canTransfer0.control_reg = (AT91C_CAN_MDLC & (0x8 << 16)); // Mailbox Data Length Code
							CAN_InitMailboxRegisters(&canTransfer0);
							CAN_Write(&canTransfer0);
							CAN_ResetTransfer(&canTransfer0);
						}
						
//    			      Mez_TC_init (MezStruct);
						break;
						
					case Mez_TU:
//    			      Mez_TU_init (MezStruct);
						break;
						
					case Mez_TT:
						Count = Mez_V.Value;	// сколько намотал счетчик
						Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Value = Mez_TT_Frequency(Count, Mez_V.Channel, Mez_V.ID);
						
						// анализировать состояние
						if (Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Value > Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Levels.Max_P_Level || Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Value < Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Levels.Min_P_Level) {
							Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].State = 0x10; // предупредительный порог
							//else
							//Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].State = 0;	  // норма
							if (Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Value > Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Levels.Max_A_Level || Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Value < Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Levels.Min_A_Level)
								Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].State = 0x11; // аварийный порог
						} else
							Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].State = 0; // норма
							
						//if (Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Value > Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Max_Value ||
						//Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Value < Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Min_Value)
						//Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].State = 0x0A; // аварийный порог
						//else
						//Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].State = 0;	  // норма
						
						if (Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Params.Mode == 0x04)
							Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].State = 0x04; // выключен
							
						tempTT = Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Value - Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].OldValue;
						temp1TT = Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].OldValue - Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Value;
						
						if (tempTT > Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Levels.Sense || temp1TT > Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Levels.Sense || Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].State == 0x0A || Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].State == 0x10 || Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].State == 0x11)
//    			    	if (abs (Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Value - Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].OldValue) > Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Params.Sense) // если ТТ изменилась
						{
							Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].OldValue = Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Value;
							ChannelNumber = Mez_V.ID * 4 + Mez_V.Channel;
							identifier = AT91C_CAN_MIDE | priority_N << 26 | identifier_TT << 18 | MSO_Addres << 10 | ChannelNumber << 4 | ParamTC; // составление идентификатора для посылки
							
							n = 0;
							while (!n) {
								for (i = 12; i < 16; i++) {
									AT91PS_CAN_MB CAN_Mailbox = (AT91PS_CAN_MB)((unsigned int)AT91C_BASE_CAN0_MB0 + (unsigned int)(0x20 * i));
									if ((CAN_Mailbox->CAN_MB_MSR & AT91C_CAN_MRDY)|| ((CAN_Mailbox->CAN_MB_MMR & AT91C_CAN_MOT)==0))n=i;
									break;
								}
							}
							// для поиска свободного мэйлбокса
							//tt[c]=n;
							//c++;
							FillCanPacket(&canTransfer1, 1, n, AT91C_CAN_MOT_TX | AT91C_CAN_PRIOR, 0x00000000, identifier); // Заполнение структуры CanTransfer с расширенным идентификатором
							
							*((float *)&(canTransfer1.data_low_reg)) = Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Value;
							canTransfer1.data_high_reg = Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].State;
							canTransfer1.control_reg = (AT91C_CAN_MDLC & (0x8 << 16)); // Mailbox Data Length Code
							CAN_InitMailboxRegisters(&canTransfer1);
							CAN_Write(&canTransfer1);
							/*    						m.cob_id=identifier;
							 m.len=8;
							 m.data_low_reg=canTransfer1.data_low_reg;
							 m.data_high_reg=canTransfer1.data_high_reg;
							 canSend(&m);*/

							/*							status = CAN_Write (&canTransfer0);
							 if (status!=0)
							 {
							 vTaskDelay(20);
							 CAN_Write (&canTransfer0);
							 }*/

							CAN_ResetTransfer(&canTransfer1);
							
							FillCanPacket(&canTransfer0, 0, n, AT91C_CAN_MOT_TX | AT91C_CAN_PRIOR, 0x00000000, identifier); // Заполнение структуры CanTransfer с расширенным идентификатором
							*((float *)&(canTransfer0.data_low_reg)) = Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Value;
							canTransfer0.data_high_reg = Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].State;
							canTransfer0.control_reg = (AT91C_CAN_MDLC & (0x8 << 16)); // Mailbox Data Length Code
							CAN_InitMailboxRegisters(&canTransfer0);
							CAN_Write(&canTransfer0);
							CAN_ResetTransfer(&canTransfer0);
							
						}
						/*    			    	NewTT = Mez_TT_Frequency (Count , Mez_V.Channel + 1, Mez_V.ID); // настоящее значение физической величины

						 Value_temp = Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Value; // предыдущее значение
						 State_temp = Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].State; // предыдущее состояние

						 if ( abs(Value_temp - NewTT) > Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Params.Sense )*/

//    			    	Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].State = 1;
//    			    	Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Value = Mez_TT_Frequency (Count , Mez_V.Channel + 1, Mez_V.ID);
//    			    	Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Value = Mez_V.Value;  // получение значения физической величины из очереди
//    			    	Real_Value.Value = Mez_TT_Frequency (Channel_V , /*Mezonin_TT*/ChannelNumber, MEZ_ID); //
						break;
						
					case Mez_TP:
//    			      Mez_TP_init (MezStruct);
						break;
						
					case Mez_TI:
//    			      Mez_TI_init (MezStruct);
						break;
						
					case Mez_NOT:
						//    default:
//    			      Mez_NOT_init ();
						break;
				}
//		    	Mezonin_TT[Mez_V.ID].Channel[Mez_V.Channel].Value = Mez_V.Value;  // получение значения физической величины из очереди
				
//    			Mezonin_TT[0].Channel[3].Value = 10;
			}
		}
//    	vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}
//------------------------------------------//*//------------------------------------------------
void MezRec(void *p) // распознование типа мезонина
{
	unsigned char RedLeds = 0;
	unsigned char GreenLeds = 0;
	int i;
	signed char a;
	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, ( portTickType ) 10 ) == pdTRUE) {
			MSO_Addres = Switch8_Read(); // Определение адреса МСО
		}
		xSemaphoreGive(xTWISemaphore);
	}
	
	for (i = 0; i < 4; i++) {
		Mezonin_TU[i].ID = i;
		Mezonin_TT[i].ID = i;
		Mezonin_TC[i].ID = i;
		if (xTWISemaphore != NULL) {
			if (xSemaphoreTake( xTWISemaphore, ( portTickType ) 10 ) == pdTRUE) {
				mezonin_my[i].Mez_Type = Mez_Recognition_new(i); // Определение типа мезонина
				xSemaphoreGive(xTWISemaphore);
			}
		}
		Mez_init(mezonin_my[i].Mez_Type, &mezonin_my[i]);

		switch (i) {
			case 0:
				a = '0';
				break;
				
			case 1:
				a = '1';
				break;
				
			case 2:
				a = '2';
				break;
				
			case 3:
				a = '3';
				break;
		}
		
		switch (mezonin_my[i].Mez_Type) {
			case Mez_TC:
				GreenLeds |= LED_ON(i);
				if ((xTWISemaphore != NULL)) {
					if (xSemaphoreTake( xTWISemaphore, ( portTickType ) 10 ) == pdTRUE) {
						if (Get_TCParams(&Mezonin_TC[i]) == 0) {
							xTaskCreate( Mez_TC_Task, ( signed portCHAR * )"Mez_TC_Task" + a, mainUIP_TASK_STACK_SIZE_MED, (void *)i, mainUIP_PRIORITY, NULL);
						} else {
							RedLeds |= LED_PWM0(i);
						}
						xSemaphoreGive(xTWISemaphore);
					}
				}
				break;
				
			case Mez_TU:
				GreenLeds |= LED_ON(i);
				if ((xTWISemaphore != NULL)) {
					if (xSemaphoreTake( xTWISemaphore, ( portTickType ) 10 ) == pdTRUE) {
						if (Get_TUParams(&Mezonin_TU[i]) == 0) {
							xTaskCreate( Mez_TU_Task, ( signed portCHAR * )"Mez_TU" + a, mainUIP_TASK_STACK_SIZE_MIN, (void *)i, mainUIP_PRIORITY, NULL);
						} else {
							RedLeds |= LED_PWM0(i);
						}
						xSemaphoreGive(xTWISemaphore);
					}
				}
				break;
				
			case Mez_TT:
				GreenLeds |= LED_ON(i);
				if ((xTWISemaphore != NULL)/* && (mezonin_my[i].Mez_Type ==3)*/) {
					if (xSemaphoreTake( xTWISemaphore, ( portTickType ) 10 ) == pdTRUE) {
						if (Get_TTParams(&Mezonin_TT[i]) || Get_TTCoeffs(&Mezonin_TT[i]) || Get_TTLevels(&Mezonin_TT[i]) == 0) {
							xTaskCreate( Mez_TT_Task, ( signed portCHAR * )"MEZ_TT_Task" + a, mainUIP_TASK_STACK_SIZE_MED, (void *)i, mainUIP_PRIORITY, NULL);
						} else {
							RedLeds |= LED_PWM0(i);
						}
						xSemaphoreGive(xTWISemaphore);
					}
				}

				break;
				
			case Mez_TP:
				GreenLeds |= LED_ON(i);
				xTaskCreate( Mez_TP_Task, ( signed portCHAR * )"Mez_TP" + a, mainUIP_TASK_STACK_SIZE_MED, (void *)i, mainUIP_PRIORITY, NULL);
				break;
				
			case Mez_TI:
				GreenLeds |= LED_ON(i);
				xTaskCreate( Mez_TI_Task, ( signed portCHAR * )"MEZ_TI_Task" + a, mainUIP_TASK_STACK_SIZE_MED, (void *)i, mainUIP_PRIORITY, NULL);
				break;
				
			case Mez_NOT:
				GreenLeds |= LED_OFF(i);
				break;

			default:
				RedLeds |= LED_ON(i);
				break;
		}
	}
	if (xTWISemaphore != NULL) {
		if (xSemaphoreTake( xTWISemaphore, ( portTickType ) 10 ) == pdTRUE) {
			TWI_StartWrite(AT91C_BASE_TWI, (unsigned char)PCA9532_address, (PCA9532_LS0 | PCA9532_AUTO_INCR), 1);
			TWI_WriteByte(AT91C_BASE_TWI, RedLeds);
			Wait_THR_Ready();
			TWI_WriteByte(AT91C_BASE_TWI, GreenLeds);
			Wait_THR_Ready();
			TWI_Stop(AT91C_BASE_TWI );
			Wait_TXCOMP();
			AT91C_BASE_TWI ->TWI_RHR;
			xSemaphoreGive( xTWISemaphore);
		}
	}
	
	xTaskCreate( CanHandler, ( signed portCHAR * )"CanHandler", mainUIP_TASK_STACK_SIZE_MED, NULL, mainUIP_PRIORITY, NULL);
	vTaskDelete(*((xTaskHandle *)p));
	/*	if (xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE) {
	 if (Value != Mez_TT) {
	 Value = Mez_TT;
	 a = MEZ_memory_Write(0, 0x00, 0x00, 1, &Value); // присвоение мезонину типа ТT
	 }
	 xSemaphoreGive( xSemaphore);
	 }*/

}
//------------------------------------------//*//------------------------------------------------
/*
 * Starts all the other tasks, then starts the scheduler.
 */
int main(void)
{
	Mez_PreInit(&mezonin_my[0], &mezonin_my[1], &mezonin_my[2], &mezonin_my[3]);
	
	prvSetupHardware();
	vSemaphoreCreateBinary( xTWISemaphore);
	xCanQueue = xQueueCreate( 64, sizeof( Message ));
	xMezQueue = xQueueCreate(16, sizeof(Mez_Value));
	xMezTUQueue = xQueueCreate(16, sizeof(Mez_Value));

	xTaskCreate( LedBlinkTask, ( signed portCHAR * )"LedBlink", mainUIP_TASK_STACK_SIZE_MIN, 0, mainUIP_PRIORITY, NULL);
	xTaskCreate( MezRec, ( signed portCHAR * )"Recognition", mainUIP_TASK_STACK_SIZE_MED, &xMezHandle, mainUIP_PRIORITY, &xMezHandle);
	xTaskCreate( MezValue, ( signed portCHAR * )"Value", mainUIP_TASK_STACK_SIZE_MED, NULL, mainUIP_PRIORITY, NULL);


	/* Start the scheduler.

	 NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	 The processor MUST be in supervisor mode when vTaskStartScheduler is
	 called.  The demo applications included in the FreeRTOS.org download switch
	 to supervisor mode prior to main being called.  If you are not using one of
	 these demo application projects then ensure Supervisor mode is used here. */

	vTaskStartScheduler();
	
	/* We should never get here as control is now taken by the scheduler. */
	return 0;
}
/*-----------------------------------------------------------*/

static void prvSetupHardware(void)
{
	unsigned int i;
	portDISABLE_INTERRUPTS();
	
	/* When using the JTAG debugger the hardware is not always initialised to
	 the correct default state.  This line just ensures that this does not
	 cause all interrupts to be masked at the start. */

	AT91C_BASE_AIC ->AIC_EOICR = 0;
	
	/* Most setup is performed by the low level init function called from the
	 startup asm file. */

	/* Enable the peripheral clock. */

	AT91C_BASE_PMC ->PMC_PCER = 1 << AT91C_ID_PIOA;
	AT91C_BASE_PMC ->PMC_PCER = 1 << AT91C_ID_PIOB;
	AT91C_BASE_PMC ->PMC_PCER = 1 << AT91C_ID_TWI;
	
	AT91F_PIO_CfgOutput(AT91C_BASE_PIOB, WP );
	AT91F_PIO_SetOutput(AT91C_BASE_PIOB, WP );
	
	for (i = 0; i < 4; i++) {
		AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, (1 << (4 + i)));
		AT91F_PIO_SetOutput(AT91C_BASE_PIOA, (1 << (4 + i)));
	}
	
	TWI_Pin_config();
	TWI_Configure(AT91C_BASE_TWI, TWCK, MCK);
	AIC_ConfigureIT(AT91C_ID_TWI, 0, TWI_ISR);
	AIC_EnableIT(AT91C_ID_TWI);
	
	AIC_DisableIT(mezonin_my[0].TC_ID);	// запрет прерываний TimerCounter
	AIC_DisableIT(mezonin_my[1].TC_ID);	// запрет прерываний TimerCounter
	AIC_DisableIT(mezonin_my[2].TC_ID);	// запрет прерываний TimerCounter
	AIC_DisableIT(mezonin_my[3].TC_ID);	// запрет прерываний TimerCounter
	
	PCA9532_init(); // Моргалка передними лампочками

	AIC_ConfigureIT(AT91C_ID_CAN0, AT91C_AIC_PRIOR_HIGHEST, CAN0_ISR);
	AIC_ConfigureIT(AT91C_ID_CAN1, AT91C_AIC_PRIOR_HIGHEST, CAN1_ISR);
	
	vParTestInitialise();
}
/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
	static portBASE_TYPE xTaskWoken = pdFALSE;
	int i;
	unsigned int PWM = 0;
	for (i = 0; i < 4; i++) {
		if (mezonin_my[i].Start) {
			mezonin_my[i].Start = 0;
			PWM |= mezonin_my[i].PWM_ID; //подача синхросигнала 2FIN
		}
		
	}
	if (PWM) {
		AT91C_BASE_PWMC ->PWMC_ENA = PWM;
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
	if (PWM)
		AT91C_BASE_PWMC ->PWMC_DIS = PWM;
	
	for (i = 0; i < 4; i++) {
		if (mezonin_my[i].PWM_ID & PWM)
			xSemaphoreGiveFromISR( mezonin_my[i].xSemaphore, &xTaskWoken);
	}
}
