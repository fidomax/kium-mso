/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------
#include <boards/MSO_board.h>
#include <pio/pio_config.h>
#include <pio/pio.h>
#include <aic/aic.h>
//#include <stdlib.h>
#include "can.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
//------------------------------------------------------------------------------
//         Local definitions
//------------------------------------------------------------------------------
// CAN state
#define CAN_DISABLED       0
#define CAN_HALTED         1
#define CAN_IDLE           2
#define CAN_SENDING        3
#define CAN_RECEIVING      4

// MOT: Mailbox Object Type
#define CAN_MOT_DISABLE    0 // Mailbox is disabled
#define CAN_MOT_RECEPT     1 // Reception Mailbox
#define CAN_MOT_RECEPT_OW  2 // Reception mailbox with overwrite
#define CAN_MOT_TRANSMIT   3 // Transmit mailbox
#define CAN_MOT_CONSUMER   4 // Consumer mailbox
#define CAN_MOT_PRODUCER   5 // Producer mailbox
//------------------------------------------------------------------------------
#define BOARD_MCK               ((18432000.0 * 26 / 5) / 2)

#define PIN_CAN1_TRANSCEIVER_RS  {1<<30, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}

#define PIN_CAN2_TRANSCEIVER_RS  {1<<31, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}

#define PIN_CAN1_TRANSCEIVER_TXD  {1<<27, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
///// RXD: Receive data output
#define PIN_CAN1_TRANSCEIVER_RXD  {1<<26, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/// TXD: Transmit data input
#define PIN_CAN2_TRANSCEIVER_TXD  {1<<29, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
///// RXD: Receive data output
#define PIN_CAN2_TRANSCEIVER_RXD  {1<<28, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
///// TXD pins
#define PINS_CAN_TRANSCEIVER_TXD  PIN_CAN1_TRANSCEIVER_TXD, PIN_CAN2_TRANSCEIVER_TXD
///// RXD pins
#define PINS_CAN_TRANSCEIVER_RXD  PIN_CAN1_TRANSCEIVER_RXD, PIN_CAN2_TRANSCEIVER_RXD

#define PINS_CAN_TRANSCEIVER_RS   PIN_CAN1_TRANSCEIVER_RS,  PIN_CAN2_TRANSCEIVER_RS
//------------------------------------------------------------------------------

//         Local variables
//------------------------------------------------------------------------------
#if defined (PINS_CAN_TRANSCEIVER_TXD)
static const Pin pins_can_transceiver_txd[] = {PINS_CAN_TRANSCEIVER_TXD};
#endif
#if defined (PINS_CAN_TRANSCEIVER_RXD)
static const Pin pins_can_transceiver_rxd[] = {PINS_CAN_TRANSCEIVER_RXD};
#endif

static const Pin pins_can_transceiver_rs[] = {PINS_CAN_TRANSCEIVER_RS};
#if defined (PIN_CAN_TRANSCEIVER_RXEN)
static const Pin pin_can_transceiver_rxen = PIN_CAN_TRANSCEIVER_RXEN;
#endif

Message Actual_Message;
//static CanTransfer *pCAN0Transfer = NULL;
#ifdef AT91C_BASE_CAN1
//static CanTransfer *pCAN1Transfer = NULL;
#endif
//Message Actual_Message;
extern xQueueHandle xCanQueue;
//------------------------------------------------------------------------------

//         Local functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// CAN Error Detection
/// \param status     error type
/// \param can_number can nulber
//------------------------------------------------------------------------------
static void CAN_ErrorHandling(unsigned int status, unsigned char can_number)
{
	int flag;
	if ((status & AT91C_CAN_ERRA)== AT91C_CAN_ERRA) {
//        trace_LOG( trace_ERROR, "-E- (CAN) CAN is in active Error Active mode\n\r");
		flag = 0;
	} else
		if ((status & AT91C_CAN_ERRP)== AT91C_CAN_ERRP) {
//        trace_LOG( trace_ERROR, "-E- (CAN) CAN is in Error Passive mode\n\r");
			flag = 1;
		} else
			if ((status & AT91C_CAN_BOFF)== AT91C_CAN_BOFF) {
//        trace_LOG( trace_ERROR, "-E- (CAN) CAN is in Buff Off mode\n\r");
				flag = 2;
				// CAN reset
//        trace_LOG( trace_ERROR, "-E- (CAN) CAN%d reset\n\r", can_number);
				// CAN Controller Disable
				if (can_number == 0) {
					AT91C_BASE_CAN0 ->CAN_MR &= ~AT91C_CAN_CANEN;
					// CAN Controller Enable
					AT91C_BASE_CAN0 ->CAN_MR |= AT91C_CAN_CANEN;
				}

#ifdef AT91C_BASE_CAN1
				else
					if (can_number == 1) {
						AT91C_BASE_CAN1 ->CAN_MR &= ~AT91C_CAN_CANEN;
						// CAN Controller Enable
						AT91C_BASE_CAN1 ->CAN_MR |= AT91C_CAN_CANEN;
					}
#endif
			}
	
	// Error for Frame dataframe
	// CRC error
	if ((status & AT91C_CAN_CERR)== AT91C_CAN_CERR) {
//        trace_LOG( trace_ERROR, "-E- (CAN) CRC Error\n\r");
		flag = 3;
	}
	// Bit-stuffing error
	else
		if ((status & AT91C_CAN_SERR)== AT91C_CAN_SERR) {
//        trace_LOG( trace_ERROR, "-E- (CAN) Stuffing Error\n\r");
			flag = 4;
		}
		// Bit error
		else
			if ((status & AT91C_CAN_BERR)== AT91C_CAN_BERR) {
//        trace_LOG( trace_ERROR, "-E- (CAN) Bit Error\n\r");
				flag = 5;
			}
			// Form error
			else
				if ((status & AT91C_CAN_FERR)== AT91C_CAN_FERR) {
//        trace_LOG( trace_ERROR, "-E- (CAN) Form Error\n\r");
					flag = 6;
				}
				// Acknowledgment error
				else
					if ((status & AT91C_CAN_AERR)== AT91C_CAN_AERR) {
//        trace_LOG( trace_ERROR, "-E- (CAN) Acknowledgment Error\n\r");
						flag = 7;
					}
	
	// Error interrupt handler
	// Represent the current status of the CAN bus and are not latched.
	// See CAN, par. Error Interrupt Handler
	// AT91C_CAN_WARN
	// AT91C_CAN_ERRA
}

//------------------------------------------------------------------------------
// Generic CAN Interrupt handler
/// \param can_number can nulber
//------------------------------------------------------------------------------
void CAN_Handler(unsigned char can_number)
{
	AT91PS_CAN base_can;
	AT91PS_CAN_MB CAN_Mailbox;
	
	unsigned int status;
	unsigned int can_msr;
	unsigned int* pCan_mcr;
	unsigned int message_mode;
	unsigned char numMailbox;
	portBASE_TYPE xTaskWokenByPost = pdFALSE;
	
	if (can_number == 0) {
		base_can = AT91C_BASE_CAN0;
		CAN_Mailbox = AT91C_BASE_CAN0_MB0;
	} else {
		base_can = AT91C_BASE_CAN1;
		CAN_Mailbox = AT91C_BASE_CAN1_MB0;
	}
	status = (base_can->CAN_SR) & (base_can->CAN_IMR);
	base_can->CAN_IDR = status;
	
	if ((status & 0x0000FFFF) != 0) {

		// Handle Mailbox interrupts
		for (numMailbox = 0; numMailbox < NUM_MAILBOX_MAX; numMailbox++) {

			can_msr = *(unsigned int*)((unsigned int)CAN_Mailbox + (unsigned int)(0x10 + (0x20 * numMailbox)));
			if ((AT91C_CAN_MRDY & can_msr) == AT91C_CAN_MRDY) {
				// Mailbox object type
				message_mode = ((*(unsigned int*)((unsigned int)CAN_Mailbox + (unsigned int)(0x00 + (0x20 * numMailbox)))) >> 24) & 0x7;
				if (message_mode == 0) {
				} else {
					if ((message_mode == CAN_MOT_RECEPT) || (message_mode == CAN_MOT_RECEPT_OW) || (message_mode == CAN_MOT_PRODUCER)) {

						Actual_Message.data_low_reg = (*(unsigned int*)((unsigned int)CAN_Mailbox + (unsigned int)(0x14 + (0x20 * numMailbox))));
						Actual_Message.data_high_reg = (*(unsigned int*)((unsigned int)CAN_Mailbox + (unsigned int)(0x18 + (0x20 * numMailbox))));
						Actual_Message.real_Identifier = (*(unsigned int*)((unsigned int)CAN_Mailbox + (unsigned int)(0x08 + (0x20 * numMailbox))));
						Actual_Message.canID = can_number;
						xTaskWokenByPost = xQueueSendFromISR( xCanQueue, &Actual_Message, &xTaskWokenByPost );

						base_can->CAN_TCR = 1 << numMailbox;
						base_can->CAN_IER = 1 << numMailbox;

						// Message Data has been received
						pCan_mcr = (unsigned int*)((unsigned int)CAN_Mailbox + 0x1C + (0x20 * numMailbox));
						*pCan_mcr = AT91C_CAN_MTCR;

					} else {

					}
				}
			}
		}
	}

	if ((status & 0xFFCF0000) != 0) {
		CAN_ErrorHandling(status, can_number);
	}
	
	// Now the buffer is empty we can switch context if necessary.  Note that the
	// name of the yield function required is port specific.
	if (xTaskWokenByPost) {
		taskYIELD();
	}

}
//------------------------------------------------------------------------------
//         Global functions
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
/// Configure the corresponding mailbox
/// \param pTransfer can transfer structure
//------------------------------------------------------------------------------
void CAN_InitMailboxRegisters(CanTransfer *pTransfer)
{
	AT91PS_CAN_MB CAN_Mailbox;
	
	if (pTransfer->can_number == 0) {
		CAN_Mailbox = AT91C_BASE_CAN0_MB0;
	} else {
		CAN_Mailbox = AT91C_BASE_CAN1_MB0;
	}
	CAN_Mailbox = (AT91PS_CAN_MB)((unsigned int)CAN_Mailbox + (unsigned int)(0x20 * pTransfer->mailbox_number));
	
	pTransfer->mailbox_in_use |= 1 << (pTransfer->mailbox_number);
	// MailBox Control Register
	CAN_Mailbox->CAN_MB_MCR = 0x0;
	// MailBox Mode Register
	CAN_Mailbox->CAN_MB_MMR = 0x00;
	// CAN Message Acceptance Mask Register
	CAN_Mailbox->CAN_MB_MAM = pTransfer->acceptance_mask_reg;
	// MailBox ID Register
	// Disable the mailbox before writing to CAN_MIDx registers
	if (pTransfer->identifier != 0) {
		CAN_Mailbox->CAN_MB_MAM |= AT91C_CAN_MIDE;
		CAN_Mailbox->CAN_MB_MID = pTransfer->identifier;
	} else {
		CAN_Mailbox->CAN_MB_MAM &= ~AT91C_CAN_MIDE;
	}
	// MailBox Mode Register
	CAN_Mailbox->CAN_MB_MMR = pTransfer->mode_reg;
	// MailBox Data Low Register
	CAN_Mailbox->CAN_MB_MDL = pTransfer->data_low_reg;
	// MailBox Data High Register
	CAN_Mailbox->CAN_MB_MDH = pTransfer->data_high_reg;
	// MailBox Control Register
	CAN_Mailbox->CAN_MB_MCR = pTransfer->control_reg;
}

//------------------------------------------------------------------------------
/// Reset the MBx
//------------------------------------------------------------------------------
void CAN_ResetAllMailbox(void)
{
	unsigned char i;
	CanTransfer *pCAN0Transfer = NULL;
//	CAN_ResetTransfer(pCAN0Transfer);
	for (i = 0; i < NUM_MAILBOX_MAX; i++) {
		pCAN0Transfer->can_number = 0;
		pCAN0Transfer->mailbox_number = i;
		pCAN0Transfer->mode_reg = AT91C_CAN_MOT_DIS;
		pCAN0Transfer->acceptance_mask_reg = 0;
		pCAN0Transfer->identifier = 0;
		pCAN0Transfer->data_low_reg = 0x00000000;
		pCAN0Transfer->data_high_reg = 0x00000000;
		pCAN0Transfer->control_reg = 0x00000000;
		CAN_InitMailboxRegisters(pCAN0Transfer);
	}
	if (pCAN1Transfer != NULL) {
//		CAN_ResetTransfer(pCAN1Transfer);
		for (i = 0; i < NUM_MAILBOX_MAX; i++) {
			pCAN1Transfer->can_number = 1;
			pCAN1Transfer->mailbox_number = i;
			pCAN1Transfer->mode_reg = AT91C_CAN_MOT_DIS;
			pCAN1Transfer->acceptance_mask_reg = 0;
			pCAN1Transfer->identifier = 0;
			pCAN1Transfer->data_low_reg = 0x00000000;
			pCAN1Transfer->data_high_reg = 0x00000000;
			pCAN1Transfer->control_reg = 0x00000000;
			CAN_InitMailboxRegisters(pCAN1Transfer);
		}
	}
}

//------------------------------------------------------------------------------
/// CAN reset Transfer descriptor
/// \param pTransfer can transfer structure
//------------------------------------------------------------------------------
void CAN_ResetTransfer(CanTransfer *pTransfer)
{
	pTransfer->state = CAN_IDLE;
	pTransfer->can_number = 0;
	pTransfer->mailbox_number = 0;
	pTransfer->test_can = 0;
	pTransfer->mode_reg = 0;
	pTransfer->acceptance_mask_reg = 0;
	pTransfer->identifier = 0;
	pTransfer->data_low_reg = 0;
	pTransfer->data_high_reg = 0;
	pTransfer->control_reg = 0;
	pTransfer->mailbox_in_use = 0;
	pTransfer->size = 0;
}
//------------------------------------------------------------------------------
/// Wait for CAN synchronisation
/// \return return 1 for good initialisation, otherwise return 0
//------------------------------------------------------------------------------
static unsigned char CAN_Synchronisation(unsigned char can_number)
{
	unsigned int tick;
	AT91PS_CAN base_can;
	if (can_number == 0) {
		base_can = AT91C_BASE_CAN0;
	} else {
		base_can = AT91C_BASE_CAN1;
	}
	// CAN Controller Enable
	base_can->CAN_MR = AT91C_CAN_CANEN;
	// Enable Autobaud/Listen mode
	// dangerous, CAN not answer in this mode
	for (tick = 0;
			!(base_can->CAN_SR & AT91C_CAN_WAKEUP)&& (tick < AT91C_CAN_TIMEOUT);tick++)if (tick == AT91C_CAN_TIMEOUT) {
				return 0;
			}

	return 1;
}

//------------------------------------------------------------------------------
/// Write a CAN transfer
/// \param pTransfer can transfer structure
/// \return return CAN_STATUS_SUCCESS if command passed, otherwise
///         return CAN_STATUS_LOCKED
//------------------------------------------------------------------------------
unsigned char CAN_Write(CanTransfer *pTransfer)
{
	AT91PS_CAN base_can;
	
	if (pTransfer->state == CAN_RECEIVING) {
		pTransfer->state = CAN_IDLE;
	}
	
	if (pTransfer->state != CAN_IDLE) {
		return CAN_STATUS_LOCKED;
	}
	
//    trace_LOG( trace_DEBUG, "CAN_Write\n\r");
	pTransfer->state = CAN_SENDING;
	if (pTransfer->can_number == 0) {
		base_can = AT91C_BASE_CAN0;
	}
#ifdef AT91C_BASE_CAN1
	else {
		base_can = AT91C_BASE_CAN1;
	}
#endif
	base_can->CAN_TCR = pTransfer->mailbox_in_use;
	base_can->CAN_IER = pTransfer->mailbox_in_use;
	
	vTaskDelay(20);
	
	return CAN_STATUS_SUCCESS;
	
}

//------------------------------------------------------------------------------
/// Read a CAN transfer
/// \param pTransfer can transfer structure
/// \return return CAN_STATUS_SUCCESS if command passed, otherwise
///         return CAN_STATUS_LOCKED
//------------------------------------------------------------------------------
unsigned char CAN_Read(CanTransfer *pTransfer)
{
	AT91PS_CAN base_can;
	
	/*    if (pTransfer->state == CAN_RECEIVING)  {
	 pTransfer->state = CAN_IDLE;
	 }*/

	if (pTransfer->state != CAN_IDLE) {
		return CAN_STATUS_LOCKED;
	}
	
//    trace_LOG( trace_DEBUG, "CAN_Read\n\r");
	pTransfer->state = CAN_RECEIVING;
	
	if (pTransfer->can_number == 0) {
		base_can = AT91C_BASE_CAN0;
	}
#ifdef AT91C_BASE_CAN1
	else {
		base_can = AT91C_BASE_CAN1;
	}
#endif
	// enable interrupt
	base_can->CAN_IER = pTransfer->mailbox_in_use;
	
	return CAN_STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/// Test if CAN is in IDLE state
/// \param pTransfer can transfer structure
/// \return return 0 if CAN is in IDLE, otherwise return 1
//------------------------------------------------------------------------------
unsigned char CAN_IsInIdle(CanTransfer *pTransfer)
{
	return (pTransfer->state != CAN_IDLE);
}

//------------------------------------------------------------------------------
/// Disable CAN and enter in low power
//------------------------------------------------------------------------------
void CAN_disable(void)
{
	// Disable the interrupt on the interrupt controller
	AIC_DisableIT(AT91C_ID_CAN0);
	// disable all IT
	AT91C_BASE_CAN0 ->CAN_IDR = 0x1FFFFFFF;
#if defined AT91C_BASE_CAN1
	AIC_DisableIT(AT91C_ID_CAN1);
	// disable all IT
	AT91C_BASE_CAN1 ->CAN_IDR = 0x1FFFFFFF;
#endif
	
	// Enable Low Power mode
	AT91C_BASE_CAN0 ->CAN_MR |= AT91C_CAN_LPM;
	
	// Disable CANs Transceivers
	// Enter standby mode
	PIO_Set(&pins_can_transceiver_rs[0]);
	PIO_Set(&pins_can_transceiver_rs[1]);
#if defined (PIN_CAN_TRANSCEIVER_RXEN)
	// Enable ultra Low Power mode
	PIO_Clear(&pin_can_transceiver_rxen);
#endif
	
	// Disable clock for CAN PIO
	AT91C_BASE_PMC ->PMC_PCDR = (1 << AT91C_ID_PIOA);
	
	// Disable the CAN0 controller peripheral clock
	AT91C_BASE_PMC ->PMC_PCDR = (1 << AT91C_ID_CAN0);
	
}

//------------------------------------------------------------------------------
/// baudrate calcul
/// \param base_CAN CAN base address
/// \param baudrate Baudrate value
///                 allowed values: 1000, 800, 500, 250, 125, 50, 25, 10
/// \return return 1 in success, otherwise return 0
//------------------------------------------------------------------------------
unsigned char CAN_BaudRateCalculate(AT91PS_CAN base_CAN, unsigned int baudrate)
{
	unsigned int BRP;
	unsigned int PROPAG;
	unsigned int PHASE1;
	unsigned int PHASE2;
	unsigned int SJW;
	unsigned int t1t2;
	
	base_CAN->CAN_BR = 0;
	BRP = (BOARD_MCK / (baudrate * 1000 * 16)) - 1 + 0.5;
	// timing Delay:
	// Delay Bus Driver: 50 ns
	// Delay Receiver:   30 ns
	// Delay Bus Line:  110 ns
	if ((16 * baudrate * 2 * (50 + 30 + 110) / 1000000) >= 1) {
		PROPAG = (16 * baudrate * 2 * (50 + 30 + 110) / 1000000) - 1;
	} else {
		PROPAG = 0;
	}
	t1t2 = 15 - (PROPAG + 1);
	
	if ((t1t2 & 0x01) == 0x01) {
		// ODD
		PHASE1 = ((t1t2 - 1) / 2) - 1;
		PHASE2 = PHASE1 + 1;
	} else {
		// EVEN
		PHASE1 = (t1t2 / 2) - 1;
		PHASE2 = PHASE1;
	}
	
	if (1 > (4 / (PHASE1 + 1))) {
		SJW = 3;
	} else {
		SJW = PHASE1;
	}
	// Verif
	if (BRP == 0) {
		return 0;
	}
	if ((PROPAG + PHASE1 + PHASE2) != 12) {
		return 0;
	}
	base_CAN->CAN_BR = (AT91C_CAN_PHASE2 & (PHASE2 << 0)) + (AT91C_CAN_PHASE1 & (PHASE1 << 4)) + (AT91C_CAN_PROPAG & (PROPAG << 8)) + (AT91C_CAN_SYNC & (SJW << 12)) + (AT91C_CAN_BRP & (BRP << 16)) + (AT91C_CAN_SMP & (0 << 24));
	return 1;
	
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
/// Init of the CAN peripheral
/// \param baudrate Baudrate value
///                 allowed values: 1000, 800, 500, 250, 125, 50, 25, 10
/// \param canTransfer0 CAN0 structure transfer
/// \param canTransfer1 CAN1 structure transfer
/// \return return 1 if CAN has good baudrate and CAN is synchronized,
///         otherwise return 0
//------------------------------------------------------------------------------
unsigned char CAN_Init(unsigned int baudrate)
{
	
	// CAN Transmit Serial Data
#if defined (PINS_CAN_TRANSCEIVER_TXD)
	PIO_Configure(pins_can_transceiver_txd, PIO_LISTSIZE(pins_can_transceiver_txd));
#endif
#if defined (PINS_CAN_TRANSCEIVER_RXD)
	// CAN Receive Serial Data
	PIO_Configure(pins_can_transceiver_rxd, PIO_LISTSIZE(pins_can_transceiver_rxd));
#endif
	// CAN RS
#if defined (PINS_CAN_TRANSCEIVER_RS)
	PIO_Configure(pins_can_transceiver_rs, PIO_LISTSIZE(pins_can_transceiver_rs));
#endif
	
#if defined (PIN_CAN_TRANSCEIVER_RXEN)
	// CAN RXEN
	PIO_Configure(&pin_can_transceiver_rxen, PIO_LISTSIZE(pin_can_transceiver_rxen));
#endif
	
	// Enable clock for CAN PIO
	AT91C_BASE_PMC ->PMC_PCER = (1 << AT91C_ID_PIOA);
	
	// Enable the CAN0 controller peripheral clock
	AT91C_BASE_PMC ->PMC_PCER = (1 << AT91C_ID_CAN0);
	
	// disable all IT
	AT91C_BASE_CAN0 ->CAN_IDR = 0x1FFFFFFF;
	
	// Enable CANs Transceivers
#if defined (PIN_CAN_TRANSCEIVER_RXEN)
	// Disable ultra Low Power mode
	PIO_Set(&pin_can_transceiver_rxen);
#endif
	// Normal Mode (versus Standby mode)
#if defined (PINS_CAN_TRANSCEIVER_RS)
	PIO_Clear(&pins_can_transceiver_rs[0]);
	PIO_Clear(&pins_can_transceiver_rs[1]);
#endif
	
	// Enable the interrupt on the interrupt controller
	AIC_EnableIT(AT91C_ID_CAN0);
	
	if (CAN_BaudRateCalculate(AT91C_BASE_CAN0, baudrate) == 0) {
		// Baudrate problem
		return 0;
	}
	// Enable CAN1 Clocks
	AT91C_BASE_PMC ->PMC_PCER = (1 << AT91C_ID_CAN1);

	// disable all IT
	AT91C_BASE_CAN1 ->CAN_IDR = 0x1FFFFFFF;

	// Enable the interrupt on the interrupt controller
	AIC_EnableIT(AT91C_ID_CAN1);

	if (CAN_BaudRateCalculate(AT91C_BASE_CAN1, baudrate) == 0) {
		// Baudrate problem
		return 0;
	}

	// Reset all mailbox
	CAN_ResetAllMailbox();
	
	// Enable the interrupt with all error cases
	AT91C_BASE_CAN0 ->CAN_IER = AT91C_CAN_CERR  // (CAN) CRC Error
	| AT91C_CAN_SERR  // (CAN) Stuffing Error
	| AT91C_CAN_BERR  // (CAN) Bit Error
	| AT91C_CAN_FERR  // (CAN) Form Error
	| AT91C_CAN_AERR; // (CAN) Acknowledgment Error
	
	AT91C_BASE_CAN1 ->CAN_IER = AT91C_CAN_CERR  // (CAN) CRC Error
	| AT91C_CAN_SERR  // (CAN) Stuffing Error
	| AT91C_CAN_BERR  // (CAN) Bit Error
	| AT91C_CAN_FERR  // (CAN) Form Error
	| AT91C_CAN_AERR; // (CAN) Acknowledgment Error
	
	// Wait for CAN synchronisation
	if (CAN_Synchronisation(0) == 1) {
		if (CAN_Synchronisation(1) == 1) {
			return 1;
		}
	}
	
	return 0;
}

