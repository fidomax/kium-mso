/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "AT91SAM7A3.h"
#include "../peripherals/MSO_functions/MSO_functions.h"

extern unsigned int overflow[4];		// массив значений перполнений счетчика TC

/* Constants required to handle interrupts. */
/*#define portTIMER_MATCH_ISR_BIT		( ( unsigned char ) 0x01 )
 #define portCLEAR_VIC_INTERRUPT		( ( unsigned long ) 0 )
 */
/* Constants required to handle critical sections. */
/*#define portNO_CRITICAL_NESTING		( ( unsigned long ) 0 )
 volatile unsigned long ulCriticalNesting = 9999UL;
 volatile unsigned long zzz = 0;
 */

/*-----------------------------------------------------------*/
void TWI_ISR(void) __attribute__((naked));
void TWI_ISR(void)
{
	portSAVE_CONTEXT()
	;
	TWI_Transfer_Handler();
	
	AT91C_BASE_AIC ->AIC_EOICR = 0;
	portRESTORE_CONTEXT()
	;
	
}

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
void CAN0_ISR(void) __attribute__((naked));
void CAN0_ISR(void)
{
	portSAVE_CONTEXT()
	;
	CAN0_Handler();
	
	AT91C_BASE_AIC ->AIC_EOICR = 0;
	portRESTORE_CONTEXT()
	;
	
}

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
void CAN1_ISR(void) __attribute__((naked));
void CAN1_ISR(void)
{
	portSAVE_CONTEXT()
	;
	CAN1_Handler();
	
	AT91C_BASE_AIC ->AIC_EOICR = 0;
	portRESTORE_CONTEXT()
	;
	
}

/*-----------------------------------------------------------*/
/*	void PIT_ISR (void) __attribute__((naked));
 void PIT_ISR (void)
 {
 if (PIT_GetStatus() == AT91C_PITC_PITS)
 {
 PIT_GetPIVR();
 Timer_Handler ();
 }
 }*/
/*-----------------------------------------------------------*/
//------------------------------------------------------------------------------
// обработка прерываний для всех мезонинов
//------------------------------------------------------------------------------
void Mez_1_TT_ISR_TC(void) __attribute__((naked));
void Mez_1_TT_ISR_TC(void)
{
	portSAVE_CONTEXT()
	;
	AT91C_BASE_TC0 ->TC_SR;
	overflow[0]++;
	AT91C_BASE_AIC ->AIC_EOICR = 0;
	portRESTORE_CONTEXT()
	;
}
//------------------------------------------------------------------------------
void Mez_2_TT_ISR_TC(void) __attribute__((naked));
void Mez_2_TT_ISR_TC(void)
{
	portSAVE_CONTEXT()
	;
	AT91C_BASE_TC1 ->TC_SR;
	overflow[1]++;
	AT91C_BASE_AIC ->AIC_EOICR = 0;
	portRESTORE_CONTEXT()
	;
	
}
//------------------------------------------------------------------------------
void Mez_3_TT_ISR_TC(void) __attribute__((naked));
void Mez_3_TT_ISR_TC(void)
{
	portSAVE_CONTEXT()
	;
	AT91C_BASE_TC2 ->TC_SR;
	overflow[2]++;
	AT91C_BASE_AIC ->AIC_EOICR = 0;
	portRESTORE_CONTEXT()
	;
	
}
//------------------------------------------------------------------------------
void Mez_4_TT_ISR_TC(void) __attribute__((naked));
void Mez_4_TT_ISR_TC(void)
{
	portSAVE_CONTEXT()
	;
	AT91C_BASE_TC5 ->TC_SR;
	overflow[3]++;
	AT91C_BASE_AIC ->AIC_EOICR = 0;
	portRESTORE_CONTEXT()
	;
	
}

