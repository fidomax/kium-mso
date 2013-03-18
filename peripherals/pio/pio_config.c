#include "pio_config.h"

//*----------------------------------------------------------------------------
//* \fn    AT91F_PIO_CfgInput
//* \brief Enable PIO in input mode
//*----------------------------------------------------------------------------
void AT91F_PIO_CfgInput(AT91PS_PIO pPio,     // \arg pointer to a PIO controller
		unsigned int inputEnable)      // \arg PIO to be enabled
{
	// Disable output
	pPio->PIO_ODR = inputEnable;
	pPio->PIO_PER = inputEnable;
}
//*----------------------------------------------------------------------------
//* \fn    AT91F_PIO_CfgOutput
//* \brief Enable PIO in output mode
//*----------------------------------------------------------------------------
void AT91F_PIO_CfgOutput(AT91PS_PIO pPio,    // \arg pointer to a PIO controller
		unsigned int pioEnable)      // \arg PIO to be enabled
{
	pPio->PIO_PER = pioEnable; // Set in PIO mode
	pPio->PIO_OER = pioEnable; // Configure in Output
}
//*----------------------------------------------------------------------------
//* \fn    AT91F_PIO_SetOutput
//* \brief Set to 1 output PIO
//*----------------------------------------------------------------------------
void AT91F_PIO_SetOutput(AT91PS_PIO pPio,   // \arg  pointer to a PIO controller
		unsigned int flag) // \arg  output to be set
{
	pPio->PIO_SODR = flag;
}
//*----------------------------------------------------------------------------
//* \fn    AT91F_PIO_ClearOutput
//* \brief Set to 0 output PIO
//*----------------------------------------------------------------------------
void AT91F_PIO_ClearOutput(AT91PS_PIO pPio, // \arg  pointer to a PIO controller
		unsigned int flag) // \arg  output to be cleared
{
	pPio->PIO_CODR = flag;
}
//*----------------------------------------------------------------------------
//* \fn    AT91F_PIO_CfgPeriph
//* \brief Enable pins to be drived by peripheral
//*----------------------------------------------------------------------------
void AT91F_PIO_CfgPeriph(AT91PS_PIO pPio,    // \arg pointer to a PIO controller
		unsigned int periphAEnable,  // \arg PERIPH A to enable
		unsigned int periphBEnable)  // \arg PERIPH B to enable

{
	pPio->PIO_ASR = periphAEnable;
	pPio->PIO_BSR = periphBEnable;
	pPio->PIO_PDR = (periphAEnable | periphBEnable); // Set in Periph mode
}
//*----------------------------------------------------------------------------
//* \fn    AT91F_PIO_CfgOpendrain
//* \brief Configure PIO in open drain
//*----------------------------------------------------------------------------
void AT91F_PIO_CfgOpendrain(AT91PS_PIO pPio, // \arg pointer to a PIO controller
		unsigned int multiDrvEnable) // \arg pio to be configured in open drain
{
	// Configure the multi-drive option
	pPio->PIO_MDDR = ~multiDrvEnable;
	pPio->PIO_MDER = multiDrvEnable;
}
//----------------------------------------------------------

