#ifndef PIO_CONFIG_H
#define PIO_CONFIG_H
#include "AT91SAM7A3.h"

//*----------------------------------------------------------------------------
//* \fn    AT91F_PIO_CfgInput
//* \brief Enable PIO in input mode
//*----------------------------------------------------------------------------
void AT91F_PIO_CfgInput(AT91PS_PIO pPio,     // \arg pointer to a PIO controller
		unsigned int inputEnable);      // \arg PIO to be enabled
//*----------------------------------------------------------------------------
//* \fn    AT91F_PIO_CfgOutput
//* \brief Enable PIO in output mode
//*----------------------------------------------------------------------------
void AT91F_PIO_CfgOutput(AT91PS_PIO pPio,    // \arg pointer to a PIO controller
		unsigned int pioEnable);      // \arg PIO to be enabled
//*----------------------------------------------------------------------------
//* \fn    AT91F_PIO_SetOutput
//* \brief Set to 1 output PIO
//*----------------------------------------------------------------------------
void AT91F_PIO_SetOutput(AT91PS_PIO pPio,   // \arg  pointer to a PIO controller
		unsigned int flag); // \arg  output to be set
//*----------------------------------------------------------------------------
//* \fn    AT91F_PIO_ClearOutput
//* \brief Set to 0 output PIO
//*----------------------------------------------------------------------------
void AT91F_PIO_ClearOutput(AT91PS_PIO pPio, // \arg  pointer to a PIO controller
		unsigned int flag); // \arg  output to be cleared
//*----------------------------------------------------------------------------
//* \fn    AT91F_PIO_CfgPeriph
//* \brief Enable pins to be drived by peripheral
//*----------------------------------------------------------------------------
void AT91F_PIO_CfgPeriph(AT91PS_PIO pPio,    // \arg pointer to a PIO controller
		unsigned int periphAEnable,  // \arg PERIPH A to enable
		unsigned int periphBEnable);  // \arg PERIPH B to enable
//*----------------------------------------------------------------------------
//* \fn    AT91F_PIO_CfgOpendrain
//* \brief Configure PIO in open drain
//*----------------------------------------------------------------------------
void AT91F_PIO_CfgOpendrain(AT91PS_PIO pPio, // \arg pointer to a PIO controller
		unsigned int multiDrvEnable); // \arg pio to be configured in open drain
//----------------------------------------------------------
#endif
