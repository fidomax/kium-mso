#include "PCA9532.h"
#include "twi/twi.h"

void PCA9532_WriteByte(unsigned int PCA9532_Register, unsigned char DataToWrite)
{
	TWI_StartWrite(AT91C_BASE_TWI, PCA9532_address, PCA9532_Register, 1);
	TWI_WriteByte(AT91C_BASE_TWI, DataToWrite);
	Wait_THR_Ready();
	TWI_Stop(AT91C_BASE_TWI );
	Wait_TXCOMP();
	AT91C_BASE_TWI ->TWI_RHR;
}
//------------------------------------------------------------------------------
void PCA9532_AddByte(unsigned int PCA9532_Register, unsigned char DataToAdd)
{
	unsigned char NewData;
	
	TWI_StartRead(AT91C_BASE_TWI, PCA9532_address, PCA9532_Register, 1);
	Wait_RHR_Ready();
	NewData = TWI_ReadByte(AT91C_BASE_TWI );
	TWI_Stop(AT91C_BASE_TWI );
	Wait_TXCOMP();
	AT91C_BASE_TWI ->TWI_RHR;
	NewData |= DataToAdd;
	PCA9532_WriteByte(PCA9532_Register, NewData);
}
//------------------------------------------------------------------------------
unsigned char PCA9532_ReadRegister(unsigned int PCA9532_Register)
{
	unsigned char RegisterValue;
	
	TWI_StartRead(AT91C_BASE_TWI, PCA9532_address, PCA9532_Register, 1);
	Wait_RHR_Ready();
	RegisterValue = TWI_ReadByte(AT91C_BASE_TWI );
	TWI_Stop(AT91C_BASE_TWI );
	Wait_TXCOMP();
	AT91C_BASE_TWI ->TWI_RHR;
	return RegisterValue;
}

