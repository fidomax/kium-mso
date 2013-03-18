#include "AT91SAM7A3.h"
//#define __inline inline
//#include "lib_AT91SAM7A3.h"

#define   MCK           47923200

//========================определения для TWI===========================
#define   I2CDATA        ((unsigned int) AT91C_PIO_PA0) // TWI DATA
#define   I2CCLOCK       ((unsigned int) AT91C_PIO_PA1) // TWI CLOCK
#define   I2CCSE_1       ((unsigned int) AT91C_PIO_PA4) // Chip Select 1
#define   I2CCSE_2       ((unsigned int) AT91C_PIO_PA5) // Chip Select 2
#define   I2CCSE_3       ((unsigned int) AT91C_PIO_PA6) // Chip Select 3
#define   I2CCSE_4       ((unsigned int) AT91C_PIO_PA7) // Chip Select 4
#define   TWCK           ((unsigned int)  100000      )  // TWI clock
//========================определения для SPI===========================
#define   MISO           ((unsigned int) AT91C_PIO_PA15) // SPI Master In Slave Out
#define   MOSI           ((unsigned int) AT91C_PIO_PA16) // SPI Master Out Slave In
#define   CLK            ((unsigned int) AT91C_PIO_PA17) // SPI Clock
#define   CS_1           ((unsigned int) AT91C_PIO_PA11) // SPI Chip Select 0
#define   CS_2           ((unsigned int) AT91C_PIO_PA12) // SPI Chip Select 1
#define   CS_3           ((unsigned int) AT91C_PIO_PA13) // SPI Chip Select 2
#define   CS_4           ((unsigned int) AT91C_PIO_PA14) // SPI Chip Select 3
//========================определения для CAN===========================
#define   CAN1RX         ((unsigned int) AT91C_PIO_PA26) // CAN0 Receive
#define   CAN1TX         ((unsigned int) AT91C_PIO_PA27) // CAN0 Transmit
#define   CAN1TXE        ((unsigned int) AT91C_PIO_PA30) // CAN0 Low power mode
#define   CAN2RX         ((unsigned int) AT91C_PIO_PA28) // CAN1 Receive
#define   CAN2TX         ((unsigned int) AT91C_PIO_PA29) // CAN1 Transmit
#define   CAN2TXE        ((unsigned int) AT91C_PIO_PA31) // CAN1 Low power mode
#define   Mailbox_Number ((int)          16)             // количество ящиков
//======================================================================
#define   FIN_1          ((unsigned int) AT91C_PIO_PA18) // PWM0
#define   FIN_2          ((unsigned int) AT91C_PIO_PA19) // PWM1
#define   FIN_3          ((unsigned int) AT91C_PIO_PA20) // PWM2
#define   FIN_4          ((unsigned int) AT91C_PIO_PA21) // PWM3
#define   TC24V1         ((unsigned int) AT91C_PIO_PB0) // Interrupt input (IRQ2)
#define   TC24V2         ((unsigned int) AT91C_PIO_PB1) // Interrupt input (IRQ3)
#define   WP             ((unsigned int) AT91C_PIO_PB4) // Write protect
#define   MDATA_1        ((unsigned int) AT91C_PIO_PB9) // Timer Counter 0 external clock input
#define   MDATA_2        ((unsigned int) AT91C_PIO_PB10) // Timer Counter 1 external clock input
#define   MDATA_3        ((unsigned int) AT91C_PIO_PB11) // Timer Counter 2 external clock input
#define   MDATA_4        ((unsigned int) AT91C_PIO_PA25) // Timer Counter 5 external clock input
#define   LED_0          ((unsigned int) AT91C_PIO_PB12)
#define   LED_1          ((unsigned int) AT91C_PIO_PB13)
#define   OVF_1          ((unsigned int) AT91C_PIO_PB14)
#define   OVF_2          ((unsigned int) AT91C_PIO_PB15)
#define   OVF_3          ((unsigned int) AT91C_PIO_PB16)
#define   OVF_4          ((unsigned int) AT91C_PIO_PB17)
#define   BRK_1          ((unsigned int) AT91C_PIO_PB18)
#define   BRK_2          ((unsigned int) AT91C_PIO_PB19)
#define   BRK_3          ((unsigned int) AT91C_PIO_PB20)
#define   BRK_4          ((unsigned int) AT91C_PIO_PB21)
#define   A1_0           ((unsigned int) AT91C_PIO_PB22)
#define   A1_1           ((unsigned int) AT91C_PIO_PB23)
#define   A2_0           ((unsigned int) AT91C_PIO_PB24)
#define   A2_1           ((unsigned int) AT91C_PIO_PB25)
#define   A3_0           ((unsigned int) AT91C_PIO_PB26)
#define   A3_1           ((unsigned int) AT91C_PIO_PB27)
#define   A4_0           ((unsigned int) AT91C_PIO_PB28)
#define   A4_1           ((unsigned int) AT91C_PIO_PB29)
//==============================================================================

