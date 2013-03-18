#define   PCA9532_address     ((unsigned char)    0x60)    //PCA9532 slave address
//=========================Регистры==========================
#define   PCA9532_INPUT0      0x0     //Input register 0 (pins 0 to 7)
#define   PCA9532_INPUT1      0x1     //Input register 1 (pins 8 to 15)
#define   PCA9532_PSC0        0x2     //Frequensy prescaler 0
#define   PCA9532_PWM0        0x3     //PWM register 0
#define   PCA9532_PSC1        0x4     //Frequensy prescaler 1
#define   PCA9532_PWM1        0x5     //PWM register 1
#define   PCA9532_LS0         0x6     //LED 0-3 selector
#define   PCA9532_LS1         0x7     //LED 4-7 selector
#define   PCA9532_LS2         0x8     //LED 8-11 selector
#define   PCA9532_LS3         0x9     //LED 12-15 selector
#define   PCA9532_AUTO_INCR   0x10    //Auto-increment flag
//===================конфигурация диодов======================
//ledNumber - номер диода в регистре LSx - должен быть равен 0, 1, 2 или 3
#define   LED_OFF(ledNumber)      (0<<(ledNumber*2))  //Output is set low Hi-Z (LED off-default)
#define   LED_ON(ledNumber)       (1<<(ledNumber*2))  //Output is set low (LED on)
#define   LED_PWM0(ledNumber)     (2<<(ledNumber*2))  //Output blinks at PWM0 rate
#define   LED_PWM1(ledNumber)     (3<<(ledNumber*2))  //Output blinks at PWM1 rate
void PCA9532_WriteByte(unsigned int PCA9532_Register, unsigned char DataToWrite);

void PCA9532_AddByte(unsigned int PCA9532_Register, unsigned char DataToAdd);

unsigned char PCA9532_ReadRegister(unsigned int PCA9532_Register);
