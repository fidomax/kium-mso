///------------------------------------------------------------------------------
//         For CAN identifier
#define CAN_MIDE        (0x1 << 29) // (CAN_MB) Identifier Version
//------------------------------------------------------------------------------
//		   Приоритет по виду запрашиваемых данных
//------------------------------------------------------------------------------
#define identifier_TU    	((unsigned int) 8)
#define identifier_TR    	((unsigned int) 9)
#define identifier_TC	 	((unsigned int) 10)
#define identifier_TT	 	((unsigned int) 11)
#define identifier_Level 	((unsigned int) 15)
#define identifier_Coeff 	((unsigned int) 16)
#define identifier_ParamTT 	((unsigned int) 17)
#define identifier_ParamTC	((unsigned int) 18)
#define identifier_ParamMEZ ((unsigned int) 255)
//------------------------------------------------------------------------------
//		   Базовый приоритет
//------------------------------------------------------------------------------
#define priority_V			((unsigned int) 0x2)	// приоритетный запрос
#define priority_W			((unsigned int) 0x3)	// запись данных
#define priority_R			((unsigned int) 0x4)	// чтение данных
#define priority_N			((unsigned int) 0x7)	// без приоритета
//------------------------------------------------------------------------------
//		   Поле параметр
//------------------------------------------------------------------------------
#define ParamFV				((unsigned int) 0x0)	// параметр для физической величины
#define ParamWrite			((unsigned int)	0x0)
#define ParamTC				((unsigned int) 0x0)	// параметр для значения ТС
#define ParamMode			((unsigned int) 0x1)	// параметр для физической величины
#define ParamTime			((unsigned int)	0x2)	// время измерений
#define ParamMinD			((unsigned int)	0x3)	// минимум датчика 
#define ParamMaxD			((unsigned int)	0x4)	// максимум датчика
#define ParamMinFV			((unsigned int)	0x5)	// минимум ФВ
#define ParamMaxFV			((unsigned int)	0x6)	// максимум ФВ
//#define ParamFV				((unsigned int) 0x0)	// параметр для физической величины
//#define ParamTC				((unsigned int) 0x0)	// параметр для значения ТС
#define ParamKmin			((unsigned int) 0x1)	// Кмин
#define ParamKmax			((unsigned int)	0x2)	// Кмах
#define ParamPmin			((unsigned int)	0x3)	// Pmin
#define ParamPmax			((unsigned int)	0x4)	// Pmax
#define ParamCalib0			((unsigned int)	0x5)	// калибровка 0
#define ParamCalib20		((unsigned int)	0x6)	// калибровка 20
#define ParamCalibEnd		((unsigned int)	0x7)	// калибровка отменена
#define ParamMinPred		((unsigned int) 0x1)	// Предупредительный минимум
#define ParamMaxPred		((unsigned int) 0x2)	// Предупредительный максимум	
#define ParamMinAvar		((unsigned int) 0x3)	// Аварийный минимум	
#define ParamMaxAvar		((unsigned int) 0x4)	// Аварийный максимум
#define ParamSense			((unsigned int) 0x5)	// Чувствительность
//------------------------------------------------------------------------------
//		   Позиция в идентификаторе
//------------------------------------------------------------------------------
#define PosPriority			((unsigned int) 26)		// Позиция базового приоритета в ID
#define PosType				((unsigned int) 18)		// Позиция приоритета по типу запрашиваемых данных в ID
#define PosAdress			((unsigned int) 10)		// Позиция сетевого адреса МСО в ID	
#define PosChannel			((unsigned int) 4)		// Позиция номера канала
//------------------------------------------------------------------------------
//		   Маски для идентификатора
//------------------------------------------------------------------------------
#define MaskPriority		((unsigned int) 0b111)		// Маска для базового приоритета
#define MaskType			((unsigned int) 0xFF)		// Маска для приоритета по типу запрашиваемых данных в ID
#define MaskAdress			((unsigned int)	0xFF)		// Маска для сетевого адреса
#define MaskChannel			((unsigned int) 0x3F)		// Маска для номера канала
#define MaskParam			((unsigned int) 0b1111)		// Маска для параметра
//------------------------------------------------------------------------------
