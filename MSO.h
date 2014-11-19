/*
 * MSO.h
 *
 *  Created on: 18 нояб. 2014 г.
 *      Author: maximus
 */

#ifndef MSO_H_
#define MSO_H_
#include <stdint.h>
#include "constant.h"



typedef struct _MSO_Data
{
	uint32_t Mode;			// режим работы
	uint8_t Address;	// контрольная сумма
} MSO_Data;

extern MSO_Data MSO;


#endif /* MSO_H_ */
