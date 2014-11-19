/*
 * protocol.h
 *
 *  Created on: 18 нояб. 2014 г.
 *      Author: maximus
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_
#include "boards/MSO_board.h"
#include "MSO.h"
//         For CAN identifier

#define MAKE_CAN_ID(proirity, type, addr, channel, param) (AT91C_CAN_MIDE | proirity << 26| type << 18 | addr << 10 | channel << 4 | param)

#define MAKE_MSG_ID(proirity, type, channel, param) (MAKE_CAN_ID ( proirity, type, MSO.Address, channel, param))
#endif /* PROTOCOL_H_ */
