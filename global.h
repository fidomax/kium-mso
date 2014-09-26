#ifndef GLOBAL_H
#define GLOBAL_H
#include "FreeRTOS.h"
#include "peripherals/mezonin/mezonin.h"
extern volatile uint8_t MSO_Address;
extern mezonin mezonin_my[4];
//Message Actual_Message;
extern QueueHandle_t xCanQueue;


extern SemaphoreHandle_t xTWISemaphore;

#endif
