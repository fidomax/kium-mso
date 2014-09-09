#ifndef GLOBAL_H
#define GLOBAL_H
#include "FreeRTOS.h"
#include "peripherals/mezonin/mezonin.h"
extern volatile uint8_t MSO_Address;
extern mezonin mezonin_my[4];
//Message Actual_Message;
extern QueueHandle_t xCanQueue;
extern TT_Value Mezonin_TT[4];
extern TC_Value Mezonin_TC[4];
extern TU_Value Mezonin_TU[4];
extern TP_Value Mezonin_TP[4];

extern QueueHandle_t xMezQueue;
extern QueueHandle_t xMezTUQueue;
extern SemaphoreHandle_t xTWISemaphore;
#endif
