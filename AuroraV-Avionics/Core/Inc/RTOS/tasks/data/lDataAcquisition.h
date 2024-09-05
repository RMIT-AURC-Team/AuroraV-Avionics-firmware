#ifndef _LDATAACQUISITION_H
#define _LDATAACQUISITION_H

#include "FreeRTOS.h"
#include "event_groups.h"
#include "message_buffer.h"
#include "semphr.h"

#include "stdio.h"

#include "BMP581.h"
#include "KX134_1211.h"
#include "dataframe.h"
#include "kalmanfilter.h"
#include "membuff.h"
#include "sensors.h"
#include "stateUpdate.h"

#ifdef DUMMY
  #include "press.h"
#endif

void vLDataAcquisition(void *pvParameters);

typedef struct {
  ctxState state;
  MemBuff mem;
  // FreeRTOS objects
  EventGroupHandle_t xTaskEnableGroup;
  SemaphoreHandle_t xUsbMutex;
  MessageBufferHandle_t xUsbTxBuff;
  // Sensor objects
  BMP581 baro;
  KX134_1211 *accel;
} ctxLDataAcquisition;

#endif
