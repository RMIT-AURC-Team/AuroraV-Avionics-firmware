#ifndef __HDATAACQUISITION_H
#define __HDATAACQUISITION_H

#include "FreeRTOS.h"
#include "event_groups.h"
#include "groups.h"
#include "message_buffer.h"
#include "semphr.h"

#include "stdio.h"

#include "A3G4250D.h"
#include "KX134_1211.h"
#include "dataframe.h"
#include "membuff.h"
#include "sensors.h"
#include "stateUpdate.h"

#ifdef DUMMY
  #include "accelX.h"
  #include "accelY.h"
  #include "accelZ.h"
  #include "gyroX.h"
  #include "gyroY.h"
  #include "gyroZ.h"
#endif

void vHDataAcquisition(void *pvParameters);

typedef struct {
  ctxState state;
  MemBuff mem;
  // FreeRTOS objects
  EventGroupHandle_t xTaskEnableGroup;
  SemaphoreHandle_t xUsbMutex;
  MessageBufferHandle_t xUsbTxBuff;
  // Sensor objects
  A3G4250D gyro;
  KX134_1211 *hAccel;
  KX134_1211 *lAccel;
  KX134_1211 *accel;
} ctxHDataAcquisition;

#endif
