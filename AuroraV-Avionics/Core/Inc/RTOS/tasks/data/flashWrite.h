#ifndef _FLASHBUFFER_H
#define _FLASHBUFFER_H

#include "FreeRTOS.h"
#include "event_groups.h"

#include "flash.h"
#include "membuff.h"
#include "stateUpdate.h"
#include "stdbool.h"

void vIdle(void *pvParameters);
void vFlashBuffer(void *pvParameters);

typedef struct {
  enum State *currentState;
  MemBuff mem;
  // FreeRTOS objects
  EventGroupHandle_t xTaskEnableGroup;
} ctxIdle;

typedef struct {
  enum State *currentState;
  MemBuff mem;
  Flash flash;
  // FreeRTOS objects
  EventGroupHandle_t xTaskEnableGroup;
} ctxFlashBuffer;

#endif
