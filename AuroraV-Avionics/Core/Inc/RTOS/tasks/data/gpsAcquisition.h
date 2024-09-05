#ifndef _GPSTRANSMIT_H
#define _GPSTRANSMIT_H

#include "FreeRTOS.h"
#include "dataframe.h"
#include "event_groups.h"
#include "message_buffer.h"
#include "stateUpdate.h"

#include "stdio.h"

#include "gps.h"
#include "lora.h"

void vGpsTransmit(void *);

typedef struct {
  enum State *currentState;
} ctxGpsTransmit;

#endif
