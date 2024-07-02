#ifndef BUFFER_H
#define BUFFER_H

#include "membuff.h"

extern MemBuff membuff;
extern uint8_t buffer_data[16];  // Adjust size as needed

void initBuff(void);

#endif 