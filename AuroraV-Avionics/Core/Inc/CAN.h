#ifndef _CAN_H
#define _CAN_H

#include "stdint.h"
#include "stm32f439xx.h"

typedef struct CAN {
  unsigned int dataL;   // data high register
  unsigned int dataH;   // data low register
  unsigned int address; // CAN identifer
  uint8_t CAN_number;   // either CAN2 or CAN1

} CAN;

void CAN_configGPIO();
void CAN_configPeripheral();
uint8_t CAN_findEmptyMailboxTX(uint8_t);
uint8_t CAN_transmit(uint8_t, uint8_t, unsigned int, unsigned int, unsigned int);
uint8_t CAN_receive(CAN *);
#endif
