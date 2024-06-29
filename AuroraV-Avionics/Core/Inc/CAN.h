#ifndef _CAN_H
#define _CAN_H

#include "stdint.h"

typedef struct CAN {
  unsigned int dataL;   // data high register
  unsigned int dataH;   // data low register
  unsigned int address; // CAN identifer
  uint8_t CAN_number;   // either CAN2 or CAN1

} CAN;

void CAN_configGPIO();
void CAN_configPeripheral();
uint8_t CAN_findEmptyMailboxTX(uint8_t);

// 0 transmission good,
// 1 transmission failed,
// 255 transmission timeout,
// 100 error in CAN selection,
// 250 no mailboxes available
uint8_t CAN_transmit(uint8_t, uint8_t, unsigned int, unsigned int, unsigned int);

// 0 nothing recived
// 1 packet recieved,
// 255 CAN_number not 2 or 1
uint8_t CAN_receive(CAN *);
#endif
