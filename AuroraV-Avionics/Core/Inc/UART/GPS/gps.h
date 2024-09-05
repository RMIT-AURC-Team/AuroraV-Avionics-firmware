/**
 * @author Matt Ricci
 * @ingroup UART_API
 * @addtogroup GPS
 * @todo Add to UART group
 */

#ifndef _GPS_H
#define _GPS_H

#include "stm32f4xx.h"
#include "stdint.h"

/**
 * @addtogroup GPS
 * @{
 */

struct GPSData {
 char time[15];
 uint8_t hour;
 uint8_t minute;
 uint8_t second;
 char latitude[15];
 unsigned long latitude_num;
 char N_S[15];
 char longitude[15];
 unsigned long longitude_num;
 char E_W[15];
 char fix[15];
 char satellites[15];
 char hdop[15];
 char altitude[15];
 uint8_t lock; // 0 = no lock, 1 = lock
};

uint8_t GPS_byte();
void GPS_message(char *);
void DecodeGPS(char*, struct GPSData*);

/** @} */
#endif
