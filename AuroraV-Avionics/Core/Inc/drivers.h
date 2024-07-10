#ifndef _DRIVERS_H
#define _DRIVERS_H

#include "stm32f439xx.h"

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

void configure_RCC_APB1(void);
void configure_RCC_APB2(void);
void configure_RCC_AHB1(void);
void configure_MISC_GPIO(void);
void configure_UART3_GPS(void);
void configure_UART6_Serial_2_mini_USB(void);
void configure_external_interrupts(void);
void EXTI1_IRQHandler(void);

void TIM6init(void);
void TIM7init(void);
void buzzer(void);
void send_GPS_messege(char *pointerdata);

void DecodeGPS(char *GPS, struct GPSData *data);
void GPS_test(void);
#endif
