#ifndef _SPI_H
#define _SPI_H

#include "stm32f439xx.h"
#include <stdint.h>

typedef enum {
  SENSOR_ACCEL,
  SENSOR_GYRO,
  SENSOR_BARO
} DeviceType;

typedef struct SPI {
  DeviceType device;
  SPI_TypeDef *interface;
  GPIO_TypeDef *port;
  unsigned long cs; // Device CS address
  void (*send)(struct SPI *, uint16_t);
  void (*receive)(struct SPI *, uint16_t *);
} SPI;

void SPI_init(SPI *, DeviceType, SPI_TypeDef *, GPIO_TypeDef *, unsigned long);
void SPI_send(SPI *, uint16_t);
void SPI_receive(SPI *, uint16_t *);

#endif
