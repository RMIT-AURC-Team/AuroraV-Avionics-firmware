#ifndef _SPI_H
#define _SPI_H

#include "stm32f439xx.h"
#include <stdint.h>

#define ACCEL_PORT_1 GPIOA
#define ACCEL_CS_1   GPIO_ODR_OD1
#define ACCEL_PORT_2 GPIOB
#define ACCEL_CS_2   GPIO_ODR_OD0

#define GYRO_PORT GPIOA
#define GYRO_CS   GPIO_ODR_OD2

#define BARO_PORT GPIOA
#define BARO_CS   GPIO_ODR_OD3

#define FLASH_PORT GPIOE
#define FLASH_CS   GPIO_ODR_OD11

typedef enum {
  SENSOR_ACCEL,
  SENSOR_GYRO,
  SENSOR_BARO,
  MEMORY_FLASH
} DeviceType;

typedef struct SPI {
  DeviceType device;
  SPI_TypeDef *interface;
  GPIO_TypeDef *port;
  unsigned long cs; // Device CS address
  void (*send)(struct SPI *, uint16_t);
  void (*receive)(struct SPI *, uint16_t *);
  uint16_t (*transmit)(struct SPI *, uint16_t);
} SPI;

void SPI_init(SPI *, DeviceType, SPI_TypeDef *, GPIO_TypeDef *, unsigned long);
void SPI_send(SPI *, uint16_t);
void SPI_receive(SPI *, uint16_t *);
uint16_t SPI_transmit(SPI *, uint16_t);

#endif
