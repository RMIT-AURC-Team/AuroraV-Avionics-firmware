#ifndef _ACCEL_H
#define _ACCEL_H

#include "SPI/spi.h"
#include "sensors.h"
#include "stm32f439xx.h"

#define ACCEL_SCALE_HIGH         32
#define ACCEL_SCALE_LOW          16
#define ACCEL_SENSITIVITY_32G    (1.0f / 1024.0f)
#define ACCEL_SENSITIVITY_16G    (1.0f / 2048.0f)
#define ACCEL_SENSITIVITY(scale) ACCEL_SENSITIVITY_##scale##G
#define ACCEL_CNTL1              0x1B
#define ACCEL_CNTL1_PC1          0x80
#define ACCEL_CNTL1_RES          0x40
#define ACCEL_CNTL1_GSEL_32G     0x10
#define ACCEL_CNTL1_GSEL_16G     0x08
#define ACCEL_CNTL1_GSEL(scale)  ACCEL_CNTL1_GSEL_##scale##G
#define ACCEL_CNTL1_GSEL_8G      0x00
#define ACCEL_ODCNTL             0x21
#define ACCEL_ODCNTL_RESERVED    0x90

#define ACCEL_CS_1 GPIO_ODR_OD1
#define ACCEL_CS_2 GPIO_ODR_OD0

void Accel_init(Sensor *, unsigned long, int);
void Accel_write(Sensor *, uint8_t, uint8_t);
uint8_t Accel_read(Sensor *, uint8_t);

#endif
