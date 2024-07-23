#ifndef _SENSORS_H
#define _SENSORS_H

#include "stdarg.h"
#include "stm32f439xx.h"

#define XINDEX 0
#define YINDEX 1
#define ZINDEX 2

#define ROLL_INDEX  0
#define PITCH_INDEX 1
#define YAW_INDEX   2

#define ACCEL_SCALE_HIGH 32
#define ACCEL_SCALE_LOW  16
#define ACCEL_PORT_1     GPIOA
#define ACCEL_CS_1       GPIO_ODR_OD1
#define ACCEL_PORT_2     GPIOB
#define ACCEL_CS_2       GPIO_ODR_OD0

#define GYRO_PORT GPIOA
#define GYRO_CS   GPIO_ODR_OD2

#define BARO_PORT GPIOA
#define BARO_CS   GPIO_ODR_OD3

extern const uint8_t ACCEL_AXES_1[3];
extern const uint8_t ACCEL_AXES_2[3];
extern const uint8_t GYRO_AXES[3];

void configure_SPI1_Sensor_Suite();

#endif
