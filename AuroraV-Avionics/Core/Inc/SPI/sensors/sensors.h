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

extern const uint8_t ACCEL_AXES_1[3];
extern const uint8_t ACCEL_AXES_2[3];
extern const uint8_t GYRO_AXES[3];

void configure_SPI1_Sensor_Suite();

#endif
