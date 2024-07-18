#ifndef _SENSORS_H
#define _SENSORS_H

#include "stdarg.h"
#include "stm32f439xx.h"

#define XINDEX 					 0
#define YINDEX 					 1
#define ZINDEX 					 2

#define ACCEL_SCALE_HIGH 32
#define ACCEL_SCALE_LOW  16
#define ACCEL_PORT_1     GPIOA
#define ACCEL_CS_1       GPIO_ODR_OD1
#define ACCEL_PORT_2     GPIOB
#define ACCEL_CS_2       GPIO_ODR_OD0
extern const uint8_t ACCEL_AXES_1[3];
extern const uint8_t ACCEL_AXES_2[3];

// TODO: Get rid of all this
#define SENSOR_MULTI_SENSITIVITIES_MAXN 3

typedef struct Sensor {
  unsigned long cs;
  float sensitivity;
  void (*write)(struct Sensor *, uint8_t, uint8_t);
  uint8_t (*read)(struct Sensor *, uint8_t);
} Sensor;

// Vararg struct type for sensors with multiple outputs with varying sensitivity
// (damn barometer)
typedef struct Sensor_multi {
  unsigned long cs;
  float sensitivities[SENSOR_MULTI_SENSITIVITIES_MAXN];
  void (*write)(struct Sensor_multi *, uint8_t, uint8_t);
  uint8_t (*read)(struct Sensor_multi *, uint8_t);
} Sensor_multi;

#define MAGNET_SENSITIVITY    (1.0f / 1711.0f)
#define MAGNET_CTRL_REG1      0x20
#define MAGNET_CTRL_REG1_FAST 0x02
#define MAGNET_CTRL_REG2      0x21
#define MAGNET_CTRL_REG2_FS16 0x60

void configure_SPI1_Sensor_Suite();
void configure_Sensors();

void write_MAG(uint8_t, uint8_t);
uint8_t read_MAG(uint8_t);

#endif
