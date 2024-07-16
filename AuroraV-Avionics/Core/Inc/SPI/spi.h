#ifndef _SPI_H
#define _SPI_H

#include <stdint.h>

typedef enum {
  SENSOR_ACCEL,
  SENSOR_GYRO,
  SENSOR_BARO
} DeviceType;

typedef struct SPI {
  DeviceType device;
  unsigned long cs;                                // Device CS address
  uint8_t (*readRegister)(void *, uint8_t);        // Read from 8-bit register
  void (*writeRegister)(void *, uint8_t, uint8_t); // Write to 8-bit register
} SPI;

#endif
