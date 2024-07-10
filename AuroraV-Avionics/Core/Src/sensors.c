#include "sensors.h"

void configure_Sensors() {
  // Configure accelerometer 1
  write_ACCEL_1(ACCEL_CNTL1, ACCEL_CNTL1_RES | ACCEL_CNTL1_GSEL_32G);    // Accel select, 32g sensitivity
  uint8_t ODCNTL = read_ACCEL_1(ACCEL_ODCNTL);                           // Read from register for reserve mask
  write_ACCEL_1(ACCEL_ODCNTL, (ACCEL_ODCNTL_RESERVED & ODCNTL) | 0x2A);  // No filter, fast startup, 800Hz
  write_ACCEL_1(ACCEL_CNTL1, 0xD0);                                      // Enable PC1

  // Configure accelerometer 2
  write_ACCEL_2(ACCEL_CNTL1, ACCEL_CNTL1_RES | ACCEL_CNTL1_GSEL_32G);    // Accel select, 16g sensitivity
  ODCNTL = read_ACCEL_2(ACCEL_ODCNTL);                          				  // Read from register for reserve mask
  write_ACCEL_2(ACCEL_ODCNTL, (ACCEL_ODCNTL_RESERVED & ODCNTL) | 0x2A);  // No filter, fast startup, 800Hz
  write_ACCEL_2(ACCEL_CNTL1, 0xD0);                                      // Enable PC1

  // Configure gyroscope
  write_GYRO(GYRO_CTRL_REG1, GYRO_CTRL_REG1_ODR_800Hz | GYRO_CTRL_REG1_AXIS_ENABLE | GYRO_CTRL_REG1_PD_ENABLE);
	
  // Configure magnetometer
  write_MAG(MAGNET_CTRL_REG1, MAGNET_CTRL_REG1_FAST);
  write_MAG(MAGNET_CTRL_REG2, MAGNET_CTRL_REG2_FS16);

  // Configure barometer
  write_BARO(BARO_ODR_CFG, BARO_ODR_CFG_PWR | BARO_ODR_CFG_DEEP_DIS);
  uint8_t OSRCFG = read_BARO(BARO_OSR_CFG);
  write_BARO(BARO_OSR_CFG, (BARO_OSR_CFG_RESERVED * OSRCFG) | BARO_OSR_CFG_PRESS_EN);
}