#include "BMP581.h"

void BMP581_init(BMP581 *baro, GPIO_TypeDef *port, unsigned long cs, float tempSensitivity, float pressSensitivity) {
  baro->base.device        = SENSOR_BARO;
  baro->base.port          = port;
  baro->base.cs            = cs;
  baro->base.writeRegister = BMP581_writeRegister;
  baro->base.readRegister  = BMP581_readRegister;
  baro->tempSensitivity    = tempSensitivity;
  baro->pressSensitivity   = pressSensitivity;
  baro->readTemp           = BMP581_readTemp;
  baro->readRawTemp        = BMP581_readRawTemp;
  baro->processRawTemp     = BMP581_processRawTemp;
  baro->readPress          = BMP581_readPress;
  baro->readRawPress       = BMP581_readRawPress;
  baro->processRawPress    = BMP581_processRawPress;

  baro->base.writeRegister(baro, BMP581_ODR_CFG, BMP581_ODR_CFG_PWR | BMP581_ODR_CFG_DEEP_DIS);
  uint8_t OSRCFG = baro->base.readRegister(baro, BMP581_OSR_CFG);
  baro->base.writeRegister(baro, BMP581_OSR_CFG, (BMP581_OSR_CFG_RESERVED * OSRCFG) | BMP581_OSR_CFG_PRESS_EN);
}

/******************************** DEVICE METHODS ********************************/

/* ===============================================================================
 * READ_TEMP
 *  Read processed floating point temperature
 *
 * PARAMETERS
 *  - out: floating point temperature
 * =============================================================================== */
void BMP581_readTemp(BMP581 *baro, float *out) {
  uint8_t bytes[BMP581_DATA_TOTAL];
  baro->readRawTemp(baro, bytes);
  baro->processRawTemp(baro, bytes, out);
}

/* ===============================================================================
 * PROCESS_RAW_TEMP
 *  Process raw values to floating point temperature
 *
 * PARAMETERS
 *  - bytes: 3-byte array of raw temperature values in big-endian
 *  - out: floating point temperature
 * =============================================================================== */
void BMP581_processRawTemp(BMP581 *baro, uint8_t *bytes, float *out) {
  *out = baro->tempSensitivity * (int32_t)(((uint32_t)bytes[0] << 16) | ((uint32_t)bytes[1] << 8) | bytes[0]);
}

/* ===============================================================================
 * READ_RAW_TEMP
 *  Read in raw values from temperature registers
 *
 * PARAMETERS
 *  - out: 3-byte array of raw temperature in big-endian
 * =============================================================================== */
void BMP581_readRawTemp(BMP581 *baro, uint8_t *out) {
  out[0] = baro->base.readRegister(baro, BMP581_TEMPERATURE_MSB);  // temp high
  out[1] = baro->base.readRegister(baro, BMP581_TEMPERATURE_LSB);  // temp low
  out[2] = baro->base.readRegister(baro, BMP581_TEMPERATURE_XLSB); // temp mid
}

/* ===============================================================================
 * READ_PRESS
 *  Read processed floating point pressure
 *
 * PARAMETERS
 *  - out: floating point pressure
 * =============================================================================== */
void BMP581_readPress(BMP581 *baro, float *out) {
  uint8_t bytes[BMP581_DATA_TOTAL];
  baro->readRawPress(baro, bytes);
  baro->processRawPress(baro, bytes, out);
}

/* ===============================================================================
 * PROCESS_RAW_PRESSURE
 *  Process raw values to floating point pressure
 *
 * PARAMETERS
 *  - bytes: 3-byte array of raw pressure in big-endian
 *  - out: floating point pressure
 * =============================================================================== */
void BMP581_processRawPress(BMP581 *baro, uint8_t *bytes, float *out) {
  *out = baro->pressSensitivity * (int32_t)(((uint32_t)bytes[0] << 16) | ((uint32_t)bytes[1] << 8) | bytes[0]);
}

/* ===============================================================================
 * READ_RAW_TEMP
 *  Read in raw values from temperature registers
 *
 * PARAMETERS
 *  - out: 3-byte array of raw temperature in big-endian
 * =============================================================================== */
void BMP581_readRawPress(BMP581 *baro, uint8_t *out) {
  out[0] = baro->base.readRegister(baro, BMP581_PRESSURE_MSB);  // temp high
  out[1] = baro->base.readRegister(baro, BMP581_PRESSURE_LSB);  // temp low
  out[2] = baro->base.readRegister(baro, BMP581_PRESSURE_XLSB); // temp mid
}

/******************************** INTERFACE METHODS ********************************/
void BMP581_writeRegister(void *sensor, uint8_t address, uint8_t payload) {
  uint16_t return_value    = 0;
  uint16_t payload_to_send = 0;
  payload_to_send &= (~(0x8000));                     // 0 write, dont need this lne really
  payload_to_send |= (address << 0x8);                // load address into top 7 bits
  payload_to_send |= payload;                         // load data in to write
  ((SPI *)sensor)->port->ODR &= ~((SPI *)sensor)->cs; // lower gyro chip select
  while ((SPI1->SR & SPI_SR_TXE) == 0);               // wait for transmission to be empty
  SPI1->DR = payload_to_send;
  while ((SPI1->SR & SPI_SR_RXNE) == 0);              // wait for received data
  return_value = (uint16_t)(SPI1->DR);
  while ((SPI1->SR & SPI_SR_BSY) == 1);
  ((SPI *)sensor)->port->ODR |= ((SPI *)sensor)->cs;
}

uint8_t BMP581_readRegister(void *sensor, uint8_t address) {
  uint16_t return_value;
  uint16_t payload_to_send = 0;
  payload_to_send |= 0x8000;                          // 1 read
  payload_to_send |= (address << 0x8);                // load address into top 7 bits
  ((SPI *)sensor)->port->ODR &= ~((SPI *)sensor)->cs; // lower gyro chip select
  while ((SPI1->SR & SPI_SR_TXE) == 0);               // wait for transmission to be empty
  SPI1->DR = payload_to_send;
  while ((SPI1->SR & SPI_SR_RXNE) == 0);              // wait for received data
  return_value = (uint16_t)(SPI1->DR);
  while ((SPI1->SR & SPI_SR_BSY) == 1);
  ((SPI *)sensor)->port->ODR |= ((SPI *)sensor)->cs;
  return return_value;
}
