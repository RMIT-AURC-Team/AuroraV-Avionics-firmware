#include "A3G4250D.h"

void A3G4250D_init(A3G4250D *gyro, GPIO_TypeDef *port, unsigned long cs, float sensitivity, const uint8_t *axes) {
  gyro->base.device        = SENSOR_GYRO;
  gyro->base.port          = port;
  gyro->base.cs            = cs;
  gyro->base.writeRegister = A3G4250D_writeRegister;
  gyro->base.readRegister  = A3G4250D_readRegister;
  gyro->sensitivity        = sensitivity;
  gyro->axes               = axes;
  gyro->readGyro           = A3G4250D_readGyro;
  gyro->readRawBytes       = A3G4250D_readRawBytes;
  gyro->processRawBytes    = A3G4250D_processRawBytes;

  gyro->base.writeRegister(gyro, A3G4250D_CTRL_REG1, A3G4250D_CTRL_REG1_ODR_800Hz | A3G4250D_CTRL_REG1_AXIS_ENABLE | A3G4250D_CTRL_REG1_PD_ENABLE);
}

/******************************** DEVICE METHODS ********************************/

/* ===============================================================================
 * READ_GYRO
 *  Read processed floating point rates
 *
 * PARAMETERS
 *  - out: 3-float array of X-Y-Z rates
 * =============================================================================== */
void A3G4250D_readGyro(A3G4250D *gyro, float *out) {
  uint8_t bytes[A3G4250D_DATA_TOTAL];
  gyro->readRawBytes(gyro, bytes);
  gyro->processRawBytes(gyro, bytes, out);
}

/* ===============================================================================
 * PROCESS_RAW_BYTES
 *  Process raw values to floating point rates
 *
 * PARAMETERS
 *  - bytes: 6-byte array of raw X-Y-Z values in big-endian
 *  - out: 3-float array of X-Y-Z rates
 * =============================================================================== */
void A3G4250D_processRawBytes(A3G4250D *gyro, uint8_t *bytes, float *out) {
  out[0] = gyro->sensitivity * (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]); // gyro X
  out[1] = gyro->sensitivity * (int16_t)(((uint16_t)bytes[2] << 8) | bytes[3]); // gyro Y
  out[2] = gyro->sensitivity * (int16_t)(((uint16_t)bytes[4] << 8) | bytes[5]); // gyro Z
}

/* ===============================================================================
 * READ_RAW_BYTES
 *  Read in raw values from sensor registers
 *
 * PARAMETERS
 *  - out: 6-byte array of raw X-Y-Z values in big-endian
 * =============================================================================== */
void A3G4250D_readRawBytes(A3G4250D *gyro, uint8_t *out) {
  out[0] = gyro->base.readRegister(gyro, A3G4250D_OUT_X_H); // gyro X high
  out[1] = gyro->base.readRegister(gyro, A3G4250D_OUT_X_L); // gyro X low
  out[2] = gyro->base.readRegister(gyro, A3G4250D_OUT_Y_H); // gyro Y high
  out[3] = gyro->base.readRegister(gyro, A3G4250D_OUT_Y_L); // gyro Y low
  out[4] = gyro->base.readRegister(gyro, A3G4250D_OUT_Z_H); // gyro Z high
  out[5] = gyro->base.readRegister(gyro, A3G4250D_OUT_Z_L); // gyro Z low
}

/******************************** INTERFACE METHODS ********************************/
void A3G4250D_writeRegister(void *sensor, uint8_t address, uint8_t payload) {
  uint16_t return_value;
  uint16_t payload_to_send = 0;
  payload_to_send &= (~(0X4000));                     // clear  multiple transfer bit
  payload_to_send &= (~(0x8000));                     // set 15th bit to 0 for write
  payload_to_send |= (address << 0x8);                // load address into top 7 bits
  payload_to_send |= (payload);                       // mask in data
  ((SPI *)sensor)->port->ODR &= ~((SPI *)sensor)->cs; // lower gyro chip select
  while ((SPI1->SR & SPI_SR_TXE) == 0);               // wait for transmission to be empty
  SPI1->DR = payload_to_send;
  while ((SPI1->SR & SPI_SR_RXNE) == 0);              // wait for received data
  return_value = (uint16_t)(SPI1->DR);
  while ((SPI1->SR & SPI_SR_BSY) == 1);
  ((SPI *)sensor)->port->ODR |= ((SPI *)sensor)->cs;
}

uint8_t A3G4250D_readRegister(void *sensor, uint8_t address) {
  uint16_t return_value;
  uint16_t payload_to_send = 0;
  payload_to_send &= (~(0x4000));        // clear  multiple transfer bit
  payload_to_send |= 0x8000;
  payload_to_send |= (address << 0x8);   // load address into top 7 bits
  while ((SPI1->SR & SPI_SR_TXE) == 0);
  ((SPI *)sensor)->port->ODR &= ~((SPI *)sensor)->cs;
  SPI1->DR = payload_to_send;
  while ((SPI1->SR & SPI_SR_RXNE) == 0); // wait for received data
  ((SPI *)sensor)->port->ODR |= ((SPI *)sensor)->cs;
  return_value = (uint16_t)(SPI1->DR);
  return (uint8_t)return_value;
}
