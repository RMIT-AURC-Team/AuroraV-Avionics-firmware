#include "KX134_1211.h"

void KX134_1211_init(KX134_1211 *accel, unsigned long cs, int scale) {
  accel->base.cs            = cs;
  accel->base.writeRegister = KX134_1211_writeRegister;
  accel->base.readRegister  = KX134_1211_readRegister;
  accel->readAccel          = KX134_1211_readAccel;
  accel->readRawBytes       = KX134_1211_readRawBytes;
  accel->processRawBytes    = KX134_1211_processRawBytes;

  // Set value of GSEL and sensitivity based on selected scale
  uint8_t GSEL = 0x00;
  if (scale == 32) {
    GSEL               = KX134_1211_CNTL1_GSEL(32);
    accel->sensitivity = KX134_1211_SENSITIVITY(32);
  } else if (scale == 16) {
    GSEL               = KX134_1211_CNTL1_GSEL(16);
    accel->sensitivity = KX134_1211_SENSITIVITY(16);
  }

  // Configure accelerometer registers
  accel->base.writeRegister((SPI *)accel, KX134_1211_CNTL1, KX134_1211_CNTL1_RES | GSEL);                   // Accel select, selected sensitivity
  uint8_t ODCNTL = accel->base.readRegister((SPI *)accel, KX134_1211_ODCNTL);                               // Read from register for reserve mask
  accel->base.writeRegister((SPI *)accel, KX134_1211_ODCNTL, (KX134_1211_ODCNTL_RESERVED & ODCNTL) | 0x2A); // No filter, fast startup, 800Hz
  accel->base.writeRegister((SPI *)accel, KX134_1211_CNTL1, 0xD0);                                          // Enable PC1
}

/******************************** DEVICE METHODS ********************************/

/* ===============================================================================
 * READ_ACCEL
 *  Read processed floating point accelerations
 *
 * PARAMETERS
 *  - out: 3-float array of X-Y-Z accelerations
 * =============================================================================== */
void KX134_1211_readAccel(KX134_1211 *accel, float *out) {
  uint8_t bytes[KX134_1211_DATA_TOTAL];
  accel->readRawBytes(accel, bytes);
  accel->processRawBytes(accel, bytes, out);
}

/* ===============================================================================
 * PROCESS_RAW_BYTES
 *  Process raw values to floating point accelerations
 *
 * PARAMETERS
 *  - bytes: 6-byte array of raw X-Y-Z values in big-endian
 *  - out: 3-float array of X-Y-Z accelerations
 * =============================================================================== */
void KX134_1211_processRawBytes(KX134_1211 *accel, uint8_t *bytes, float *out) {
  out[0] = accel->sensitivity * (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]); // Accel X
  out[1] = accel->sensitivity * (int16_t)(((uint16_t)bytes[2] << 8) | bytes[3]); // Accel Y
  out[2] = accel->sensitivity * (int16_t)(((uint16_t)bytes[4] << 8) | bytes[5]); // Accel Z
}

/* ===============================================================================
 * READ_RAW_BYTES
 *  Read in raw values from sensor registers
 *
 * PARAMETERS
 *  - out: 6-byte array of raw X-Y-Z values in big-endian
 * =============================================================================== */
void KX134_1211_readRawBytes(KX134_1211 *accel, uint8_t *out) {
  out[0] = accel->base.readRegister(accel, KX134_1211_XOUT_H); // Accel X high
  out[1] = accel->base.readRegister(accel, KX134_1211_XOUT_L); // Accel X low
  out[2] = accel->base.readRegister(accel, KX134_1211_YOUT_H); // Accel Y high
  out[3] = accel->base.readRegister(accel, KX134_1211_YOUT_L); // Accel Y low
  out[4] = accel->base.readRegister(accel, KX134_1211_ZOUT_H); // Accel Z high
  out[5] = accel->base.readRegister(accel, KX134_1211_ZOUT_L); // Accel Z low
}

/******************************** INTERFACE METHODS ********************************/

void KX134_1211_writeRegister(void *sensor, uint8_t address, uint8_t payload) {
  uint16_t return_value;
  uint16_t payload_to_send = 0;
  payload_to_send &= ~0x8000;            // 0 write, dont need this lne really
  payload_to_send |= (address << 0x8);   // load address into top 7 bits
  payload_to_send |= payload;            // load data in to write
  while ((SPI1->SR & SPI_SR_TXE) == 0);  // wait for transmission to be empty
  GPIOA->ODR &= ~((SPI *)sensor)->cs;    // lower chip select
  SPI1->DR = payload_to_send;
  while ((SPI1->SR & SPI_SR_RXNE) == 0); // wait for received data
  return_value = (uint16_t)(SPI1->DR);
  while ((SPI1->SR & SPI_SR_BSY) == 1);
  GPIOA->ODR |= ((SPI *)sensor)->cs;
}

uint8_t KX134_1211_readRegister(void *sensor, uint8_t address) {
  uint16_t return_value    = 0;
  uint16_t payload_to_send = 0;
  payload_to_send |= 0x8000;
  payload_to_send |= (address << 0x8);   // load address into top 7 bits
  while ((SPI1->SR & SPI_SR_TXE) == 0);
  GPIOB->ODR &= ~((SPI *)sensor)->cs;
  SPI1->DR = payload_to_send;
  while ((SPI1->SR & SPI_SR_RXNE) == 0); // wait for received data
  GPIOB->ODR |= ((SPI *)sensor)->cs;
  return_value = (uint16_t)(SPI1->DR);
  return (uint8_t)return_value;
}
