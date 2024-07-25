#include "KX134_1211.h"

void KX134_1211_init(KX134_1211 *accel, GPIO_TypeDef *port, unsigned long cs, uint8_t scale, const uint8_t *axes) {
  SPI_init(&accel->base, SENSOR_ACCEL, SPI1, port, cs);
  accel->axes            = axes;
  accel->readAccel       = KX134_1211_readAccel;
  accel->readRawBytes    = KX134_1211_readRawBytes;
  accel->processRawBytes = KX134_1211_processRawBytes;

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
  KX134_1211_writeRegister(accel, KX134_1211_CNTL1, KX134_1211_CNTL1_RES | GSEL);                        // Accel select, selected sensitivity
  uint8_t ODCNTL = KX134_1211_readRegister(accel, KX134_1211_ODCNTL);                                    // Read from register for reserve mask
  KX134_1211_writeRegister(accel, KX134_1211_ODCNTL, (KX134_1211_ODCNTL_RESERVED & ODCNTL) | 0x2A);      // No filter, fast startup, 800Hz
  KX134_1211_writeRegister(accel, KX134_1211_CNTL1, KX134_1211_CNTL1_PC1 | KX134_1211_CNTL1_RES | GSEL); // Enable PC1
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
  out[0] = KX134_1211_readRegister(accel, KX134_1211_XOUT_H); // Accel X high
  out[1] = KX134_1211_readRegister(accel, KX134_1211_XOUT_L); // Accel X low
  out[2] = KX134_1211_readRegister(accel, KX134_1211_YOUT_H); // Accel Y high
  out[3] = KX134_1211_readRegister(accel, KX134_1211_YOUT_L); // Accel Y low
  out[4] = KX134_1211_readRegister(accel, KX134_1211_ZOUT_H); // Accel Z high
  out[5] = KX134_1211_readRegister(accel, KX134_1211_ZOUT_L); // Accel Z low
}

/******************************** INTERFACE METHODS ********************************/

void KX134_1211_writeRegister(KX134_1211 *accel, uint8_t address, uint8_t data) {
  uint16_t response;
  SPI spi = accel->base;

  // Send write command with address and data
  uint16_t payload = (address << 0x08) | data; // Load payload with address and data
  spi.port->ODR &= ~spi.cs;                    // Lower chip select
  spi.send(&spi, payload);                     // Send payload

  // Read in response from interface
  spi.receive(&spi, &response); // Read in response from receive buffer
  spi.port->ODR |= spi.cs;      // Raise chip select
}

uint8_t KX134_1211_readRegister(KX134_1211 *accel, uint8_t address) {
  uint16_t response;
  SPI spi = accel->base;

  // Send read command and address
  uint16_t payload = (address << 0x08) | 0x8000; // Load payload with address and read command
  spi.port->ODR &= ~spi.cs;                      // Lower chip select
  spi.send(&spi, payload);                       // Send payload

  // Read in response from interface
  spi.receive(&spi, &response); // Read in response from receive buffer
  spi.port->ODR |= spi.cs;      // Raise chip select
  return (uint8_t)response;
}
