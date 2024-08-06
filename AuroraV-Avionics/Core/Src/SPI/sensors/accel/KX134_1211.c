/**
 * @author Matt Ricci
 * @file KX134_1211.c
 */

#include "KX134_1211.h"

/* =============================================================================== */
/**
 * @brief Initialiser for a KX134-1211 accelerometer.
 * @param *accel 			Pointer to KX134-1211 struct to be initialised.
 * @param *port 			Pointer to GPIO port struct.
 * @param cs 					Device chip select address.
 * @param scale       Selected scale for read accelerations.
 * @param *axes       Array defining sensor mounting axes.
 * @return @c NULL.
 **
 * =============================================================================== */
void KX134_1211_init(KX134_1211 *accel, GPIO_TypeDef *port, unsigned long cs, uint8_t scale, const uint8_t *axes, const int8_t *sign) {
  SPI_init(&accel->base, SENSOR_ACCEL, SPI1, port, cs);
  accel->update          = KX134_1211_update;
  accel->readAccel       = KX134_1211_readAccel;
  accel->readRawBytes    = KX134_1211_readRawBytes;
  accel->processRawBytes = KX134_1211_processRawBytes;
  memcpy(accel->axes, axes, KX134_1211_DATA_COUNT);
  memcpy(accel->sign, sign, KX134_1211_DATA_COUNT);

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

/********************************** DEVICE METHODS *********************************/

/* =============================================================================== */
/**
 * @brief Read 3-axis floating point accelerations.
 * @param 	*accel 		Pointer to accel struct.
 * @param 	*out 		  Floating point acceleration array.
 * @returns @c NULL.
 **
 * =============================================================================== */
void KX134_1211_readAccel(KX134_1211 *accel, float *out) {
  accel->update(accel);
  out = accel->accelData;
}

/* =============================================================================== */
/**
 * @brief Updates internally stored acceleration readings.
 * @param 	*accel 		Pointer to accel struct.
 * @returns @c NULL.
 **
 * =============================================================================== */
void KX134_1211_update(KX134_1211 *accel) {
  accel->readRawBytes(accel, accel->rawAccelData);
  accel->processRawBytes(accel, accel->rawAccelData, accel->accelData);
}

/* =============================================================================== */
/**
 * @brief Process raw 3-axis data to floating point accelerations.
 * @param 	*accel 		Pointer to accel struct.
 * @param 	*bytes 		Raw 3-axis data array.
 * @param 	*out 			Processed 3-axis data array to write.
 * @returns @c NULL.
 **
 * =============================================================================== */
void KX134_1211_processRawBytes(KX134_1211 *accel, uint8_t *bytes, float *out) {
  out[0] = accel->sign[0] * accel->sensitivity * (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]); // Accel X
  out[1] = accel->sign[1] * accel->sensitivity * (int16_t)(((uint16_t)bytes[2] << 8) | bytes[3]); // Accel Y
  out[2] = accel->sign[2] * accel->sensitivity * (int16_t)(((uint16_t)bytes[4] << 8) | bytes[5]); // Accel Z
}

/* =============================================================================== */
/**
 * @brief Read raw 3-axis data.
 * @param 	*accel 		Pointer to accel struct.
 * @param 	*out 			Raw 3-axis data array to write.
 * @returns @c NULL.
 **
 * =============================================================================== */
void KX134_1211_readRawBytes(KX134_1211 *accel, uint8_t *out) {
// Map raw indices to mounting axis
#define INDEX_AXES(index, byte) 2 * accel->axes[index] + byte
  out[INDEX_AXES(0, 0)] = KX134_1211_readRegister(accel, KX134_1211_XOUT_H); // Accel X high
  out[INDEX_AXES(0, 1)] = KX134_1211_readRegister(accel, KX134_1211_XOUT_L); // Accel X low
  out[INDEX_AXES(1, 0)] = KX134_1211_readRegister(accel, KX134_1211_YOUT_H); // Accel Y high
  out[INDEX_AXES(1, 1)] = KX134_1211_readRegister(accel, KX134_1211_YOUT_L); // Accel Y low
  out[INDEX_AXES(2, 0)] = KX134_1211_readRegister(accel, KX134_1211_ZOUT_H); // Accel Z high
  out[INDEX_AXES(2, 1)] = KX134_1211_readRegister(accel, KX134_1211_ZOUT_L); // Accel Z low
#undef INDEX_AXES
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
