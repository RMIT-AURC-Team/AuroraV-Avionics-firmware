/**
 * @author Matt Ricci
 * @file lora.c
 * @addtogroup LoRa
 * @todo Implement adjustable packet size
 * @{
 */

#include "lora.h"

/* SPI3 LORA
 * ------------------------------------
 * Flash Pin  | MCU GPIO Pin  | SIGNAL TYPE
 * -----------|---------------|------------
 * SDI        | PC12          | DATA
 * SDO        | PC11          | DATA
 * SCLK       | PC10          | DATA
 * RST        | PD7           | CONTROL
 * DI0        | PD1           | DATA
 * CS         | PD0           | CONTROL      */

/* =============================================================================== */
/**
 * @brief
 * @param *lora 			Pointer to LoRa struct to be initialised.
 * @param *port 			Pointer to GPIO port struct.
 * @param cs 					Device chip select address.
 * @param bw
 * @param sf
 * @param cr
 * @return @c NULL.
 **
 * =============================================================================== */
void LoRa_init(LoRa *lora, GPIO_TypeDef *port, unsigned long cs, Bandwidth bw, SpreadingFactor sf, CodingRate cr) {
  SPI_init(&lora->base, COMM_LORA, SPI3, port, cs);
  lora->transmit = LoRa_transmit;

  _LoRa_setMode(lora, SLEEP); // Set mode to sleep

  // Set interrupt pin
  LoRa_writeRegister(lora, RegDioMapping1, 0x40);

  /* clang-format off */
  LoRa_writeRegister(lora, LORA_REG_OP_MODE, 
     0x01 << LORA_REG_OP_MODE_LONG_RANGE_Pos  // Enable LoRa
  ); 

  LoRa_writeRegister(lora, LORA_REG_MODEM_CONFIG1, 
    bw   << LORA_REG_MODEM_CONFIG1_BW_Pos     // Set bandwidth
  | cr   << LORA_REG_MODEM_CONFIG1_CR_Pos     // Set coding rate
  | 0x00 << LORA_REG_MODEM_CONFIG1_CRC_Pos    // Enable CRC
  );
  /* clang-format on */

	/** @todo set spreading factor */	
  LoRa_writeRegister(lora, LORA_REG_MODEM_CONFIG2, 0x94);

  // Set payload length
  LoRa_writeRegister(lora, LORA_REG_PAYLOAD_LENGTH, LORA_MSG_LENGTH);
  LoRa_writeRegister(lora, LORA_REG_MAX_PAYLOAD_LENGTH, LORA_MSG_LENGTH);

  _LoRa_setMode(lora, STDBY); // Set mode to standby
}

/********************************** PRIVATE METHODS ********************************/

#ifndef DOXYGEN_PRIVATE

/* =============================================================================== */
/**
 * @brief
 * @param *LoRa			Pointer to LoRa struct.
 * @param Mode
 * @return @c NULL.
 **
 * =============================================================================== */
void _LoRa_setMode(LoRa *lora, Mode mode) {
  uint8_t regOpMode = LoRa_readRegister(lora, LORA_REG_OP_MODE);
  regOpMode &= ~0x07; // Mask to mode bits
  regOpMode |= mode;  // Set mode
  LoRa_writeRegister(lora, LORA_REG_OP_MODE, regOpMode);
}

#endif

/********************************** STATIC METHODS *********************************/

/* =============================================================================== */
/**
 * @brief
 * @param id
 * @param *accelData
 * @param lenAccel
 * @return LoRa_Packet.
 **
 * =============================================================================== */
LoRa_Packet LoRa_AVData(
	uint8_t id, 
	uint8_t currentState,
	uint8_t *lAccelData, 
	uint8_t *hAccelData, 
	uint8_t lenAccel, 
  uint8_t *gyroData,
	uint8_t lenGyro,
	float altitude,
	float velocity
) {
  LoRa_Packet msg;

	// Convert altitude float to byte array
	union {
		float f;
		uint8_t b[4];
	} a;
	a.f = altitude;
	
	// Convert velocity float to byte array
	union {
		float f;
		uint8_t b[4];
	} v;
	v.f = velocity;

	int idx = 0;
  // Append to struct data array
	msg.id = id;
	msg.data[idx++] = currentState;
	memcpy(&msg.data[idx], lAccelData, lenAccel);
	memcpy(&msg.data[idx += lenAccel], hAccelData, lenAccel);
	memcpy(&msg.data[idx += lenAccel], gyroData, lenGyro);
	memcpy(&msg.data[idx += lenGyro], a.b, sizeof(float));
	memcpy(&msg.data[idx += sizeof(float)], v.b, sizeof(float));

  return msg;
}

/********************************** DEVICE METHODS *********************************/

/* =============================================================================== */
/**
 * @brief
 * @param lora
 * @param pointerdata
 **
 * =============================================================================== */
void LoRa_transmit(LoRa *lora, uint8_t *pointerdata) {
  LoRa_writeRegister(lora, LORA_REG_IRQ_FLAGS, 0x08);     // clears the status flags
  LoRa_writeRegister(lora, LORA_REG_FIFO_ADDR_PTR, 0x80); // set pointer adddress to TX

  // Load data into transmit FIFO
  for (int i = 0; i < 32; i++) {
    LoRa_writeRegister(lora, LORA_REG_FIFO, pointerdata[i]);
  }

  _LoRa_setMode(lora, TX);

	/** 
	 * @todo Implement interrupt on TxComplete
	 * 
	 * @attention Polling the pin takes too much time, as a result delay timers on various tasks will
	 * have expired in the time waiting and CPU will never idle (meaning flashing does not occur).
	 *
	 * @attention Implementing interrupt on TxComplete signal should solve this since the handler should only
	 * run over a few clock cycles.
	 */
	//while (!(GPIOD->IDR & 0x2));
  LoRa_writeRegister(lora, LORA_REG_IRQ_FLAGS, 0x08); // clears the status flags
}

/******************************** INTERFACE METHODS ********************************/

void LoRa_writeRegister(LoRa *lora, uint8_t address, uint8_t data) {
  SPI spi          = lora->base;

  uint16_t payload = (address << 0x08) | (1 << 0x0F); // Load payload with address and write command
  payload |= data;                                    // Append data to payload
  spi.port->ODR &= ~spi.cs;                           // Lower chip select
  spi.transmit(&spi, payload);                        // Send payload over SPI
  spi.port->ODR |= spi.cs;                            // Raise chip select
}

uint8_t LoRa_readRegister(LoRa *lora, uint8_t address) {
  SPI spi = lora->base;
  uint16_t response;

  uint16_t payload = (address << 0x08);   // Load payload with address and read command
  spi.port->ODR &= ~spi.cs;               // Lower chip select
  response = spi.transmit(&spi, payload); // Receive payload over SPI
  spi.port->ODR |= spi.cs;                // Raise chip select
  return (uint8_t)response;
}

/** @} */
