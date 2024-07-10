/*
 * spi_driver.c
 *
 *  Created on: May 13, 2024
 *      Author: aidencontini
 */

#ifndef SRC_SPI_DRIVER_C_
#define SRC_SPI_DRIVER_C_

#include "spi_driver.h"

HAL_StatusTypeDef spi_sendOp_readByte(uint8_t* opcode, SPI_HandleTypeDef *hspi, uint8_t* data_ptr, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_CS) {
	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin_CS, GPIO_PIN_RESET);	// Set the chip select pin to low to initiate SPI comms
	HAL_SPI_Transmit(hspi, opcode, 1, TIMEOUT_SML);			// Write the write enable value to the Flash to initial reading
	status = HAL_SPI_Receive(hspi, data_ptr, 1, TIMEOUT_SML);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin_CS, GPIO_PIN_SET);
	return status;
}

HAL_StatusTypeDef perform_operation(uint8_t* opcode, SPI_HandleTypeDef *hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_CS) {
	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin_CS, GPIO_PIN_RESET);				// Set the chip select pin to low to initiate SPI comms
	status = HAL_SPI_Transmit(hspi, opcode, 1, TIMEOUT_SML);				// Write the opcode value
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin_CS, GPIO_PIN_SET);

	return status;
}

HAL_StatusTypeDef spi_read_data(uint8_t* opcode, uint16_t data_size, uint8_t data_read[data_size], SPI_HandleTypeDef *hspi, uint32_t addr, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_CS) {
	HAL_StatusTypeDef status;

	// Split the 24 bit address into three 8 bit ints
	uint8_t addrL = addr & 0xFF;  // Get the low byte
	uint8_t addrM = (addr >> 8) & 0xFF;  // Get the middle byte
	uint8_t addrH = (addr >> 16) & 0xFF;  // Get the high byte

	// Read the data denoted by the given address
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin_CS, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, opcode, 1, TIMEOUT_SML);	// Write the read enable value

	// Transmit the 24 bit address of the page to initialise read from
	HAL_SPI_Transmit(hspi, &addrH, 1, TIMEOUT_SML);
	HAL_SPI_Transmit(hspi, &addrM, 1, TIMEOUT_SML);
	HAL_SPI_Transmit(hspi, &addrL, 1, TIMEOUT_SML);

	status = HAL_SPI_Receive(hspi, data_read, data_size, TIMEOUT_LRG);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin_CS, GPIO_PIN_SET);

	return status;
}

HAL_StatusTypeDef spi_write_data(uint8_t* opcode, uint16_t data_size, uint8_t data_write[data_size], SPI_HandleTypeDef *hspi, uint32_t addr, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_CS) {
	HAL_StatusTypeDef status;

	// Split the 24 bit address into three 8 bit ints
	uint8_t addrL = addr & 0xFF;  			// Get the low byte
	uint8_t addrM = (addr >> 8) & 0xFF;  	// Get the middle byte
	uint8_t addrH = (addr >> 16) & 0xFF;  	// Get the high byte

	// Send the PageWrite command
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin_CS, GPIO_PIN_RESET);	// Set the chip select pin to low to initiate SPI comms
	HAL_SPI_Transmit(hspi, opcode, 1, TIMEOUT_SML);					// Write the write enable value to the Flash to initial writing

	// Transmit the 24 bit address of the page to write to
	HAL_SPI_Transmit(hspi, &addrH, 1, TIMEOUT_SML);
	HAL_SPI_Transmit(hspi, &addrM, 1, TIMEOUT_SML);
	HAL_SPI_Transmit(hspi, &addrL, 1, TIMEOUT_SML);

	// Transmit the data to be written
	status = HAL_SPI_Transmit(hspi, data_write, data_size, TIMEOUT_LRG);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin_CS, GPIO_PIN_SET);	// Set the chip select pin to high to remove the Flash from the SPI bus

	return status;
}

HAL_StatusTypeDef spi_write_data_dma(uint8_t* opcode, uint16_t data_size, uint8_t data_write[data_size], SPI_HandleTypeDef *hspi, uint32_t addr, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_CS) {
    HAL_StatusTypeDef status;

    // Split the 24 bit address into three 8 bit ints
    uint8_t addrL = addr & 0xFF;  			// Get the low byte
    uint8_t addrM = (addr >> 8) & 0xFF;  	// Get the middle byte
    uint8_t addrH = (addr >> 16) & 0xFF;  	// Get the high byte

    // Send the PageWrite command
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin_CS, GPIO_PIN_RESET);	// Set the chip select pin to low to initiate SPI comms
    HAL_SPI_Transmit(hspi, opcode, 1, TIMEOUT_SML);					// Write the write enable value to the Flash to initial writing

    // Transmit the 24 bit address of the page to write to
    HAL_SPI_Transmit(hspi, &addrH, 1, TIMEOUT_SML);
    HAL_SPI_Transmit(hspi, &addrM, 1, TIMEOUT_SML);
    HAL_SPI_Transmit(hspi, &addrL, 1, TIMEOUT_SML);

    // Start the DMA transfer
    status = HAL_SPI_Transmit_DMA(hspi, data_write, data_size);

    return status;
}

#endif /* SRC_SPI_DRIVER_C_ */
