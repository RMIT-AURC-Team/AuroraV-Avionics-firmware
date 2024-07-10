/*
 * data_management.c
 *
 *  Created on: Apr 6, 2024
 *      Author: aidencontini
 */

#include "data_management.h"

uint8_t FLASH_WREN 		= 0x06;			// Write enable to begin writing data
uint8_t FLASH_WRDIS 	= 0x04;			// Write disable to disable writing data
uint8_t FLASH_PGWR 		= 0x02;			// Page write to write an entire page out
uint8_t FLASH_READSR1 	= 0x05;			// Read status register value
uint8_t FLASH_READEN 	= 0x03;			// Read enable to begin reading data
uint8_t FLASH_READ_MANU	= 0x90;			// Instruction code to read the manufacturer ID
uint8_t FLASH_32K_ERS	= 0x52;			// Erase all memory within a 32K-byte block
uint8_t FLASH_ERASE		= 0xC7;			// Erase the entire chip
uint8_t RST_EN			= 0x66;			// Reset enable
uint8_t DEV_RST			= 0x99;			// Reset device

uint8_t check_busy(SPI_HandleTypeDef *hspi, GPIO_Config config, int timeout) {
	uint8_t ret_val = 0x00;

    // Get the current time
	uint32_t startTick = HAL_GetTick();

	// Read the status register to ensure no write is currently in progress
	uint8_t busy = 0x01;
	while(busy) {
		// Check if there is a write in progress
		busy = (check_status_register(hspi, config) & 0x01);

		uint32_t currTick = HAL_GetTick();

        // Check if the timeout has been reached
		if ((currTick - startTick) >= timeout) {
        	ret_val = 0x01;
            break;
        }
	}

	return ret_val;
}

uint8_t check_status_register(SPI_HandleTypeDef *hspi, GPIO_Config config) {
	uint8_t status_reg = 0x00;
	spi_sendOp_readByte(&FLASH_READSR1, hspi, &status_reg, config.GPIOx, config.GPIO_Pin);
	return status_reg;
}

