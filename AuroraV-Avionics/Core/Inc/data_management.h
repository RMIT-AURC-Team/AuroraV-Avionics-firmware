/*
 * data_management.h
 *
 *  Created on: Mar 24, 2024
 *      Author: aidencontini
 */

#ifndef INC_DATA_MANAGEMENT_H_
#define INC_DATA_MANAGEMENT_H_
#include "stm32f4xx_hal.h"
#include "spi_driver.h"
#include "gpio_struct.h"

#define PAGE_SIZE 		256 			// 2048 bits
#define NUM_OF_PAGES 	65536
#define BLOCK_32K		256000
#define READ_SIZE		20
#define BSY_TIMEOUT_MS	5
#define BLK_ERS_32K_TO	1600
#define CHIP_ERASE_TO	200000

extern uint8_t FLASH_WREN;			// Write enable to begin writing data
extern uint8_t FLASH_WRDIS;			// Write disable to disable writing data
extern uint8_t FLASH_PGWR;			// Page write to write an entire page out
extern uint8_t FLASH_READSR1;		// Read status register value
extern uint8_t FLASH_READEN;		// Read enable to begin reading data
extern uint8_t FLASH_READ_MANU;		// Instruction code to read the manufacturer ID
extern uint8_t FLASH_32K_ERS;		// Erase all memory within a 32K-byte block
extern uint8_t FLASH_ERASE;			// Erase the entire chip
extern uint8_t RST_EN;				// Reset enable
extern uint8_t DEV_RST;				// Reset device

/**
 * This function checks if a write operation is in progress and waits until it's completed or a timeout occurs.
 *
 * @param hspi     A pointer to the SPI_HandleTypeDef structure that contains the configuration information for the specified SPI module.
 * @param flashNo  The number of the flash memory to check.
 * @param timeout  The maximum time (ms) to wait for the write operation to complete.
 *
 * @return         Returns 0x00 if the write operation completes within the specified timeout.
 *                 Returns 0x01 if a timeout occurs before the write operation completes.
 */
uint8_t check_busy(SPI_HandleTypeDef *hspi, GPIO_Config config, int timeout);

uint8_t check_status_register(SPI_HandleTypeDef *hspi, GPIO_Config config);

#endif /* INC_DATA_MANAGEMENT_H_ */
