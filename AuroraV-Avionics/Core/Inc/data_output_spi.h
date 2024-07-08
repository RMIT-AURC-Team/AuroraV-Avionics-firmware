/*
 * data_output.h
 *
 *  Created on: Mar 20, 2024
 *      Author: aidencontini
 */

#ifndef INC_DATA_OUTPUT_SPI_H_
#define INC_DATA_OUTPUT_SPI_H_
#include "data_management.h"

void write_enable_spi(SPI_HandleTypeDef *hspi, GPIO_Config config);
void write_disable_spi(SPI_HandleTypeDef *hspi, GPIO_Config config);
HAL_StatusTypeDef erase_chip_spi(SPI_HandleTypeDef *hspi, GPIO_Config config);
HAL_StatusTypeDef erase_32k_spi(SPI_HandleTypeDef *hspi, uint32_t addr, GPIO_Config config);
uint8_t write_data_spi(uint8_t page[PAGE_SIZE], SPI_HandleTypeDef *hspi, uint32_t addr, GPIO_Config config);
uint8_t write_data_spi_dma(uint8_t page[PAGE_SIZE], SPI_HandleTypeDef *hspi, uint32_t addr, GPIO_Config config);
void software_reset(SPI_HandleTypeDef *hspi, GPIO_Config config);

#endif /* INC_DATA_OUTPUT_SPI_H_ */
