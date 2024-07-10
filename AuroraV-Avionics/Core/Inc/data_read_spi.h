/*
 * data_read.h
 *
 *  Created on: Mar 24, 2024
 *      Author: aidencontini
 */

#ifndef INC_DATA_READ_SPI_H_
#define INC_DATA_READ_SPI_H_
#include "data_management.h"

void read_page_spi(uint8_t data_read[PAGE_SIZE], SPI_HandleTypeDef *hspi, uint32_t addr, GPIO_Config config);
uint32_t find_next_blank_page(SPI_HandleTypeDef *hspi, GPIO_PinState *end_of_flash_ptr, GPIO_Config config);
void read_manufacturer_id(uint8_t manu[2], SPI_HandleTypeDef *hspi, GPIO_Config config);

#endif /* INC_DATA_READ_SPI_H_ */
