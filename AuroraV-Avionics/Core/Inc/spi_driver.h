/*
 * spi_driver.h
 *
 *  Created on: May 13, 2024
 *      Author: aidencontini
 */

#ifndef INC_SPI_DRIVER_H_
#define INC_SPI_DRIVER_H_

#include "stm32f4xx_hal.h"

#define TIMEOUT_SML		2
#define TIMEOUT_LRG		100

/**
 * @brief Sends an operation code via SPI and reads a byte of data.
 *
 * This function sends an operation code to a device over SPI, then reads a byte of data from the device.
 * The chip select pin is set to low to initiate SPI communication, then reset to high after the operation.
 *
 * @param opcode Pointer to the operation code to be sent.
 * @param hspi Handle to the SPI peripheral to use for communication.
 * @param GPIOx Pointer to the GPIO port to which the chip select pin is connected.
 * @param GPIO_Pin_CS Pin number of the chip select pin.
 *
 * @return The byte of data read from the device.
 */
HAL_StatusTypeDef spi_sendOp_readByte(uint8_t* opcode, SPI_HandleTypeDef *hspi, uint8_t* data_ptr, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_CS);

/**
 * @brief Performs an operation by sending an opcode via SPI.
 *
 * This function sends an operation code to a device over SPI. The chip select pin is set to low to initiate SPI communication,
 * the opcode is transmitted, and then the chip select pin is reset to high to end the operation.
 *
 * @param opcode Pointer to the operation code to be sent.
 * @param hspi Handle to the SPI peripheral to use for communication.
 * @param GPIOx Pointer to the GPIO port to which the chip select pin is connected.
 * @param GPIO_Pin_CS Pin number of the chip select pin.
 */
HAL_StatusTypeDef perform_operation(uint8_t* opcode, SPI_HandleTypeDef *hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_CS);

/**
 * @brief Reads data from a specific address via SPI.
 *
 * This function reads the entire contents of a page starting from the given address. It first splits the 24-bit address into three 8-bit integers.
 * Then it sets the chip select pin to low to initiate SPI communication, transmits the opcode, and the 24-bit address of the page to initialise read from.
 * Finally, it receives the data and sets the chip select pin to high to end the operation.
 *
 * @param opcode Pointer to the operation code to be sent.
 * @param data_size Size of the data to be read.
 * @param data_read Pointer to the buffer where the read data will be stored.
 * @param hspi Handle to the SPI peripheral to use for communication.
 * @param addr 24-bit address from where the data will be read.
 * @param GPIOx Pointer to the GPIO port to which the chip select pin is connected.
 * @param GPIO_Pin_CS Pin number of the chip select pin.
 */
HAL_StatusTypeDef spi_read_data(uint8_t* opcode, uint16_t data_size, uint8_t data_read[data_size], SPI_HandleTypeDef *hspi, uint32_t addr, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_CS);

/**
 * @brief Writes data to a specific address via SPI.
 *
 * This function writes a page of data starting from the given address. It first splits the 24-bit address into three 8-bit integers.
 * Then it sets the chip select pin to low to initiate SPI communication, transmits the opcode, and the 24-bit address of the page to initialise writing to.
 * Finally, it transmits the data and sets the chip select pin to high to end the operation.
 *
 * @param opcode Pointer to the operation code to be sent.
 * @param data_size Size of the data to be written.
 * @param data_write Pointer to the data to be written.
 * @param hspi Handle to the SPI peripheral to use for communication.
 * @param addr 24-bit address where the data will be written.
 * @param GPIOx Pointer to the GPIO port to which the chip select pin is connected.
 * @param GPIO_Pin_CS Pin number of the chip select pin.
 */
HAL_StatusTypeDef spi_write_data(uint8_t* opcode, uint16_t data_size, uint8_t data_write[data_size], SPI_HandleTypeDef *hspi, uint32_t addr, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_CS);

HAL_StatusTypeDef spi_write_data_dma(uint8_t* opcode, uint16_t data_size, uint8_t data_write[data_size], SPI_HandleTypeDef *hspi, uint32_t addr, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_CS);


#endif /* INC_SPI_DRIVER_H_ */
