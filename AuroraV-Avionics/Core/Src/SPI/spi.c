#include "spi.h"

/* =============================================================================== */
/**
 * @brief Initialiser for an SPI device interface.
 * @param *spi 				Pointer to SPI struct to be initialised.
 * @param device 			Enum specifier for device type.
 * @param *interface 	Pointer to SPI interface struct.
 * @param *port 			Pointer to GPIO port struct.
 * @param cs 					Device chip select address.
 * @return @c NULL.
 **
 * =============================================================================== */
void SPI_init(SPI *spi, DeviceType device, SPI_TypeDef *interface, GPIO_TypeDef *port, unsigned long cs) {
  spi->device    = device;
  spi->interface = interface;
  spi->port      = port;
  spi->cs        = cs;
  spi->send      = SPI_send;
  spi->receive   = SPI_receive;
  spi->transmit  = SPI_transmit;
}

/* =============================================================================== */
/**
 * @brief Instance method to communicate a SPI transaction with slave device.
 * @param 	*spi 			Pointer to SPI struct.
 * @param 	data 			Data payload to be sent to slave device.
 * @retval 	response 	Returns the slave device response from the transaction.
 **
 * =============================================================================== */
uint16_t SPI_transmit(SPI *spi, uint16_t data) {
  uint16_t response;
  SPI_send(spi, data);
  SPI_receive(spi, &response);
  while ((spi->interface->SR & SPI_SR_BSY) == SPI_SR_BSY);
  return response;
}

/* =============================================================================== */
/**
 * @brief Send data through the SPI interface.
 * @param 	*spi 			Pointer to SPI struct.
 * @param   data      The data to send.
 * @return @c NULL.
 **
 * =============================================================================== */
void SPI_send(SPI *spi, uint16_t data) {
  while ((spi->interface->SR & SPI_SR_TXE) == 0);
  spi->interface->DR = data;
}

/* =============================================================================== */
/**
 * @brief Receive data through the SPI interface.
 * @param 	*spi 			Pointer to SPI struct.
 * @param   data      Pointer to variable to receive data into.
 * @return @c NULL.
 **
 * =============================================================================== */
void SPI_receive(SPI *spi, uint16_t *data) {
  while ((spi->interface->SR & (SPI_SR_RXNE)) == 0);
  *data = spi->interface->DR;
}
