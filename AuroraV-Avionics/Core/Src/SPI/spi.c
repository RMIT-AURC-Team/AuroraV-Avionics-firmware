#include "spi.h"

void SPI_init(SPI *spi, DeviceType device, SPI_TypeDef *interface, GPIO_TypeDef *port, unsigned long cs) {
  spi->device    = device;
  spi->interface = interface;
  spi->port      = port;
  spi->cs        = cs;
  spi->send      = SPI_send;
  spi->receive   = SPI_receive;
  spi->transmit  = SPI_transmit;
}

uint16_t SPI_transmit(SPI *spi, uint16_t data) {
  uint16_t response;
  SPI_send(spi, data);
  SPI_receive(spi, &response);
  return response;
}

void SPI_send(SPI *spi, uint16_t data) {
  while ((spi->interface->SR & SPI_SR_TXE) == 0);
  spi->interface->DR = data;
  while ((spi->interface->SR & SPI_SR_BSY) == 1);
}

void SPI_receive(SPI *spi, uint16_t *data) {
  while ((spi->interface->SR & (SPI_SR_RXNE)) == 0);
  *data = spi->interface->DR;
}
