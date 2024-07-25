#include "spi.h"

void SPI_init(SPI *spi, DeviceType device, SPI_TypeDef *interface, GPIO_TypeDef *port, unsigned long cs) {
  spi->device    = device;
  spi->interface = interface;
  spi->port      = port;
  spi->cs        = cs;
  spi->send      = SPI_send;
  spi->receive   = SPI_receive;
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
