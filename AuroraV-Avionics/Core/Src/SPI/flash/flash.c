#include "flash.h"

/* SPI2 FLASH
 * ---------------------------------------------
 * Flash Pin       | MCU GPIO Pin  | SIGNAL TYPE
 * ----------------|---------------|------------
 * SDI             | PE14          | DATA
 * SDO             | PE13          | DATA
 * SCLK            | PE12          | DATA
 * CS              | PE11          | CONTROL
 * Memory Hold     | PE10          | CONTROL
 * Write Protect   | PE9           | CONTROL    */

void Flash_init(Flash *flash, GPIO_TypeDef *port, unsigned long cs) {
  SPI_init(&flash->base, MEMORY_FLASH, SPI4, port, cs);
  flash->erase     = Flash_erase;
  flash->readPage  = Flash_readPage;
  flash->writePage = Flash_writePage;
}

/******************************** PRIVATE METHODS ********************************/

void _Flash_writeEnable(Flash *flash) {
  SPI spi = flash->base;

  spi.port->ODR &= ~spi.cs;
  spi.transmit(&spi, 0x06);
  spi.port->ODR |= spi.cs;
}

void _Flash_readStatus1(Flash *flash, uint8_t *status) {
  SPI spi = flash->base;

  spi.port->ODR &= ~spi.cs;
  *status = spi.transmit(&spi, 0x05);
  *status = spi.transmit(&spi, 0x0F);
  spi.port->ODR |= spi.cs;
}

void _Flash_readStatus2(Flash *flash, uint8_t *status) {
  SPI spi = flash->base;

  spi.port->ODR &= ~spi.cs;
  *status = spi.transmit(&spi, 0x35);
  *status = spi.transmit(&spi, 0x0F);
  spi.port->ODR |= spi.cs;
}

void _Flash_readStatus3(Flash *flash, uint8_t *status) {
  SPI spi = flash->base;

  spi.port->ODR &= ~spi.cs;
  *status = spi.transmit(&spi, 0x15);
  *status = spi.transmit(&spi, 0x0F);
  spi.port->ODR |= spi.cs;
}

/******************************** DEVICE METHODS ********************************/

void Flash_erase(Flash *flash) {
  _Flash_writeEnable(flash);
  SPI spi        = flash->base;
  uint8_t status = 0;

  // Send Erase Chip instruction
  spi.port->ODR &= ~spi.cs;
  spi.transmit(&spi, 0x60);
  spi.port->ODR |= spi.cs;

  // Wait until chip BUSY is clear
  do {
    _Flash_readStatus1(flash, &status);
  } while (status & 0x01);
}

void Flash_writePage(Flash *flash, uint32_t address, uint8_t *data) {
  _Flash_writeEnable(flash);
  uint8_t response = 0;
  uint8_t status   = 0;
  SPI spi          = flash->base;

  do {
    _Flash_readStatus1(flash, &status);
  } while (status & 0x1);

  spi.port->ODR &= ~spi.cs;

  spi.transmit(&spi, 0x02);
  spi.transmit(&spi, (address & 0xFF0000) >> 16);
  spi.transmit(&spi, (address & 0xFF00) >> 8);
  spi.transmit(&spi, (address & 0xFF));

  // sent to flash
  for (int x = 0; x < 255; x++) {
    response = spi.transmit(&spi, data[x]);
  }

  spi.port->ODR |= spi.cs;
}

void Flash_readPage(Flash *flash, uint32_t address, uint8_t *data) {
  uint8_t status        = 0;
  uint16_t return_value = 0;
  SPI spi               = flash->base;

  do {
    _Flash_readStatus1(flash, &status);
  } while (status & 0x1);

  spi.port->ODR &= ~spi.cs;

  spi.transmit(&spi, 0x03);
  spi.transmit(&spi, (address & 0xFF0000) >> 16);
  spi.transmit(&spi, (address & 0xFF00) >> 8);
  spi.transmit(&spi, (address & 0xFF));

  // sent to flash
  for (int x = 0; x < 255; x++) {
    data[x] = spi.transmit(&spi, 0x01);
  }

  spi.port->ODR |= spi.cs;
}
