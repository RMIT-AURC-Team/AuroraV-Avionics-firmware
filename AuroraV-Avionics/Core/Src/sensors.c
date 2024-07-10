#include "sensors.h"

void configure_Sensors() {
  // Configure accelerometer 1
  write_ACCEL_1(ACCEL_CNTL1, ACCEL_CNTL1_RES | ACCEL_CNTL1_GSEL_32G);    // Accel select, 32g sensitivity
  uint8_t ODCNTL = read_ACCEL_1(ACCEL_ODCNTL);                           // Read from register for reserve mask
  write_ACCEL_1(ACCEL_ODCNTL, (ACCEL_ODCNTL_RESERVED & ODCNTL) | 0x2A);  // No filter, fast startup, 800Hz
  write_ACCEL_1(ACCEL_CNTL1, 0xD0);                                      // Enable PC1

  // Configure accelerometer 2
  write_ACCEL_2(ACCEL_CNTL1, ACCEL_CNTL1_RES | ACCEL_CNTL1_GSEL_32G);    // Accel select, 16g sensitivity
  ODCNTL = read_ACCEL_2(ACCEL_ODCNTL);                          				  // Read from register for reserve mask
  write_ACCEL_2(ACCEL_ODCNTL, (ACCEL_ODCNTL_RESERVED & ODCNTL) | 0x2A);  // No filter, fast startup, 800Hz
  write_ACCEL_2(ACCEL_CNTL1, 0xD0);                                      // Enable PC1

  // Configure gyroscope
  write_GYRO(GYRO_CTRL_REG1, GYRO_CTRL_REG1_ODR_800Hz | GYRO_CTRL_REG1_AXIS_ENABLE | GYRO_CTRL_REG1_PD_ENABLE);
	
  // Configure magnetometer
  write_MAG(MAGNET_CTRL_REG1, MAGNET_CTRL_REG1_FAST);
  write_MAG(MAGNET_CTRL_REG2, MAGNET_CTRL_REG2_FS16);

  // Configure barometer
  write_BARO(BARO_ODR_CFG, BARO_ODR_CFG_PWR | BARO_ODR_CFG_DEEP_DIS);
  uint8_t OSRCFG = read_BARO(BARO_OSR_CFG);
  write_BARO(BARO_OSR_CFG, (BARO_OSR_CFG_RESERVED * OSRCFG) | BARO_OSR_CFG_PRESS_EN);
}

uint8_t read_ACCEL_1(uint8_t address) {
  uint16_t return_value;
  uint16_t payload_to_send  = 0;
  payload_to_send          |= 0x8000;
  payload_to_send          |= (address << 0x8); // load address into top 7 bits
  while ((SPI1->SR & SPI_SR_TXE) == 0);
  GPIOA->ODR &= (~(GPIO_ODR_OD1));
  SPI1->DR    = payload_to_send;
  while ((SPI1->SR & SPI_SR_RXNE) == 0);        // wait for received data
  GPIOA->ODR   |= (GPIO_ODR_OD1);
  return_value  = (uint8_t)(SPI1->DR);
  return (uint8_t)return_value;
}

void write_ACCEL_1(uint8_t address, uint8_t payload) {
  uint16_t return_value;
  uint16_t payload_to_send  = 0;
  payload_to_send          &= (~(0x8000));      // 0 write, dont need this lne really
  payload_to_send          |= (address << 0x8); // load address into top 7 bits
  payload_to_send          |= payload;          // load data in to write
  while ((SPI1->SR & SPI_SR_TXE) == 0);         // wait for transmission to be empty
  GPIOA->ODR &= (~(GPIO_ODR_OD1));              // lower gyro chip select
  SPI1->DR    = payload_to_send;
  while ((SPI1->SR & SPI_SR_RXNE) == 0);        // wait for received data
  return_value = (uint16_t)(SPI1->DR);
  while ((SPI1->SR & SPI_SR_BSY) == 1);
  GPIOA->ODR |= (GPIO_ODR_OD1);
}

uint8_t read_ACCEL_2(uint8_t address) {
		uint16_t return_value = 0;
		uint16_t payload_to_send = 0;
		payload_to_send |= 0x8000;
		payload_to_send |= (address << 0x8);// load address into top 7 bits
		while((SPI1->SR & SPI_SR_TXE)== 0);
		GPIOB->ODR &= (~(GPIO_ODR_OD0));
		SPI1->DR = payload_to_send;
		while((SPI1->SR & SPI_SR_RXNE)==0);// wait for received data
		GPIOB->ODR |= (GPIO_ODR_OD0);
		return_value = (uint16_t)(SPI1->DR);
		return (uint8_t)return_value;
}
	
	
void write_ACCEL_2(uint8_t address, uint8_t payload) {
	uint16_t return_value;
	uint16_t payload_to_send = 0;
	payload_to_send &= (~(0x8000));// 0 write, dont need this lne really
	payload_to_send |= (address << 0x8);// load address into top 7 bits
	payload_to_send |= payload;// load data in to write
	GPIOB->ODR &= (~(GPIO_ODR_OD0));// lower gyro chip select
	while((SPI1->SR & SPI_SR_TXE)== 0);// wait for transmission to be empty
	SPI1->DR = payload_to_send;  
	while((SPI1->SR & SPI_SR_RXNE)==0);// wait for received data
	return_value = (uint16_t)(SPI1->DR);
	while((SPI1->SR & SPI_SR_BSY)== 1);
	GPIOB->ODR |= (GPIO_ODR_OD0);
}


uint8_t read_GYRO(uint8_t address) {
  uint16_t return_value;
  uint16_t payload_to_send  = 0;
  payload_to_send          &= (~(0x4000));      // clear  multiple transfer bit
  payload_to_send          |= 0x8000;
  payload_to_send          |= (address << 0x8); // load address into top 7 bits
  while ((SPI1->SR & SPI_SR_TXE) == 0);
  GPIOA->ODR &= (~(GPIO_ODR_OD2));
  SPI1->DR    = payload_to_send;
  while ((SPI1->SR & SPI_SR_RXNE) == 0);        // wait for received data
  GPIOA->ODR   |= (GPIO_ODR_OD2);
  return_value  = (uint16_t)(SPI1->DR);

  return (uint8_t)return_value;
}

void write_GYRO(uint8_t address, uint8_t payload) {
			uint16_t return_value;
			uint16_t payload_to_send = 0;
			payload_to_send &= (~(0X4000)); // clear  multiple transfer bit
			payload_to_send &= (~(0x8000));// set 15th bit to 0 for write
			payload_to_send |= (address << 0x8);// load address into top 7 bits
			payload_to_send |= (payload);// mask in data
			GPIOA->ODR &= (~(GPIO_ODR_OD2));// lower gyro chip select
			while((SPI1->SR & SPI_SR_TXE)== 0);// wait for transmission to be empty
			SPI1->DR = payload_to_send;
			while((SPI1->SR & SPI_SR_RXNE)==0);// wait for received data
			return_value = (uint16_t)(SPI1->DR);
			while((SPI1->SR & SPI_SR_BSY)== 1);
			GPIOA->ODR |= (GPIO_ODR_OD2);
}

void write_MAG(uint8_t address, uint8_t payload) {
  uint16_t return_value     = 0;
  uint16_t payload_to_send  = 0;
  payload_to_send          &= (~(0X4000));       // clear  multiple transfer bit
  payload_to_send          &= (~(0x8000));       // set 15th bit to 0 for write
  payload_to_send          |= (address << 0x8);  // load address into top 7 bits
  payload_to_send          |= (payload);         // mask in data
  GPIOA->ODR               &= (~(GPIO_ODR_OD4)); // lower gyro chip select
  while ((SPI1->SR & SPI_SR_TXE) == 0);          // wait for transmission to be empty
  SPI1->DR = payload_to_send;
  while ((SPI1->SR & SPI_SR_RXNE) == 0);         // wait for received data
  return_value = (uint16_t)(SPI1->DR);
  while ((SPI1->SR & SPI_SR_BSY) == 1);
  GPIOA->ODR |= (GPIO_ODR_OD4);
}

uint8_t read_MAG(uint8_t address) {
  uint16_t return_value     = 0;
  uint16_t payload_to_send  = 0;
  payload_to_send          &= (~(0X4000));      // clear  multiple transfer bit
  payload_to_send          |= 0x8000;
  payload_to_send          |= (address << 0x8); // load address into top 7 bits
  // payload_to_send |= (payload<<0X8);// mask in data
  GPIOA->ODR &= (~(GPIO_MODER_MODE2));   // lower gyro chip select
  while ((SPI1->SR & SPI_SR_TXE) == 0);
  GPIOA->ODR &= (~(GPIO_ODR_OD4));
  SPI1->DR    = payload_to_send;
  while ((SPI1->SR & SPI_SR_RXNE) == 0); // wait for received data
  GPIOA->ODR   |= (GPIO_ODR_OD4);
  return_value  = (uint16_t)(SPI1->DR);
  return (uint8_t)return_value;
}

void write_BARO(uint8_t address, uint8_t payload) {
  uint16_t return_value     = 0;
  uint16_t payload_to_send  = 0;
  payload_to_send          &= (~(0x8000));       // 0 write, dont need this lne really
  payload_to_send          |= (address << 0x8);  // load address into top 7 bits
  payload_to_send          |= payload;           // load data in to write
  GPIOA->ODR               &= (~(GPIO_ODR_OD3)); // lower gyro chip select
  while ((SPI1->SR & SPI_SR_TXE) == 0);          // wait for transmission to be empty
  SPI1->DR = payload_to_send;
  while ((SPI1->SR & SPI_SR_RXNE) == 0);         // wait for received data
  return_value = (uint16_t)(SPI1->DR);
  while ((SPI1->SR & SPI_SR_BSY) == 1);
  GPIOA->ODR |= (GPIO_ODR_OD3);
}

uint8_t read_BARO(uint8_t address) {
  uint16_t return_value;
  uint16_t payload_to_send  = 0;
  payload_to_send          |= 0x8000;            // 1 read
  payload_to_send          |= (address << 0x8);  // load address into top 7 bits
  GPIOA->ODR               &= (~(GPIO_ODR_OD3)); // lower gyro chip select
  while ((SPI1->SR & SPI_SR_TXE) == 0);          // wait for transmission to be empty
  SPI1->DR = payload_to_send;
  while ((SPI1->SR & SPI_SR_RXNE) == 0);         // wait for received data
  return_value = (uint16_t)(SPI1->DR);
  while ((SPI1->SR & SPI_SR_BSY) == 1);
  GPIOA->ODR |= (GPIO_ODR_OD3);
  return return_value;
}
