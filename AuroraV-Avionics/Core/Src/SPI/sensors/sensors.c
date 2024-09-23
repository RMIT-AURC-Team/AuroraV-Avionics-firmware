/***********************************************************************************
 * @file        sensors.c                                                          *
 * @author      Matt Ricci                                                         *
 *                                                                                 *
 * @todo Move initialisation of SPI interface to individual devices initialising   *
 *       relevant registers and peripherals.                                       *
 ***********************************************************************************/

#include "sensors.h"

void configure_SPI1_Sensor_Suite(void) { // change
	// SPI 1 SDI SDO SCLK PA7/6/5 respectively
  GPIOA->MODER &= (~(GPIO_MODER_MODE5_Msk | GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE7_Msk));
  GPIOA->MODER |= ((0x2 << GPIO_MODER_MODE5_Pos) | (0x2 << GPIO_MODER_MODE6_Pos) | (0x2 << GPIO_MODER_MODE7_Pos));
  GPIOA->AFR[0] &= (~(0xFFF00000));                                                                                        // clears AFRL 5, 6 and 7
  GPIOA->AFR[0] |= (0x55500000);                                                                                           // sets ports 5,6,7 to AF5

  GPIOA->OTYPER &= (~(GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7));                                               // configure as push pull
  GPIOA->OSPEEDR &= (~(GPIO_OSPEEDR_OSPEED5_Msk | GPIO_OSPEEDR_OSPEED6_Msk | GPIO_OSPEEDR_OSPEED7_Msk));                   // clears OSPEED
  GPIOA->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED5_Pos | 0x2 << GPIO_OSPEEDR_OSPEED6_Pos | 0x2 << GPIO_OSPEEDR_OSPEED7_Pos); // sets as high speed
  // pins PC2, PC3, PC4 need to be set to interrupt pins
  //
  //PA3, PA2 for ACC 1 CS and BAR CS add GYRO chip select as PF14
  GPIOA->MODER &= (~( GPIO_MODER_MODE2_Msk | GPIO_MODER_MODE3_Msk ));
  GPIOA->MODER |= ( (0x1 << GPIO_MODER_MODE2_Pos) | (0x1 << GPIO_MODER_MODE3_Pos) );
  GPIOA->OTYPER &= (uint32_t)(~(GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3));                                                             // sets 0xboth as push-pull
  GPIOA->OSPEEDR &= (~( GPIO_OSPEEDR_OSPEED2_Msk | GPIO_OSPEEDR_OSPEED3_Msk ));                                  // clears Port 14 and 15 section
  GPIOA->OSPEEDR |= ((0x2 << GPIO_OSPEEDR_OSPEED2_Pos) | (0x2 << GPIO_OSPEEDR_OSPEED3_Pos) ); // sets slew rate as high speed
  // PA2 Gryo Chip Select - set to high for default
  //  PA1 Accel 1 Chip Select - set to high for default
  // PA4 Mag - chip select - set to high for dafault

  GPIOA->ODR |= (GPIO_ODR_OD2 | GPIO_ODR_OD3); // sets to high to disable chip select
	
// CHANGE to PB1 for ACC 2 CS
  GPIOB->MODER &= (~(GPIO_MODER_MODE1_Msk));
  GPIOB->MODER |= (0x1 << GPIO_MODER_MODE1_Pos);
  GPIOB->OTYPER &= (~(GPIO_OTYPER_OT1));
  GPIOB->OSPEEDR &= (~(GPIO_OSPEEDR_OSPEED1_Msk));
  GPIOB->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED1_Pos);
 //Chip Select - set to high for default
  GPIOB->ODR |= (GPIO_ODR_OD1);
	// GRYO Chip Select PF14 configuration
  GPIOF->MODER &= (~(GPIO_MODER_MODE15_Msk));
  GPIOF->MODER |= (0x1 << GPIO_MODER_MODE15_Pos);
  GPIOF->OTYPER &= (~(GPIO_OTYPER_OT15));
  GPIOF->OSPEEDR &= (~(GPIO_OSPEEDR_OSPEED15_Msk));
  GPIOF->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED15_Pos);
  // Chip Select - set to high for default
  GPIOF->ODR |= (GPIO_ODR_OD15);
	//
	// SPI peripheral Configurations
  // Clear the First Control register of the SPI peripheral.
  	SPI1->CR1 &= 0xFFFF0000;

  	// Configure the SCLK to be divide by 8,
  	SPI1->CR1 |= (0x02 << SPI_CR1_BR_Pos) | (1 << SPI_CR1_CPOL_Pos) | (1 << SPI_CR1_CPHA_Pos) | (0 << SPI_CR1_DFF_Pos);


  	// Set to full duplex, master mode.
  	// In full duplex, both the MISO and MOSI pins are required.
  	SPI1->CR1 &= ~(SPI_CR1_BIDIMODE);
  	SPI1->CR1 &= ~(SPI_CR1_RXONLY);

  	// Set the slave select - software management.
  	SPI1->CR1 |= (SPI_CR1_SSM | SPI_CR1_SSI);

  	// Specify master operation.
  	SPI1->CR1 |= SPI_CR1_MSTR;

  	// Manually raise the chip select.
  	GPIOA->ODR |= (1 << GPIO_ODR_OD4_Pos) | (1 << GPIO_ODR_OD3_Pos) | (1 << GPIO_ODR_OD2_Pos) | (1 << GPIO_ODR_OD1_Pos);

  	// Enable the SPI peripheral
  	SPI1->CR1 |= SPI_CR1_SPE;



}
