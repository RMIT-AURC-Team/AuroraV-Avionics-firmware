/***********************************************************************************
 * @file        uart.c                                                             *
 * @author      Matt Ricci                                                         *
 * @addtogroup  UART                                                               *
 * @brief       Brief description of the file's purpose.                           *
 *                                                                                 *
 * @todo Tidy up `_UART_setup`                                                     *
 * @todo Implement printf                                                          *
 * @todo Add println function                                                      *
 ***********************************************************************************/

#include "uart.h"

/* =============================================================================== */
/**
 * @brief Initialiser for a UART device interface.
 *
 * @param *uart 			Pointer to UART struct to be initialised.
 * @param *interface 	Pointer to UART interface struct.
 * @param *port 			Pointer to GPIO port struct.
 * @param baud 				UART baud rate.
 * @param over8 			Oversampling mode.
 * @return @c NULL.
 **
 * =============================================================================== */
DeviceHandle_t UART_init(
    UART *uart,
    char name[DEVICE_NAME_LENGTH],
    USART_TypeDef *interface,
    GPIO_TypeDef *port,
    uint32_t baud,
    OversampleMode over8
) {
  uart->send      = UART_send;
  uart->sendBytes = UART_sendBytes;
  uart->print     = UART_print;
  uart->receive   = UART_receive;
  uart->interface = interface;
  uart->port      = port;
  uart->baud      = baud;
  uart->over8     = over8;

  _UART_setup(uart);
  DeviceHandle_t handle;
  strcpy(handle.name, name);
  handle.device = uart;

  return handle;
}

/********************************** PRIVATE METHODS *********************************/

#ifndef DOXYGEN_PRIVATE

/* =============================================================================== */
/**
 * @brief Configures the UART interface for communication.
 *
 * @param *uart Pointer to UART struct containing configuration parameters.
 * @return @c NULL.
 **
 * =============================================================================== */
void _UART_setup(UART *uart) {
  GPIO_TypeDef *port       = uart->port;
  USART_TypeDef *interface = uart->interface;

  port->MODER &= ~(GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE7_Msk | GPIO_MODER_MODE8_Msk);
  port->MODER |= (0x02 << GPIO_MODER_MODE6_Pos) | (0x02 << GPIO_MODER_MODE7_Pos) | (0x2 << GPIO_MODER_MODE8_Pos);
  port->AFR[0] &= ~0xFF000000; // clears AFRL 6 and 7 and
  port->AFR[0] |= 0x88000000;  // sets PC 6 and 7 to AF8
  port->PUPDR |= (0x01 << GPIO_PUPDR_PUPD7_Pos);

  port->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED6;
  port->OSPEEDR |= (0x03 << GPIO_OSPEEDR_OSPEED6_Pos);

  uint16_t usartDiv = 168000000 / ((2 - (uart->over8)) * uart->baud);
  interface->BRR &= 0xFFFF0000; // clear mantissa and div in baud rate reg
  interface->BRR |= (usartDiv); // set mantissa and div in baud rate reg to 9600

  interface->CR1 &= ~0x400;     // disable parity
  interface->CR2 &= ~0xE00;     // disable synchrnous mode
  interface->CR3 &= ~0x300;     // disable flow control
  interface->CR1 |= 0x202C;     // enable usart, enable receive and transmitt
}

#endif

/********************************** INTERFACE METHODS ********************************/

/* =============================================================================== */
/**
 * @brief Sends a single byte of data over the UART interface.
 *
 * @param *uart  Pointer to UART struct.
 * @param data   Byte of data to be sent.
 * @return @c NULL.
 **
 * =============================================================================== */
void UART_send(UART *uart, uint8_t data) {
  USART_TypeDef *interface = uart->interface;
  while ((interface->SR & USART_SR_TXE) == 0);
  interface->DR = data;
  while ((interface->SR & USART_SR_TC) == 0);
}

/* =============================================================================== */
/**
 * @brief Sends an array of bytes over the UART interface.
 *
 * @param *uart   Pointer to UART struct.
 * @param *data   Pointer to the array of bytes to be sent.
 * @param length  Number of bytes to send.
 * @return @c NULL.
 **
 * =============================================================================== */
void UART_sendBytes(UART *uart, uint8_t *data, int length) {
  for (int i = 0; i < length; i++)
    UART_send(uart, data[i]);
}

/* =============================================================================== */
/**
 * @brief Sends a string of characters over the UART interface.
 *
 * @param *uart  Pointer to UART struct.
 * @param *data  Pointer to the string of characters to be sent.
 * @return @c NULL.
 **
 * =============================================================================== */

void UART_print(UART *uart, char *data) {
  int i = 0;
  while (data[i] != '\0')
    UART_send(uart, data[i++]);
}

/* =============================================================================== */
/**
 * @brief Receives a single byte of data from the UART interface.
 *
 * @param *uart  Pointer to UART struct.
 * @return       The received byte of data.
 **
 * =============================================================================== */
uint8_t UART_receive(UART *uart) {
  USART_TypeDef *interface = uart->interface;
  while ((interface->SR & USART_SR_RXNE) == 0);
  return (uint8_t)(interface->DR & 0xFF);
}
