/**
 * @author Matt Ricci
 * @file uart.c
 * @addtogroup UART
 * @todo Tidy up `_UART_setup`
 */

#include "uart.h"

/* =============================================================================== */
/**
 * @brief Initialiser for a UART device interface.
 * @param *uart 			Pointer to UART struct to be initialised.
 * @param *interface 	Pointer to UART interface struct.
 * @param *port 			Pointer to GPIO port struct.
 * @param baud 				UART baud rate.
 * @param over8 			Oversampling mode.
 * @return @c NULL.
 **
 * =============================================================================== */
void UART_init(UART *uart, USART_TypeDef *interface, GPIO_TypeDef *port, uint32_t baud, OversampleMode over8) {
  uart->send      = UART_send;
  uart->sendBytes = UART_sendBytes;
  uart->receive   = UART_receive;
  uart->interface = interface;
  uart->port      = port;
  uart->baud      = baud;
  uart->over8     = over8;

  _UART_setup(uart);
}

/********************************** PRIVATE METHODS *********************************/

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
  interface->CR1 |= 0x200C;     // enable usart, enable receive and transmitt
}

/********************************** INTERFACE METHODS ********************************/

void UART_send(UART *uart, uint8_t data) {
  USART_TypeDef *interface = uart->interface;
  while ((interface->SR & USART_SR_TXE) == 0);
  interface->DR = data;
  while ((interface->SR & USART_SR_TC) == 0);
}

void UART_sendBytes(UART *uart, uint8_t *data, int length) {
  for (int i = 0; i < length; i++)
    UART_send(uart, data[i]);
}

uint8_t UART_receive(UART *uart) {
  USART_TypeDef *interface = uart->interface;
  while ((interface->SR & USART_SR_RXNE) == 0);
  return (uint8_t)(interface->DR & 0xFF);
}
