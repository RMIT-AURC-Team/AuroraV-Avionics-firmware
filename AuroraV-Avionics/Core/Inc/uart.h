/**
 * @author Matt Ricci
 * @defgroup UART
 */

#ifndef _UART_H
#define _UART_H

#include "stdbool.h"
#include "stm32f439xx.h"

/**
 * @addtogroup UART
 * @brief UART interface
 * @{
 */

typedef enum {
  OVER8,
  OVER16
} OversampleMode;

/**
 * @brief Struct definition for \ref UART "UART interface"
 * foobar
 */
typedef struct UART {
  USART_TypeDef *interface;
  GPIO_TypeDef *port;
  uint32_t baud;
  OversampleMode over8;
  void (*send)(struct UART *, uint8_t);             //!< UART send method.   	             @see UART_send
  void (*sendBytes)(struct UART *, uint8_t *, int); //!< UART send multiple bytes method.  @see UART_sendBytes
  uint8_t (*receive)(struct UART *);                //!< UART receive method.              @see UART_receive
} UART;

void UART_init(UART *, USART_TypeDef *, GPIO_TypeDef *, uint32_t, OversampleMode);
void _UART_setup(UART *);

void UART_send(UART *, uint8_t data);
void UART_sendBytes(UART *, uint8_t *data, int length);
uint8_t UART_receive(UART *);

/** @} */
#endif
