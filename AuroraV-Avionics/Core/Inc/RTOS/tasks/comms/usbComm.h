#ifndef _USBTRANSCEIVE_H
#define _USBTRANSCEIVE_H

#include "FreeRTOS.h"
#include "message_buffer.h"
#include "stdint.h"
#include "stm32f439xx.h"

#include "shell.h"
#include "uart.h"

#define USB_TX_SIZE     4096
#define USB_RX_SIZE     128

#define SIGINT          0x03
#define BACKSPACE       0x08
#define LINE_FEED       0x0A
#define CARRIAGE_RETURN 0x0D

void vUsbReceive(void *);
void vUsbTransmit(void *);

typedef struct {
  UART usb;
} ctxUsbTransmit;

typedef struct {
  UART usb;
  Shell shell;
} ctxUsbReceive;

#endif
