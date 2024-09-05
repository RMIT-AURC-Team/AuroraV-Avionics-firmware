/**
 * @author Matt Ricci
 * @file shell.h
 */

#ifndef _SHELL_H
#define _SHELL_H

#include "FreeRTOS.h"
#include "stdbool.h"
#include "stdint.h"
#include "string.h"
#include "task.h"

#include "flash.h"
#include "uart.h"

#define CMD_CLEAR           "clear"
#define CMD_FLASH           "flash"
#define CMD_FLASH_ERASE     "erase"
#define CMD_FLASH_READ_PAGE "read page"
#define CMD_FLASH_READ_ALL  "read all"

extern Flash flash;
extern UART usb;

typedef struct Shell {
  UART usb;
  Flash flash;
  bool (*parse)(struct Shell *, uint8_t *);
  bool (*runClear)(struct Shell *);
  bool (*runFlash)(struct Shell *, char *);
} Shell;

void Shell_init(Shell *, UART, Flash);
bool Shell_parse(Shell *, uint8_t *);

bool Shell_runClear(Shell *);
bool Shell_runFlash(Shell *, char *);

bool usbCommandParse(Flash *, uint8_t *);
bool usbClearCommandExecute();
bool usbFlashCommandExecute(Flash *, char *);

#endif
