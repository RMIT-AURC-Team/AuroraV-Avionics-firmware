/**
 * @author Matt Ricci
 * @file control.h
 */

#ifndef _CONTROL_H
#define _CONTROL_H

#include "stdbool.h"
#include "stdint.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"

#include "flash.h"
#include "uart.h"

#define CMD_CLEAR 						"clear"
#define CMD_FLASH       			"flash"
#define CMD_FLASH_ERASE 			"erase"
#define CMD_FLASH_READ_PAGE 	"read page"
#define CMD_FLASH_READ_ALL 		"read all"

extern Flash flash;
extern UART usb;

bool usbCommandParse(uint8_t *);

bool usbClearCommandExecute();
bool usbFlashCommandExecute(uint8_t *);

#endif
