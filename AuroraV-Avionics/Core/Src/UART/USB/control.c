/***********************************************************************************
 * @file        control.c                                                          *
 * @author      Matt Ricci                                                         *
 ***********************************************************************************/

#include "control.h"

bool usbCommandParse(uint8_t *cmd) {
	bool ret;

	char *token = strtok((char *)cmd, " ");
	char *flags = strchr(token, '\0') + 1;
		
	if (!strcmp(token, CMD_CLEAR))
    ret = usbClearCommandExecute();
  else if (!strcmp(token, CMD_FLASH))
    ret = usbFlashCommandExecute(flags);
	
  return ret;
}

/* =============================================================================== */
/**
 * @brief Send clear sequence to host terminal
 *
 * @details `usbClearCommandExecute` transmits over UART an ANSI control sequence for 
 * clearing the host terminal window.
 **
 * =============================================================================== */
bool usbClearCommandExecute() {
	usb.sendBytes(&usb, (uint8_t *) "\033[3J\033[H\033[2J", 11);
	return true;
}

/* =============================================================================== */
/**
 * @brief Execute flash commands on target
 *
 * @details `usbFlashCommandExecute` parses and executes flash related commands 
 * according to the flags passed in by the top level command parser. 
 * @details Currently implemented commands include:
 * 	- Erase
 * 	- Read all
 **
 * =============================================================================== */
bool usbFlashCommandExecute(char *flags) {
  if (flags == NULL)
    return false;

	// flash erase
  if (!strcmp(flags, CMD_FLASH_ERASE)) {
		usb.print(&usb, "Clearing flash... " );
    flash.erase(&flash);
		usb.print(&usb, "Done.\n\r");
  }
	// flash read all
	else if (!strcmp(flags, CMD_FLASH_READ_ALL)) {
		volatile uint8_t pageData[256];
		for (long i = 0; i < flash.pageCount; i++) {
			flash.readPage(&flash, i * 0x100, pageData);
			for (int j = 0; j < flash.pageSize; j++)
				usb.send(&usb, pageData[j]);
		}		
	}
	
  return false;
}
