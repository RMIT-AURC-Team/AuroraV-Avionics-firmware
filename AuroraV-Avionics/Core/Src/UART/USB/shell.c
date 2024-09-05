/***********************************************************************************
 * @file        control.c                                                          *
 * @author      Matt Ricci                                                         *
 ***********************************************************************************/

#include "shell.h"

void Shell_init(Shell *shell, UART usb, Flash flash) {
  shell->usb      = usb;
  shell->flash    = flash;
  shell->parse    = Shell_parse;
  shell->runClear = Shell_runClear;
  shell->runFlash = Shell_runFlash;
}

bool Shell_parse(Shell *shell, uint8_t *cmd) {
  bool ret;

  char *token = strtok((char *)cmd, " ");
  char *flags = strchr(token, '\0') + 1;

  if (!strcmp(token, CMD_CLEAR))
    ret = shell->runClear(shell);
  else if (!strcmp(token, CMD_FLASH))
    ret = shell->runFlash(shell, flags);

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
bool Shell_runClear(Shell *shell) {
  shell->usb.sendBytes(&shell->usb, (uint8_t *)"\033[3J\033[H\033[2J", 11);
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
bool Shell_runFlash(Shell *shell, char *flags) {
  if (flags == NULL)
    return false;

  // flash erase
  if (!strcmp(flags, CMD_FLASH_ERASE)) {
    shell->usb.print(&shell->usb, "Clearing flash... ");
    shell->flash.erase(&shell->flash);
    shell->usb.print(&shell->usb, "Done.\n\r");
  }
  // flash read all
  else if (!strcmp(flags, CMD_FLASH_READ_ALL)) {
    volatile uint8_t pageData[256];
    for (long i = 0; i < shell->flash.pageCount; i++) {
      shell->flash.readPage(&shell->flash, i * 0x100, pageData);
      for (int j = 0; j < shell->flash.pageSize; j++)
        shell->usb.send(&shell->usb, pageData[j]);
    }
  }

  return false;
}
