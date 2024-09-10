/* ===================================================================== *
 *                            FLASH HANDLING                             *
 * ===================================================================== */

#include "flashWrite.h"

extern EventGroupHandle_t xTaskEnableGroup;

/**
 * @brief Idle hook for writing to flash.
 *
 * Uses idle time to check if there is a page available in the buffer. If a page
 * is available and the rocket state is past `LAUNCH`, it sets the flash write
 * flag to trigger the flash buffer task.
 */
void vIdle(void *argument) {

  ctxIdle *ctx = (ctxIdle *)argument;

  for (;;) {
    // Write if a page is available in the buffer
    if (*ctx->currentState >= LAUNCH && ctx->mem.pageReady)
      xEventGroupSetBits(xTaskEnableGroup, GROUP_TASK_ENABLE_FLASH);
  }
}

/**
 * @brief Flash buffer write task.
 *
 * Monitors flash enable flag to determine when to flush data from the memory buffer to
 * flash memory. The task waits for the flag to be set, reads data into a buffer, writes
 * the data to flash, and then updates the page address.
 *
 * @bug Data written to flash currently shows signs of corruption, potentially due
 *      to issues with buffering. <b>This is a critical error.</b>
 */
void vFlashBuffer(void *argument) {
  const TickType_t timeout = pdMS_TO_TICKS(1);
  uint32_t pageAddr        = 0;

  Flash *flash             = DeviceHandle_getHandle("Flash").device;
  ctxFlashBuffer *ctx      = (ctxFlashBuffer *)argument;
  uint8_t outBuff[flash->pageSize];

  for (;;) {
    // Wait for write flag to be ready, clear flag on exit
    EventBits_t uxBits = xEventGroupWaitBits(xTaskEnableGroup, GROUP_TASK_ENABLE_FLASH, pdTRUE, pdFALSE, timeout);
    if (uxBits & GROUP_TASK_ENABLE_FLASH) {
      bool success = ctx->mem.readPage(&ctx->mem, outBuff); // Flush data to output buffer
      if (success) {
        // Write data to flash memory
        flash->writePage(flash, pageAddr, outBuff);
        pageAddr += 0x100;
      }
    }
  }
}
