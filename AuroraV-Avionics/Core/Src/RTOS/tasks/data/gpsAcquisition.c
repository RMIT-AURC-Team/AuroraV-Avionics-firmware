/* ===================================================================== *
 *                                   GPS                                 *
 * ===================================================================== */

#include "gpsAcquisition.h"

extern MessageBufferHandle_t xLoRaTxBuff;
extern MessageBufferHandle_t xUsbTxBuff;
extern SemaphoreHandle_t xUsbMutex;

void vGpsTransmit(void *argument) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(500);
  const TickType_t blockTime  = pdMS_TO_TICKS(0);
  char gpsString[100];

  struct GPSData gps;
  ctxGpsTransmit *ctx = (ctxGpsTransmit *)argument;

  for (;;) {
    // Block until 500ms interval
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    GPS_message(gpsString);
    DecodeGPS(gpsString, &gps);

#ifdef DEBUG
    //! @todo extract debug print to function
    //! @todo move debug function to new source file with context as parameter
    if ((xSemaphoreTake(xUsbMutex, pdMS_TO_TICKS(0))) == pdTRUE) {
      char debugStr[100];
      snprintf(debugStr, 100, "[GPS] %d:%d:%d\n\r", gps.hour, gps.minute, gps.second);
      xMessageBufferSend(xUsbTxBuff, (void *)debugStr, 100, 0);
      xSemaphoreGive(xUsbMutex);
    }
#endif

    LoRa_Packet gpsData = LoRa_GPSData(
        LORA_HEADER_GPS_DATA,
        gps.latitude,
        gps.longitude,
        (*ctx->currentState << 4) | gps.lock
    );
    // Add packet to queue
    xMessageBufferSend(xLoRaTxBuff, &gpsData, LORA_MSG_LENGTH, blockTime);
  }
}
