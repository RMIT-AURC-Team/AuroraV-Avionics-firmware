/* ===================================================================== *
 *                                   GPS                                 *
 * ===================================================================== */

#include "gpsAcquisition.h"

extern MessageBufferHandle_t xLoRaTxBuff;
extern MessageBufferHandle_t xUsbTxBuff;
extern SemaphoreHandle_t xUsbMutex;

void vGpsTransmit(void *argument) {
  const TickType_t xFrequency = pdMS_TO_TICKS(500);
  const TickType_t blockTime  = pdMS_TO_TICKS(0);
  char gpsString[100];
	
  enum State *flightState = StateHandle_getHandle("FlightState").state;

  for (;;) {
    // Block until 500ms interval
		TickType_t xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

		struct GPSData gps;
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
        (*flightState << 4) | gps.lock
    );
    // Add packet to queue
    xMessageBufferSend(xLoRaTxBuff, &gpsData, LORA_MSG_LENGTH, blockTime);
  }
}
