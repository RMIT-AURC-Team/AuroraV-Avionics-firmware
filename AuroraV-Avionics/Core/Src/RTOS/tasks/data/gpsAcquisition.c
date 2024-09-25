/* ===================================================================== *
 *                                   GPS                                 *
 * ===================================================================== */

#include "gpsAcquisition.h"

extern MessageBufferHandle_t xLoRaTxBuff;
extern MessageBufferHandle_t xUsbTxBuff;
extern SemaphoreHandle_t xUsbMutex;

void vGpsTransmit(void *argument) {
  const TickType_t xFrequency = pdMS_TO_TICKS(500);
  const TickType_t blockTime  = pdMS_TO_TICKS(250);
  char gpsString[100];
	
	GPS *gps 								= DeviceHandle_getHandle("GPS").device;
  enum State *flightState = StateHandle_getHandle("FlightState").state;

  for (;;) {
    // Block until 500ms interval
		TickType_t xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

  	struct GPS_Data gpsData;
    gps->message(gps, gpsString);
    gps->decode(gps, gpsString, &gpsData);

		#ifdef DEBUG
				//! @todo extract debug print to function
				//! @todo move debug function to new source file with context as parameter
				if ((xSemaphoreTake(xUsbMutex, pdMS_TO_TICKS(0))) == pdTRUE) {
					char debugStr[100];
					snprintf(debugStr, 100, "[GPS] %d:%d:%d\n\r", gpsData.hour, gpsData.minute, gpsData.second);
					xMessageBufferSend(xUsbTxBuff, (void *)debugStr, 100, 0);
					xSemaphoreGive(xUsbMutex);
				}
		#endif

    LoRa_Packet gpsPacket = LoRa_GPSData(
        LORA_HEADER_GPS_DATA,
        gpsData.latitude,
        gpsData.longitude,
        (*flightState << 4) | gpsData.lock
    );
    // Add packet to queue
    xMessageBufferSend(xLoRaTxBuff, &gpsPacket, LORA_MSG_LENGTH, blockTime);
  }
}
