/* ===================================================================== *
 *                                PAYLOAD                                *
 * ===================================================================== */

#include "payloadComm.h"

extern MessageBufferHandle_t xLoRaTxBuff;
extern MessageBufferHandle_t xUsbTxBuff;
extern SemaphoreHandle_t xUsbMutex;

void payloadCANRequest(unsigned int can, uint16_t id, unsigned int *out) {
	// Request data from payload
	CAN_TX(can, 8, 0x00, 0x00, id);
		
	// Wait until response is received
	struct CAN_RX_data payloadRx = {.CAN_number = can};
	while(CAN_RX(&payloadRx) != 0x01 && payloadRx.address != id);
	out[0] = payloadRx.dataL;
	out[1] = payloadRx.dataH;
}

void vPayloadTransmit(void *argument) {
  const TickType_t xFrequency = pdMS_TO_TICKS(500);
  const TickType_t blockTime  = pdMS_TO_TICKS(0);

  for (;;) {
    // Block until 250ms interval
		TickType_t xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

		unsigned int payloadState[2];
	  payloadCANRequest(CAN_PAYLOAD_AV, CAN_HEADER_PAYLOAD_STATUS, payloadState);
		
		unsigned int payloadAccel[2];
		payloadCANRequest(CAN_PAYLOAD_AV, CAN_HEADER_PAYLOAD_ACCEL, payloadAccel);
    		
		LoRa_Packet payloadData = LoRa_PayloadData(
        LORA_HEADER_PAYLOAD_DATA,
				(uint8_t) payloadState[0],
        (uint8_t *) payloadAccel,
        PAYLOAD_ACCEL_TOTAL
    );
    // Add packet to queue
    xMessageBufferSend(xLoRaTxBuff, &payloadData, LORA_MSG_LENGTH, blockTime);
  }
}
