/* ===================================================================== *
 *                                PAYLOAD                                *
 * ===================================================================== */

#include "payloadComm.h"

extern MessageBufferHandle_t xLoRaTxBuff;
extern MessageBufferHandle_t xUsbTxBuff;
extern SemaphoreHandle_t xUsbMutex;

// 1: BARO1, 2: BARO2, 3: ACCEL, 4: STATUS, 7: RDY
EventGroupHandle_t xCANPayloadGroup;

void payloadCANRequest(unsigned int can, uint16_t id, unsigned int *out) {
	uint8_t groupBit = id - CAN_HEADER_PAYLOAD_BASE;
	xEventGroupSetBits(xCANPayloadGroup, groupBit);	// Set request bit in group
	
	// Request data from payload
	CAN_TX(can, 8, 0x00, 0x00, id);
		
	// Wait until response is received
	xEventGroupWaitBits(xCANPayloadGroup, groupBit | GROUP_CAN_PAYLOAD_RDY, pdTRUE, pdTRUE, portMAX_DELAY);
	
	// Set output data
	struct CAN_RX_data payloadRx = {.CAN_number = can};
	CAN_RX(&payloadRx);
	out[0] = payloadRx.dataL;
	out[1] = payloadRx.dataH;
}

void vPayloadTransmit(void *argument) {
  const TickType_t xFrequency = pdMS_TO_TICKS(500);
  const TickType_t blockTime  = pdMS_TO_TICKS(0);
	
	xCANPayloadGroup = xEventGroupCreate();

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

#if CAN_PAYLOAD_AV == 1
void CAN1_RX1_IRQHandler(void) {
#else
void CAN2_RX1_IRQHandler(void) {
#endif
	BaseType_t xHigherPriorityTaskWoken = pdFALSE, xResult;
	
	// Retrieve information on current communication status
	unsigned int id  = ((CAN_PAYLOAD_AV_INTF->sFIFOMailBox[1].RIR & 0xFFE00040) >> 21); // Get ID from mailbox
	uint8_t groupBit = (id - CAN_HEADER_PAYLOAD_BASE);																	// Determine test bit
	EventBits_t bits = xEventGroupGetBitsFromISR(xCANPayloadGroup);											// Retrieve set bits in group

	// Set ready bit if received ID matches currently set group bit
	if(bits & groupBit) {
		xResult = xEventGroupSetBitsFromISR(
				xCANPayloadGroup,
				GROUP_CAN_PAYLOAD_RDY,
				&xHigherPriorityTaskWoken
		);
	}

  if (xResult != pdFAIL)
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}