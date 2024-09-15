/* ===================================================================== *
 *                             LORA HANDLING                             *
 * ===================================================================== */

#include "loraComm.h"

extern EventGroupHandle_t xMsgReadyGroup;
extern MessageBufferHandle_t xLoRaTxBuff;

/**
 * @brief LoRa transmit task.
 *
 * Handles transmission of data to the SX1272 transceiver. It waits for the LoRa
 * module to be ready, then reads a message from the LoRa transmit buffer and
 * sends it via the SX1272. The ready flag is cleared after transmission.
 */
void vLoRaTransmit(void *argument) {
  const TickType_t blockTime = portMAX_DELAY;
  uint8_t rxData[LORA_MSG_LENGTH];

  ctxLoRaTransmit *ctx = (ctxLoRaTransmit *)argument;

  for (;;) {
    // Wait for SX1272 to be ready for transmission
    EventBits_t uxBits = xEventGroupWaitBits(xMsgReadyGroup, GROUP_MESSAGE_READY_LORA, pdFALSE, pdFALSE, blockTime);
    if ((uxBits & GROUP_MESSAGE_READY_LORA)) {
      // Wait to receive message in buffer
      size_t xReceivedBytes = xMessageBufferReceive(
          xLoRaTxBuff,
          (void *)rxData,
          sizeof(rxData),
          blockTime
      );
      // Transmit if message is available
      if (xReceivedBytes) {
        ctx->lora.transmit(&ctx->lora, rxData);
        xEventGroupClearBits(xMsgReadyGroup, GROUP_MESSAGE_READY_LORA);
      }
    }
  }
}

/**
 * @brief LoRa sample task.
 *
 * Samples current sensor data from RAM every 250ms and queues it to be transmitted
 * by `vLoRaTransmit`. The task creates a LoRa packet containing accelerometer,
 * gyroscope, altitude, and velocity data, which is then appended to the transmission
 * queue.
 */
void vLoRaSample(void *argument) {
  const TickType_t blockTime  = pdMS_TO_TICKS(0);
  const TickType_t xFrequency = pdMS_TO_TICKS(250);

  ctxLoRaSample *ctx          = (ctxLoRaSample *)argument;

  for (;;) {
    // Block until 250ms interval
		TickType_t xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Create AVData packet with current data
    LoRa_Packet avData = LoRa_AVData(
        LORA_HEADER_AV_DATA,
        ctx->state.currentState,
        ctx->lAccel.rawAccelData,
        ctx->hAccel.rawAccelData,
        KX134_1211_DATA_TOTAL,
        ctx->gyro.rawGyroData,
        A3G4250D_DATA_TOTAL,
        ctx->state.altitude,
        ctx->state.velocity
    );

    // Add packet to queue
    xMessageBufferSend(xLoRaTxBuff, &avData, LORA_MSG_LENGTH, blockTime);
  }
}

/**
 * @brief LoRa Tx complete interrupt handler.
 *
 * Handles the external interrupt triggered by the Tx complete signal from the
 * SX1272 transceiver. Upon interrupt, the LoRa ready flag is set in
 * `xMsgReadyGroup`.
 */
void EXTI1_IRQHandler(void) {
  EXTI->PR |= (0x02);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE, xResult;

  //
  xResult = xEventGroupSetBitsFromISR(
      xMsgReadyGroup,
      GROUP_MESSAGE_READY_LORA,
      &xHigherPriorityTaskWoken
  );

  if (xResult != pdFAIL)
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
