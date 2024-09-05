/* ===================================================================== *
 *                            STATE MANAGEMENT                           *
 * ===================================================================== */

#include "stateUpdate.h"

/**
 * @brief State update task.
 *
 * Handles transitions between different flight states, sends CAN messages
 * for aerobrakes and altitude data, and enables or disables various data
 * acquisition tasks based on the current state.
 *
 * @todo Add definition for update period and replace assignments for frequency
 *       (e.g. xFrequency = pdMS_TO_TICKS(STATE_UPDATE_PERIOD);).
 */
void vStateUpdate(void *argument) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz

  unsigned int CANHigh        = 0;
  unsigned int CANLow         = 0;
  unsigned int id             = 0;

  float avgPressCurrent       = 0;
  float avgPressPrevious      = 0;
  float avgVelCurrent         = 0;
  float avgVelPrevious        = 0;

  ctxFlightState *ctx         = (ctxFlightState *)argument;

  for (;;) {
    // Block until 20ms interval
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Emergency aerobrakes for excessive tilt
    if (ctx->state.tilt >= 30.0f) {
      // CAN payload for aerobrakes retract
      CANHigh = 0x00000000;
      CANLow  = 0x00000000;
      id      = CAN_HEADER_AEROBRAKES_RETRACT;
      CAN_TX(2, 8, CANHigh, CANLow, id);
    }

    switch (ctx->state.currentState) {
    case PRELAUNCH:
      if (ctx->accel->accelData[ZINDEX] >= ACCEL_LAUNCH) {
#ifdef FLIGHT_TEST
        GPIOB->ODR ^= 0x8000;
        GPIOD->ODR ^= 0x8000;
#endif
#ifndef DEBUG
        vTaskDelete(ctx->handles.xUsbTransmitHandle);
        vTaskDelete(ctx->handles.xUsbReceiveHandle);
#endif
        vTaskDelete(ctx->handles.xGpsTransmitHandle);
        xEventGroupSetBits(ctx->xTaskEnableGroup, GROUP_TASK_ENABLE_FLASH);   // Enable flash
        xEventGroupSetBits(ctx->xTaskEnableGroup, GROUP_TASK_ENABLE_HIGHRES); // Enable high resolution data acquisition
        xEventGroupSetBits(ctx->xTaskEnableGroup, GROUP_TASK_ENABLE_LOWRES);  // Enable low resolution data acquisition
        ctx->state.currentState = LAUNCH;
      }
      break;

    case LAUNCH:
      ctx->state.avgVel.calculateMovingAverage(&ctx->state.avgVel, &avgVelCurrent);
      // Send altitude to aerobrakes via CAN
      CANHigh = 0x00000000;
      memcpy(&CANLow, &ctx->state.altitude, sizeof(float));
      id = CAN_HEADER_AEROBRAKES_DATA;
      CAN_TX(2, 8, CANHigh, CANLow, id);
      // Transition to motor burnout state on velocity decrease
      if ((avgVelCurrent - avgVelPrevious) < 0) {
#ifdef FLIGHT_TEST
        GPIOB->ODR ^= 0x8000;
        GPIOD->ODR ^= 0x8000;
#endif
        ctx->state.currentState = COAST;
      }
      avgVelPrevious = avgVelCurrent;
      break;

    case COAST:
      ctx->state.avgPress.calculateMovingAverage(&ctx->state.avgPress, &avgPressCurrent);
      // Send altitude to aerobrakes via CAN
      CANHigh = 0x00000000;
      memcpy(&CANLow, &ctx->state.altitude, sizeof(float));
      id = CAN_HEADER_AEROBRAKES_DATA;
      CAN_TX(2, 8, CANHigh, CANLow, id);
      // Transition to apogee state on three way vote of altitude, velocity, and tilt
      // apogee is determined as two of three conditions evaluating true
      if ((((avgPressCurrent - avgPressPrevious) > 0) + (ctx->state.tilt >= 90) + (ctx->state.velocity < 0.0f)) >= 2) {
#ifdef FLIGHT_TEST
        GPIOB->ODR ^= 0x8000;
        GPIOD->ODR ^= 0x8000;
#endif
        vTaskDelete(ctx->handles.xHDataAcquisitionHandle);
        vTaskDelete(ctx->handles.xLDataAcquisitionHandle);
        vTaskDelete(ctx->handles.xLoRaSampleHandle);
        // xTaskCreate(ctx->handles.vGpsRead, "GpsRead", 512, NULL, configMAX_PRIORITIES - 6, &ctx->handles.xGpsReadHandle);
        ctx->state.currentState = APOGEE;
        // Send transmission to trigger apogee E-matches
      }
      avgPressPrevious = avgPressCurrent;
      break;

    case APOGEE:
      // Retract aerobrakes
      CANHigh = 0x00000000;
      CANLow  = 0x00000000;
      id      = CAN_HEADER_AEROBRAKES_RETRACT;
      CAN_TX(2, 8, CANHigh, CANLow, id);
      // Deploy drogue chute
      GPIOD->ODR |= 0x8000;
      // Transition to descent state when below main deployment altitude
      if (ctx->state.altitude <= MAIN_ALTITUDE_METERS) {
#ifdef FLIGHT_TEST
        GPIOB->ODR ^= 0x8000;
        GPIOD->ODR ^= 0x8000;
#endif
        ctx->state.currentState = DESCENT;
        // Add descent event dataframe to buffer
      }
      break;

    case DESCENT:
      // Handle descent state actions
      break;
    }
  }
}
