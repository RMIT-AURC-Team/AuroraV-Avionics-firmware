
/* ===================================================================== *
 *                    LOW RESOLUTION DATA ACQUISITION                    *
 * ===================================================================== */

#include "lDataAcquisition.h"

extern long lDummyIdx;
char LdebugStr[100] = {};

/**
 * @brief Low-frequency data acquisition and altitude estimation function.
 *
 * Performs data acquisition for barometric pressure at a 50Hz rate.
 * Altitude is calculated from barometric pressure using the hypsometric formula
 * Kalman filter state matrices (A, Q, R, P) are initialized within the function.
 * Optionally, dummy data can be used for testing if the `DUMMY` macro is defined.
 *
 * Velocity and altitude state estimates are calculated with the Kalman filter
 * if enabled.
 *
 * @todo Add definition for sample period and replace assignments for dt and
 *       frequency (e.g. dt = 1/SAMPLE_PERIOD_LOW;).
 */
void vLDataAcquisition(void *argument) {
  float dt = 0.020;
  KalmanFilter kf;
  KalmanFilter_init(&kf);

  //! @todo Move kalman filter matrices into context struct
  // Initialise filter parameters
  float A[9] = {
      1.0, dt, 0.5 * (dt * dt),
      0.0, 1.0, dt,
      0.0, 0.0, 1.0
  };
  kf.A.pData = A;
  float Q[9] = {
      99.52, 0.0, 0.0,
      0.0, 1.42, 0.0,
      0.0, 0.0, 6.27
  };
  kf.Q.pData = Q;
  float R[4] = {
      97.92, 0.0,
      0.0, 0.61
  };
  kf.R.pData = R;
  float P[9] = {
      1, 0.0, 0.0,
      0.0, 0.1, 0.0,
      0.0, 0.0, 100.0
  };
  kf.P.pData = P;

  // Initialise measurement matrix
  arm_matrix_instance_f32 z;
  float zData[2] = {0.0, 0.0};
  arm_mat_init_f32(&z, 2, 1, zData);

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz
  const TickType_t blockTime  = pdMS_TO_TICKS(0);

  ctxLDataAcquisition *ctx    = (ctxLDataAcquisition *)argument;

  for (;;) {
    // Block until 20ms interval
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Update baro data
#ifdef DUMMY
    const unsigned long press_length = 0x00003A5C;
    if (lDummyIdx < PRESS_LENGTH - 1) {
      uint32_t tempPress = (uint32_t)press[lDummyIdx + 1] << 16 | press[lDummyIdx];
      memcpy(&ctx->baro->press, &tempPress, sizeof(float));
      lDummyIdx += 2;
    }
#else
    ctx->baro.update(&ctx->baro);
#endif

    // Calculate altitude
    ctx->state.altitude = 44330 * (1.0 - pow(ctx->baro.press / ctx->baro.groundPress, 0.1903));

    // Add sensor data and barometer data to dataframe
    ctx->mem.append(&ctx->mem, HEADER_LOWRES);
    ctx->mem.appendBytes(&ctx->mem, ctx->baro.rawTemp, BMP581_DATA_SIZE);
    ctx->mem.appendBytes(&ctx->mem, ctx->baro.rawPress, BMP581_DATA_SIZE);

    // Only run calculations when enabled
    EventBits_t uxBits = xEventGroupWaitBits(ctx->xTaskEnableGroup, GROUP_TASK_ENABLE_LOWRES, pdFALSE, pdFALSE, blockTime);
    if (uxBits & GROUP_TASK_ENABLE_LOWRES) {
      // Calculate state
      z.pData[0] = ctx->state.altitude;
      z.pData[1] = (ctx->state.cosine * 9.81 * ctx->accel->accelData[ZINDEX] - 9.81); // Acceleration measured in m/s^2
      kf.update(&kf, &z);

      ctx->state.velocity = kf.x.pData[1];
      ctx->state.avgPress.append(&ctx->state.avgPress, ctx->baro.press);
      ctx->state.avgVel.append(&ctx->state.avgVel, ctx->state.velocity);
    }

#ifdef DEBUG
    //! @todo extract debug print to function
    //! @todo move debug function to new source file with context as parameter
    if ((xSemaphoreTake(ctx->xUsbMutex, pdMS_TO_TICKS(0))) == pdTRUE) {
      char debugStr[100];
      snprintf(debugStr, 100, "[LDataAcq] %d\tBaro\tPressure: %.0f\n\r", lDummyIdx / 2, ctx->baro.press);
      xMessageBufferSend(ctx->xUsbTxBuff, (void *)debugStr, 100, pdMS_TO_TICKS(10));
      xSemaphoreGive(ctx->xUsbMutex);
    }
#endif
  }
}
