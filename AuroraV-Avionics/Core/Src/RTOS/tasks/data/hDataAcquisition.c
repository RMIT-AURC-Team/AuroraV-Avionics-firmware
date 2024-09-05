/* ===================================================================== *
 *                  HIGH RESOLUTION DATA ACQUISITION                     *
 * ===================================================================== */

#include "HDataAcquisition.h"

extern long hDummyIdx;
char HdebugStr[100] = {};

extern SemaphoreHandle_t xUsbMutex;
extern MessageBufferHandle_t xUsbTxBuff;

/**
 * @brief High-frequency data acquisition task.
 *
 * Acquires sensor data at a 500Hz rate. It selects the appropriate accelerometer
 * based on current data, processes sensor data and appends the processed data
 * to a dataframe. Optionally, dummy data can be used for testing if the `DUMMY`
 * macro is defined.
 *
 * Quaternion integration and tilt angle calculations are performed if enabled.
 *
 * @todo Add definition for sample period and replace assignments for dt and
 *       frequency (e.g. dt = 1/SAMPLE_PERIOD_HIGH;).
 * @todo Refactor *ctx definition to *ctxPtr, add dereferenced context after
 *       task unblock to improve readability of context access.
 */
void vHDataAcquisition(void *argument) {
  float dt = 0.002;

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(2); // 500Hz
  const TickType_t blockTime  = pdMS_TO_TICKS(0);

  ctxHDataAcquisition *ctx    = (ctxHDataAcquisition *)argument;
  KX134_1211 *accel           = ctx->accel;

  for (;;) {
    // Block until 2ms interval
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Select which accelerometer to use
    ctx->accel = (accel->accelData[ZINDEX] < 15) ? ctx->lAccel : ctx->hAccel;

#ifdef DUMMY
    // Load bearing definition???
    const unsigned long accelX_length = 0x00007568;
    /*
     * Update sensor data with dummy values
     * These arrays are defined in the files under /Data and are generated from
     * past flight data binaries with srec_cat.
     */
    if (hDummyIdx < ACCELX_LENGTH - 1) {
      // Shift in floating point values and add to processed accelerometer array
      uint32_t tempX = (uint32_t)accelX[hDummyIdx + 1] << 16 | accelX[hDummyIdx];
      uint32_t tempY = (uint32_t)accelY[hDummyIdx + 1] << 16 | accelY[hDummyIdx];
      uint32_t tempZ = (uint32_t)accelZ[hDummyIdx + 1] << 16 | accelZ[hDummyIdx];
      memcpy(&accel->accelData[0], &tempX, sizeof(float));
      memcpy(&accel->accelData[1], &tempY, sizeof(float));
      memcpy(&accel->accelData[2], &tempZ, sizeof(float));

      // Back convert to raw data
      uint16_t xRaw          = (short)(accel->accelData[0] / accel->sensitivity);
      uint16_t yRaw          = (short)(accel->accelData[1] / accel->sensitivity);
      uint16_t zRaw          = (short)(accel->accelData[2] / accel->sensitivity);
      accel->rawAccelData[0] = xRaw >> 8;
      accel->rawAccelData[1] = xRaw;
      accel->rawAccelData[2] = yRaw >> 8;
      accel->rawAccelData[3] = yRaw;
      accel->rawAccelData[4] = zRaw >> 8;
      accel->rawAccelData[5] = zRaw;

      // Shift in floating point values and add to processed gyroscope array
      tempX = (uint32_t)gyroX[hDummyIdx + 1] << 16 | gyroX[hDummyIdx];
      tempY = (uint32_t)gyroY[hDummyIdx + 1] << 16 | gyroY[hDummyIdx];
      tempZ = (uint32_t)gyroZ[hDummyIdx + 1] << 16 | gyroZ[hDummyIdx];
      memcpy(&ctx->gyro.gyroData[0], &tempX, sizeof(float));
      memcpy(&ctx->gyro.gyroData[1], &tempY, sizeof(float));
      memcpy(&ctx->gyro.gyroData[2], &tempZ, sizeof(float));

      // Back convert to raw data
      xRaw                     = (short)(ctx->gyro.gyroData[0] / ctx->gyro.sensitivity);
      yRaw                     = (short)(ctx->gyro.gyroData[1] / ctx->gyro.sensitivity);
      zRaw                     = (short)(ctx->gyro.gyroData[2] / ctx->gyro.sensitivity);
      ctx->gyro.rawGyroData[0] = xRaw >> 8;
      ctx->gyro.rawGyroData[1] = xRaw;
      ctx->gyro.rawGyroData[2] = yRaw >> 8;
      ctx->gyro.rawGyroData[3] = yRaw;
      ctx->gyro.rawGyroData[4] = zRaw >> 8;
      ctx->gyro.rawGyroData[5] = zRaw;

      hDummyIdx += 2;
    }
#else
    (*ctx->lAccel).update(ctx->lAccel);
    (*ctx->hAccel).update(ctx->hAccel);
    ctx->gyro.update(&ctx->gyro);
#endif

    // Add sensor data to dataframe
    ctx->mem.append(&ctx->mem, HEADER_HIGHRES);
    ctx->mem.appendBytes(&ctx->mem, accel->rawAccelData, KX134_1211_DATA_TOTAL);
    ctx->mem.appendBytes(&ctx->mem, ctx->gyro.rawGyroData, A3G4250D_DATA_TOTAL);

    // Only run calculations when enabled
    EventBits_t uxBits = xEventGroupWaitBits(ctx->xTaskEnableGroup, GROUP_TASK_ENABLE_HIGHRES, pdFALSE, pdFALSE, blockTime);
    if (uxBits & GROUP_TASK_ENABLE_HIGHRES) {
      // Integrate attitude quaternion from rotations
      Quaternion qDot;
      Quaternion_init(&qDot);
      qDot.fromEuler(
          &qDot,
          (float)(dt * ctx->gyro.gyroData[ROLL_INDEX]),
          (float)(dt * ctx->gyro.gyroData[PITCH_INDEX]),
          (float)(dt * ctx->gyro.gyroData[YAW_INDEX])
      );
      ctx->state.qRot = Quaternion_mul(&ctx->state.qRot, &qDot);
      ctx->state.qRot.normalise(&ctx->state.qRot); // New attitude quaternion

      // Apply rotation to z-axis unit vector
      ctx->state.qRot.fRotateVector3D(&ctx->state.qRot, ctx->state.zUnit, ctx->state.vAttitude);

      // Calculate tilt angle
      // tilt = cos^-1(attitude · initial)
      ctx->state.cosine = ctx->state.zUnit[0] * ctx->state.vAttitude[0] + ctx->state.zUnit[1] * ctx->state.vAttitude[1] + ctx->state.zUnit[2] * ctx->state.vAttitude[2];
      ctx->state.tilt   = acos(ctx->state.cosine) * 180 / M_PI;
    }

#ifdef DEBUG
    //! @todo extract debug print to function
    //! @todo move debug function to new source file with context as parameter
    if ((xSemaphoreTake(xUsbMutex, pdMS_TO_TICKS(0))) == pdTRUE) {
      memset(HdebugStr, 100, sizeof(char));

      snprintf(HdebugStr, 100, "[HDataAcq] %d\tAccel\tX: %.3f\tY: %.3f\tZ: %.3f\n\r", hDummyIdx / 2, accel->accelData[0], accel->accelData[1], accel->accelData[2]);
      xMessageBufferSend(xUsbTxBuff, (void *)HdebugStr, 100, pdMS_TO_TICKS(0));

      snprintf(HdebugStr, 100, "[HDataAcq] %d\tGyro\tX: %.3f\tY: %.3f\tZ: %.3f\n\r", hDummyIdx / 2, ctx->gyro.gyroData[0], ctx->gyro.gyroData[1], ctx->gyro.gyroData[2]);

      xMessageBufferSend(xUsbTxBuff, (void *)HdebugStr, 100, pdMS_TO_TICKS(0));
      xSemaphoreGive(xUsbMutex);
    }
#endif
  }
}
