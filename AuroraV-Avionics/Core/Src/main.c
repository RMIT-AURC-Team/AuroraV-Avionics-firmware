#include "main.h"

#include "baccel.h"
#include "baro.h"
#include "gyroX.h"
#include "gyroY.h"
#include "gyroZ.h"

// Task Handles
TaskHandle_t xDataAqcquisitionHHandle = NULL;
TaskHandle_t xDataAqcquisitionLHandle = NULL;

// Unsure of actual fix for linker error
// temporary (lol) solution
void _init() {}

Quaternion qRot;                // Global attitude quaternion
float vAttitude[3] = {0, 0, 1}; // Attitude vector
float zUnit[3]     = {0, 0, 1}; // Z unit vector
float cosine       = 0;         // Tilt angle cosine
float tilt         = 0;         // Tilt angle

float altitude = 0;
float velocity = 0;
float accel    = 0;

#define BUFF_SIZE 32000
#define PAGE_SIZE 256
MemBuff mem;
uint8_t buff[BUFF_SIZE];

void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);

int main(void) {
  //  configure_RCC_APB1();
  //  configure_RCC_APB2();
  //  configure_RCC_AHB1();

  SystemInit();

  MemBuff_init(&mem, buff, BUFF_SIZE, PAGE_SIZE);
  Quaternion_init(&qRot);

  // Create task handles
  xTaskCreate(vDataAcquisitionH, "DataHighRes", 128, NULL, configMAX_PRIORITIES - 1, &xDataAqcquisitionHHandle);
  xTaskCreate(vDataAcquisitionL, "DataLowRes", 256, NULL, configMAX_PRIORITIES - 2, &xDataAqcquisitionLHandle);

  vTaskStartScheduler();

  // Should never reach here
  while (1) {
  }
}

// High-Resolution Task Function
void vDataAcquisitionH(void *argument) {

  unsigned int index = 0;
  float dt           = 0.002;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(2);

  float roll, pitch, yaw;

  for (;;) {

    /* ========================================
     *             Read sensor data
     * ======================================== */

    memcpy(&accel, baccel + index, sizeof(accel));
    memcpy(&roll, gyroX + index, sizeof(roll));
    memcpy(&pitch, gyroY + index, sizeof(pitch));
    memcpy(&yaw, gyroZ + index, sizeof(yaw));

    /* ========================================
     *            Append to dataframe
     * ======================================== */

    // Add sensor data and quaternion to dataframe
    mem.append(&mem, HEADER_HIGHRES);
    // TODO: Add sync
    //
    // mem.append(&mem, accel[0]); // Accel X high byte
    // mem.append(&mem, accel[1]); // Accel X low byte
    // mem.append(&mem, accel[2]); // Accel Y high byte
    // mem.append(&mem, accel[3]); // Accel Y low byte
    // mem.append(&mem, accel[4]); // Accel Z high byte
    // mem.append(&mem, accel[5]); // Accel Z low byte
    //
    // mem.append(&mem, gyro[0]); // Gyro X high byte
    // mem.append(&mem, gyro[1]); // Gyro X low byte
    // mem.append(&mem, gyro[2]); // Gyro Y high byte
    // mem.append(&mem, gyro[3]); // Gyro Y low byte
    // mem.append(&mem, gyro[4]); // Gyro Z high byte
    // mem.append(&mem, gyro[5]); // Gyro Z low byte

    /* ========================================
     *            Calculate attitude
     * ======================================== */

    // Integrate attitude quaternion from rotations
    Quaternion qDot;
    Quaternion_init(&qDot);
    qDot.fromEuler(
        &qDot,
        (float)(dt * roll),
        (float)(dt * pitch),
        (float)(dt * yaw)
    );
    qRot = Quaternion_mul(&qRot, &qDot);
    qRot.normalise(&qRot); // New attitude quaternion

    // Apply rotation to z-axis unit vector
    qRot.fRotateVector3D(&qRot, zUnit, vAttitude);

    // Calculate tilt angle
    // tilt = cos^-1(attitude · initial)
    cosine = zUnit[0] * vAttitude[0] + zUnit[1] * vAttitude[1] + zUnit[2] * vAttitude[2];
    tilt   = acos(cosine) * 180 / M_PI;

    index += 4;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// High-Resolution Task Function
void vDataAcquisitionL(void *argument) {

  unsigned int index = 0;
  float dt           = 0.020;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(20);

  /* ========================================
   *       Initialise filter parameters
   * ======================================== */

  KalmanFilter kf;
  KalmanFilter_init(&kf);

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

  for (;;) {

    /* ========================================
     *             Read sensor data
     * ======================================== */

    memcpy(&altitude, baro + index, sizeof(altitude));

    /* ========================================
     *            Append to dataframe
     * ======================================== */

    // Add sensor data and quaternion to dataframe
    mem.append(&mem, HEADER_LOWRES);
    // TODO: Add sync
    //
    // mem.append(&mem, baro[0]);
    // mem.append(&mem, baro[1]);
    // mem.append(&mem, baro[2]);
    // mem.append(&mem, baro[3]);

    /* ========================================
     *              Calculate state
     * ======================================== */

    z.pData[0] = altitude;
    z.pData[1] = (cosine * 9.81 * accel - 9.81); // Acceleration measured in m/s^2
    kf.update(&kf, &z);

    index += 4;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName) {
  xTaskHandle bad_task_handle = xTask;      // this seems to give me the crashed task handle
  char *bad_task_name         = pcTaskName; // this seems to give me a pointer to the name of the crashed task
  while (1);
}

void vApplicationIdleHook(void) {
}
