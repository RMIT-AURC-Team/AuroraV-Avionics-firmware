#include "main.h"
#include "data_output_spi.h"
#include "gpio_struct.h"

SPI_HandleTypeDef hspi4;

// Task Handles
TaskHandle_t xDataAqcquisitionHHandle = NULL;
TaskHandle_t xDataAqcquisitionLHandle = NULL;
TaskHandle_t xFlashBufferHandle       = NULL;
TaskHandle_t xStateUpdateHandle       = NULL;
TaskHandle_t xLoRaCommunicateHandle   = NULL;
EventGroupHandle_t xTaskEnableGroup; // 0: FLASH,  1: HIGHRES, 2: LOWRES, 7: IDLE

// Calculated attitude variables
Quaternion qRot;                // Global attitude quaternion
float vAttitude[3] = {0, 0, 1}; // Attitude vector
float zUnit[3]     = {0, 0, 1}; // Z unit vector
float cosine       = 0;         // Tilt angle cosine
float tilt         = 0;         // Tilt angle

// Angular rotations
float roll  = 0;
float pitch = 0;
float yaw   = 0;

// Flight dynamics state variables
float altitude                = 0; // Current altitude
float velocity                = 0; // Current vertical velocity
float accelZ                  = 0; // Current body-axis acceleration
float previousvelocity        = 0;
uint8_t velocitydecreasecount = 0;
float previousaltitude        = 0;
uint8_t altitudedecreasecount = 0;

uint8_t accel[6]              = {0, 0, 0, 0, 0, 0};
uint8_t gyro[6];
uint8_t pressure[6];
uint8_t temperature[6];
float pressGround = 0;

#define BUFF_SIZE 21000 // 2s worth of data in buffer
#define PAGE_SIZE 256
MemBuff mem;
uint8_t buff[BUFF_SIZE];
uint8_t outBuff[PAGE_SIZE];

MemBuff Altitudemem;
uint8_t AltitudeBuff[15];
uint8_t Altitudeoutbuff[15];

MemBuff Velocitymem;
uint8_t VelocityBuff[10];
uint8_t VelocityoutBuff[10];

enum State currentState = PRELAUNCH; // Boot in prelaunch
int interval2ms         = 0;

GPIO_Config spi4_cs;
Sensor lAccel_s;
Sensor hAccel_s;
Sensor gyro_s;
Sensor_multi baro_s;

const struct LoRa_Registers LoRa_Regs = {0, 1, 0xd, 0xE, 0xF, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0X1C, 0X1D, 0X1E, 0X1F, 0X20, 0X21, 0X22, 0X23, 0X24, 0X25, 0X28, 0X29, 0X2A, 0X2C, 0X31, 0X33, 0X37, 0X39, 0X3B, 0X3D, 0X40, 0X41};
static void MX_SPI4_Init(void);

int main(void) {
  // Bring up RCC
  configure_RCC_APB1();
  configure_RCC_APB2();
  configure_RCC_AHB1();
  configure_MISC_GPIO();
  configure_UART3_GPS();
  configure_UART6_Serial_2_mini_USB();
  configure_SPI1_Sensor_Suite();
  configure_SPI2_Flash();
  configure_SPI3_LoRa();

  // Initialise timers
  TIM6init();
  TIM7init();

  // Configure peripherals
  CANGPIO_config();
  CAN_Peripheral_config();
  configure_LoRa_module();

  configure_Sensors();
  Accel_init(&lAccel_s, ACCEL_CS_1, ACCEL_SCALE_LOW);
  Accel_init(&hAccel_s, ACCEL_CS_2, ACCEL_SCALE_HIGH);
  Gyro_init(&gyro_s, GYRO_CS, GYRO_SENSITIVITY);
  Baro_init(&baro_s, BARO_CS, BARO_SENSITIVITY_COUNT, BARO_TEMP_SENSITIVITY, BARO_PRESS_SENSITIVITY);

  configure_LoRa_module();

  MX_SPI4_Init();
  spi4_cs = create_GPIO_Config(GPIOE, GPIO_PIN_11);

  HAL_GPIO_WritePin(spi4_cs.GPIOx, spi4_cs.GPIO_Pin, GPIO_PIN_RESET);
  // erase_chip_spi(&hspi4, spi4_cs);
  // while(1);
  //	uint8_t readBuff[256];
  //	read_page_spi(readBuff, &hspi4, 0x00, spi4_cs);

  unsigned int CANHigh = 0;
  unsigned int CANLow  = 0;
  unsigned int id      = 0x603;
  CAN_TX(1, 8, CANHigh, CANLow, id);

  MemBuff_init(&mem, buff, BUFF_SIZE, PAGE_SIZE);
  Quaternion_init(&qRot);

  MemBuff_init(&Altitudemem, AltitudeBuff, 10, 10);
  MemBuff_init(&Velocitymem, VelocityBuff, 5, 5);

  xTaskEnableGroup = xEventGroupCreate();
  xEventGroupClearBits(xTaskEnableGroup, 0xFF);

  // Calculate ground pressure
  uint8_t hPress = baro_s.read(&baro_s, 0x22);
  uint8_t lPress = baro_s.read(&baro_s, 0x21);
  uint8_t xPress = baro_s.read(&baro_s, 0x20);
  pressGround    = baro_s.sensitivities[1] * (((int32_t)hPress << 16) | ((int32_t)lPress << 8) | xPress);

  // Create task handles
  xTaskCreate(vDataAcquisitionH, "HDataAcq", 256, NULL, configMAX_PRIORITIES - 2, &xDataAqcquisitionHHandle);
  xTaskCreate(vDataAcquisitionL, "LDataAcq", 256, NULL, configMAX_PRIORITIES - 3, &xDataAqcquisitionLHandle);
  xTaskCreate(vStateUpdate, "StateUpdate", 256, NULL, configMAX_PRIORITIES - 4, &xStateUpdateHandle);
  xTaskCreate(vFlashBuffer, "FlashData", 256, NULL, configMAX_PRIORITIES - 1, &xFlashBufferHandle);
  // xTaskCreate(vLoRaCommunicate, "LoRa", 256, NULL, configMAX_PRIORITIES - 1, &xLoRaCommunicateHandle);

  vTaskStartScheduler();

  // Should never reach here
  while (1) {
  }
}

/* =====================================================================
 *                            FLASH HANDLING
 * ===================================================================== */

// Use idle time to write flash
void vApplicationIdleHook(void) {
  // Write if a page is available in the buffer
  if (currentState == LAUNCH && mem.pageReady)
    xEventGroupSetBits(xTaskEnableGroup, 0x01);
}

void vFlashBuffer(void *argument) {
  const TickType_t timeout = pdMS_TO_TICKS(1);
  uint32_t pageAddr        = 0;
  for (;;) {
    // Wait for write flag to be ready, clear flag on exit
    EventBits_t uxBits = xEventGroupWaitBits(xTaskEnableGroup, 0x01, pdTRUE, pdFALSE, timeout);
    if ((uxBits & 0x01) == 0x01) {
      _Bool success = mem.readPage(&mem, outBuff); // Flush data to output buffer
      if (success) {
        // Write data to flash memory
        // Flash_Page_Program(flashAddress, outBuff, sizeof(outBuff));
        write_data_spi(outBuff, &hspi4, pageAddr, spi4_cs);
        pageAddr += 0x100;
      }
    }
  }
}

static void MX_SPI4_Init(void) {
  /* SPI4 parameter configuration*/
  RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;
  RCC->APB2RSTR |= RCC_APB2RSTR_SPI4RST;
  __ASM("NOP");
  __ASM("NOP");
  RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI4RST;
  hspi4.Instance               = SPI4;
  hspi4.Init.Mode              = SPI_MODE_MASTER;
  hspi4.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi4.Init.NSS               = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial     = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK) {
    // Error_Handler();
  }
}

/* =====================================================================
 *                             LORA HANDLING
 * ===================================================================== */

void vLoRaCommunicate(void *argument) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(500); // 2Hz
  for (;;) {
    char payload[16] = {
        LORA_HEADER_AVD1,
        accel[0], accel[1], accel[2],
        accel[3], accel[4], accel[5],
        ':', '/',
        ':', ')'
    };
    Load_And_Send_To_LoRa(payload, &LoRa_Regs);
    // Block until transmit enable task group bit is set
    //  this is managed within an ISR triggered by the LoRa module
    // ...
    // Repeat 2 more times to delivery all payloads
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/* =====================================================================
 *                            STATE MANAGEMENT
 * ===================================================================== */

void vStateUpdate(void *argument) {
  unsigned int ms = 0;
  union {
    unsigned int i;
    uint8_t a[4];
  } u;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz

  unsigned int CANHigh        = 0;
  unsigned int CANLow         = 0;
  unsigned int id             = 0;

  for (;;) {
    // Block until 20ms interval
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    // Emergency aerobrakes for excessive tilt
    if (tilt >= 30.0f) {
      // CAN payload for aerobrakes retract
      CANHigh = 0x00000000;
      CANLow  = 0x00000000;
      id      = 0x602;
      CAN_TX(1, 8, CANHigh, CANLow, id);
    }

    switch (currentState) {
    case PRELAUNCH:
      if (isAccelerationAbove5Gs(accelZ)) {
        // load and send to lora
        xEventGroupSetBits(xTaskEnableGroup, 0x80); // Enable flash
        xEventGroupSetBits(xTaskEnableGroup, 0x06); // Enable data acquisition
        currentState = LAUNCH;
        // Add launch event dataframe to buffer
        mem.append(&mem, HEADER_EVENT_LAUNCH);
        // Append timestamp
        mem.append(&mem, u.a[0]);
        mem.append(&mem, u.a[1]);
        mem.append(&mem, u.a[2]);
        mem.append(&mem, u.a[3]);
      }
      break;
    case LAUNCH:
      // Send altitude to aerobrakes via CAN
      CANHigh = 0x00000000;
      CANLow  = (unsigned int)altitude;
      id      = 0x601;
      CAN_TX(1, 8, CANHigh, CANLow, id);
      // Transition to motor burnout state on velocity decrease
      if (isVelocityDecreasing(velocity, previousvelocity, velocitydecreasecount)) {
        currentState = MOTOR_BURNOUT;
        // Add motor burnout event dataframe to buffer
        mem.append(&mem, HEADER_EVENT_MOTOR);
        // Append timestamp
        mem.append(&mem, u.a[0]);
        mem.append(&mem, u.a[1]);
        mem.append(&mem, u.a[2]);
        mem.append(&mem, u.a[3]);
      }
      break;
    case MOTOR_BURNOUT:
      // Send altitude to aerobrakes via CAN
      CANHigh = 0x00000000;
      CANLow  = (unsigned int)altitude;
      id      = 0x601;
      CAN_TX(1, 8, CANHigh, CANLow, id);
      // Transition to apogee state on three way vote of altitude, velocity, and tilt
      // apogee is determined as two of three conditions evaluating true
      if (isAltitudeDropping(altitude, previousaltitude, altitudedecreasecount) + isTiltAngleAbove90(tilt) + isNegativeVelocity(velocity) >= 2) {
        currentState = APOGEE;
        // Add apogee event dataframe to buffer
        mem.append(&mem, HEADER_EVENT_APOGEE);
        // Append timestamp
        mem.append(&mem, u.a[0]);
        mem.append(&mem, u.a[1]);
        mem.append(&mem, u.a[2]);
        mem.append(&mem, u.a[3]);
        // Send transmission to trigger apogee E-matches
      }
      break;
    case APOGEE:
      if (isAltitude1300ft(altitude)) {
        currentState = DESCENT;
        // Enable descent state
        mem.append(&mem, HEADER_EVENT_DESCENT);
        // Append timestamp
        mem.append(&mem, u.a[0]);
        mem.append(&mem, u.a[1]);
        mem.append(&mem, u.a[2]);
        mem.append(&mem, u.a[3]);
        // Add descent event dataframe to buffer
      }
      break;
    case DESCENT:
      // Handle descent state actions
      break;
    default:
      // Handle unexpected state
      break;
    }
    ms += 2;
    u.i = ms;
  }
}

/* =====================================================================
 *                  HIGH RESOLUTION DATA ACQUISITION
 * ===================================================================== */

void vDataAcquisitionH(void *argument) {
  float dt = 0.002;
  float roll, pitch, yaw;
  Sensor acc;
  uint8_t axisBase = 0;

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(2); // 500Hz
  const TickType_t blockTime  = pdMS_TO_TICKS(0); // Don't need to block
  for (;;) {
    // Block until 2ms interval
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    acc      = lAccel_s;
    axisBase = 2;                    // Y axis on the vertical
    accel[0] = acc.read(&acc, 0x09); // Accel X high
    accel[1] = acc.read(&acc, 0x08); // Accel X low
    accel[2] = acc.read(&acc, 0x0B); // Accel Y high
    accel[3] = acc.read(&acc, 0x0A); // Accel Y low
    accel[4] = acc.read(&acc, 0x0D); // Accel Z high
    accel[5] = acc.read(&acc, 0x0C); // Accel Z low
    accelZ   = -acc.sensitivity * (int16_t)(((uint16_t)accel[axisBase] << 8) | accel[axisBase + 1]);

    // Read and process gyroscope
    gyro[0] = gyro_s.read(&gyro_s, 0x29); // Gyro X high
    gyro[1] = gyro_s.read(&gyro_s, 0x28); // Gyro X low
    gyro[2] = gyro_s.read(&gyro_s, 0x2B); // Gyro Y high
    gyro[3] = gyro_s.read(&gyro_s, 0x2A); // Gyro Y low
    gyro[4] = gyro_s.read(&gyro_s, 0x2D); // Gyro Z high
    gyro[5] = gyro_s.read(&gyro_s, 0x2C); // Gyro Z low
    // Need to adjust this to actual axis
    roll  = gyro_s.sensitivity * (int16_t)(((uint16_t)gyro[0] << 8) | gyro[1]);
    pitch = gyro_s.sensitivity * (int16_t)(((uint16_t)gyro[4] << 8) | gyro[5]);
    yaw   = gyro_s.sensitivity * (int16_t)(((uint16_t)gyro[2] << 8) | gyro[3]);

    // Add sensor data and quaternion to dataframe
    mem.append(&mem, HEADER_HIGHRES);
    mem.append(&mem, accel[0]); // Accel X high byte
    mem.append(&mem, accel[1]); // Accel X low byte
    mem.append(&mem, accel[2]); // Accel Y high byte
    mem.append(&mem, accel[3]); // Accel Y low byte
    mem.append(&mem, accel[4]); // Accel Z high byte
    mem.append(&mem, accel[5]); // Accel Z low byte
    mem.append(&mem, gyro[0]);  // Gyro X high byte
    mem.append(&mem, gyro[1]);  // Gyro X low byte
    mem.append(&mem, gyro[2]);  // Gyro Y high byte
    mem.append(&mem, gyro[3]);  // Gyro Y low byte
    mem.append(&mem, gyro[4]);  // Gyro Z high byte
    mem.append(&mem, gyro[5]);  // Gyro Z low byte

    // Only run calculations when enabled
    EventBits_t uxBits = xEventGroupWaitBits(xTaskEnableGroup, 0x02, pdFALSE, pdFALSE, blockTime);
    if ((uxBits & 0x02) == 0x02) {
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
    }
  }
}

/* =====================================================================
 *                    LOW RESOLUTION DATA ACQUISITION
 * ===================================================================== */

void vDataAcquisitionL(void *argument) {
  float dt = 0.020;
  KalmanFilter kf;
  KalmanFilter_init(&kf);

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
  const TickType_t blockTime  = pdMS_TO_TICKS(0);  // Don't need to block

  for (;;) {
    // Block until 20ms interval
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Read baro data
    temperature[0] = baro_s.read(&baro_s, 0x1F);
    temperature[1] = baro_s.read(&baro_s, 0x1E);
    temperature[2] = baro_s.read(&baro_s, 0x1D);
    float temp     = baro_s.sensitivities[0] * (((int32_t)temperature[0] << 16) | ((int32_t)temperature[1] << 8) | temperature[2]);

    pressure[0]    = baro_s.read(&baro_s, 0x22);
    pressure[1]    = baro_s.read(&baro_s, 0x21);
    pressure[2]    = baro_s.read(&baro_s, 0x20);
    float press    = baro_s.sensitivities[1] * (((int32_t)pressure[0] << 16) | ((int32_t)pressure[1] << 8) | pressure[2]);

    altitude       = 44330 * (1.0 - pow(press / pressGround, 0.1903));

    // Add sensor data and barometer data to dataframe
    mem.append(&mem, HEADER_LOWRES);
    mem.append(&mem, temperature[0]);
    mem.append(&mem, temperature[1]);
    mem.append(&mem, temperature[2]);
    mem.append(&mem, pressure[0]);
    mem.append(&mem, pressure[1]);
    mem.append(&mem, pressure[2]);

    // Only run calculations when enabled
    EventBits_t uxBits = xEventGroupWaitBits(xTaskEnableGroup, 0x04, pdFALSE, pdFALSE, blockTime);
    if ((uxBits & 0x04) == 0x04) {
      // Calculate state
      z.pData[0] = altitude;
      z.pData[1] = (cosine * 9.81 * accelZ - 9.81); // Acceleration measured in m/s^2
      kf.update(&kf, &z);

      union {
        float f;
        uint8_t a[4];
      } u;
      u.f = kf.x.pData[1];
    }
  }
}

// Unsure of actual fix for linker error
// temporary (lol) solution
void _init() {}
