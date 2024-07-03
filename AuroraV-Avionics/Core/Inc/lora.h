#ifndef _LORA_H
#define _LORA_H

#include "stm32f439xx.h"

void configure_SPI3_LoRa();
void Load_And_Send_To_LoRa(char *pointerdata);
void configure_LoRa_module();
void write_lora_packet(uint8_t address, uint8_t payload);
uint8_t receive_lora_data(uint8_t address);

// =========================================================
//               SPI DATA PINS (SCLK, SDI, SDO)
// =========================================================
#define LORA_DATAC_AFRH_Msk   (GPIO_AFRH_AFRH2 | GPIO_AFRH_AFRH3 | GPIO_AFRH_AFRH4)
#define LORA_DATAC_OTYPE_Msk  (GPIO_OTYPER_OT10_Msk | GPIO_OTYPER_OT11_Msk | GPIO_OTYPER_OT12_Msk)
#define LORA_DATAC_MODE_Msk   (GPIO_MODER_MODE10_Msk | GPIO_MODER_MODE11_Msk | GPIO_MODER_MODE12_Msk)
#define LORA_DATAC_PUPD_Msk   (GPIO_PUPDR_PUPD10_Msk | GPIO_PUPDR_PUPD11_Msk | GPIO_PUPDR_PUPD12_Msk)
#define LORA_DATAC_OSPEED_Msk (GPIO_OSPEEDR_OSPEED10_Msk | GPIO_OSPEEDR_OSPEED11_Msk | GPIO_OSPEEDR_OSPEED12_Msk)

#define LORA_CONTROLD_MODE_Msk (GPIO_MODER_MODE0_Msk | GPIO_MODER_MODE7_Msk)
#define LORA_DATAD_MODE_Msk    (GPIO_MODER_MODE1_Msk)

// clang-format off
#define LORA_DATAC_MODE_Set(value) ( \
    value << GPIO_MODER_MODE10_Pos   \
  | value << GPIO_MODER_MODE11_Pos   \
  | value << GPIO_MODER_MODE12_Pos   \
)

#define LORA_DATAC_PUPD_Set(value) ( \
    value << GPIO_PUPDR_PUPD10_Pos   \
  | value << GPIO_PUPDR_PUPD11_Pos   \
  | value << GPIO_PUPDR_PUPD12_Pos   \
)

#define LORA_DATAC_SPEED_Set(value) (  \
    value << GPIO_OSPEEDR_OSPEED10_Pos \
  | value << GPIO_OSPEEDR_OSPEED11_Pos \
  | value << GPIO_OSPEEDR_OSPEED12_Pos \
)

#define LORA_DATAC_AFRH_Set(value) ( \
    value << GPIO_AFRH_AFSEL10_Pos   \
  | value << GPIO_AFRH_AFSEL11_Pos   \
  | value << GPIO_AFRH_AFSEL12_Pos   \
)
// clang-format on

// =========================================================
//              SPI CONTROL PINS (WP, HOLD, CS)
// =========================================================
#define LORA_CONTROL_ODR_Msk    (GPIO_ODR_OD9_Msk | GPIO_ODR_OD10_Msk | GPIO_ODR_OD11_Msk)
#define LORA_CONTROL_OTYPE_Msk  (GPIO_OTYPER_OT9_Msk | GPIO_OTYPER_OT10_Msk | GPIO_OTYPER_OT11_Msk)
#define LORA_CONTROL_MODE_Msk   (GPIO_MODER_MODE9_Msk | GPIO_MODER_MODE10_Msk | GPIO_MODER_MODE11_Msk)
#define LORA_CONTROL_OSPEED_Msk (GPIO_OSPEEDR_OSPEED9_Msk | GPIO_OSPEEDR_OSPEED10_Msk | GPIO_OSPEEDR_OSPEED11_Msk)

// clang-format off
#define LORA_CONTROLD_MODE_Set(value) ( \
    value << GPIO_MODER_MODE0_Pos       \
  | value << GPIO_MODER_MODE7_Pos       \
)

#define LORA_CONTROLC_TYPE_Set(value) ( \
    value << GPIO_OTYPE_OT9_Pos   \
  | value << GPIO_OTYPE_OT10_Pos  \
  | value << GPIO_OTYPE_OT11_Pos  \
)
// clang-format on

struct LoRa_Registers {
  uint8_t RegFifo;
  uint8_t RegOpMode;
  uint8_t RegFifoAddrPtr;
  uint8_t RegFifoTxBaseAddr;
  uint8_t RegFifoRxBaseAddr;
  uint8_t RegFifoRxCurrentAddr;
  uint8_t RegIrqFlagsMask;
  uint8_t RegIrqFlags;
  uint8_t RegRxNbBytes;
  uint8_t RegRxHeaderCntValueMsb;
  uint8_t RegRxHeaderCntValueLsb;
  uint8_t RegRxPacketCntValueMsb;
  uint8_t RegRxPacketCntValueLsb;
  uint8_t RegModemStat;
  uint8_t RegPktSnrValue;
  uint8_t RegPktRssiValue;
  uint8_t RegRssiValue;
  uint8_t RegHopChannel;
  uint8_t RegModemConfig1;
  uint8_t RegModemConfig2;
  uint8_t RegSymbTimeoutLsb;
  uint8_t RegPreambleMsb;
  uint8_t RegPreambleLsb;
  uint8_t RegPayloadLength;
  uint8_t RegMaxPayloadLength;
  uint8_t RegHopPeriod;
  uint8_t RegFifoRxByteAddr;
  uint8_t RegFeiMsb;
  uint8_t RegFeiMid;
  uint8_t RegFeiLsb;
  uint8_t RegRssiWideband;
  uint8_t RegDetectOptimize;
  uint8_t RegInvertIQ;
  uint8_t RegDetectionThreshold;
  uint8_t RegSyncWord;
  uint8_t RegInvertQ2;
  uint8_t RegChirpFilter;
  uint8_t RegDioMapping1;
  uint8_t RegDioMapping2;
};
extern const struct LoRa_Registers LoRa_Registers;

#endif
