#ifndef _LORA_H
#define _LORA_H

#include "stm32f439xx.h"

void configure_SPI3_LoRa();
void configure_LoRa_module();

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

#endif
