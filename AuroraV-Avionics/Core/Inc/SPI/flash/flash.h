#ifndef _FLASH_H
#define _FLASH_H

#include "spi.h"
#include "stm32f439xx.h"

// =========================================================
//               SPI DATA PINS (SCLK, SDI, SDO)
// =========================================================
#define FLASH_DATA_AFRH_Msk   (GPIO_AFRH_AFRH4 | GPIO_AFRH_AFRH5 | GPIO_AFRH_AFRH6)
#define FLASH_DATA_OTYPE_Msk  (GPIO_OTYPER_OT12_Msk | GPIO_OTYPER_OT13_Msk | GPIO_OTYPER_OT14_Msk)
#define FLASH_DATA_MODE_Msk   (GPIO_MODER_MODE12_Msk | GPIO_MODER_MODE13_Msk | GPIO_MODER_MODE14_Msk)
#define FLASH_DATA_OSPEED_Msk (GPIO_OSPEEDR_OSPEED12_Msk | GPIO_OSPEEDR_OSPEED13_Msk | GPIO_OSPEEDR_OSPEED14_Msk)

// clang-format off
#define FLASH_DATA_MODE_Set(value) ( \
    value << GPIO_MODER_MODE12_Pos   \
  | value << GPIO_MODER_MODE13_Pos   \
  | value << GPIO_MODER_MODE14_Pos   \
)

#define FLASH_DATA_AFRH_Set(value) ( \
    value << GPIO_AFRH_AFSEL12_Pos   \
  | value << GPIO_AFRH_AFSEL13_Pos   \
  | value << GPIO_AFRH_AFSEL14_Pos   \
)
// clang-format on

// =========================================================
//              SPI CONTROL PINS (WP, HOLD, CS)
// =========================================================
#define FLASH_CONTROL_ODR_Msk    (GPIO_ODR_OD9_Msk | GPIO_ODR_OD10_Msk | GPIO_ODR_OD11_Msk)
#define FLASH_CONTROL_OTYPE_Msk  (GPIO_OTYPER_OT9_Msk | GPIO_OTYPER_OT10_Msk | GPIO_OTYPER_OT11_Msk)
#define FLASH_CONTROL_MODE_Msk   (GPIO_MODER_MODE9_Msk | GPIO_MODER_MODE10_Msk | GPIO_MODER_MODE11_Msk)
#define FLASH_CONTROL_OSPEED_Msk (GPIO_OSPEEDR_OSPEED9_Msk | GPIO_OSPEEDR_OSPEED10_Msk | GPIO_OSPEEDR_OSPEED11_Msk)

// clang-format off
#define FLASH_CONTROL_MODE_Set(value) ( \
    value << GPIO_MODER_MODE9_Pos       \
  | value << GPIO_MODER_MODE10_Pos      \
  | value << GPIO_MODER_MODE11_Pos      \
)

#define FLASH_CONTROL_SPEED_Set(value) ( \
    value << GPIO_OSPEEDR_OSPEED9_Pos    \
  | value << GPIO_OSPEEDR_OSPEED10_Pos   \
  | value << GPIO_OSPEEDR_OSPEED11_Pos   \
)
// clang-format on

typedef struct Flash {
  SPI base;
  void (*erase)(struct Flash *);
  void (*readPage)(struct Flash *, uint32_t, uint8_t *);
  void (*writePage)(struct Flash *, uint32_t, uint8_t *);
} Flash;

void configure_SPI4_Flash();
void Flash_init(Flash *, GPIO_TypeDef *, unsigned long);
void Flash_readPage(Flash *, uint32_t, uint8_t *);
void Flash_writePage(Flash *, uint32_t, uint8_t *);
void Flash_erase(Flash *);
void _Flash_writeEnable(Flash *);
void _Flash_readStatus1(Flash *, uint8_t *);
void _Flash_readStatus2(Flash *, uint8_t *);
void _Flash_readStatus3(Flash *, uint8_t *);

#endif
