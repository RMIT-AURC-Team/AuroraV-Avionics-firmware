/**
 * @author Matt Ricci
 * @defgroup SPI_API SPI
 */

#ifndef _SPI_H
#define _SPI_H

#include "stm32f439xx.h"
#include <stdint.h>

/**
 * @ingroup SPI_API
 * @addtogroup SPI Interface
 * @brief SPI interface from which slave devices inherit.
 * @todo Add in code block examples to API documentation.
 * @{
 */

/**
 * @brief Device type enum
 * Describes the type of peripheral implementing an SPI interface.
 */
typedef enum {
  SENSOR_ACCEL,  //!< Accelerometer.
  SENSOR_GYRO,   //!< Gyroscope.
  SENSOR_BARO,   //!< Barometer.
  MEMORY_FLASH,  //!< Flash memory.
	COMM_LORA 		 //!< LoRa module
} DeviceType;

/**
 * @brief Struct definition for \ref SPI "SPI interface".
 * Provides the interface for API consumers to interact with the SPI peripheral.
 */
typedef struct SPI {
  DeviceType device;                            //!< Enum specifier for device type.
  SPI_TypeDef *interface;                       //!< Pointer to SPI interface struct.
  GPIO_TypeDef *port;                           //!< Pointer to GPIO port struct.
  unsigned long cs;                             //!< Device chip select address.
  void (*send)(struct SPI *, uint16_t);         //!< SPI send method.     @see SPI_send
  void (*receive)(struct SPI *, volatile uint16_t *);    //!< SPI receive method.  @see SPI_receive
  uint16_t (*transmit)(struct SPI *, uint16_t); //!< SPI transmit method. @see SPI_transmit
} SPI;

void SPI_init(SPI *, DeviceType, SPI_TypeDef *, GPIO_TypeDef *, unsigned long);
void SPI_send(SPI *, uint16_t);
void SPI_receive(SPI *, volatile uint16_t *);
uint16_t SPI_transmit(SPI *, uint16_t);

/** @} */
#endif


