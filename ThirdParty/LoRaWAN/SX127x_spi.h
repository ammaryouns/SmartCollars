/**
 * @file   	SX127x_spi.h
 * @author 	Hassan
 * @version	
 * @date	Mar 17, 2016
 * 
 * @brief   
 */

#ifndef SX127x_SPI_H_
#define SX127x_SPI_H_

#include <stdint.h>
#include <SX127x_hal.h>

/**
 * @defgroup SX127x_enum	SX127x Enumeration Definitions
 * @{
 */


/**
 * @brief Return status defines for SX127x functions
 */
typedef enum{

	SX127x_OK = 0,//!< SX127x_OK
	SX127x_ERROR,  //!< SX127x_ERROR
    SX127x_TIMEOUT
    
}SX127xReturn_t;

/** @}*/

/**
 * @defgroup SX127x_public_macros	SX127x Public Macro Definitions
 * @{
 */

#define SX127x_ASSERT_RETURN( returnValue, expectedValue )	if (returnValue != expectedValue)\
															{\
																return returnValue;\
															}\

/** @}*/

#ifdef __cplusplus
extern "C"{
#endif

SX127xReturn_t SX127x_readRegister	(uint8_t regAddr, uint8_t* pRetValue);
SX127xReturn_t SX127x_writeRegister	(uint8_t regAddr, uint8_t value);
SX127xReturn_t SX127x_modifyRegister(uint8_t regAddr, uint8_t mask, uint8_t value);
SX127xReturn_t SX127x_writeBurst 	(uint8_t regAddr, uint8_t* buffer, uint8_t len);
SX127xReturn_t SX127x_readBurst		(uint8_t regAddr, uint8_t* buffer, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* SX127x_SPI_H_ */
