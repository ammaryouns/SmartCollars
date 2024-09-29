/**
 * @file   	SX127x_hal.h
 * @author 	Hassan Abid
 * @version	v2.0.0
 * @date	Mar 18, 2016
 * 
 * @brief   
 */


#include <stdbool.h>
#include "BSP.h"

#ifndef SX127x_HAL_H_
#define SX127x_HAL_H_


#define SX127x_RESET_TXRX()     { SX127x_GPIO_Reset( SX127x_RFSwitch_CNTR_1 ); \
                                  SX127x_GPIO_Reset( SX127x_RFSwitch_CNTR_2 ); }

#define SX127x_SET_TX()         { SX127x_GPIO_Set( SX127x_RFSwitch_CNTR_1 ); \
                                  SX127x_GPIO_Reset( SX127x_RFSwitch_CNTR_2 ); }

#define SX127x_SET_RX()         { SX127x_GPIO_Reset( SX127x_RFSwitch_CNTR_1 ); \
                                  SX127x_GPIO_Set( SX127x_RFSwitch_CNTR_2 ); }

#define SX127x_SET_NSEL()       SX127x_GPIO_Set( SX127x_NSS )
#define SX127x_RESET_NSEL()     SX127x_GPIO_Reset( SX127x_NSS )

#define SX127x_SET_NRST()       SX127x_GPIO_Set( SX127x_RESET )
#define SX127x_RESET_NRST()     SX127x_GPIO_Reset( SX127x_RESET )


#define SX127x_spiTransmit(txBuffer, len)      HAL_SPI_Transmit(&SX127x_hspi,\
                                                                    txBuffer,\
                                                                    len,\
                                                                    20)
                                    
#define SX127x_spiReceive(rxBuffer, len)      HAL_SPI_Receive(&SX127x_hspi,\
                                                                rxBuffer,\
                                                                len,\
                                                                20)
                                    
#define SX127x_spiTransmitReceive(txBuffer, rxBuffer, len)      HAL_SPI_TransmitReceive(&SX127x_hspi,\
                                                                                        txBuffer,\
                                                                                        rxBuffer,\
                                                                                        len,\
                                                                                        20)


#define SX127x_READ_DIO(pin)	(HAL_GPIO_ReadPin(SX127x_DIO##pin##_GPIO_Port, SX127x_DIO##pin##_Pin) == GPIO_PIN_SET)



#ifdef __cplusplus
extern "C"{
#endif 


#ifdef __cplusplus
}
#endif

#endif /* SX127x_HAL_H_ */
