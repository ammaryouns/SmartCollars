/**
  ******************************************************************************
  * @file    Cowlar-node-v8.h
  * @author  Cowlar
  * @version V8.0.0
  * @date    7-Nov-2018
  * @brief   This file provides low level functionalities for Cowlar node hardware
  *          version 8
  ******************************************************************************
  */ 

  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COWLAR_NODE_V8_H
#define __COWLAR_NODE_V8_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "accelerometer.h"
#include "gyroscope.h"
#include "Flash_Memory_Map.h"
#include "adc.h"
  

#define PRINT_DEBUG
#define PRINT_TASK

#define printSensorPacket(...)
#define printLogPacket(...)

#define WHOLE_SYSTEM_UNIT_TEST          1

#define BATTERY_ADC_UNIT_TEST           1
#define MPU_UNIT_TEST                   1
#define RADIO_UNIT_TEST                 1
#define NFC_UNIT_TEST                   1
//#define LORA_RANGE_TEST
//#define DISABLE_UC_SLEEP
//#define DEBUG_SNIFFER
//#define DEBUG_DISABLE_FUELGAUGE
//#define DEBUG_DISABLE_BATTERY
//#define DEBUG_DISABLE_IWDG              
//#define DEBUG_DISABLE_DS18B20
//#define DEBUG_DISABLE_SX1276
//#define DEBUG_DISABLE_FIRMWARE_UPDATE
//#define DEBUG_DISABLE_MPU
//#define DEBUG_DISABLE_MPU_RAW
//#define MPU6500_RAW_DATA
//#define DISABLE_UC_SLEEP
//#define FSK_DATA_TEST
//#define RADIO_FSK
//#define RADIO_FSK_GATEWAY


#if defined (WHOLE_SYSTEM_UNIT_TEST)
  #define BATTERY_ADC_UNIT_TEST           1
  #define MPU_UNIT_TEST                   1
  #define RADIO_UNIT_TEST                 1
  #define NFC_UNIT_TEST                   1
  #define DEBUG_ON                        1
  #define PRINT_DEBUG
  #define PRINT_TASK


#endif


#if defined( BATTERY_ADC_UNIT_TEST)
  #define DEBUG_ON                        1
  #define PRINT_DEBUG
  #define PRINT_TASK

#endif 

#if defined( RADIO_UNIT_TEST)
  #define DEBUG_ON                        1
  #define PRINT_DEBUG
  #define PRINT_TASK
  #define DEBUG_DISABLE_OTA_UPDATE
  #define DEBUG_DISABLE_LORA
#endif 

//#ifdef ANN_TEST
//    #define DEBUG_DISABLE_SX1276
//    #define DEBUG_DISABLE_FIRMWARE_UPDATE
//    #define DEBUG_DISABLE_IWDG
//    #define DISABLE_UC_SLEEP
//    #define DEBUG_DISABLE_FUELGAUGE
//    #define DEBUG_DISABLE_DS18B20
//    #define DEBUG_DISABLE_BATTERY
//#endif


//#ifdef RADIO_FSK
//    #define DEBUG_DISABLE_SX1276
//    #define DEBUG_DISABLE_FIRMWARE_UPDATE
//    #define DEBUG_DISABLE_IWDG
//#endif

//#ifdef FSK_DATA_TEST 
//    #define DEBUG_DISABLE_BATTERY
//    #define DEBUG_DISABLE_FUELGAUGE
//    #define DEBUG_DISABLE_SX1276
//    //#undef  configconfigUSE_TICKLESS_IDLE  
//    #define DISABLE_UC_SLEEP    
//    #define DEBUG_DISABLE_FIRMWARE_UPDATE
//	#define DEBUG_DISABLE_IWDG
//    #define DEBUG_DISABLE_DS18B20
//    #define DEBUG_DISABLE_MPU
//#endif 

//#ifdef LORA_RANGE_TEST 
//    #define DEBUG_DISABLE_BATTERY
//    #define DEBUG_DISABLE_FUELGAUGE
//    #define DEBUG_DISABLE_SX1276
//    //#undef  configconfigUSE_TICKLESS_IDLE  
//    #define DEBUG_DISABLE_FIRMWARE_UPDATE
//	#define DEBUG_DISABLE_IWDG
//    #define DEBUG_DISABLE_DS18B20
//    #define DEBUG_DISABLE_MPU
//#endif 


//#define RAW_DATA_COLLECTION               //enabled sd card logging
//#define POWER_DATA_LOGGING                //enabled Current,voltage and power logging
//#define GPS_DATA_LOGGING                  //Log GPS time with device time
#define POWER_DATA_LOGGING_TIME_PERIOD    1000 // ms

#if defined(MPU_UNIT_TEST) 
  #define DEBUG_ON                        1
  #define PRINT_DEBUG
  #define PRINT_TASK
  #undef CLASSIFICATION_APP
  #define DEFAULT_MPU_HZ        26        // Hz   
#else   
  #ifdef RAW_DATA_COLLECTION
    #define DEFAULT_MPU_HZ        26       // Hz
  #else
    #define DEFAULT_MPU_HZ        26        // Hz   
  #endif

  #define CLASSIFICATION_APP                //enabled CLASSIFICATION_APP
  //#define ANN_TEST                          // This will get data from serial port istead of MPU and calculate result
#endif
   
   
//#define CORE_TEMP_LOGGING                 //Get the Core Temprature and log
#define LSE_TEMP_COMPENSATION             //Get the Core Temprature and log
#define TEMPERATURE_COEFFICIENT           (-0.034f)       // ppm/°C²

/**
 * @defgroup   Battery_ADC_BSP				Battery ADC BSP
 * @brief		Voltage divider circuit used to measure voltage level of the battery.
 * @{
 */

#define BATTERY_VOLTAGE_MAX		(4.2f)           
#define BATTERY_VOLTAGE_MIN		(1.8f) 
   
#define BATTERY_ADC_CHANNEL		ADC_CHANNEL_5
#define BATTERY_ADC_Vref		  (1.8f)           //vref voltage 1.8


#define ADC_BATTERY_ENABLE()    
#define ADC_BATTERY_DISABLE() 

#define BATTERY_VOLTAGE_SCALING_FACTOR_R1 (335.0f)
#define BATTERY_VOLTAGE_SCALING_FACTOR_R2 (335.0f)
#define BATTERY_VOLTAGE_SCALING_FACTOR    (float)((BATTERY_VOLTAGE_SCALING_FACTOR_R1+BATTERY_VOLTAGE_SCALING_FACTOR_R2)/BATTERY_VOLTAGE_SCALING_FACTOR_R2)

/** @}*/


 /**
 * @defgroup   SX1262				Lora Module
 * @brief		
 * @{
 */
  
#define wait_ms osDelay
extern SPI_HandleTypeDef hspi2;
#define SPI_SX1262_Handle   hspi2

#define LoRa_PowerEN(...)
#define LoRa_PowerDIS(...)

/** @}*/

/**
 * @defgroup   IPC				  Inter process Communication 
 * @brief		
 * @{
 */





/** @}*/

/** @addtogroup BSP
  * @{
  */

/** @addtogroup SENSORTILE
  * @{
  */
      
  /** @addtogroup SENSORTILE_LOW_LEVEL
  * @{
  */

/** @defgroup SENSORTILE_LOW_LEVEL_Exported_Types SENSORTILE_LOW_LEVEL Exported Types
  * @{
  */

typedef enum
{
//  TEMPERATURE_SENSORS_AUTO = -1, /* Always first element and equal to -1 */
  LSM6DSM = 0,                  /* LSM6DSM. */
} SPI_Device_t;

/**
  * @}
  */ 

/** @defgroup SENSORTILE_LOW_LEVEL_Exported_Constants SENSORTILE_LOW_LEVEL Exported Constants
  * @{
  */ 


#define LSM6DSM_INT1_GPIO_PORT           GPIOA
#define LSM6DSM_INT1_GPIO_CLK_ENABLE()   __GPIOA_CLK_ENABLE()
#define LSM6DSM_INT1_GPIO_CLK_DISABLE()  __GPIOA_CLK_DISABLE()
#define LSM6DSM_INT1_PIN                 GPIO_PIN_1
#define LSM6DSM_INT1_EXTI_IRQn           EXTI1_IRQn

#define LSM6DSM_INT2_GPIO_PORT           GPIOA
#define LSM6DSM_INT2_GPIO_CLK_ENABLE()   __GPIOA_CLK_ENABLE()
#define LSM6DSM_INT2_GPIO_CLK_DISABLE()  __GPIOA_CLK_DISABLE()
#define LSM6DSM_INT2_PIN                 GPIO_PIN_2
#define LSM6DSM_INT2_EXTI_IRQn           EXTI2_IRQn


                                              
#define LSM6DSM_SENSORS_SPI                    SPI1

#define LSM6DSM_SENSORS_SPI_Port               GPIOA
#define LSM6DSM_SENSORS_SPI_MOSI_Pin           MPU_MOSI_Pin
#define LSM6DSM_SENSORS_SPI_SCK_Pin            MPU_SCK_Pin

#define LSM6DSM_SENSORS_SPI_CLK_ENABLE()       __SPI2_CLK_ENABLE()
#define LSM6DSM_SENSORS_SPI_GPIO_CLK_ENABLE()  __GPIOA_CLK_ENABLE()

#define LSM6DSM_SPI_CS_Port	          		MPU_NSS_GPIO_Port
#define LSM6DSM_SPI_CS_Pin     	  				MPU_NSS_Pin
#define LSM6DSM_SPI_CS_GPIO_CLK_ENABLE()  	__GPIOA_CLK_ENABLE()
   
                                                
/**
  * @}
  */ 


/** @defgroup BOARD_LOW_LEVEL_Exported_Macros SENSORTILE_LOW_LEVEL Exported Macros
  * @{
  */  
/**
  * @}
  */ 
/** @defgroup BOARD_LOW_LEVEL_Exported_Functions SENSORTILE_LOW_LEVEL Exported Functions
  * @{
  */
uint32_t         BSP_GetVersion(void);  
DrvStatusTypeDef Sensor_IO_SPI_Init( void );
DrvStatusTypeDef Sensor_IO_SPI_DeInit( void );
uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
uint8_t Sensor_IO_SPI_Read_DMA( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
uint8_t Sensor_IO_SPI_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
uint8_t Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
uint8_t Sensor_IO_SPI_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
uint8_t Sensor_IO_SPI_CS_Init(void *handle);
uint8_t Sensor_IO_SPI_CS_Enable(void *handle);
uint8_t Sensor_IO_SPI_CS_Disable(void *handle);
DrvStatusTypeDef LSM6DSM_Sensor_IO_ITConfig( void );


/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __COWLAR_NODE_V8_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
