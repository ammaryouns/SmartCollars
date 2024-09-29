/**
  ******************************************************************************
  * @file    x_nucleo_nfc04a1.h
  * @author  MMY Application Team
  * @version $Revision: 3351 $
  * @date    $Date: 2017-01-25 17:28:08 +0100 (Wed, 25 Jan 2017) $
  * @brief   This file contains definitions for the x_nucleo_nfc04a1.c
  *          board specific functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
  *
  * Licensed under ST MYLIBERTY SOFTWARE LICENSE AGREEMENT (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/myliberty  
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __X_NUCLEO_NFC04A1_H
#define __X_NUCLEO_NFC04A1_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "bsp.h"
#include "st25dv.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup X_NUCLEO_NFC04A1
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup X_NUCLEO_NFC04A1_Exported_Types
  * @{
  */
/**
 * @brief  NFC04A1 Led enumerator definition
 */
typedef enum 
{
  GREEN_LED = 0,
  BLUE_LED,
  YELLOW_LED
}NFC04A1_Led_E;

/**
 * @brief  NFC04A1 Ack Nack enumerator definition
 */
typedef enum 
{
  I2CANSW_ACK = 0,
  I2CANSW_NACK
}NFC04A1_I2CANSW_E;

/**
 * @brief  NFC04A1 Led structure definition
 */
typedef struct
{
  uint16_t          NFC04A1_LED_PIN;
  GPIO_TypeDef *    NFC04A1_LED_PIN_PORT;
}NFC04A1_Led_TypeDef;

/**
  * @}
  */
/* Exported constants --------------------------------------------------------*/
/** @defgroup X_NUCLEO_NFC04A1_Exported_Constants
  * @{
  */
/* I/O Definition of ST25DV_NUCLEO board */
/* HAL connector names */
#define NFC04A1_SDA_PIN                   GPIO_PIN_9
#define NFC04A1_SDA_PIN_PORT              GPIOB
#define NFC04A1_SCL_PIN                   GPIO_PIN_8
#define NFC04A1_SCL_PIN_PORT              GPIOB
#define NFC04A1_GPO_PIN                   GPIO_PIN_13
#define NFC04A1_GPO_PIN_PORT              GPIOC
#define NFC04A1_LPD_PIN                   GPIO_PIN_7
#define NFC04A1_LPD_PIN_PORT              GPIOB


#define NFC04A1_GPO_EXTI                  EXTI15_10_IRQn
#define NFC04A1_GPO_IRQHandler            EXTI15_10_IRQHandler

/* ST25DV I2C Caracteristics for ST25DV_NUCLEO board */
#define NFC04A1_I2C_AF                    GPIO_AF4_I2C1

#define NFC04A1_I2C_CLK_ENABLE()          __HAL_RCC_I2C1_CLK_ENABLE( )
#define NFC04A1_I2C_CLK_DISABLE()         __HAL_RCC_I2C1_CLK_DISABLE( )
#define NFC04A1_I2C_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE( ) 

#define NFC04A1_I2C_FORCE_RESET()         __HAL_RCC_I2C1_FORCE_RESET( )
#define NFC04A1_I2C_RELEASE_RESET()       __HAL_RCC_I2C1_RELEASE_RESET( )

#define NFC04A1_INIT_CLK_GPO_RFD()        __HAL_RCC_GPIOC_CLK_ENABLE( );
#define NFC04A1_INIT_CLK_LPD_RFD()        __HAL_RCC_GPIOB_CLK_ENABLE( );


#define NFC04A1_ST25DV_I2C_SPEED_10K    10000
#define NFC04A1_ST25DV_I2C_SPEED_100K   100000
#define NFC04A1_ST25DV_I2C_SPEED_200K   200000
#define NFC04A1_ST25DV_I2C_SPEED_400K   400000
#define NFC04A1_ST25DV_I2C_SPEED_800K   800000
#define NFC04A1_ST25DV_I2C_SPEED_1M     1000000


#define ST25DV_I2C_SPEED                  NFC04A1_ST25DV_I2C_SPEED_1M
#define NFC04A1_I2C_TIMEOUT               ST25DV_I2C_TIMEOUT
/**
  * @}
  */

/* External variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/** @defgroup ST25DV_NUCLEO_Exported_Functions
  * @{
  */
void NFC04A1_LED_Init( void );
void NFC04A1_LED_DeInit( NFC04A1_Led_E led );
void NFC04A1_LED_ON( const NFC04A1_Led_E led );
void NFC04A1_LED_OFF( const NFC04A1_Led_E led );
void NFC04A1_LED_Toggle( const NFC04A1_Led_E led );
void NFC04A1_GPO_Init( void );
void NFC04A1_GPO_DeInit( void );
GPIO_PinState NFC04A1_GPO_ReadPin( void );
void NFC04A1_LPD_Init( void );
void NFC04A1_LPD_DeInit( void );
GPIO_PinState NFC04A1_LPD_ReadPin( void );
void NFC04A1_LPD_WritePin( GPIO_PinState LpdPinState );
extern void PowerOffNFC(void);
extern void PowerOnNFC(void);

#ifndef AZURE_MOTOR
  void NFC04A1_SelectI2cSpeed( uint8_t i2cspeedchoice );
#endif /* AZURE_MOTOR */
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
#endif /* __X_NUCLEO_NFC04A1_H */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
