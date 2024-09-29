/**
  ******************************************************************************
  * @file    nfc04a1.c
  * @author  MMY Application Team
  * @version $Revision: 3351 $
  * @date    $Date: 2017-01-25 17:28:08 +0100 (Wed, 25 Jan 2017) $
  * @brief   This file provides nfc04a1 specific functions
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

/* Includes ------------------------------------------------------------------*/
#include "nfc04a1.h"
#include "nfc04a1_nfctag.h"
#include "i2c.h"

/** @addtogroup BSP
 * @{
 */

/** @defgroup NFC04A1
 * @{
 */
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/ 
/* Global variables ----------------------------------------------------------*/
/** @defgroup NFC04A1_Global_Variables
 * @{
 */
 
#define hNFC04A1_i2c hi2c1

//#ifndef SMARTAG
//  I2C_HandleTypeDef hNFC04A1_i2c;
//#else /* SMARTAG */
//  /* For using the Global I2C Allocation */
//  #ifdef USE_STM32L0XX_NUCLEO
//    #include "stm32l0xx_I2C.h"
//  #endif /* USE_STM32L0XX_NUCLEO */

//  /* Global Handle */
//  #define hNFC04A1_i2c I2CHandle

//  /* Global I2C Initialization Function */
//  #define STM32_I2C1_Init(i2cspeed) I2C_Global_Init()
//#endif /* SMARTAG */
/**
 * @}
 */

/* Private function prototypes -----------------------------------------------*/
#ifndef SMARTAG
  static HAL_StatusTypeDef STM32_I2C1_Init( uint32_t i2cspeed );
  static HAL_StatusTypeDef STM32_I2C1_DeInit( void );
#endif /* SMARTAG */
static HAL_StatusTypeDef STM32_I2C1_MemWrite( const uint8_t * const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size );
static HAL_StatusTypeDef STM32_I2C1_MemRead( uint8_t * const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size );
static HAL_StatusTypeDef STM32_I2C1_Read( uint8_t * const pData, const uint8_t DevAddr, const uint16_t Size );
static uint8_t STM32_I2C1_IsNacked( void );
static HAL_StatusTypeDef STM32_I2C1_IsDeviceReady( const uint8_t DevAddr, const uint32_t Trials );
#ifndef SMARTAG
static void STM32_I2C1_MspInit( void );
static void STM32_I2C1_MspDeInit( void );
#endif /* SMARTAG */

static NFCTAG_StatusTypeDef NFCTAG_IO_Init( void );
NFCTAG_StatusTypeDef NFCTAG_IO_MemWrite( const uint8_t * const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size );
NFCTAG_StatusTypeDef NFCTAG_IO_MemRead( uint8_t * const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size );
NFCTAG_StatusTypeDef NFCTAG_IO_Read( uint8_t * const pData, const uint8_t DevAddr, const uint16_t Size );
uint8_t NFCTAG_IO_IsNacked( void );
NFCTAG_StatusTypeDef NFCTAG_IO_IsDeviceReady( const uint8_t DevAddr, const uint32_t Trials );

NFCTAG_StatusTypeDef ST25DV_IO_Init( void );
NFCTAG_StatusTypeDef ST25DV_IO_MemWrite( const uint8_t * const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size );
NFCTAG_StatusTypeDef ST25DV_IO_MemRead( uint8_t * const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size );
NFCTAG_StatusTypeDef ST25DV_IO_Read( uint8_t * const pData, const uint8_t DevAddr, const uint16_t Size );
uint8_t ST25DV_IO_IsNacked( void );
NFCTAG_StatusTypeDef ST25DV_IO_IsDeviceReady( const uint8_t DevAddress, const uint32_t Trials );
NFCTAG_StatusTypeDef NFCTAG_ConvertStatus( const HAL_StatusTypeDef status );

/* Functions Definition ------------------------------------------------------*/
/** @defgroup NFC04A1_Public_Functions
 * @{
 */
void PowerOnNFC(void)
{
  NFC04A1_LPD_WritePin(GPIO_PIN_RESET);
}

void PowerOffNFC(void)
{
  NFC04A1_LPD_WritePin(GPIO_PIN_SET);
}
/**
  * @brief  This function initialize the GPIO to manage the Leds
  * @brief  through GPIO
  * @param  None
  * @retval None
  */
void NFC04A1_LED_Init( void )
{
}

/**
  * @brief  DeInit LEDs.
  * @param  Led: LED to be de-init. 
  * @note Led DeInit does not disable the GPIO clock nor disable the Mfx
  */
void NFC04A1_LED_DeInit( NFC04A1_Led_E led )
{
}

/**
  * @brief  This function light on selected Led
  * @param  led : Led to be lit on
  * @retval None
  */
void NFC04A1_LED_ON( const NFC04A1_Led_E led )
{
}

/**
  * @brief  This function light off selected Led
  * @param  led : Led to be lit off
  * @retval None
  */
void NFC04A1_LED_OFF( const NFC04A1_Led_E led )
{
}

/**
  * @brief  Toggles the selected LED
  * @param  led : Specifies the Led to be toggled
  * @retval None
  */
void NFC04A1_LED_Toggle( const NFC04A1_Led_E led )
{
}

/**
  * @brief  This function initialize the GPIO to manage the NFCTAG GPO pin
  * @param  None
  * @retval None
  */
void NFC04A1_GPO_Init( void )
{
//  GPIO_InitTypeDef GPIO_InitStruct;
//  NFC04A1_INIT_CLK_GPO_RFD( );

//  GPIO_InitStruct.Pin   = NFC04A1_GPO_PIN;
//  GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.Pull  = GPIO_PULLUP;
//  HAL_GPIO_Init( NFC04A1_GPO_PIN_PORT, &GPIO_InitStruct );
}

/**
  * @brief  DeInit GPO.
  * @param  None.
  * @note GPO DeInit does not disable the GPIO clock nor disable the Mfx
  */
void NFC04A1_GPO_DeInit( void )
{
  GPIO_InitTypeDef  gpio_init_structure;

  /* DeInit the GPIO_GPO pin */
  gpio_init_structure.Pin = NFC04A1_GPO_PIN;
  HAL_GPIO_DeInit( NFC04A1_GPO_PIN_PORT, gpio_init_structure.Pin);
}

/**
  * @brief  This function get the GPIO value through GPIO
  * @param  None
  * @retval HAL GPIO pin status
  */
GPIO_PinState NFC04A1_GPO_ReadPin( void )
{
  return HAL_GPIO_ReadPin( NFC04A1_GPO_PIN_PORT, NFC04A1_GPO_PIN );
}

/**
  * @brief  This function initialize the GPIO to manage the NFCTAG LPD pin
  * @param  None
  * @retval None
  */
void NFC04A1_LPD_Init( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;
  NFC04A1_INIT_CLK_LPD_RFD( );

  GPIO_InitStruct.Pin   = NFC04A1_LPD_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init( NFC04A1_LPD_PIN_PORT, &GPIO_InitStruct );
  
  HAL_GPIO_WritePin( NFC04A1_LPD_PIN_PORT, NFC04A1_LPD_PIN, GPIO_PIN_RESET );
  osDelay(200);
}

/**
  * @brief  DeInit LPD.
  * @param  None.
  * @note LPD DeInit does not disable the GPIO clock nor disable the Mfx
  */
void NFC04A1_LPD_DeInit( void )
{
  GPIO_InitTypeDef  gpio_init_structure;

  /* DeInit the GPIO_GPO pin */
  gpio_init_structure.Pin = NFC04A1_LPD_PIN;
  HAL_GPIO_DeInit( NFC04A1_LPD_PIN_PORT, gpio_init_structure.Pin);
}

/**
  * @brief  This function get the GPIO value through GPIO
  * @param  None
  * @retval HAL GPIO pin status
  */
GPIO_PinState NFC04A1_LPD_ReadPin( void )
{
  return HAL_GPIO_ReadPin( NFC04A1_LPD_PIN_PORT, NFC04A1_LPD_PIN );
}

/**
  * @brief  This function get the GPIO value through GPIO
  * @param  None
  * @retval HAL GPIO pin status
  */
void NFC04A1_LPD_WritePin( GPIO_PinState LpdPinState )
{
  HAL_GPIO_WritePin( NFC04A1_LPD_PIN_PORT, NFC04A1_LPD_PIN, LpdPinState );
}

#ifndef SMARTAG
/**
  * @brief  This function select the I2C1 speed to communicate with NFCTAG
  * @param  i2cspeedchoice Number from 0 to 5 to select i2c speed
  * @retval HAL GPIO pin status
  */
void NFC04A1_SelectI2cSpeed( uint8_t i2cspeedchoice )
{
  if( HAL_I2C_GetState(&hNFC04A1_i2c) != HAL_I2C_STATE_RESET )
  {
    /* DeInit the I2C */
    STM32_I2C1_DeInit( );
  }
  
  switch( i2cspeedchoice )
  {
    case 0:
      
      STM32_I2C1_Init( NFC04A1_ST25DV_I2C_SPEED_10K );
      break;
    
    case 1:
      
      STM32_I2C1_Init( NFC04A1_ST25DV_I2C_SPEED_100K );
      break;
    
    case 2:
      
      STM32_I2C1_Init( NFC04A1_ST25DV_I2C_SPEED_200K );
      break;
    
    case 3:
      
      STM32_I2C1_Init( NFC04A1_ST25DV_I2C_SPEED_400K );
      break;
    
    case 4:
      
      STM32_I2C1_Init( NFC04A1_ST25DV_I2C_SPEED_800K );
      break;
    
    case 5:
      
      STM32_I2C1_Init( NFC04A1_ST25DV_I2C_SPEED_1M );
      break;
    
    default:
      
      STM32_I2C1_Init( NFC04A1_ST25DV_I2C_SPEED_1M );
      break;
  }    
    
}
#endif /* SMARTAG */

/**
 * @}
 */

/** @defgroup NFC04A1_Private_Functions
 * @{
 */
/******************************** LINK EEPROM COMPONENT *****************************/

/**
  * @brief  Initializes peripherals used by the I2C NFCTAG driver
  * @param  None
  * @retval NFCTAG enum status
  */
NFCTAG_StatusTypeDef ST25DV_IO_Init( void )
{
  return NFCTAG_IO_Init( );
}

/**
  * @brief  Write data, at specific address, through i2c to the ST25DV
  * @param  pData: pointer to the data to write
  * @param  DevAddr : Target device address
  * @param  TarAddr : I2C data memory address to write
  * @param  Size : Size in bytes of the value to be written
  * @retval NFCTAG enum status
  */
NFCTAG_StatusTypeDef ST25DV_IO_MemWrite( const uint8_t * const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size )
{
  NFCTAG_StatusTypeDef pollstatus;
  NFCTAG_StatusTypeDef ret;
  uint32_t tickstart;
  
  ret = NFCTAG_IO_MemWrite( pData, DevAddr, TarAddr, Size );
  if( ret == NFCTAG_OK )
  {
    /* Poll until EEPROM is available */
    tickstart = HAL_GetTick();
    /* Wait until ST25DV is ready or timeout occurs */
    do
    {
      pollstatus = ST25DV_IO_IsDeviceReady( DevAddr, 1 );
    } while( ( (HAL_GetTick() - tickstart) < NFC04A1_I2C_TIMEOUT) && (pollstatus != NFCTAG_OK) );
    
    if( pollstatus != NFCTAG_OK )
    {
      ret = NFCTAG_TIMEOUT;
    }
  }
  else
  {
    /* Check if Write was NACK */
    if( ST25DV_IO_IsNacked() == I2CANSW_NACK )
    {
      ret = NFCTAG_NACK;
    }
  }
  
  return ret;
}

/**
  * @brief  Reads data at a specific address from the NFCTAG.
  * @param  pData: pointer to store read data
  * @param  DevAddr : Target device address
  * @param  TarAddr : I2C data memory address to read
  * @param  Size : Size in bytes of the value to be read
  * @retval NFCTAG enum status
  */
NFCTAG_StatusTypeDef ST25DV_IO_MemRead( uint8_t * const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size )
{
  return NFCTAG_IO_MemRead( pData, DevAddr, TarAddr, Size );
}

/**
  * @brief  Reads data at current address from the NFCTAG.
  * @param  pData: pointer to store read data
  * @param  DevAddr : Target device address
  * @param  Size : Size in bytes of the value to be read
  * @retval NFCTAG enum status
  */
NFCTAG_StatusTypeDef ST25DV_IO_Read( uint8_t * const pData, const uint8_t DevAddr, const uint16_t Size )
{
  return NFCTAG_IO_Read( pData, DevAddr, Size );
}

/**
  * @brief  Checks if NACK was received from I2C Slave
  * @param  None
  * @retval 0 ACK, 1 NACK
  */
uint8_t ST25DV_IO_IsNacked( void )
{
  return NFCTAG_IO_IsNacked( );
}

/**
* @brief  Checks if target device is ready for communication
* @note   This function is used with Memory devices
* @param  DevAddr : Target device address
* @param  Trials : Number of trials
* @retval NFCTAG enum status
*/
NFCTAG_StatusTypeDef ST25DV_IO_IsDeviceReady( const uint8_t DevAddr, const uint32_t Trials )
{ 
  return NFCTAG_IO_IsDeviceReady( DevAddr, Trials );
}

/******************************** LINK NFCTAG *****************************/
/**
  * @brief  This functions converts HAL status to NFCTAG status
  * @param  status : HAL status to convert
  * @retval NFCTAG enum status
  */
NFCTAG_StatusTypeDef NFCTAG_ConvertStatus( const HAL_StatusTypeDef status )
{
  switch( status )
  {
    case HAL_OK:
      return NFCTAG_OK;
    case HAL_ERROR:
      return NFCTAG_ERROR;
    case HAL_BUSY:
      return NFCTAG_BUSY;
    case HAL_TIMEOUT:
      return NFCTAG_TIMEOUT;
    
    default:
      return NFCTAG_TIMEOUT;
  }
}

/**
  * @brief  Configures nfctag I2C interface
  * @param  None
  * @retval NFCTAG enum status
  */
static NFCTAG_StatusTypeDef NFCTAG_IO_Init( void )
{
#ifndef SMARTAG
  NFC04A1_GPO_Init( );
  NFC04A1_LPD_Init( );
#endif /* SMARTAG */
  
  return NFCTAG_ConvertStatus( STM32_I2C1_Init( ST25DV_I2C_SPEED ) );
}

/**
  * @brief  Write at specific address nfctag memory
  * @param  pData : pointer to the data to write
  * @param  DevAddr : Target device address
  * @param  TarAddr : I2C data memory address to write
  * @param  Size : Size in bytes of the value to be written
  * @retval NFCTAG enum status
  */
NFCTAG_StatusTypeDef NFCTAG_IO_MemWrite( const uint8_t * const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size )
{
  return NFCTAG_ConvertStatus( STM32_I2C1_MemWrite( pData, DevAddr, TarAddr, Size ) );
}

/**
  * @brief  Read at specific address on nfctag
  * @param  pData : pointer to store read data
  * @param  DevAddr : Target device address
  * @param  TarAddr : I2C data memory address to read
  * @param  Size : Size in bytes of the value to be read
  * @retval NFCTAG enum status
  */
NFCTAG_StatusTypeDef NFCTAG_IO_MemRead( uint8_t * const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size )
{
  return NFCTAG_ConvertStatus( STM32_I2C1_MemRead( pData, DevAddr, TarAddr, Size ) );
}

/**
  * @brief  Read at current address on nfctag
  * @param  pData : pointer to store read data
  * @param  DevAddr : Target device address
  * @param  Size : Size in bytes of the value to be read
  * @retval NFCTAG enum status
  */
NFCTAG_StatusTypeDef NFCTAG_IO_Read( uint8_t * const pData, const uint8_t DevAddr, const uint16_t Size )
{
  return NFCTAG_ConvertStatus( STM32_I2C1_Read( pData, DevAddr, Size ) );
}

/**
  * @brief  Checks if NACK was received from I2C Slave
  * @param  None
  * @retval 0 ACK, 1 NACK
  */
uint8_t NFCTAG_IO_IsNacked( void )
{
  return STM32_I2C1_IsNacked( );
}

/**
  * @brief  Check nfctag availability
  * @param  DevAddr : Target device address
  * @param  Trials : Number of trials
  * @retval NFCTAG enum status
  */
NFCTAG_StatusTypeDef NFCTAG_IO_IsDeviceReady( const uint8_t DevAddr, const uint32_t Trials )
{
  return NFCTAG_ConvertStatus( STM32_I2C1_IsDeviceReady( DevAddr, Trials ) );
}

/******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/
#ifndef SMARTAG
/**
  * @brief  Configures I2C interface.
  * @param  None
  * @retval HAL status
  */
static HAL_StatusTypeDef STM32_I2C1_Init( uint32_t i2cspeed )
{
  HAL_StatusTypeDef ret_val = HAL_OK;
  MX_I2C1_Init();
  
  if( HAL_I2C_GetState(&hNFC04A1_i2c) == HAL_I2C_STATE_RESET )
  {
    /* NFC04A1_I2Cx peripheral configuration */

//    hNFC04A1_i2c.Instance = I2C1;
//    hNFC04A1_i2c.Init.ClockSpeed = i2cspeed;
//    hNFC04A1_i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//    hNFC04A1_i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
//    hNFC04A1_i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
//    hNFC04A1_i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;

    /* Init the I2C */
    STM32_I2C1_MspInit();
    ret_val = HAL_I2C_Init( &hNFC04A1_i2c );

    if( i2cspeed > NFC04A1_ST25DV_I2C_SPEED_400K )
    {
#if (defined (USE_STM32L0XX_NUCLEO))
      /* I2C Fast mode Plus enable */
      HAL_I2CEx_EnableFastModePlus( I2C_FASTMODEPLUS_I2C1 );
#elif (defined (USE_STM32F4XX_NUCLEO))
      /* Configure Analogue filter for fast mode plus */
      ret_val |= HAL_I2CEx_ConfigAnalogFilter( &hNFC04A1_i2c, I2C_ANALOGFILTER_DISABLE );
      /* Configure Digital filter for fast mode plus */
      HAL_I2CEx_ConfigDigitalFilter( &hNFC04A1_i2c, 0x02);
#endif
    }
  }
  
  return ret_val;
}

/**
  * @brief  Deinitializes I2C interface.
  * @param  None
  * @retval HAL status
  */
static HAL_StatusTypeDef STM32_I2C1_DeInit( void )
{
  HAL_StatusTypeDef ret_val = HAL_OK;
  
  STM32_I2C1_MspDeInit();
  ret_val = HAL_I2C_DeInit( &hNFC04A1_i2c );
  
  return ret_val;
}
#endif /* SMARTAG */
/**
  * @brief  Write data in a register of the device through the bus
  * @param  pData : pointer to the data to write
  * @param  DevAddr : Target device address
  * @param  TarAddr : I2C data memory address to write
  * @param  Size : Size in bytes of the value to be written
  * @retval HAL status
  */

static HAL_StatusTypeDef STM32_I2C1_MemWrite( const uint8_t * const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size )
{
  uint8_t *pbuffer = (uint8_t *)pData;
  
  return HAL_I2C_Mem_Write( &hNFC04A1_i2c, DevAddr, TarAddr, I2C_MEMADD_SIZE_16BIT, pbuffer, Size, NFC04A1_I2C_TIMEOUT );
}

/**
  * @brief  Read the value of a register of the device through the bus
  * @param  pData : pointer to store read data
  * @param  DevAddr : Target device address
  * @param  TarAddr : I2C data memory address to read
  * @param  Size : Size in bytes of the value to be read
  * @retval HAL status.
  */
static HAL_StatusTypeDef STM32_I2C1_MemRead( uint8_t * const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size )
{
  uint8_t *pbuffer = (uint8_t *)pData;
  HAL_StatusTypeDef ret;

  ret = HAL_I2C_Mem_Read( &hNFC04A1_i2c, DevAddr, TarAddr, I2C_MEMADD_SIZE_16BIT, pbuffer, Size, NFC04A1_I2C_TIMEOUT );

  return ret;
}

/**
  * @brief  Read the value of a register of the device through the bus
  * @param  pData : pointer to store read data
  * @param  DevAddr : the device address on bus
  * @param  Size : Size in bytes of the value to be read
  * @retval HAL status
  */
static HAL_StatusTypeDef STM32_I2C1_Read( uint8_t * const pData, const uint8_t DevAddr, const uint16_t Size )
{
  uint8_t *pbuffer = (uint8_t *)pData;
  HAL_StatusTypeDef ret;
  
  ret = HAL_I2C_Master_Receive( &hNFC04A1_i2c, DevAddr, pbuffer, Size, NFC04A1_I2C_TIMEOUT );
  
  return ret;
}

/**
* @brief  Checks if NACK was received from I2C Slave
* @param  None
* @retval 0 ACK, 1 NACK
*/
static uint8_t STM32_I2C1_IsNacked( void )
{
  if( hNFC04A1_i2c.ErrorCode == HAL_I2C_ERROR_AF )
  {
    return I2CANSW_NACK;
  }
  return I2CANSW_ACK;
}

/**
* @brief  Checks if target device is ready for communication
* @param  DevAddr : Target device address
* @param  Trials : Number of trials
* @retval HAL status
*/
static HAL_StatusTypeDef STM32_I2C1_IsDeviceReady( const uint8_t DevAddr, const uint32_t Trials )
{ 
  return HAL_I2C_IsDeviceReady( &hNFC04A1_i2c, DevAddr, Trials, NFC04A1_I2C_TIMEOUT );
}
#ifndef SMARTAG
/**
  * @brief  I2C MSP Initialization 
  *         This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - DMA configuration for transmission request by peripheral 
  *           - NVIC configuration for DMA interrupt request enable
  * @param  None
  * @retval None
  */
static void STM32_I2C1_MspInit( void )
{  
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable NFC04A1 I2C clock */
  NFC04A1_I2C_CLK_ENABLE( );

  /* Reset NFC04A1 I2C */
  NFC04A1_I2C_FORCE_RESET();
  NFC04A1_I2C_RELEASE_RESET();
  
  /* Enable GPIO clock */
  NFC04A1_I2C_GPIO_CLK_ENABLE( );
  
  /* NFC04A1 I2C SCL/SDA GPIO pin configuration  */
  GPIO_InitStruct.Pin       = NFC04A1_SCL_PIN | NFC04A1_SDA_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = NFC04A1_I2C_AF;
  
  HAL_GPIO_Init( NFC04A1_SCL_PIN_PORT, &GPIO_InitStruct );
}

/**
  * @brief  I2C MSP DeInitialization 
  * @param  None
  * @retval None
  */
static void STM32_I2C1_MspDeInit( void )
{  
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* NFC04A1 I2C SCL/SDA GPIO pin configuration  */
  GPIO_InitStruct.Pin       = NFC04A1_SCL_PIN | NFC04A1_SDA_PIN;
  
  HAL_GPIO_DeInit( NFC04A1_SCL_PIN_PORT, GPIO_InitStruct.Pin );
  
  /* Disable NFC04A1 I2C clock */
  NFC04A1_I2C_CLK_DISABLE( );
}
#endif /* SMARTAG */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
