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

/* Includes ------------------------------------------------------------------*/
#include "Cowlar-node-v8.h"
#include "spi.h"
#include "SmarTagConf.h"
#include "cmsis_os.h"


/** @addtogroup BSP
* @{
*/ 


/** @addtogroup BOARD_LOW_LEVEL
* @brief This file provides a set of low level firmware functions 
* @{
*/

/** @defgroup BOARD_LOW_LEVEL_Private_TypesDefinitions SENSORTILE_LOW_LEVEL Private Typedef
* @{
*/

/**
* @}
*/

/** @defgroup BOARD_LOW_LEVEL__Private_Defines SENSORTILE_LOW_LEVEL Private Defines
* @{
*/

//#define SYNCHRO_WAIT(NB)       for(int i=0; i<NB; i++){__asm("dsb\n");}
//#define SYNCHRO_SPI_DELAY     (1)

/**
* @brief BOARD BSP Driver version number V1.0.0
*/
#define __Cowlar_node_v8_BSP_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __Cowlar_node_v8_BSP_VERSION_SUB1   (0x00) /*!< [23:16] sub1 version */
#define __Cowlar_node_v8_BSP_VERSION_SUB2   (0x00) /*!< [15:8]  sub2 version */
#define __Cowlar_node_v8_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __Cowlar_node_v8_BSP_VERSION         ((__Cowlar_node_v8_BSP_VERSION_MAIN << 24)\
|(__Cowlar_node_v8_BSP_VERSION_SUB1 << 16)\
  |(__Cowlar_node_v8_BSP_VERSION_SUB2 << 8 )\
    |(__Cowlar_node_v8_BSP_VERSION_RC))


/**
* @}
*/

/** @defgroup BOARD_LOW_LEVEL_Private_Variables SENSORTILE_LOW_LEVEL Private Variables 
* @{
*/
#define SPI_Sensor_Handle   hspi1




/**
* @}
*/


/** @defgroup SENSORTILE_LOW_LEVEL_Private_Functions SENSORTILE_LOW_LEVEL Private Functions
* @{
*/ 


void SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val);
void SPI_Read_nBytes(SPI_HandleTypeDef* xSpiHandle, uint8_t *val, uint16_t nBytesToRead );
void SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t val);


/**
* @}
*/



/** @defgroup SENSORTILE_LOW_LEVEL_Exported_Functions SENSORTILE_LOW_LEVEL Exported Functions
  * @{
  */


/**
* @brief  This method returns the STM32446E EVAL BSP Driver revision
* @param  None
* @retval version: 0xXYZR (8bits for each decimal, R for RC)
*/
uint32_t BSP_GetVersion(void)
{
  return __Cowlar_node_v8_BSP_VERSION;
}




/**
 * @brief  Configures sensor SPI interface.
 * @param  None
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef Sensor_IO_SPI_Init( void )
{
 
  if(HAL_SPI_GetState( &SPI_Sensor_Handle) == HAL_SPI_STATE_RESET )
  {
    MX_SPI1_Init();
  }  
  return COMPONENT_OK;
}
/**
 * @brief  Configures sensor SPI interface.
 * @param  None
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef Sensor_IO_SPI_DeInit( void )
{
 
  if(HAL_SPI_GetState( &SPI_Sensor_Handle) != HAL_SPI_STATE_RESET )
  {
    HAL_SPI_DeInit(&SPI_Sensor_Handle);
  }  
  return COMPONENT_OK;
}

uint8_t Sensor_IO_SPI_CS_Init(void *handle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  
  switch(ctx->spiDevice)
  {
  case LSM6DSM:
    LSM6DSM_SPI_CS_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = LSM6DSM_SPI_CS_Pin;
    /* Set the pin before init to avoid glitch */
    HAL_GPIO_WritePin(LSM6DSM_SPI_CS_Port, LSM6DSM_SPI_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_Init(LSM6DSM_SPI_CS_Port, &GPIO_InitStruct);
    break;  
  default:
    return COMPONENT_NOT_IMPLEMENTED;
  }
  return COMPONENT_OK;
}
/**
 * @brief  Writes a buffer to the sensor
 * @param  handle instance handle
 * @param  WriteAddr specifies the internal sensor address register to be written to
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToWrite number of bytes to be written
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite )
{
  
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx->ifType == 0)
  {

    return COMPONENT_ERROR;
  }
  
  if(ctx->ifType == 1)
  {
    if ( nBytesToWrite > 1 ) 
    {

    }
    return Sensor_IO_SPI_Write( handle, WriteAddr, pBuffer, nBytesToWrite );
  }
  
  return COMPONENT_ERROR;
}


/**
 * @brief  Reads from the sensor to a buffer
 * @param  handle instance handle
 * @param  ReadAddr specifies the internal sensor address register to be read from
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToRead number of bytes to be read
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx->ifType == 0)
  {
    
    return COMPONENT_ERROR;
  }
  
  if(ctx->ifType == 1 )
  {
    if ( nBytesToRead > 5 ) {
   return Sensor_IO_SPI_Read_DMA( handle, ReadAddr, pBuffer, nBytesToRead );

    }
   return Sensor_IO_SPI_Read( handle, ReadAddr, pBuffer, nBytesToRead );
  }
  
  return COMPONENT_ERROR;
}




/**
 * @brief  Writes a buffer to the sensor
 * @param  handle instance handle
 * @param  WriteAddr specifies the internal sensor address register to be written to
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToWrite number of bytes to be written
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_SPI_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite )
{
  HAL_StatusTypeDef halStatus;
  
// Select the correct device
  Sensor_IO_SPI_CS_Enable(handle);
  
  halStatus = HAL_SPI_Transmit(&SPI_Sensor_Handle, &WriteAddr, 1, 20);
  halStatus = HAL_SPI_Transmit(&SPI_Sensor_Handle, pBuffer, nBytesToWrite, 20);
  
  //  SPI_Write(&SPI_Sensor_Handle, WriteAddr);

//  for(i=0;i<nBytesToWrite;i++)
//  {
//    SPI_Write(&SPI_Sensor_Handle, pBuffer[i]);
//  }
// Deselect the device
  Sensor_IO_SPI_CS_Disable(handle);
  if (HAL_OK == halStatus)
    return COMPONENT_OK;
  else
    return COMPONENT_ERROR;
}
/**
 * @brief  Reads a from the sensor to buffer
 * @param  handle instance handle
 * @param  ReadAddr specifies the internal sensor address register to be read from
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToRead number of bytes to be read
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_SPI_Read_DMA( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead )
{
  static int counter= 0;
  /* Select the correct device */
  Sensor_IO_SPI_CS_Enable(handle);
  counter = 0;
  HAL_StatusTypeDef halStatus;
  ReadAddr |= 0x80;
  halStatus = HAL_SPI_Transmit(&SPI_Sensor_Handle, &ReadAddr, 1, 20);
  halStatus = HAL_SPI_Receive_DMA(&SPI_Sensor_Handle, pBuffer, nBytesToRead);
  while(SPI_Sensor_Handle.State != HAL_SPI_STATE_READY)
  {
    wait_ms((nBytesToRead / 800)+ 1);counter++;
  }
  /* Deselect the device */
  Sensor_IO_SPI_CS_Disable(handle);  
  

  if (HAL_OK == halStatus)
    return COMPONENT_OK;
  else
    return COMPONENT_ERROR;
}

/**
 * @brief  Reads a from the sensor to buffer
 * @param  handle instance handle
 * @param  ReadAddr specifies the internal sensor address register to be read from
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToRead number of bytes to be read
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_SPI_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead )
{
  /* Select the correct device */
  Sensor_IO_SPI_CS_Enable(handle);
  
  HAL_StatusTypeDef halStatus;
  ReadAddr |= 0x80;
  halStatus = HAL_SPI_Transmit(&SPI_Sensor_Handle, &ReadAddr, 1, 20);
  halStatus = HAL_SPI_Receive(&SPI_Sensor_Handle, pBuffer, nBytesToRead, 20);
 
  /* Deselect the device */
  Sensor_IO_SPI_CS_Disable(handle);  
  

  if (HAL_OK == halStatus)
    return COMPONENT_OK;
  else
    return COMPONENT_ERROR;
}


uint8_t Sensor_IO_SPI_CS_Enable(void *handle)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  switch(ctx->spiDevice)
  {
  case LSM6DSM:
    HAL_GPIO_WritePin(LSM6DSM_SPI_CS_Port, LSM6DSM_SPI_CS_Pin, GPIO_PIN_RESET);
    break;
  }
  return COMPONENT_OK;
}

uint8_t Sensor_IO_SPI_CS_Disable(void *handle)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  switch(ctx->spiDevice)
  {
  case LSM6DSM:
    HAL_GPIO_WritePin(LSM6DSM_SPI_CS_Port, LSM6DSM_SPI_CS_Pin, GPIO_PIN_SET);
    break;
  }
  return COMPONENT_OK;
}

/**
 * @brief  Configures sensor interrupts interface for LSM6DSM sensor.
 * @param  None
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef LSM6DSM_Sensor_IO_ITConfig( void )
{

  /* At the moment this feature is only implemented for LSM6DSM */
  GPIO_InitTypeDef GPIO_InitStructureInt2;
  
  /* Enable INT2 GPIO clock */
  LSM6DSM_INT2_GPIO_CLK_ENABLE();
  
  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructureInt2.Pin = LSM6DSM_INT2_PIN;
  GPIO_InitStructureInt2.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructureInt2.Speed = GPIO_SPEED_FAST;
  GPIO_InitStructureInt2.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(LSM6DSM_INT2_GPIO_PORT, &GPIO_InitStructureInt2);
  
  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(LSM6DSM_INT2_EXTI_IRQn, 0x08, 0x00);
  HAL_NVIC_EnableIRQ(LSM6DSM_INT2_EXTI_IRQn);
  
  return COMPONENT_OK;
}

//#if defined(__ICCARM__)
//#pragma optimize=none
//#endif

/**
 * @brief  This function reads a single byte on SPI 3-wire.
 * @param  xSpiHandle : SPI Handler.
 * @param  val : value.
 * @retval None
 */
void SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val)
{
  /* In master RX mode the clock is automaticaly generated on the SPI enable.
  So to guarantee the clock generation for only one data, the clock must be
  disabled after the first bit and before the latest bit */
  /* Interrupts should be disabled during this operation */
  
  __disable_irq();
  
  __HAL_SPI_ENABLE(xSpiHandle);
  __asm("dsb\n");
  __asm("dsb\n");
  __HAL_SPI_DISABLE(xSpiHandle);
  
  __enable_irq();
  
  while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
  /* read the received data */
  *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

/**
 * @brief  This function reads multiple bytes on SPI 3-wire.
 * @param  xSpiHandle: SPI Handler.
 * @param  val: value.
 * @param  nBytesToRead: number of bytes to read.
 * @retval None
 */
void SPI_Read_nBytes(SPI_HandleTypeDef* xSpiHandle, uint8_t *val, uint16_t nBytesToRead)
{
  /* Interrupts should be disabled during this operation */
  __disable_irq();
  __HAL_SPI_ENABLE(xSpiHandle);
  
  /* Transfer loop */
  while (nBytesToRead > 1U)
  {
    /* Check the RXNE flag */
    if (xSpiHandle->Instance->SR & SPI_FLAG_RXNE)
    {
      /* read the received data */
      *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
      val += sizeof(uint8_t);
      nBytesToRead--;
    }
  }
  /* In master RX mode the clock is automaticaly generated on the SPI enable.
  So to guarantee the clock generation for only one data, the clock must be
  disabled after the first bit and before the latest bit of the last Byte received */
  /* __DSB instruction are inserted to garantee that clock is Disabled in the right timeframe */
  
  __DSB();
  __DSB();
  __HAL_SPI_DISABLE(xSpiHandle);
  
  __enable_irq();
  
  while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
  /* read the received data */
  *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

/**
 * @brief  This function writes a single byte on SPI 3-wire.
 * @param  xSpiHandle: SPI Handler.
 * @param  val: value.
 * @retval None
 */
void SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t val)
{
  /* check TXE flag */
  while ((xSpiHandle->Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE);
  
  /* Write the data */
  *((__IO uint8_t*) &xSpiHandle->Instance->DR) = val;
  
  /* Wait BSY flag */
  while ((xSpiHandle->Instance->SR & SPI_FLAG_FTLVL) != SPI_FTLVL_EMPTY);
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}


/******************************* I2C Routines *********************************/




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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
