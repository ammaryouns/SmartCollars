/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
*
* File Name          : LSM6DSM_ACC_GYRO_driver_fifo.h
* Author             : Cowlar Firmware Team
* Version            : v0.1
* Date               : 01 Nov 2017 
* Description        : LSM6DSM ACC_GYRO driver for FIFO source file
*                      
* Reviewed by: 
*
*-------------------------------------------------------------------------------
*
*
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LSM6DSM_ACC_GYRO_DRIVER_FIFO__H
#define __LSM6DSM_ACC_GYRO_DRIVER_FIFO__H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "LSM6DSM_ACC_GYRO_driver_HL.h"
#include "LSM6DSM_ACC_GYRO_driver.h"
/* Exported types ------------------------------------------------------------*/


/* Exported common structure --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

status_t LSM6DSM_ACC_GYRO_Enable_FIFO_stream(void *handle, u8_t enableWaterMark, u16_t fifoWaterMark);
status_t LSM6DSM_ACC_GYRO_Disable_FIFO_stream(void *handle);
status_t LSM6DSM_ACC_GYRO_Get_GetFIFOnData(void *handle, u8_t *buff, u16_t samples); 
status_t LSM6DSM_ACC_GYRO_ConvertToRaw_ACC(void *handle, i16_t *buff, float *outBuff, u16_t len);


#endif
