
/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
* File Name          : LSM6DSM_ACC_GYRO_driver_fifo.c
* Author             : Cowlar Firmware Team
* Version            : v0.1
* Date               : 01 Nov 2017 
* Description        : LSM6DSM ACC_GYRO driver for FIFO source file
*                      
* Reviewed by: 
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

/* Includes ------------------------------------------------------------------*/
#include "LSM6DSM_ACC_GYRO_driver_fifo.h"

/* Imported function prototypes ----------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/
/*
 * Following is the table of sensitivity values for each case.
 * Values are espressed in ug/digit.
 */
static const long long LSM6DSM_ACC_Sensitivity_List[4] = {
      61,	/* FS @2g */
      122,	/* FS @4g */
      244,	/* FS @8g */
      488,	/* FS @16g */
};
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name		: LSM6DSM_ACC_GYRO_Enable_FIFO_stream
* Description			: Enable the FIFO in Continuous mode
*									: 					
* Input       		: LSM6DSM_ACC_GYRO_FIFO_MODE_t
* Output					: Data REad
* Return					:  Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DSM_ACC_GYRO_Enable_FIFO_stream(void *handle, u8_t enableWaterMark, u16_t fifoWaterMark) 
{
	/* FIFO mode selection */
	if ( LSM6DSM_ACC_GYRO_W_FIFO_MODE( (void *)handle, LSM6DSM_ACC_GYRO_FIFO_MODE_DYN_STREAM_2 ) == MEMS_ERROR )
	{
		return MEMS_ERROR;
	} 
	/* FIFO ODR selection */
	if ( LSM6DSM_ACC_GYRO_W_ODR_FIFO( (void *)handle, LSM6DSM_ACC_GYRO_ODR_FIFO_6600Hz ) == MEMS_ERROR )
	{
		return MEMS_ERROR;
	}	
	
	if(enableWaterMark)
	{
		fifoWaterMark /= 2;   
		if ( LSM6DSM_ACC_GYRO_W_FIFO_Watermark( (void *)handle, fifoWaterMark ) == MEMS_ERROR )
		{
			return MEMS_ERROR;
		}   
		if ( LSM6DSM_ACC_GYRO_W_STOP_ON_FTH( (void *)handle, LSM6DSM_ACC_GYRO_STOP_ON_FTH_DISABLED ) == MEMS_ERROR )
		{
			return MEMS_ERROR;
		}
	}
	
	//Temp Logging Enable/Disable 
	if ( LSM6DSM_ACC_GYRO_W_FIFO_TEMP( (void *)handle, LSM6DSM_ACC_GYRO_FIFO_TEMP_DISABLE ) == MEMS_ERROR )
	{
		return MEMS_ERROR;
	}
		
	// Step Logging Enable/Disable                                  
	if ( LSM6DSM_ACC_GYRO_W_TIM_PEDO_FIFO_Write_En( (void *)handle, LSM6DSM_ACC_GYRO_TIM_PEDO_FIFO_DRDY_DISABLED ) == MEMS_ERROR )
	{
		return MEMS_ERROR;
	}	
	// Step Logging Enable/Disable
	if ( LSM6DSM_ACC_GYRO_W_TIM_PEDO_FIFO_En( (void *)handle, LSM6DSM_ACC_GYRO_TIM_PEDO_FIFO_EN_DISABLED ) == MEMS_ERROR )
	{
		return MEMS_ERROR;
	}
	
	// Accl Logging Enable/Disable
	if ( LSM6DSM_ACC_GYRO_W_DEC_FIFO_XL( (void *)handle, LSM6DSM_ACC_GYRO_DEC_FIFO_XL_NO_DECIMATION ) == MEMS_ERROR )
	{
		return MEMS_ERROR;
	}	
	// gyro Logging Enable/Disable
	if ( LSM6DSM_ACC_GYRO_W_DEC_FIFO_G( (void *)handle, LSM6DSM_ACC_GYRO_DEC_FIFO_G_DATA_NOT_IN_FIFO ) == MEMS_ERROR )
	{
		return MEMS_ERROR;
	}	
		
	if ( LSM6DSM_ACC_GYRO_W_DEC_FIFO_DS3( (void *)handle, LSM6DSM_ACC_GYRO_DEC_FIFO_DS3_DATA_NOT_IN_FIFO ) == MEMS_ERROR )
	{
		return MEMS_ERROR;
	}	
	if ( LSM6DSM_ACC_GYRO_W_DEC_FIFO_DS4( (void *)handle, LSM6DSM_ACC_GYRO_DEC_FIFO_DS4_DATA_NOT_IN_FIFO ) == MEMS_ERROR )
	{
		return MEMS_ERROR;
	}	
	
	
  /* Enable basic Interrupts */
  if ( LSM6DSM_ACC_GYRO_W_BASIC_INT( (void *)handle, LSM6DSM_ACC_GYRO_BASIC_INT_ENABLED ) == MEMS_ERROR )
  {
    return MEMS_ERROR;
  }

	if ( LSM6DSM_ACC_GYRO_W_FIFO_TSHLD_on_INT2( (void *)handle, LSM6DSM_ACC_GYRO_INT2_FTH_ENABLED ) == MEMS_ERROR )
	{
		return MEMS_ERROR;
	}	
	if ( LSM6DSM_ACC_GYRO_W_LIR( (void *)handle, LSM6DSM_ACC_GYRO_LIR_DISABLED ) == MEMS_ERROR )
	{
		return MEMS_ERROR;
	}	
	

	return MEMS_SUCCESS;

}
/*******************************************************************************
* Function Name		: LSM6DSM_ACC_GYRO_Disable_FIFO_stream
* Description			: Disable the FIFO 
*									: 					
* Input       		: LSM6DSM_ACC_GYRO_FIFO_MODE_t
* Output					: Data REad
* Return					:  Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DSM_ACC_GYRO_Disable_FIFO_stream(void *handle) 
{ 
  /* FIFO mode selection */
	if ( LSM6DSM_ACC_GYRO_W_FIFO_MODE( (void *)handle, LSM6DSM_ACC_GYRO_FIFO_MODE_BYPASS ) == MEMS_ERROR )
	{
		return MEMS_ERROR;
	} 
  return MEMS_SUCCESS;

}
  
/*******************************************************************************
* Function Name  : LSM6DSM_ACC_GYRO_Get_GetFIFOnData
* Description    : Read GetFIFOData output register fof n samples
*                : Each samples is 6 bytes
* Input          : pointer to [u8_t]
* Input          : samples number of sampes
* Output         : GetFIFOData buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DSM_ACC_GYRO_Get_GetFIFOnData(void *handle, u8_t *buff, u16_t samples) 
{
	u16_t len = samples * 6;
	
	if( !LSM6DSM_ACC_GYRO_ReadMem(handle, LSM6DSM_ACC_GYRO_FIFO_DATA_OUT_L, &buff[0], len))
		return MEMS_ERROR;

  return MEMS_SUCCESS; 
}

/*
 * Returned values are espressed in mg.
 * Regesterd are in 16 bit
 * 3 Axis for Accl
 * output in float for raw use
 */
status_t LSM6DSM_ACC_GYRO_ConvertToRaw_ACC(void *handle, i16_t *buff, float *outBuff, u16_t len)
{
	int32_t i = 0;
  
//  LSM6DSM_ACC_GYRO_FS_XL_t fs;
//  long long sensitivity=0;
//  /* Read out current odr, fs, hf setting */
//  if(!LSM6DSM_ACC_GYRO_R_FS_XL(handle, &fs)) {
//    return MEMS_ERROR;
//  }

//  /* Determine the sensitivity according to fs */
//  switch(fs) {
//  case LSM6DSM_ACC_GYRO_FS_XL_2g:
//    sensitivity = LSM6DSM_ACC_Sensitivity_List[0];
//    break;

//  case LSM6DSM_ACC_GYRO_FS_XL_4g:
//    sensitivity = LSM6DSM_ACC_Sensitivity_List[1];
//    break;

//  case LSM6DSM_ACC_GYRO_FS_XL_8g:
//    sensitivity = LSM6DSM_ACC_Sensitivity_List[2];
//    break;

//  case LSM6DSM_ACC_GYRO_FS_XL_16g:
//    sensitivity = LSM6DSM_ACC_Sensitivity_List[3];
//    break;
//  }

	for(i = 0; i< len; i++)
	{
		/* Apply proper shift and sensitivity    LSM6DSM_ACC_GYRO_FS_XL_2g */
		outBuff[i*3 + 0] = (buff[i*3 + 0] * (float)(0.000061));
		outBuff[i*3 + 1] = (buff[i*3 + 1] * (float)(0.000061));
		outBuff[i*3 + 2] = (buff[i*3 + 2] * (float)(0.000061));
	}
  return MEMS_SUCCESS;
}

