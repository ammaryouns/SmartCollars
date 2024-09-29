
/**
  ******************************************************************************
  * @file    GSM_UART.c
  * @author  E4 R&D Team
  * @version V1.0.0
  * @date    03/11/2014
  * @brief   Peripheral Configurations
  ******************************************************************************
*/

//------Copied from GPS_tracker project-----------------

#include "GSM_UART.h"
#include "BSP.h"

/* Private typedefs -----------------------------------------------------------*/



/* Private define ------------------------------------------------------------*/

#define GSM_UART_TIMER_MAX      20

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Public variables ----------------------------------------------------------*/


uint8_t  SIMBufferNewCommand=0;
int8_t   GSMUARTtimerIndex = -1;
uint8_t  GSMreceiveComplete = 0;
uint8_t  GSMFlagByteRecieved = 0;
uint8_t  SIMBuffer[SIMBUFFERSIZE]={'\0'};				// Meter buffer of fixed size to store response from Sim900
uint16_t SIMBufferIndex=0;  				              // Meter buffer of Write index
uint16_t SIMBufferCount=0;				                // Meter buffer of size
/* Private functions  --------------------------------------------------------*/


/**
  * @brief  set timer for gsm uart
  * @param  None
  * @retval None
  */
void GSM_UART_Configuration(void){

    HAL_UART_Receive_DMA(&GSM_HUART, SIMBuffer, SIMBUFFERSIZE);
    __HAL_DMA_DISABLE_IT(GSM_HUART.hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(GSM_HUART.hdmarx, DMA_IT_TE);
    __HAL_UART_ENABLE_IT(&GSM_HUART, UART_IT_IDLE);
    
    
}


uint32_t GSMgetByte(uint8_t * byte)
{
  if (SIMBufferCount  >0)
  {
    *byte = SIMBuffer[SIMBufferRead];
    SIMBufferCount--;
  }
  else
  {
    *byte = '\0';
  }
	return SIMBufferCount;
}


void GSM_DMA_IRQHandler(void){

    if (__HAL_DMA_GET_FLAG(GSM_HUART.hdmarx, DMA_FLAG_TC5)){
        __HAL_DMA_CLEAR_FLAG(GSM_HUART.hdmarx, DMA_FLAG_TC5);
        SIMBufferCount += SIMBUFFERSIZE - SIMBufferIndex;
        if (SIMBufferCount >= SIMBUFFERSIZE) 
            SIMBufferCount = SIMBUFFERSIZE;
        SIMBufferIndex  = 0;
    }
}





void GSM_UART_IRQHandler(void)
{

    if (__HAL_UART_GET_FLAG(&GSM_HUART, UART_FLAG_IDLE)){
        
        __HAL_UART_CLEAR_IDLEFLAG(&GSM_HUART);
        
        GSMreceiveComplete  = 1;
        SIMBufferCount     += ((2*SIMBUFFERSIZE-GSM_HUART.hdmarx->Instance->CNDTR) - SIMBufferIndex)%SIMBUFFERSIZE;
        if (SIMBufferCount >= SIMBUFFERSIZE)
            SIMBufferCount = SIMBUFFERSIZE;
        SIMBufferIndex      = SIMBUFFERSIZE - GSM_HUART.hdmarx->Instance->CNDTR;
        
    }
    
//    tmp_flag = __HAL_UART_GET_FLAG(&GSM_HUART, UART_FLAG_PE);
//    tmp_it_source = __HAL_UART_GET_IT_SOURCE(&GSM_HUART, UART_IT_PE);  
//    /* UART parity error interrupt occurred ------------------------------------*/
//    if((tmp_flag != RESET) && (tmp_it_source != RESET))
//    { 
//        __HAL_UART_CLEAR_PEFLAG(&GSM_HUART);
//    }
//    tmp_flag = __HAL_UART_GET_FLAG(&GSM_HUART, UART_FLAG_FE);
//    tmp_it_source = __HAL_UART_GET_IT_SOURCE(&GSM_HUART, UART_IT_ERR);
//    /* UART frame error interrupt occurred -------------------------------------*/
//    if((tmp_flag != RESET) && (tmp_it_source != RESET))
//    { 
//        __HAL_UART_CLEAR_FEFLAG(&GSM_HUART);
//    }

//    
//    tmp_flag = __HAL_UART_GET_FLAG(&GSM_HUART, UART_FLAG_NE);
//    /* UART noise error interrupt occurred -------------------------------------*/
//    if((tmp_flag != RESET) && (tmp_it_source != RESET))
//    { 
//        __HAL_UART_CLEAR_NEFLAG(&GSM_HUART);

//    }

//    tmp_flag = __HAL_UART_GET_FLAG(&GSM_HUART, UART_FLAG_ORE);
//    /* UART Over-Run interrupt occurred ----------------------------------------*/
//    if((tmp_flag != RESET) && (tmp_it_source != RESET))
//    { 
//        __HAL_UART_CLEAR_OREFLAG(&GSM_HUART);
//    
//    }


//    tmp_flag = __HAL_UART_GET_FLAG(&GSM_HUART, UART_FLAG_TXE);
//    tmp_it_source = __HAL_UART_GET_IT_SOURCE(&GSM_HUART, UART_IT_TXE);
//    /* UART in mode Transmitter ------------------------------------------------*/
//    if((tmp_flag != RESET) && (tmp_it_source != RESET))
//    {
//        __HAL_UART_CLEAR_FLAG(&GSM_HUART,UART_FLAG_TXE);
//    }


//    tmp_flag = __HAL_UART_GET_FLAG(&GSM_HUART, UART_FLAG_TC);
//    tmp_it_source = __HAL_UART_GET_IT_SOURCE(&GSM_HUART, UART_IT_TC);
//    /* UART in mode Transmitter end --------------------------------------------*/
//    if((tmp_flag != RESET) && (tmp_it_source != RESET))
//    {
//        __HAL_UART_CLEAR_FLAG(&GSM_HUART,UART_FLAG_TC);
//    }




}



