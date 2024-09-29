/**
  ******************************************************************************
  * @file    DEBUG_UART.c
  * @author  Umer Ilyas
  * @version V1.0.0
  * @date    03/11/2014
  * @brief   Peripheral Configurations
  ******************************************************************************
*/

//-----------Copied from GPS-tracker project.

#include "DEBUG_UART.h"
#include "bsp.h"
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#ifdef DEBUG_USB
#include <usb_device.h>
#include <usbd_cdc_if.h>
#endif



/* Private functions  --------------------------------------------------------*/

#ifdef PRINT_DEBUG
//int fputc(int ch, FILE * p_file) 
//{   
//    HAL_UART_Transmit(&DEBUG_huart, (uint8_t *)&ch, 1, 1);
//    return 0;
//}
 int MX_USART1_Mode_halfduplex = 0;

uint32_t UART_SOFT_ENABLE = 1;

#define uartBufferRxSize 254
uint8_t uartBufferRx[uartBufferRxSize];
uint32_t uartBufferRxCount =0;

void printDEBUG_function(const char* format, ...){

	#ifdef DEBUG_USB
	if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED){
		static char buff[512];
		va_list args;
		va_start(args, format);
		vsnprintf(buff, sizeof(buff) - 1,  format, args);
		CDC_Transmit_FS((uint8_t*)buff, strlen(buff));
		va_end(args);
	}
	#elif defined HAL_UART_MODULE_ENABLED
	
        if (MX_USART1_Mode_halfduplex)
        {
            printDEBUG_int();
        }
//        if(HAL_UART_GetState(&DEBUG_huart) != HAL_UART_STATE_READY &&
//            HAL_UART_GetState(&DEBUG_huart) != HAL_UART_STATE_BUSY_RX)
//        {
//            DEBUG_Uart_init();    HAL_UART_Receive_IT(&DEBUG_huart, (uint8_t*)uartBufferRx, 1);
//        }
        static char buff[512];
		va_list args;
		va_start(args, format);
		vsnprintf(buff,sizeof(buff) - 1, format, args);
        HAL_UART_Transmit(&DEBUG_huart, (uint8_t*)buff, strlen(buff), 1000); /// @todo: make it work with DMA and able to work with temprature sensor.
        va_end(args);
	#endif
	
	
}
void printDEBUG_int(void)
{
    DEBUG_Uart_init();
}

#endif




