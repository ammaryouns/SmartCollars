/**
 * @file   	bsp.h
 * @author 	Hassan Abid
 * @version	v7.0.0
 * @date	Oct 25, 2016
 * 
 * @brief   Board Support Package File for Cowlar Node v7 Hardware.
 */

#ifndef BSP_H_
#define BSP_H_
#include "Cowlar-node-v8.h"
#include "stm32l4xx.h"

#include "cmsis_os.h"


#include <stdbool.h>
#include <stddef.h>

#include "Device_Params.h"
#include "usart.h"
#include "DEBUG_UART.h"

#include "gpio.h"
#include "spi.h"

#define DEBUG_huart                     hlpuart1
#define DEBUG_Uart_init                 MX_LPUART1_UART_Init


#define SX126x_SPI_Handle       hspi2
#define DelayMs( ms )       osDelay(ms)

#define BoardDisableIrq()
#define BoardEnableIrq()


#define sizeof_array( array )           (sizeof(array) / sizeof(array[0]))
#define sizeof_member(type, member)     (sizeof( ((type*)0)->member ))

#define HAL_GPIO_SetPin( NAME )			    HAL_GPIO_WritePin( NAME##_GPIO_Port, NAME##_Pin, GPIO_PIN_SET)
#define HAL_GPIO_ResetPin( NAME )		    HAL_GPIO_WritePin( NAME##_GPIO_Port, NAME##_Pin, GPIO_PIN_RESET)

#define HAL_GPIO_getState( NAME )       HAL_GPIO_ReadPin( NAME##_GPIO_Port, NAME##_Pin)

extern __INLINE uint32_t min_ui32(uint32_t a, uint32_t b)
{
    return a < b ? a : b;
}
extern __INLINE uint32_t max_ui32(uint32_t a, uint32_t b)
{
    return a > b ? a : b;
}


//#define _Error_Handler(...)


/** @}*/

#endif /* BSP_H_ */
