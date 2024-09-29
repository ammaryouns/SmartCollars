/**
  ******************************************************************************
  * @file    GSM_UART.h
  * @author  E4 R&D Tean
  * @version V1.0.0
  * @date    03/11/2014
  * @brief   Peripheral Configurations
  ******************************************************************************
*/

#ifndef __GSM_UART_H__
#define __GSM_UART_H__

#include "BSP.h"
#include "usart.h"

/* Exported defines -----------------------------------------------------------*/

#define SIMBUFFERSIZE			          512

#define SIMBufferRead               ((SIMBufferIndex + SIMBUFFERSIZE) - SIMBufferCount)%SIMBUFFERSIZE

#define GSM_UART_SendChars( string )        HAL_UART_Transmit(&GSM_HUART, (uint8_t*)string, strlen((char *)string), 1000);
#define GSM_UART_SendnChars(string, len)    HAL_UART_Transmit(&GSM_HUART, (uint8_t*)string, len, 1000);
#define GSM_UART_SendChar( ch )             HAL_UART_Transmit(&GSM_HUART, (uint8_t*)&ch, 1, 10);



extern uint8_t  SIMBuffer[SIMBUFFERSIZE];		// Source: ..._it.c __ Use: (Unused)
extern uint16_t SIMBufferIndex;  				              // Meter buffer of Write index
extern uint16_t SIMBufferCount;				                // Meter buffer of size
extern uint8_t  SIMBufferNewCommand;
extern int8_t   GSMUARTtimerIndex;
extern uint8_t  GSMreceiveComplete;
extern uint8_t  GSMFlagByteRecieved;

/* Exported macros -----------------------------------------------------------*/


#define   printGSM                  P_UART = GSM_UART;\
                                    printf
extern uint8_t GSMLastRecievedByte;
/* Exported functions -----------------------------------------------------------*/

void     GSM_UART_Configuration (void);


uint8_t  SIM_ReceiveByte        (void);

uint32_t GSMgetByte             (uint8_t * byte);
void     GSMSendByte            (uint8_t byte);
void     GSM_UART_IRQHandler    (void);
void     GSM_DMA_IRQHandler     (void);
#endif
