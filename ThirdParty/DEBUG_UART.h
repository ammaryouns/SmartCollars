/**
  ******************************************************************************
  * @file    DEBUG_UART.h
  * @author  Umer Ilyas
  * @version V1.0.0
  * @date    03/11/2014
  * @brief   Peripheral Configurations
  ******************************************************************************
*/

#ifndef __DEBUG_UART_H__
#define __DEBUG_UART_H__

#include "bsp.h"
#include <stdio.h>


/* Exported defines -----------------------------------------------------------*/

extern int MX_USART1_Mode_halfduplex;

/* Exported macros -----------------------------------------------------------*/


//#define DEBUG_USB

#define PRINT_LOG
#define PRINT_ERROR

extern uint32_t UART_SOFT_ENABLE;
#ifdef ANN_TEST
#define PRINT_DEBUG
#endif

#ifdef PRINT_DEBUG
void printDEBUG_int(void);
void printDEBUG(const char* format, ...);

extern void printDEBUG_function(const char* format, ...);
#define printDEBUG(fmt, ...) do{if(UART_SOFT_ENABLE) printDEBUG_function(fmt, ##__VA_ARGS__);}while(0)
#define printd(fmt, ...)      do{if(UART_SOFT_ENABLE) printDEBUG_function(fmt, ##__VA_ARGS__);}while(0)

#else

#define printDEBUG_int(...)  
#define printDEBUG(...)  
#define printd(...)  
#define printDEBUG_function(...)
#endif


#ifdef ANN_TEST
#ifdef PRINT_TASK
#undef PRINT_TASK
#endif
#endif


#ifdef PRINT_TASK
#define printTask(fmt, ...)       printDEBUG("%s$ "fmt"\n", pcTaskGetTaskName(NULL), ##__VA_ARGS__);
#else
#define printTask(fmt, ...)       
#endif

#ifdef PRINT_LOG
#define printl(fmt, ...)          printDEBUG("$\t"fmt, ##__VA_ARGS__);
#else
#define printl(...)  
#endif

#ifdef PRINT_ERROR
#define printe(fmt, ...)          printDEBUG("Error:"fmt, ##__VA_ARGS__);
#else
#define printe(...)  
#endif

#endif
