/**
 * @file   	MAC_Application.h
 * @author 	Hassan
 * @version	
 * @date	May 18, 2016
 * 
 * @brief   
 */

#ifndef MAC_APPLICATION_H_
#define MAC_APPLICATION_H_

#include "bsp.h"
#include "LoRa.h"

extern TaskHandle_t xTaskRadioIRQ;
extern TaskHandle_t xTaskRadioNFC;

extern osMutexId      hmutex_MACRadio;

extern osThreadId     hthread_MACWaitForQuery;
extern osThreadId     hthread_rangeTest;

#ifdef __cplusplus
extern "C"{
#endif 

void thread_MAC_Query(const void* param);
void thread_MAC_RangeTest(const void* param);
void thread_MAC_UnitTest(const void* param);

#ifdef __cplusplus
}
#endif

#endif /* MAC_APPLICATION_H_ */
