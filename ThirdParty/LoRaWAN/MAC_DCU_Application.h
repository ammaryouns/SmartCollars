/**
 * @file   	MAC_DCU_Application.h
 * @author 	Hassan
 * @version	
 * @date	May 19, 2016
 * 
 * @brief   
 */

#ifndef MAC_DCU_APPLICATION_H_
#define MAC_DCU_APPLICATION_H_

#include "BSP.h"
#include "LoRa.h"
#include "MAC.h"

/**
 * @defgroup MAC_DCU_Application_Public_Structs			MAC DCU Application Public Structures
 * @{
 */



/** @}*/


extern osMutexId       hmutex_MACRadio;
extern osSemaphoreId   hsmphr_RadioIRQ;

extern osThreadId      hthread_MACBeaconTx;
extern osThreadId      hthread_MACSendQuery;
extern osThreadId      hthread_FirmwareUpdate;

#ifdef __cplusplus
extern "C"{
#endif 

void thread_MAC_Beacon      (const void* param);
void thread_MACSendQuery    (const void* param);
void thread_FirmwareUpdate  (const void* param);


#ifdef __cplusplus
}
#endif

#endif /* MAC_DCU_APPLICATION_H_ */
