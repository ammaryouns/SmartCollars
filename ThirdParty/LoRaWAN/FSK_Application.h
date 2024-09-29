/**
 * @file   	FSK_Application.h
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Jan 24, 2017
 * 
 * @brief   
 */

#ifndef FSK_APPLICATION_H_
#define FSK_APPLICATION_H_

#include "MAC.h"


extern osThreadId   hthread_FSK_Node;
extern osThreadId   hthread_FSK_Gateway;

#ifdef __cplusplus
extern "C"{
#endif 

void thread_FSK_Node    (const void* param);
void thread_FSK_Gateway (const void* param);

#ifdef __cplusplus
}
#endif

#endif /* FSK_APPLICATION_H_ */
