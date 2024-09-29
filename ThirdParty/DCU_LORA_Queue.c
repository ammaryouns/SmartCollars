/**
 * @file   	SensorDefs.c
 * @author 	Hassan
 * @version	
 * @date	Mar 23, 2016
 * 
 * @brief   
 */

#include <DCU_LORA_Queue.h>
#include "DEBUG_UART.h"
#include "Device_Params.h"
#include "IPC_Defs.h"
#include "IPC_Master.h"

extern osThreadId      hthread_InitDrivers;

#define NODE_PACKET_QUEUE_SIZE          100
#define ERROR_LOG_QUEUE_SIZE            500


const mQueue_Interface_t NodeDataQueue_ifc = 
{

    .init           = mQueue_init,
	.mem_read	    = mQueue_memcpy,
	.mem_write      = mQueue_memcpy,
	.pullStruct     = mQueue_pullStruct,
	.commitStruct	= NodeDataQueue_commitStruct,
    .get_Count      = mQueue_get_Count,
    .get_Size       = mQueue_get_Size
};



mQueue_Def( NodeDataQueue, SensorsData_t, NODE_PACKET_QUEUE_SIZE, NodeDataQueue_ifc);

mQueue_Def( ErrorLogQueue, LOG_NodeErrorPacket_t, ERROR_LOG_QUEUE_SIZE, NodeDataQueue_ifc);
///@todo create EEPROM Queue


__inline mQueueReturn_t NodeDataQueue_commitStruct(mQueueStruct_t* q)
{
    osSignalSet(hthread_IPC_Master,  IPC_EVENT_WRITE_REQUEST);
    return mQueue_OK;
}


