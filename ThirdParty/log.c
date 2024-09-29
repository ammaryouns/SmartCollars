#include "log.h"
#include "DEBUG_UART.h"
#include "mRTC.h"
#include "Device_Params.h"

mQueue_List_Def( logs_Queue_List );

//#define DEBUG_DISABLE_LOG

/**
 * @brief			Save the specified error message to the queue.
 * @param source	Source of error.
 * @param error		Type of Error.
 * @param message	Additional information about error.
 * @return			#LOG_Return_t
 */
LOG_Return_t LOG_Error(LOG_ErrorSource_t source, LOG_ErrorDef_t error, uint32_t message){
    
    #ifdef DEBUG_DISABLE_LOG
    return LOG_OK;
    #else
    
    static LOG_ErrorPacket_t errorPacket;
    
    errorPacket.timeStamp       = RTC_getUnixTime();
    errorPacket.source          = source;
    errorPacket.error           = error;
    errorPacket.message.value   = message;
    
    return LOG_NodeError(0x00, &errorPacket);    
    #endif
}


/**
 * @brief			Remove specified number of error log entries from the queue.
 * @param count		Number of log entries to remove from queue.
 * @return			#LOG_Return_t
 */
LOG_Return_t LOG_clear( uint32_t count )
{
    if (mQueue_OK != mQueue_List_PopMultiple_Back(&logs_Queue_List, count))
        return LOG_ERROR;
    
    return LOG_OK;
}

/**
 * @brief		Get number of log entries currently stored in all queues in list.
 * @return		Number of log entries stored.
 */
uint32_t LOG_get_Count( void )
{
    return mQueue_List_get_elementCount(&logs_Queue_List);
}


/**
 * @brief				Save the error log entry along with the device ID.
 * @param id			ID of the device.
 * @param errorPacket	Error packet containing log entry
 * @return				#LOG_Return_t
 */
__weak LOG_Return_t LOG_NodeError(uint16_t id, LOG_ErrorPacket_t* errorPacket){

    #ifdef DEBUG_DISABLE_LOG
	return LOG_OK;
    #else
    
    static LOG_NodeErrorPacket_t nodeErrorPacket;
    
    printLogPacket("\nID : %u, Error : %u, %u, %u \n", id, errorPacket->source, errorPacket->error, errorPacket->message.value);

    
    memcpy(&nodeErrorPacket.errorPacket, errorPacket, sizeof(LOG_ErrorPacket_t));
    nodeErrorPacket.deviceID = id;
    
    if (mQueue_OK != mQueue_List_push(&logs_Queue_List, &nodeErrorPacket))
        return LOG_ERROR;
    
    return LOG_OK;
    #endif
}

/**
 * @brief					Get log entry from the queues in the list.
 *
 * @param errorPacket[out]	Log entry from the queues.
 * @return					#LOG_Return_t
 *
 * @see		LOG_readErrorAt()
 */
__inline LOG_Return_t LOG_readError(void* errorPacket)
{
    return LOG_readErrorAt(0, errorPacket);
}

/**
 * @brief					Read log entry at the specified offset in the queues list.
 *
 * @details					The function reads the error packet from the list of queues.
 * 							The type of the structure is controlled by element type
 * 							specified while creating the queues in the list.
 * 							i.e For nodes, the queues will have the element type LOG_ErrorPacket_t
 * 							and thus the return value will be of the type LOG_ErrorPacket_t
 * 							whereas for the DCU, the queues are to be created with the element type
 * 							of LOG_NodeErrorPacket_t and therefore, the output of this function
 * 							will of the type LOG_NodeErrorPacket_t.
 *
 * @param index				Offset of the entry in the record. For details about the
 * 							offset, see #mQueue_List_peekAt_Back()
 * @param errorPacket[out]	Log entry at the specified offset.
 * @return					#LOG_Return_t
 */
LOG_Return_t LOG_readErrorAt(uint32_t index, void* errorPacket)
{
    if(mQueue_OK != mQueue_List_peekAt_Back(&logs_Queue_List, index, (uint8_t*)errorPacket))
        return LOG_ERROR;

    return LOG_OK;

}




