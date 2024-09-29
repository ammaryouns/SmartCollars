/**
 * @file   	MAC_Node_FirmwareApp.h
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Feb 7, 2017
 * 
 * @brief   
 */

#ifndef MAC_NODE_FIRMWAREAPP_H_
#define MAC_NODE_FIRMWAREAPP_H_

#include "MAC.h"

typedef struct FwUpdate_s FwUpdate_t;
extern FwUpdate_t FwUpdate;

extern osThreadId     hthread_FirmwareUpdate;


#ifdef __cplusplus
extern "C"{
#endif 

MAC_Return_t MAC_request_FwUpdate         (FwUpdate_t* fwUpdate);
MAC_Return_t MAC_Firmware_Update_Callback (MAC_Firmware_Packet_t* fwPacket, uint32_t expectedCommands);
bool         MAC_isFirmwareUpdateSlot     (uint32_t slot);
    
void thread_FirmwareUpdate(const void* param);

#ifdef __cplusplus
}
#endif

#endif /* MAC_NODE_FIRMWAREAPP_H_ */
