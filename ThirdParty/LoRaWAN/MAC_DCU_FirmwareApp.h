/**
 * @file   	MAC_DCU_FirmwareApp.h
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Feb 6, 2017
 * 
 * @brief   
 */

#ifndef MAC_DCU_FIRMWAREAPP_H_
#define MAC_DCU_FIRMWAREAPP_H_

#include "MAC.h"


#ifdef __cplusplus
extern "C"{
#endif 

MAC_Return_t MAC_Firmware_Update_Callback(MAC_Node_t* node, MAC_Firmware_Packet_t* fwPacket, uint32_t expectedCommands);

bool MAC_isFirmwareUpdateSlot(uint32_t slot);    

#ifdef __cplusplus
}
#endif

#endif /* MAC_DCU_FIRMWAREAPP_H_ */
