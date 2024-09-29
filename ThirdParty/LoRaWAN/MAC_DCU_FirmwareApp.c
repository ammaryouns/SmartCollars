/**
 * @file   	MAC_DCU_FirmwareApp.c
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Feb 6, 2017
 * 
 * @brief   
 */

#include "MAC.h"
#include "crc32.h"
#include "mIWDG.h"
#include "mRTC.h"
#include "LoRa.h"

#include "DEBUG_UART.h"

#define FW_UPDATE_TX_SIZE            255
#define FW_UPDATE_PAGE_TIME          30
#define FW_UPDATE_MAX_PAGE_RETRIES   3

#define FwUpdate_GetPage( offset )          (offset / FW_UPDATE_PAGE_SIZE)




/**
 * @defgroup MAC_DCU_FirmwareApp_Private_Members			MAC DCU Firmware Application Private Members
 * @{
 */

osThreadId     hthread_FirmwareUpdate;
extern osThreadId      hthread_MACSendQuery;
extern osMutexId       hmutex_MACRadio;

uint8_t fwPage[FW_UPDATE_PAGE_SIZE];


/**
 * @brief	Firmware update status and information struct.
 */
typedef struct{

    Firmware_Header_t fwHeader;
	int32_t  offset;			/*!< current offset in firmware download*/
	int32_t  bytesLeftInPage;	/*!< number of bytes left in page*/
	int32_t  pageRetries;       /*!< Maximum transmission retries for a single page.*/
    uint32_t currentPage;       /*!< Current page of firmware being transmitted. */
    uint32_t lastRxTick;		/*!< tick of last firmware update package received*/
	uint32_t pageCRC;			/*!< accumulated CRC of firmware upto the page*/


	enum{

		FW_UPDATE_IN_PROGRESS = 0,
		FW_UPDATE_PAUSED,
		FW_UPDATE_WAIT_FOR_START,
		FW_UPDATE_STOPPED,
		FW_UPDATE_COMPLETE

	}state;						/*!< Current state of firmware update.*/

}FwUpdate_t;



FwUpdate_t FwUpdate =
{
    .fwHeader = {0},
    .offset  = 0,
    .bytesLeftInPage = 0,
    .lastRxTick = 0,
    .pageCRC = CRC_SEED,
    .state   = FW_UPDATE_STOPPED
};
/** @}*/


/**
 * @defgroup MAC_DCU_FirmwareApp_Private_Functions				MAC DCU Firmware Application Private Functions
 * @{
 */

/**
 * @brief			Reset the firmware update params to start of firmware file.
 * @param fwUpdate	Firmware update status struct
 */
void resetFwUpdate(FwUpdate_t* fwUpdate){

    memset(fwUpdate, 0, sizeof(FwUpdate_t));
    fwUpdate->state = FW_UPDATE_STOPPED;
    fwUpdate->pageCRC = CRC_SEED;

}

/**
 *
 * @param fwPacket
 * @return
 */
uint8_t getFwCommandSize(MAC_Firmware_Packet_t* fwPacket)
{
	switch(fwPacket->header.type)
	{
	case MAC_FW_CMD_START:
		return sizeof(fwPacket->fwStart);

	case MAC_FW_CMD_UPDATE_REQUEST:
		return sizeof(fwPacket->fwRequest);

	case MAC_FW_CMD_RESEND_LAST_FW_PAGE:
		return sizeof(fwPacket->fwResendRequest);
    
    default:
        return 0;
	}

}


/**
 * @brief
 * @param node
 * @param fwPacket
 * @return
 */
__STATIC_INLINE MAC_Return_t MAC_sendFwCommand(MAC_Node_t* node, uint8_t controlFlags, MAC_Firmware_Packet_t* fwPacket)
{
	return MAC_sendData(node != NULL ? node->addr : 0,
						MAC_PORT_FIRMWARE_UPDATE,
						controlFlags,
						(uint8_t*)fwPacket,
						getFwCommandSize(fwPacket) + sizeof(MAC_Firmware_Packet_Header_t));
}



MAC_Return_t MAC_send_FwStartCommand(FwUpdate_t* fwUpdate)
{
	static MAC_Firmware_Packet_t fwCmd;

	fwCmd.header.type = MAC_FW_CMD_START;

    fwCmd.fwStart.fileCRC    = FwUpdate.fwHeader.crc;
    fwCmd.fwStart.pageCRC    = FwUpdate.pageCRC;
    fwCmd.fwStart.version    = FwUpdate.fwHeader.version;
    fwCmd.fwStart.fwSize     = FwUpdate.fwHeader.size;
    fwCmd.fwStart.offset	 = FwUpdate.offset;

    return MAC_sendFwCommand(NULL, MAC_CNTRL_ACKNOWLEDGE, &fwCmd);
}

/**
 *
 * @param offset
 * @param data
 * @param length
 * @return
 */
MAC_Return_t MAC_sendFwData(uint8_t controlFlags, uint32_t offset, uint8_t* data, uint8_t length)
{
	static MAC_Firmware_Packet_t fwDataPacket;

	fwDataPacket.header.type = MAC_FW_DATA;

	if (length > sizeof(fwDataPacket.fwData.data))
		return MAC_ERROR;

	memcpy(fwDataPacket.fwData.data, data, length);

	fwDataPacket.fwData.offset = offset;

	return MAC_sendData(0, MAC_PORT_FIRMWARE_UPDATE, controlFlags, (uint8_t*)&fwDataPacket, length + offsetof(MAC_Firmware_Packet_t, fwData.data));
}

/**
 * @brief	    Callback for Firmware request command packet.
 * @param node
 * @param cmd
 * @return	    #MAC_Return_t
 */
MAC_Return_t MAC_FW_CMD_Update_Request_Callback(MAC_Node_t* node, MAC_Firmware_Packet_t* fwPacket)
{
    static MAC_Firmware_Packet_t reply;

    printMAC("Fw update request received");

    //initialize reply type.
    //This is returned incase of any errors.
    reply.header.type = MAC_FW_CMD_UPDATE_REQUEST_FAILED;
    if (mFile_OK != FwFile_getHeader(&fwFile_Node, &FwUpdate.fwHeader))
    {
        goto error;
    }

    printMAC("App ID : %u, version : %u", NODE_TYPE, FwUpdate.fwHeader.version );
    printMAC(", Offset : %d", fwPacket->fwRequest.offset);

    //if the firmware update offset variable has become corrupted,
    //use the offset requested by this node.
    if (FwUpdate.offset > FwUpdate.fwHeader.size)
    {
        FwUpdate.offset = fwPacket->fwRequest.offset;
    }

    if (fwPacket->fwRequest.offset == 0)
    {
        FwUpdate.pageCRC = 0;
        fwPacket->fwRequest.fileCRC = FwUpdate.fwHeader.crc;
    }
    else
    {
//            //read corresponding page CRC.
        FwUpdate.pageCRC = FwFile_computeCRC(&fwFile_Node, 4, fwPacket->fwRequest.offset - 4);

        //The compute CRC function returns the CRC_SEED constant if there was an error
        //getting the CRC.
        if (FwUpdate.pageCRC == CRC_SEED)
        {
            goto error;
        }
    }

    if ((FwUpdate.fwHeader.crc      == fwPacket->fwRequest.fileCRC) &&
        (FwUpdate.pageCRC           == fwPacket->fwRequest.pageCRC) &&
        (FwUpdate.fwHeader.version  == fwPacket->fwRequest.version))
    {
        printMAC("Fw CRC match, resuming firmware download");
        if ((FwUpdate.state == FW_UPDATE_STOPPED) ||
            (FwUpdate.offset > fwPacket->fwRequest.offset))
        {
            FwUpdate.offset = fwPacket->fwRequest.offset;
        }
        reply.header.type = MAC_FW_CMD_AWAIT_FW_UPDATE;
    }
    else
    {

        printMAC("Fw CRC failed, restarting firmware download");
        FwUpdate.offset = 0;
        reply.header.type = MAC_FW_CMD_RESTART_FW_UPDATE;
    }

    //The both conditionals above, the reply is to wait for firmware update,
    //whether it is to resume or to restart.
    FwUpdate.state  = FW_UPDATE_WAIT_FOR_START;

    //Jumps to here incase there is an error, in which case the
    //DCU responds with request denied.
    error:
    return MAC_sendFwCommand(node, MAC_CNTRL_ACKNOWLEDGE,  &reply);
}


/**
 *
 * @param node
 * @param fwPacket
 * @return
 */
MAC_Return_t MAC_FW_CMD_Resend_Last_Page_Callback(MAC_Node_t* node, MAC_Firmware_Packet_t* fwPacket)
{

	printMAC("NACK received, offset received : %4X, current Offset : %4X" , fwPacket->fwResendRequest.offset, FwUpdate.offset);
	if ((FwUpdate.offset - fwPacket->fwResendRequest.offset) <= (FW_UPDATE_PAGE_SIZE))
	{       //if the node is too far behind, do not retransmit page.
		if (FwUpdate.pageRetries > 0)
		{
			FwUpdate.pageRetries--;
			//revert offset to start of page.
			FwUpdate.offset = FwUpdate_GetPage(fwPacket->fwResendRequest.offset)*FW_UPDATE_PAGE_SIZE;
			printMAC(", Offset reverted to : %4X", FwUpdate.offset);
		}
		else
		{
			FwUpdate.pageRetries = 0;
			printMAC(", Page retransmissions maxed out.");
		}
	}
	else
		printMAC(", Received Offset too far behind");

	return MAC_OK;
}

/** @}*/


/**
 * @defgroup MAC_DCU_FirmwareApp_Public_Functions				MAC DCU Firmware Application Public Functions
 * @{
 */

/**
 *
 * @param slot
 * @return
 */
bool MAC_isFirmwareUpdateSlot(uint32_t slot)
{
    return ((FwUpdate.state == FW_UPDATE_WAIT_FOR_START || 
             FwUpdate.state == FW_UPDATE_IN_PROGRESS) &&
             slot >= (MAC_TIME_SLOTS_MAX / 2));
}


/**
 *
 * @param node
 * @param expectedCommands
 * @return
 */
MAC_Return_t MAC_Firmware_Update_Callback(MAC_Node_t* node, MAC_Firmware_Packet_t* fwPacket, uint32_t expectedCommands)
{
	MAC_Return_t ret = MAC_ERROR;

	///@todo figure out a way to specify expected firmware packet and return error if
	/// type does not match

	switch (fwPacket->header.type)
	{

	case MAC_FW_CMD_UPDATE_REQUEST:
		ret = MAC_FW_CMD_Update_Request_Callback(node, fwPacket);
		break;


	case MAC_FW_CMD_RESEND_LAST_FW_PAGE:
		ret = MAC_FW_CMD_Resend_Last_Page_Callback(node, fwPacket);
		break;

	default :
		return MAC_ERROR;
	}

	return ret;
}



void thread_FirmwareUpdate(const void* param)
{
    static int timeLeftInSlot = 0;
    
    uint32_t length;
    uint32_t ticks = 0;
    int32_t pageLoadErrorRetries = 5;
    static uint8_t probabilityTx = 100;
    
    mIWDG_HandleTypeDef* hmIWDG_firmwareUpdate;
    
    
    MAC_Return_t ret;
    mFile_Return_t file_ret = mFile_ERROR;
    
    bool NACKreceived = false;
    
    hmIWDG_firmwareUpdate = IWDG_initWDG((MAC_SLOT_DURATION + 5)*1000);
    IWDG_startWDG(hmIWDG_firmwareUpdate);
    
    srand(RTC_getUnixTime());
    
    
    for(;;)
    {
        
        timeLeftInSlot = RTC_secondsTillMark( MAC_REQUERY_TIME / 2 );   //wait for the firmware update slot to begin.
        
        //wait for the firmware update slot to begin.
        //if already in the update slot, wait until the beginning of the next slot.
        //TODO : stop firmware update here, so that the node can request for firmware update
        //again.
        printMAC("Waiting till next Fw Slot mark");
        RTC_waitTillMark( MAC_REQUERY_TIME / 2, MAC_SLOT_DURATION, hmIWDG_firmwareUpdate);
        
        if (RTC_secondsTillMark(MAC_REQUERY_TIME / 2) != RTC_secondsTillMark(MAC_REQUERY_TIME))
        {
            printMAC("Start of Query cycle.");
            continue;
        }
        
//        //if not waiting for firmware update, go to sleep again.
        if (FwUpdate.state != FW_UPDATE_WAIT_FOR_START){
            printMAC("No Fw update in progress");
            continue;
        }

        osMutexWait(hmutex_MACRadio, osWaitForever);
        
        
        
        //send firmware update initial command
        //TODO : replace this with proper functions, once the MAC code has been cleaned up.
        printMAC("Transmitting Firmware Update Start command.");
        
        
        
        //read firmware header parameters
        file_ret = FwFile_getHeader(&fwFile_Node, &FwUpdate.fwHeader);
        if (file_ret != mFile_OK)
        {
            FwUpdate.state = FW_UPDATE_STOPPED;
            continue;
        }
        
        LoRa_setGain(MAC_RADIO_CONFIG_FW_CMD(deviceSettings.DCU_LoRa_Params.networkID));
        MAC_Port_open( MAC_PORT_FIRMWARE_UPDATE );
        MAC_Port_open( MAC_PORT_NONE );
        MAC_send_FwStartCommand(&FwUpdate);
        MAC_transmitFrame(MAC_RX1_DELAY);
        //wait for ACK. All nodes would be transmitting an ACK. If atleast one ACK is received,
        //continue with the firmware update. Otherwise, assume that no nodes are listening and abort.
        ret = MAC_WaitForPacket(MAC_RX1_WINDOW + MAC_RX1_DELAY, true);
        if (ret == MAC_OK || ret == MAC_ERROR)
        {       //Failed means CRC error. Implies that some nodes did respond, and they are listening.
            
            FwUpdate.state = FW_UPDATE_IN_PROGRESS;
            printMAC("ACK received, Starting Firmware Update Broadcast.");
            
            //Disable Query Thread      //TODO : suspend query thread in itself and launch the firmware update thread from there.
            osThreadSuspend(hthread_MACSendQuery);
            
        }
        else if (ret == MAC_TIMEOUT)
        {   //no response
            
            //abort firmware update.
            
            printMAC("No response received for Update start command");
            osMutexRelease(hmutex_MACRadio);
            MAC_Port_closeAll();
            FwUpdate.state = FW_UPDATE_STOPPED;
            LoRa_sleepMode();
            continue;
        }
        
        osDelay(1000);
        FwUpdate.currentPage = FwUpdate_GetPage( FwUpdate.offset ) - 1;
        FwUpdate.pageRetries = FW_UPDATE_MAX_PAGE_RETRIES;
        
        
        
        while (FwUpdate.offset < FwUpdate.fwHeader.size && 
               FwUpdate.state == FW_UPDATE_IN_PROGRESS)
        {
            
            IWDG_refreshWDG(hmIWDG_firmwareUpdate);
            
            timeLeftInSlot = RTC_milliSecondsTillMark(FW_UPDATE_PAGE_TIME);

            
            FwUpdate.bytesLeftInPage =  FwUpdate.offset % FW_UPDATE_PAGE_SIZE;
            FwUpdate.bytesLeftInPage = FW_UPDATE_PAGE_SIZE - FwUpdate.bytesLeftInPage;
            
            if (FwUpdate.bytesLeftInPage > (FwUpdate.fwHeader.size - FwUpdate.offset))
                FwUpdate.bytesLeftInPage = FwUpdate.fwHeader.size - FwUpdate.offset;
            
            //If transmitting a new page, load the page from firmware file into RAM buffer first.
            if (FwUpdate.currentPage != FwUpdate_GetPage( FwUpdate.offset ))
            {

                if (mFile_OK != FwFile_read(&fwFile_Node, FwUpdate.offset, fwPage, sizeof(fwPage)))
                {
                    printMAC("Read from Firmware File failed.");
                    pageLoadErrorRetries--;
                    osDelay(1*1000);
                    if (pageLoadErrorRetries > 0)
                        continue;                
                    else
                        break;
                }
                pageLoadErrorRetries = 5;
                
                FwUpdate.currentPage = FwUpdate_GetPage( FwUpdate.offset );
                FwUpdate.pageRetries = FW_UPDATE_MAX_PAGE_RETRIES;
            }
                        
            
            
            //transmit at max 3*1024 bytes(see definition of FW_UPDATE_PAGE_SIZE) in less than 20 seconds. 
            //next ten seconds are to listen for any re transmit requests.
            
            LoRa_setGain(MAC_RADIO_CONFIG_FW_DATA(deviceSettings.DCU_LoRa_Params.networkID));
            while (FwUpdate.bytesLeftInPage > 0 && timeLeftInSlot > (10*1000)) 
            {           //should have atleast ten seconds left for requeries. 
                NACKreceived = false;
                
                ticks = osKernelSysTick();          //RTC or systick ????

                length = min_ui32( FwUpdate.bytesLeftInPage, sizeof_member(MAC_Firmware_Packet_t, fwData.data));
                
                if (length % 8  != 0)
                  length = (length >> 3) << 3;        //length must be a multiple of 8.
                

                
                if ((timeLeftInSlot >= 13*1000) && FwUpdate.bytesLeftInPage > length)
                    ret = MAC_sendFwData(   MAC_CNTRL_MORE_DATA,
                                            FwUpdate.offset, 
                                            fwPage + (FwUpdate.offset % FW_UPDATE_PAGE_SIZE), 
                                            length);
                else
                    ret = MAC_sendFwData( 0,   
                                            FwUpdate.offset, 
                                            fwPage + (FwUpdate.offset % FW_UPDATE_PAGE_SIZE), 
                                            length);
                
                if (ret == MAC_OK)
                {    
                    printMAC("offset:%4X, length:%d,", FwUpdate.offset, length);
                                   
                    probabilityTx = 100;
                    if (MAC_transmitFrame(MAC_RX1_DELAY) == MAC_OK && probabilityTx > 10)
                    {       //Emulate channel noise. 10% packet loss.
                    
                        FwUpdate.offset += length;           //TODO : datasheet does not mention write time to flash.
                        FwUpdate.bytesLeftInPage -= length;            //add appropriate delay after determining flash write time.
                        
                        printMAC(" Transmitted");
                        
                        if (!(MAC_txHeader->frameHeader.controlFlags & MAC_CNTRL_MORE_DATA)){        //Dirty redundant code. Improve code structure later.
                            timeLeftInSlot -= osKernelSysTick() - ticks;    
                            break;
                        }
                        
                    }else
                        printMAC(" Tx Failed");
 
                }
                osDelay(1000);
                timeLeftInSlot -= osKernelSysTick() - ticks;
                
            }
            
            timeLeftInSlot = RTC_milliSecondsTillMark(FW_UPDATE_PAGE_TIME);
            
            
            
            LoRa_setGain(MAC_RADIO_CONFIG_FW_CMD(deviceSettings.DCU_LoRa_Params.networkID));
            while (timeLeftInSlot > 0){
                
                ticks = osKernelSysTick();
                
                if (!NACKreceived){
                 
                    ret = MAC_WaitForPacket(timeLeftInSlot, false);
                    probabilityTx = 100;
                    if (ret == MAC_OK && probabilityTx > 10)
                    {          //10% received packet loss.
                        ret = MAC_Firmware_Update_Callback(NULL, (MAC_Firmware_Packet_t*)MAC_rxPayload, 0);
                        NACKreceived = true;
                    }
                }
                else
                    osDelay(timeLeftInSlot);
                
                timeLeftInSlot -= osKernelSysTick() - ticks;
            }
        }   
        
        LoRa_sleepMode();
        resetFwUpdate(&FwUpdate);
        osMutexRelease(hmutex_MACRadio);
        MAC_Port_closeAll();
        osThreadResume(hthread_MACSendQuery);
       
    }
    
    
}



/** @}*/
