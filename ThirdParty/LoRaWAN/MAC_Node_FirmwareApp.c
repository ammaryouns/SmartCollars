/**
 * @file   	MAC_Node_FirmwareApp.c
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Feb 7, 2017
 * 
 * @brief   
 */

#include "MAC.h"
#include "crc32.h"
#include "FwFile.h"
#include "mIWDG.h"
#include "mRTC.h"
#include "LoRa.h"
#include "DEBUG_UART.h"

#define FW_UPDATE_TX_SIZE            255
#define FW_UPDATE_PAGE_TIME          30
#define FW_UPDATE_PAGE_NACK_TIME     6
#define FW_UPDATE_MAX_PAGE_RETRIES   3
#define FW_UPDATE_PAGE_SIZE                 (FLASH_PAGE_SIZE)


#define FW_UPDATE_SLOT_NUM(unixTime)        (( unixTime % (MAC_REQUERY_TIME / 2) ) / FW_UPDATE_PAGE_TIME)

#define FwUpdate_GetPage( offset )          (offset / FW_UPDATE_PAGE_SIZE)


/**
 * @defgroup MAC_FirmwareApp_Private_Members				MAC_Firmware Application Private Members
 * @{
 */

osThreadId     hthread_FirmwareUpdate;

extern osThreadId     hthread_MACWaitForQuery;
extern mIWDG_HandleTypeDef* hmIWDG_MACWaitForQuery;
extern osMutexId       hmutex_MACRadio;

/**
 * @brief	Firmware update status and information struct.
 */
typedef struct FwUpdate_s{

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
 * @brief			Reset the firmware update params to start of firmware file.
 * @param fwUpdate	Firmware update status struct
 */
void resetFwUpdate(FwUpdate_t* fwUpdate){

    memset(fwUpdate, 0, sizeof(FwUpdate_t));
    fwUpdate->state = FW_UPDATE_STOPPED;
    fwUpdate->pageCRC = CRC_SEED;

}

/**
 * @brief			Get length in bytes of the firmware command packet specified.
 * @param fwPacket	Firmware packet containing the command.
 * @return			Length of command in the firmware packet.
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
 * @brief			Pause the firmware update.
 * @param fwUpdate	Firmware update status struct
 */
void pauseFwUpdate(FwUpdate_t* fwUpdate)
{
    //       reset offset to start of current page.
    //       calculate page CRC uptill the last page.
    //       change fw update state to fw update paused.


    FwUpdate.offset  = FwUpdate_GetPage(FwUpdate.offset)*FW_UPDATE_PAGE_SIZE;

    if (FwUpdate.offset == 0)
    {
        resetFwUpdate(&FwUpdate);
    }
    else
    {
        FwUpdate.state   = FW_UPDATE_PAUSED;
        FwUpdate.pageCRC = FwFile_computeCRC(&fwFile, 4, FwUpdate.offset - 4);
    }
    printMAC("\nFirmware Download Paused.\n");
    printMAC("Fw File offset           : %04X\n", FwUpdate.offset);
    printMAC("CRC @ offset             : %04X\n", FwUpdate.pageCRC);
}


/**
 * @brief
 * @param node
 * @param fwPacket
 * @return
 */
__STATIC_INLINE MAC_Return_t MAC_sendFwCommand(uint8_t addr, uint8_t controlFlags, MAC_Firmware_Packet_t* fwPacket)
{
	return MAC_sendData(addr,
						MAC_PORT_FIRMWARE_UPDATE,
						controlFlags,
						(uint8_t*)fwPacket,
						getFwCommandSize(fwPacket) + sizeof(MAC_Firmware_Packet_Header_t));
}

/**
 * @brief			Send a request to DCU for firmware update.
 * @param fwUpdate	Firmware update status struct.
 * @return			#MAC_Return_t
 */
MAC_Return_t MAC_request_FwUpdate(FwUpdate_t* fwUpdate)
{
    MAC_Firmware_Packet_t fwCmd;

    fwCmd.header.type = MAC_FW_CMD_UPDATE_REQUEST;

    if (deviceSettings.fwVersion <= NODE_FW_VERSION ||
    	fwUpdate->state == FW_UPDATE_WAIT_FOR_START)
    {
    	return MAC_ERROR;
    }

    ///Firmware update is reset, if a firmware update is not already pending
    ///or the firmware version sent by the DCU is different
    ///than the one being downloaded.
    if (FwUpdate.state != FW_UPDATE_PAUSED ||
        FwUpdate.fwHeader.version != deviceSettings.fwVersion)
    {
        printMAC("\nSending request for new firmware.\n");
        resetFwUpdate(fwUpdate);
        FwUpdate.fwHeader.version = deviceSettings.fwVersion;
    }
    else
    {
        printMAC("\nRequesting to resume firmware download\n");
    }

    fwCmd.fwRequest.offset  = fwUpdate->offset;
    fwCmd.fwRequest.fwSize  = fwUpdate->fwHeader.size;
    fwCmd.fwRequest.fileCRC = fwUpdate->fwHeader.crc;
    fwCmd.fwRequest.pageCRC = fwUpdate->pageCRC;
    fwCmd.fwRequest.version = fwUpdate->fwHeader.version;

    return MAC_sendFwCommand(deviceSettings_crc.DcuID,
    					     MAC_CNTRL_ACKNOWLEDGE | MAC_CNTRL_ACKNOWLEDGED,
							 &fwCmd);
}

/**
 *
 * @param fwUpdate
 * @return
 */
MAC_Return_t MAC_FW_requestLastPage(FwUpdate_t* fwUpdate)
{
	static MAC_Firmware_Packet_t fwCmd;

	fwUpdate->offset = FwUpdate_GetPage(fwUpdate->offset) * FW_UPDATE_PAGE_SIZE;
    fwCmd.header.type = MAC_FW_CMD_RESEND_LAST_FW_PAGE;
    fwCmd.fwResendRequest.offset = fwUpdate->offset;

    printMAC("Sending NACK @ page offset %04X", FwUpdate.offset);
    return MAC_sendFwCommand(deviceSettings_crc.DcuID, 0, &fwCmd);
}


/**
 * @brief
 * @param header
 * @param fwCmd
 * @return
 */
MAC_Return_t MAC_FW_requestResponse_Callback(MAC_Firmware_Packet_t* fwCmd)
{
	uint8_t controlFlags = MAC_CNTRL_ACKNOWLEDGED;
	MAC_Return_t ret = MAC_ERROR;

	if (Sensors_get_PacketCount() > 0 || LOG_get_Count() > 0)
	{
		controlFlags = MAC_CNTRL_ACKNOWLEDGE | MAC_CNTRL_MORE_DATA;
	}

	ret = MAC_sendACK(deviceSettings_crc.DcuID, controlFlags);
	MAC_ASSERT_RETURN(ret, MAC_OK);

	return MAC_transmitFrame(MAC_RX1_DELAY);
}

/**
 *
 * @param fwCmd
 * @return
 */
MAC_Return_t MAC_FW_CMD_Start_Callback(MAC_Firmware_Packet_t* fwCmd)
{
	MAC_Return_t ret = MAC_ERROR;
	bool restartFwUpdate = false;

	//If the update is resuming from an offset
    if (FwUpdate.offset != 0)
    {

        if ((FwUpdate.fwHeader.version != fwCmd->fwStart.version) ||
            (FwUpdate.fwHeader.size    != fwCmd->fwStart.fwSize)  ||
            (FwUpdate.fwHeader.crc	   != fwCmd->fwStart.fileCRC))
        {
            restartFwUpdate = true;
        }
        else if (FwUpdate.offset < fwCmd->fwStart.offset)
        {
            pauseFwUpdate(&FwUpdate);
            return MAC_OK;
        }
        else if (fwCmd->fwStart.offset  != 0)
        {
            if (FwFile_computeCRC(&fwFile, 4, fwCmd->fwStart.offset - 4)
                != fwCmd->fwStart.pageCRC)
            {
                restartFwUpdate = true;
            }

        }

    }
    else restartFwUpdate = true;

    if (restartFwUpdate)
    {
        resetFwUpdate(&FwUpdate);

        if (fwCmd->fwStart.offset != 0)
        {
            printMAC("\nFirmware Update status reset. Cannot continue.\n");
            return MAC_ERROR;
        }

        FwUpdate.fwHeader.crc 		= fwCmd->fwStart.fileCRC;
        FwUpdate.fwHeader.size    	= fwCmd->fwStart.fwSize;
        FwUpdate.fwHeader.version 	= fwCmd->fwStart.version;
        FwFile_erase(&fwFile);
    }
    int32_t ToA = Radio.TimeOnAir( MODEM_LORA, sizeof(MAC_Firmware_Packet_t) );

    RTC_setTimeMs(fwCmd->fwStart.unixTime + 4, fwCmd->fwStart.unixTimeMs);
    printMAC("\n******************Firmware Update Command received\n");
    printMAC("Firmware CRC     : %X\n",   fwCmd->fwStart.fileCRC);
    printMAC("Firmware Version : %.2f\n", (float)fwCmd->fwStart.version/100);
    printMAC("Firmware Size    : %u\n",   fwCmd->fwStart.fwSize);
    printMAC("Firmware Offset  : %X\n",   fwCmd->fwStart.offset);
   
    FwUpdate.offset = FwUpdate_GetPage(FwUpdate.offset)*FW_UPDATE_PAGE_SIZE;
    HAL_FLASH_ErasePages(FwUpdate.offset, (int32_t)(fwFile.maxSize - FwUpdate.offset) ); // erase the remaing pages 

    ret = MAC_sendACK(deviceSettings_crc.DcuID, MAC_CNTRL_ACKNOWLEDGED);       //send ACK.
    MAC_ASSERT_RETURN(ret, MAC_OK);

    MAC_transmitFrame(MAC_RX1_DELAY);
    return ret;
}


/**
 *
 * @param fwCmd
 * @return
 */
MAC_Return_t MAC_FW_Data_Callback(MAC_Firmware_Packet_t* fwPacket)
{
    uint32_t rxOffset = fwPacket->fwData.offset;
    uint8_t rxDataLength = MAC_status.rxFrameSize - sizeof(MAC_Header_t) - offsetof(MAC_Firmware_Packet_t, fwData.data);

    printMAC("Off:%X L:%u\n",   fwPacket->fwData.offset, rxDataLength);
  
    FwUpdate.lastRxTick = osKernelSysTick();
    //printPacket();

    //packet ID contains the current offset.
    if (rxOffset < FwUpdate.offset)
    {        //retransmission, ignore packets.
        return MAC_OK;
    }
    else if (rxOffset > FwUpdate.offset)
    {      //missed a few packets in between.
        printMAC("missed a few packets in between");
        //check if the node and the DCU are on the same page.
        if (FwUpdate_GetPage(rxOffset) > FwUpdate_GetPage(FwUpdate.offset))
        {
          printMAC("Not even on the same page!!");
            //If the DCU is too far ahead, pause firmware update.
            pauseFwUpdate(&FwUpdate);
        }

        //Set the offset to the start of the current page.
        FwUpdate.offset = FwUpdate_GetPage(FwUpdate.offset)*FW_UPDATE_PAGE_SIZE;
        HAL_FLASH_ErasePages(FwUpdate.offset, FW_UPDATE_PAGE_SIZE);  //  // erase the the requested pages 
        
        return MAC_ERROR;
    }

    if (mFile_OK != FwFile_write(   &fwFile,
    								rxOffset,
                                    fwPacket->fwData.data,
                                    rxDataLength))
    {
        printMAC("Something is wrong with flash write");
        pauseFwUpdate(&FwUpdate);
        return MAC_ERROR;
    }



    // is it necessary to compare the received packet and the one in memory like in
    //DCU firmware update????
    FwUpdate.offset = rxOffset + rxDataLength;
    FwUpdate.bytesLeftInPage -= rxDataLength;


    return MAC_OK;
}


/**
 * @brief	Callback for Completion of firmware download.
 * @return	#MAC_Return_t
 */
MAC_Return_t MAC_Firmware_Update_Complete_Callback()
{
    Firmware_Header_t fwHeader;

    FwFile_getHeader(&fwFile, &fwHeader);
    //       verify CRC.
    //       if match reset core.
    //       else reset fw update and clear flash.
    printMAC("Firmware Download Complete.\n");
    FwUpdate.state = FW_UPDATE_COMPLETE;
    FwUpdate.pageCRC = FwFile_computeCRC(&fwFile, 4, fwHeader.size - 4);

    FwUpdate.fwHeader.crc = fwHeader.crc;
    if (FwUpdate.pageCRC == FwUpdate.fwHeader.crc)
    {
        printMAC("CRC match. Rebooting\n");
        HAL_NVIC_SystemReset();
    }
    else
    {
        printMAC("CRC check Failed.\n");
        resetFwUpdate(&FwUpdate);
    }
    return MAC_ERROR;
}

/** @}*/

/**
 * @defgroup MAC_Node_FirmwareApp_Public_Functions				MAC Node Firmware Application Public Functions
 * @{
 */


/**
 *
 * @param node
 * @param expectedCommands
 * @return
 */
MAC_Return_t MAC_Firmware_Update_Callback(MAC_Firmware_Packet_t* fwPacket, uint32_t expectedCommands)
{

	///@todo figure out a way to specify expected firmware packet and return error if
	/// type does not match
//  printMAC("header Type %d\n",fwPacket->header.type);

	switch (fwPacket->header.type)
	{

	case MAC_FW_CMD_AWAIT_FW_UPDATE:
		printMAC("******************Fw Update request accepted. Waiting for update slot.");
		FwUpdate.state = FW_UPDATE_WAIT_FOR_START;
		return MAC_FW_requestResponse_Callback(fwPacket);


	case MAC_FW_CMD_RESTART_FW_UPDATE:
		printMAC("******************Fw Update resume failed. Restarting Fw Update");
		resetFwUpdate(&FwUpdate);
		FwUpdate.state = FW_UPDATE_WAIT_FOR_START;
		return MAC_FW_requestResponse_Callback(fwPacket);


	case MAC_FW_CMD_UPDATE_REQUEST_FAILED:
		printMAC("******************Fw Update request Failed.");
		return MAC_FW_requestResponse_Callback(fwPacket);

	case MAC_FW_CMD_START:
        return MAC_FW_CMD_Start_Callback(fwPacket);

	case MAC_FW_DATA:
        return MAC_FW_Data_Callback(fwPacket);
	
    default :
		return MAC_ERROR;
	}

}


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
//#define OTA_PACKET_LOSS_TEST
void thread_FirmwareUpdate(const void* param)
{

    //This thread does not yeild control once the firmware update is started, until 
    // the update is either complete or for some reason has failed, after which this thread is 
    //terminated.
    
    MAC_Return_t  ret = MAC_ERROR;
    mIWDG_HandleTypeDef* hmIWDG_firmwareUpdate;
     
    uint32_t ticks                 = 0;
    static uint32_t lastFwPacketRx = 0;
    static int timeLeftInSlot      = 0;
    static bool sendNACK           = false;
#ifdef OTA_PACKET_LOSS_TEST
    static uint32_t packetLossProbability = 0;
#endif
 
    hmIWDG_firmwareUpdate = IWDG_initWDG((MAC_REQUERY_TIME + 5)*1000);
    
#ifdef OTA_PACKET_LOSS_TEST
    srand(RTC_getUnixTime() + HAL_GetTick());
#endif
    IWDG_startWDG(hmIWDG_firmwareUpdate);
    uint32_t currSlotFW = FW_UPDATE_SLOT_NUM(RTC_getUnixTime());
    uint32_t currSlotFWLast = FW_UPDATE_SLOT_NUM(RTC_getUnixTime());

    for(;;)
    {
    
        //wait for the firmware update slot to begin.
        //if already in the update slot, wait until the beginning of the next slot.
        //TODO : stop firmware update here, so that the node can request for firmware update
        //again.
        printMAC("\nWaiting till next Fw Slot mark\n");
        RTC_waitTillMark( MAC_REQUERY_TIME / 2, MAC_REQUERY_TIME / 2, hmIWDG_firmwareUpdate);
        
        if (RTC_secondsTillMark(MAC_REQUERY_TIME / 2) != RTC_secondsTillMark(MAC_REQUERY_TIME))
        {
            printMAC("\nStart of Query cycle.\n");
            continue;
        }
        
//        //if not waiting for firmware update, go to sleep again.
        if (FwUpdate.state != FW_UPDATE_WAIT_FOR_START){
            printMAC("\nNo Fw update in progress\n");
            continue;
        }
        
         if (osMutexWait(hmutex_MACRadio, MAC_REQUERY_TIME / 2) != osOK)
         {
           printMAC("MutexTimeout\n");          
           continue;
         }
        osDelay(1000);
        IWDG_refreshWDG(hmIWDG_firmwareUpdate);

        
        IWDG_stopWDG(hmIWDG_MACWaitForQuery);
        osThreadSuspend(hthread_MACWaitForQuery);
        MAC_Init();
        LoRa_setGain(MAC_RADIO_CONFIG_FW_CMD(deviceSettings_crc.DcuID));
        LoRa_SetTxPower(SX1262_POWER_TX_MAX);
 
        MAC_Port_closeAll();

        //The nodes wait for an initial firmware update start command, which also relays
        //the firmware size and firmware version to the nodes.
        
        //The firmware size is used as the terminating condition for firmware update.
        //The node will only received packets upto the specified offset after which 
        //it assumes that the whole of the firmware has been downloaded.
        //CRC check is used to ensure there were no errors in the file transfer.

        
        timeLeftInSlot = 40;    //wait 20 seconds for the firmware update start command.
        
        LoRa_setGain(MAC_RADIO_CONFIG_FW_CMD(deviceSettings_crc.DcuID));
        MAC_Port_open(MAC_PORT_FIRMWARE_UPDATE);
        printMAC("\nWaiting For Fw update start command\n");
        ret = MAC_WaitForPacket(timeLeftInSlot*1000, true);
        
        
        if (ret == MAC_OK)
        { 
          osDelay(1000);
          ret = MAC_Firmware_Update_Callback((MAC_Firmware_Packet_t*)MAC_rxPayload, 0);
            if (ret != MAC_OK)
            {
                printMAC("\nFirmware update Invalid command\n");
                pauseFwUpdate(&FwUpdate);
                LOG_Error(LOG_ErrorSource_Radio, 
                          LOG_RadioError_FwUpdateFailed,
                          LOG_FwUpdateError_InvalidCommand);
            }
            else
            {
                FwUpdate.state   = FW_UPDATE_IN_PROGRESS;
            }
            //TODO : find a way to signal the DCU that the nodes have a newer firmware than specified in the settings.
  
        }
        else 
        {
            //firmware update start command not received. 
            //go to sleep.
            //retain previous update settings to resume in next slot.
            pauseFwUpdate(&FwUpdate);
            printMAC("\nFirmware Update Initial Command not received\n");
            LOG_Error(LOG_ErrorSource_Radio, 
                      LOG_RadioError_FwUpdateFailed, 
                      LOG_FwUpdateError_InitialCommandNotReceived);  
        }
        
        
        //firmware update started, 

        lastFwPacketRx = osKernelSysTick();
        while (FwUpdate.state == FW_UPDATE_IN_PROGRESS)
        {
    
            printMAC("Start of next window");
            //watchdog, 30 + 5 seconds, reset after every slot.
            //since in the node, the radio threads yeild control of radio 
            //by suspension, there is no need to stop watchdogs for
            //query and beacon threads.
            IWDG_refreshWDG(hmIWDG_firmwareUpdate);
            
            //not computing individual page CRC in node, 
            //If the update is paused, the FWUpdate struct should revert to
            //last complete page downloaded and compute the entire CRC then.
            
            timeLeftInSlot = RTC_milliSecondsTillMark(FW_UPDATE_PAGE_TIME);
            currSlotFW = FW_UPDATE_SLOT_NUM(RTC_getUnixTime());         
            if (timeLeftInSlot < 4*1000)
            {
              timeLeftInSlot += FW_UPDATE_PAGE_TIME*1000;
              currSlotFW++;
            }
            printMAC("Start of next window %d", currSlotFW);
            
            //MAC_Init();
            
            LoRa_setGain(MAC_RADIO_CONFIG_FW_DATA(deviceSettings_crc.DcuID));
            
            while (timeLeftInSlot > FW_UPDATE_PAGE_NACK_TIME*1000)
            {            //allow 2 extra seconds, incase a transmission occurs
                         // just on the boundary of the time limit.
                ticks = osKernelSysTick();
                ret = MAC_WaitForPacket(timeLeftInSlot - (FW_UPDATE_PAGE_NACK_TIME*1000), true);     
                
#ifdef OTA_PACKET_LOSS_TEST
                packetLossProbability = (rand()%200) + 1;
                
//                //20% Packet error rate, 
//                if (packetLossProbability <= 5){
//                    ret = MAC_ERROR;
//                }
#endif                
                if (ret == MAC_OK)
                {
                    LoRa_SetTxPower(SX1262_POWER_TX_MAX);
                    lastFwPacketRx = osKernelSysTick();
                    ret = MAC_Firmware_Update_Callback((MAC_Firmware_Packet_t*)MAC_rxPayload, 0);
                    if (ret == MAC_ERROR)
                    {
                        sendNACK = true;
                        break;
                    }
                    
                    if (MAC_rxHeader->frameHeader.controlFlags & MAC_CNTRL_MORE_DATA)
                    {      
                           
                    }
                    else
                    {
                        //last packet in page, break and wait for next page.
                      printMAC("lastpage"); 
                      sendNACK = false;
                        break;  
                    }
                }
                else if(ret == MAC_ERROR)
                {            //CRC failed.
                    printMAC("CRC failed");
                }
                else if (ret == MAC_TIMEOUT)
                {          //no transmission received.
                
                    //no packet received in the 30 second window, 
                    //send NACK for retransmission of page.
                    printMAC("Rx Timeout");
                }  
                timeLeftInSlot = RTC_milliSecondsTillMark(FW_UPDATE_PAGE_TIME);
                // timeLeftInSlot -= osKernelSysTick() - ticks;
                
            }
            printMAC("little time Left In Slot %d",timeLeftInSlot);
           
            
            
            //no packet received for the last 180 seconds. 
            //either SNR is too low or DCU has stopped tranmitting.
            if ((lastFwPacketRx + (2*FW_UPDATE_PAGE_TIME*1000) < osKernelSysTick()))
            {
                printMAC("no packet received for the last %d seconds ",(osKernelSysTick() - lastFwPacketRx));
               //pause firmware update here.
                pauseFwUpdate(&FwUpdate);
                break; 
            }
            else if (FwUpdate.state == FW_UPDATE_PAUSED)
            {
                break;
            }
            else if (FwUpdate.offset == FwUpdate.fwHeader.size)
            {
                MAC_Firmware_Update_Complete_Callback();
                //Since, on success, the controller will reset, the following code
                //will execute only incase of a failure.
                resetFwUpdate(&FwUpdate);
                break;
            }
            
            LoRa_sleepMode();
            
            //if the last packet received was not the last packet
            //transmitted, send NACK to DCU.
            if (MAC_rxHeader->frameHeader.controlFlags & MAC_CNTRL_MORE_DATA)
                sendNACK = true;
            
            
            timeLeftInSlot = RTC_milliSecondsTillMark(FW_UPDATE_PAGE_TIME);
            //wait until start of NACK transmit window.
            while (timeLeftInSlot > FW_UPDATE_PAGE_NACK_TIME*1000)
            {    
                ticks = osKernelSysTick();
                osDelay(timeLeftInSlot - (FW_UPDATE_PAGE_NACK_TIME*1000));
                timeLeftInSlot = RTC_milliSecondsTillMark(FW_UPDATE_PAGE_TIME);
            }
            
            //Transmit NACK incase of missing packets in page.
            //Continue transmission until start of next slot.
            LoRa_setGain(MAC_RADIO_CONFIG_FW_CMD(deviceSettings_crc.DcuID));
            while (timeLeftInSlot > 3000 && currSlotFW == FW_UPDATE_SLOT_NUM(RTC_getUnixTime()))
            {
                ticks = osKernelSysTick();
                
                if (sendNACK)
                {    
                    ret = MAC_FW_requestLastPage(&FwUpdate);
                    if (ret == MAC_OK)
                    {
                        printMAC("Transmit NACK");
                        MAC_transmitFrame(MAC_RX1_DELAY);  
                    }
                    
                    osDelay(500);
                }
                else
                    osDelay(timeLeftInSlot - 2000);
                    
                timeLeftInSlot = RTC_milliSecondsTillMark(FW_UPDATE_PAGE_TIME);
                printMAC("       time Left In next Slot %d",timeLeftInSlot);

            }
            
        }
  
        deviceParams_commitToFlash(&deviceSettings_crc);
    
        //The firmware update thread yeilds control to the beacon thread, 
        //assuming that since the firmware update was not successful,
        //the node has disconnected from the network.
        MAC_Port_closeAll();
        LoRa_sleepMode();
        osMutexRelease(hmutex_MACRadio);
        
        printMAC("END of OTA thread\n");
        
        IWDG_startWDG(hmIWDG_MACWaitForQuery);
        IWDG_refreshWDG(hmIWDG_MACWaitForQuery);
        osThreadResume(hthread_MACWaitForQuery);
    }
    

    //osThreadResumeAll();
    //osThreadSuspend(hthread_MACWaitForQuery);
    osThreadTerminate(NULL);

}



/** @}*/

