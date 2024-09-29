/**
 * @file   	MAC_Application.c
 * @author 	Hassan
 * @version	
 * @date	May 18, 2016
 * 
 * @brief   
 */

#include "MAC.h"
#include "MAC_Application.h"
#include "log.h"
#include "mRTC.h"
#include "SensorDefs.h"
#include "mFLASH.h"
#include "MAC_Node_FirmwareApp.h"

#include "DEBUG_UART.h"





/**
 * @defgroup MAC_Application_Private_Structs		MAC Application Private Structs
 * @{
 */





/** @}*/

/**
 * @defgroup MAC_Application_Public_Members			MAC Application Public Members
 * @{
 */

extern float batteryVoltage;

osMutexId       hmutex_MACRadio;


osThreadId      hthread_MACWaitForQuery;
osThreadId      hthread_rangeTest;
/** @}*/


/**
 * @defgroup MAC_Application_Private_Members		MAC Application Private Members
 * @{
 */
 
mIWDG_HandleTypeDef* hmIWDG_MACWaitForQuery;
 
 
 

/** @}*/

/**
 * @defgroup MAC_App_Private_Functions			MAC Application Private Functions
 * @{
 */




/**
 * @brief	Callback for MAC_CNTRL_REPLY_SETTINGS flag in the MAC header control byte
 * @return	#MAC_Return_t
 */
MAC_Return_t MAC_Settings_Callback(MAC_CMD_t* cmd)
{
	//re checking received settings CRC is redundant here.
	///If the MAC_CNTRL_REPLY_SETTINGS flag is set in the query packet,
	///the payload has the node settings parameters in the order as defined in
    /// CLOUD_NODE_SETTINGS_LIST Macro
    printMAC("Node_Cloud_Params updating\n");
    memcpy(&deviceSettings, &cmd->query.settings, sizeof(Node_Cloud_Params_t));
    deviceParams_commitToFlash(&deviceSettings_crc);
    return MAC_OK;
}

/**
 *
 * @param controlFlags
 * @param sensorData
 * @param requestSettings
 * @return
 */
MAC_Return_t MAC_sendSensorData(uint8_t controlFlags, SensorsData_t* sensorData, bool requestSettings)
{
	static MAC_SensorData_t sensorDataPacket;
	uint32_t length = sizeof(sensorDataPacket.header);;

	sensorDataPacket.header.flags = 0;
	sensorDataPacket.header.requestSettings = requestSettings;
	sensorDataPacket.header.hasPayload = (sensorData != NULL);
  controlFlags |= MAC_CNTRL_ACKNOWLEDGE;     // ACKNOWLEDGE required 

	if (sensorData != NULL)
		length += Sensors_compressData(sensorData, sensorDataPacket.data);



	return MAC_sendData(deviceSettings_crc.DcuID,
						MAC_PORT_SENSOR_DATA,
						controlFlags,
						(uint8_t*)&sensorDataPacket,
						length);
}


/**
 *
 * @param controlFlags
 * @param errorPacket
 * @param requestSettings
 * @return
 */
MAC_Return_t MAC_sendErrorLog(uint8_t controlFlags, LOG_ErrorPacket_t* errorPacket, bool requestSettings)
{
	static MAC_ErrorLog_t errorLogPacket;
	uint32_t length = sizeof(errorLogPacket.header);

	errorLogPacket.header.flags = 0;
	errorLogPacket.header.requestSettings = requestSettings;
	errorLogPacket.header.hasPayload = (errorPacket != NULL);

	length += sizeof(LOG_ErrorPacket_t);

	if (errorPacket != NULL)
		memcpy(&errorLogPacket.log, errorPacket, sizeof(errorLogPacket.log));

	return MAC_sendData(deviceSettings_crc.DcuID,
						MAC_PORT_ERROR_LOG,
						controlFlags,
						(uint8_t*)&errorLogPacket,
						length);
}



/**
 *
 * @return
 */
uint32_t numACK = 0;
MAC_Return_t MAC_ACK_Callback()
{
  numACK++;
    printMAC("ACK received");
    if (MAC_txHeader->frameHeader.port == MAC_PORT_SENSOR_DATA)
        Sensors_clearPackets(1);
    else if (MAC_txHeader->frameHeader.port == MAC_PORT_ERROR_LOG)
        LOG_clear(1);
    
    return MAC_OK;
}


uint32_t synced = 0;

/**
 *
 * @param header
 * @param cmd
 * @return
 */
uint32_t numQuery = 0;
MAC_Return_t MAC_CMD_Query_Callback(MAC_Header_t* header, MAC_CMD_t* cmd)
{
  numQuery++;
  static SensorsData_t 		sensorPacket;
  static LOG_ErrorPacket_t    errorPacket;
  MAC_Return_t ret = MAC_ERROR;
  uint32_t sensorPacketCount =0;
  uint32_t logCount = 0;
  uint8_t controlFlags = MAC_CNTRL_ACKNOWLEDGED;
  bool requestSettings = false;
  uint32_t OldSeconds = 0;
  uint32_t OldMilliSeconds = 0;
  uint32_t milliSeconds = 0;
  uint32_t unixSeconds = 0;
	
  if (cmd->hdr.subCmdType == MAC_CMD_QUERY_FIRST)
  {

    RTC_getUnixTimeMs(&OldSeconds, &OldMilliSeconds);
    if(cmd->query.first.rtcMiliSeconds < 1000)    // backward compatibility either it is sttetings CRC or milliseconds
    {
      milliSeconds = cmd->query.first.rtcMiliSeconds + OVER_THE_AIR_TIME() + (MAC_RX1_DELAY);	
      unixSeconds = cmd->query.first.rtc + (milliSeconds / 1000);
      milliSeconds %= 1000;
    }
    else
    { 
      milliSeconds = OVER_THE_AIR_TIME() + (MAC_RX1_DELAY);
      unixSeconds = cmd->query.first.rtc + (milliSeconds / 1000);		
      milliSeconds %= 1000;
    }
    //Synchronize RTC using the packet ID of the query packet.
    if(unixSeconds < 1483228800)
    {
      // this is not possible dont update
    }
    else  
    { 
        int32_t deltaTime = (int32_t)unixSeconds - (int32_t)OldSeconds;
        deltaTime *= 1000; // convert to millisecond
        deltaTime += milliSeconds;
        deltaTime -= OldMilliSeconds;
      
        if(deltaTime < 0) deltaTime *=-1;
        if ((deltaTime)  > 1000) // if there is time differnen update the time
        {
          RTC_setTimeMs( unixSeconds, milliSeconds);
          synced++;
//          LOG_Error(LOG_ErrorSource_SecondarySystem, LOG_Error_RTCreSync, OldSeconds);
        }
    }
    struct tm* dateTime = localtime(&unixSeconds); 
    printDEBUG("Time Recived on RF_FIR %02d:%02d:%02d.%03d \t ", dateTime->tm_hour, dateTime->tm_min, dateTime->tm_sec, milliSeconds);
    struct tm* dateTimeOld = localtime(&OldSeconds); 
    printDEBUG("Time of Node %02d:%02d:%02d.%03d \n", dateTimeOld->tm_hour, dateTimeOld->tm_min, dateTimeOld->tm_sec, OldMilliSeconds);
    printDEBUG("\t\t\tsender %d - Query \n", header->dcuAddr);


    ///If the settings CRC in the query packet does not match
    /// the current CRC of the node, the MAC_CNTRL_REPLY_SETTINGS flag
    /// is set in the response to signal the DCU to send the settings in
    /// the next query.

    requestSettings = cmd->query.first.nodeParamCRC != deviceParams_getCRC();

  }
	else if (cmd->hdr.subCmdType == MAC_CMD_QUERY_SETTINGS)
	{
		MAC_Settings_Callback(cmd);
	}


	printMAC("Query received                                          %X \n", header->frameHeader.controlFlags );


	//If the query packet is not the first, then this is a response
	//to a previous query and so, a packet should be removed from the queue.

	///@todo Define flag to signal to the node that the last packet was received successfully.
	if (header->frameHeader.controlFlags & MAC_CNTRL_ACKNOWLEDGED)
	{
        MAC_ACK_Callback();
    }

	if (!(header->frameHeader.controlFlags & MAC_CNTRL_ACKNOWLEDGE))
		return MAC_OK;



	//Save the packet counts in variables so that the
	//Queues are read only once.
  sensorPacketCount = Sensors_get_PacketCount();
  logCount = LOG_get_Count();

    ///Attempt to request firmware update. The function will return
    /// error if firmware update is not pending.
    
    if(MAC_OK == MAC_request_FwUpdate(&FwUpdate))
    {
        ret = MAC_OK;
    }
    else if (sensorPacketCount > 0)
	{
		//If there are more packets waiting in the queue,
		//signal the DCU for more data.
		if (sensorPacketCount > 1 || logCount)
    {
			controlFlags |= MAC_CNTRL_MORE_DATA;
    }

		controlFlags |= MAC_CNTRL_ACKNOWLEDGE;
    Sensors_readPacket(&sensorPacket);
		//load sensor data packet.
		ret = MAC_sendSensorData(controlFlags, &sensorPacket, requestSettings);

		MAC_ASSERT_RETURN(ret, MAC_OK);
	}
	else if(logCount > 0)
	{
		//if there are no sensor data,
		//send error log packets if any.
		LOG_readError(&errorPacket);      //TODO : bad code.

		if (logCount > 1)
			controlFlags |= MAC_CNTRL_MORE_DATA;

		controlFlags |= MAC_CNTRL_ACKNOWLEDGE;
		ret = MAC_sendErrorLog(	controlFlags, &errorPacket, requestSettings);

		MAC_ASSERT_RETURN(ret, MAC_OK);
	}
	else if (requestSettings)
	{
		//if there are no sensor data packet or log error packets in the queue,
		//respond with an empty sensor data packet with Acknowledged flag set.
		//This is for the case when there is no data to transmit but the node
		//settings need to be updated. Otherwise, the More Data flag signals the
		//end of transmission.
		ret = MAC_sendSensorData(controlFlags, NULL, requestSettings);
		MAC_ASSERT_RETURN(ret, MAC_OK);
	}
	else
	{
		ret = MAC_sendACK(deviceSettings_crc.DcuID, controlFlags);
		MAC_ASSERT_RETURN(ret, MAC_OK);
	}

	//A packet is being transmitted in all execution paths.
	return MAC_transmitFrame(MAC_RX1_DELAY);
}

    uint32_t RTC_UpdateunixSeconds = 0;	
    uint32_t RTC_UpdatemilliSeconds = 0;

/**
 *
 * @param header
 * @param cmd
 * @return
 */
MAC_Return_t MAC_CMD_RTC_Update_Callback(MAC_Header_t* header, MAC_CMD_t* cmd)
{
    uint32_t OldSeconds = 0;
    uint32_t OldMilliSeconds = 0;
    RTC_getUnixTimeMs(&OldSeconds, &OldMilliSeconds);
    if(cmd->rtc_Update.rtcMiliSeconds < 1000)    // backward compatibility either it is sttetings CRC or milliseconds
    {
      RTC_UpdatemilliSeconds = cmd->rtc_Update.rtcMiliSeconds + OVER_THE_AIR_TIME() + (MAC_RX1_DELAY);	
      RTC_UpdateunixSeconds = cmd->rtc_Update.rtc + (RTC_UpdatemilliSeconds / 1000);
      RTC_UpdatemilliSeconds %= 1000;
    }
    else
    {
      RTC_UpdatemilliSeconds = OVER_THE_AIR_TIME() + (MAC_RX1_DELAY);
      RTC_UpdateunixSeconds = cmd->rtc_Update.rtc + (RTC_UpdatemilliSeconds / 1000);		
      RTC_UpdatemilliSeconds %= 1000;
    }
	//Synchronize RTC using the packet ID of the query packet.
    if(RTC_UpdateunixSeconds < 1483228800)
    {
        // this is not possible dont update
    }
    else //if (!((RTC_UpdateunixSeconds - 1) < OldSeconds && (RTC_UpdateunixSeconds + 1) > OldSeconds))  // if there is time differnen update the time
    {
        int32_t deltaTime = (int32_t)RTC_UpdateunixSeconds - (int32_t)OldSeconds;
        deltaTime *= 1000; // convert to millisecond
        deltaTime += RTC_UpdatemilliSeconds;
        deltaTime -= OldMilliSeconds;
      
        if(deltaTime < 0) deltaTime *=-1;
        if ((deltaTime)  > 1000) // if there is time differnen update the time
        {
          RTC_setTimeMs( RTC_UpdateunixSeconds, RTC_UpdatemilliSeconds);
          synced++;
//          LOG_Error(LOG_ErrorSource_SecondarySystem, LOG_Error_RTCreSync, OldSeconds);
        }
    }
    struct tm* dateTime = localtime(&RTC_UpdateunixSeconds); 
    printDEBUG("Time Recived on RF_RTC %02d:%02d:%02d.%03d \t ", dateTime->tm_hour, dateTime->tm_min, dateTime->tm_sec, RTC_UpdatemilliSeconds);
    struct tm* dateTimeOld = localtime(&OldSeconds); 
    printDEBUG("Time on Node %02d:%02d:%02d.%03d \n", dateTimeOld->tm_hour, dateTimeOld->tm_min, dateTimeOld->tm_sec, OldMilliSeconds);
    printDEBUG("\t\t\tsender %d - RTC_update \n", header->dcuAddr);

    MAC_status.isWaitingForReply = false;
	return MAC_OK;
}

/**
 *
 * @param header
 * @param cmd
 * @return
 */
MAC_Return_t MAC_CMD_set_Network_Address_Callback(MAC_Header_t* header, MAC_CMD_t* cmd)
{
	if (memcmp((uint8_t*)UID_BASE, cmd->set_NetworkAddr.devUID, sizeof(cmd->set_NetworkAddr.devUID)) != 0)
        return MAC_ERROR;
    
    deviceSettings_crc.networkID = cmd->set_NetworkAddr.nodeNetworkID;
    deviceSettings_crc.DcuID     = cmd->set_NetworkAddr.DCUNetworkID;
    deviceParams_commitToFlash(&deviceSettings_crc);
    
    if (header->frameHeader.controlFlags & MAC_CNTRL_ACKNOWLEDGE)
        return MAC_sendACK(header->dcuAddr, MAC_CNTRL_ACKNOWLEDGED);
    
    return MAC_OK;
}

/**
 *
 * @param cmd
 * @return
 */
MAC_Return_t MAC_CMD_Callback(MAC_Header_t* header, MAC_CMD_t* cmd)
{
	MAC_Return_t ret = MAC_ERROR;
  printMAC("cmdType %d\n",cmd->hdr.cmdType);

	switch (cmd->hdr.cmdType)
	{
	case MAC_CMD_QUERY:
		ret = MAC_CMD_Query_Callback(header, cmd);
		break;

	case MAC_CMD_RTC_UPDATE:
		ret = MAC_CMD_RTC_Update_Callback(header, cmd);
		break;

	case MAC_CMD_SET_NETWORK_ADDR:
		ret = MAC_CMD_set_Network_Address_Callback(header, cmd);
		break;

	}

	return ret;
}




/**
 *
 * @param slot
 * @return
 */
bool MAC_isQuerySlot(uint32_t slot)
{
	uint32_t assignedSlot = MAC_getAssignedSlot(deviceSettings_crc.networkID);

    if (deviceSettings_crc.networkID == 0x00 ||
    	slot == assignedSlot)
    {
        return true;
    }
    else if (MAC_isFirmwareUpdateSlot(slot))
    {
        return false;
    }
    else if (MAC_status.firsRun)
    {
        return true;
    }
        
    return false;
}

/**
 *
 * @param currSlot
 * @return
 */
MAC_Return_t MAC_QuerySlot_Error_Callback(uint32_t currSlot)
{
    if (MAC_status.querySlotErrorCount < MAC_QUERY_SLOT_ERRORS_MAX)
    {
        MAC_status.querySlotErrorCount++;
    }   
    return MAC_OK;
}

/**
 *
 * @return
 */
MAC_Return_t MAC_QuerySlot_Success_Callback()
{
    MAC_status.querySlotErrorCount = 0;
    return MAC_OK;
}

/**
 *
 * @param timeLeftInSlot
 * @return
 */
MAC_Return_t MAC_QuerySlot(int32_t timeLeftInSlot)
{
    MAC_Return_t ret = MAC_ERROR;
    uint32_t ticks = 0;
    bool queryComplete = false;
    uint32_t waitPeriod = 10*1000;
    
    ticks = osKernelSysTick();
    if (osMutexWait(hmutex_MACRadio, timeLeftInSlot) != osOK)
    {
        printMAC("MutexTimeout\n");          
        LOG_Error(LOG_ErrorSource_Radio, LOG_RadioError_MutexTimeout, 0);
        return MAC_ERROR;
    }
    timeLeftInSlot -= osKernelSysTick() - ticks;
    printMAC("timeLeftInSlot %d\n ", timeLeftInSlot);
    
    //The waitPeriod is initially set to 10 seconds, 
    //so that if no packets are received within the first 10 seconds of the slot, 
    //the radio is put back to sleep.  
    //If node has not made contact with the DCU in a while, and is in
    //error state, keep the node awake for the entire duration of the
    //slot instead.
    if (MAC_status.querySlotErrorCount >= MAC_QUERY_SLOT_ERRORS_MAX || RTC_getUnixTime() < 1483228800 || MAC_status.firsRun)
        waitPeriod = timeLeftInSlot + (60  * 1000) ;     // wait at least two MAC slot windows  
    else
        waitPeriod = min_ui32(timeLeftInSlot, MAC_SLOT_DURATION*1000);   // in normal conditions max wait time should no greater than SLOT Time 
    
    // if there is old packet in the buffer discard those
    LoRa_IrqHandler();
    LoRa_clearIRQFlags( IRQ_RADIO_ALL );
    
    
    MAC_status.isWaitingForReply = true;
    ///The data query is done at 500 KHz bandwidth and spread factor of 11.
    LoRa_setGain(MAC_RADIO_CONFIG_QUERY(deviceSettings_crc.DcuID));
    //wait for query from DCU
    MAC_Port_closeAll();
    MAC_Port_open(MAC_PORT_COMMAND);
    MAC_Port_open(MAC_PORT_NONE);
    MAC_Port_open(MAC_PORT_FIRMWARE_UPDATE);

    while (timeLeftInSlot > 0)
    {
        ticks = osKernelSysTick();
        
        
        printMAC("Waiting for query \n Time Left : %d\n waitPeriod : %d", timeLeftInSlot, waitPeriod);

        ret = MAC_WaitForPacket(waitPeriod, true);
        printMAC("Waiting End\n");
        
        if (ret == MAC_OK)
        {
            LoRa_calcAndSetTxPower(LoRa_getSNR());
            //flush tx buffer before attempting to reply to query.
            MAC_status.hasFrameInBuffer = false;
            printMAC("port %d\n",MAC_rxHeader->frameHeader.port);
            
            if (MAC_rxHeader->frameHeader.port == MAC_PORT_COMMAND)
            {
              printMAC("MAC_PORT_COMMAND\n");
            	ret = MAC_CMD_Callback(MAC_rxHeader, (MAC_CMD_t*)MAC_rxPayload);
            }
            else if (MAC_rxHeader->frameHeader.port == MAC_PORT_FIRMWARE_UPDATE)
            {
              printMAC("MAC_PORT_FIRMWARE_UPDATE\n");
            	ret = MAC_Firmware_Update_Callback((MAC_Firmware_Packet_t*)MAC_rxPayload, 0);
              if (ret != MAC_OK)
              {
                queryComplete = true;
                break;
              }
            }
            else if (MAC_rxHeader->frameHeader.port == MAC_PORT_NONE &&
                    (MAC_rxHeader->frameHeader.controlFlags & MAC_CNTRL_ACKNOWLEDGED))
            {
              printMAC("MAC_PORT_NONE\n");
                ret = MAC_ACK_Callback();
            }
        }
        else if (ret == MAC_TIMEOUT)
        {      //no query received in time slot. disconnected from network.
            printMAC("\nThread : MACWaitForQuery \n No Query received.\n");
            LOG_Error(LOG_ErrorSource_Radio, LOG_RadioError_QueryRxTimeout, 0);
            break;
        }
        else if (ret == MAC_ERROR)
        {
            //Invalid packet received.
            printMAC("\nThread : MACWaitForQuery \n Rx Failed.\n");
        }

        timeLeftInSlot -= osKernelSysTick() - ticks;
        ticks = osKernelSysTick();
        waitPeriod = timeLeftInSlot;
        printMAC("timeLeftInSlot %d\n ", timeLeftInSlot);
        
        
        if (ret == MAC_OK && MAC_status.isWaitingForReply == false)
        {
            printMAC("queryComplete %d,%d\n", ret, MAC_status.isWaitingForReply);
            queryComplete = true;
            break;
        }

    }

    printMAC("END of query function timeLeftInSlot %d\n ", timeLeftInSlot);
    osMutexRelease(hmutex_MACRadio);
    MAC_Port_closeAll();
    deviceParams_commitToFlash(&deviceSettings_crc);
    
    if (!queryComplete) 
    {
        return MAC_ERROR;
    }
    
    return MAC_OK;
}


/** @}*/



/**
 * @defgroup	MAC_App_Public_Functions		MAC Application Public Functions
 * @{
 */

uint32_t currSlot = 0;
uint32_t mySlotTime = 0;
int32_t  MAC_querySleepTime = 0;
uint32_t prevSlot = 0;
uint32_t MAC_QueryUnixTime = 0;
uint32_t MAC_QueryUnixTimeLast = 0;
extern uint32_t globalDeviceOperatinStatus;
TaskHandle_t xTaskRadioNFC = NULL;

void thread_MAC_Query(const void* param)
{
	//Query thread watchdog.
	


	int32_t secondsToNextSlot;

  MAC_Return_t    ret;
  MAC_QueryUnixTime = RTC_getUnixTime();
  currSlot = ( MAC_QueryUnixTime % MAC_REQUERY_TIME ) / MAC_SLOT_DURATION;
  prevSlot = currSlot - 1;
	//the watchdog is set to 1 minute and 5 seconds, because the thread
	//wakes up every minute to check whether the assigned time slot has arrived.
	hmIWDG_MACWaitForQuery = IWDG_initWDG((MAC_REQUERY_TIME + MAC_SLOT_DURATION)*1000);
	IWDG_startWDG(hmIWDG_MACWaitForQuery);

    osMutexWait(hmutex_MACRadio, osWaitForever);
  LoRa_Init();
  MAC_Init();
  LoRa_setGain(MAC_RADIO_CONFIG_FW_CMD(deviceSettings_crc.DcuID));
  MAC_status.firsRun = true;
  
  LoRa_sleepMode();
  LoRa_SetTxPower(SX1262_POWER_TX_MAX);
    osMutexRelease(hmutex_MACRadio);
  uint32_t ulNotificationValue;
  
	for (;;)
	{
    while(globalDeviceOperatinStatus  != 0x55)
    {
      if(globalDeviceOperatinStatus  != 0x55)
      { 
        printMAC("\n********************\nRadio turned off by nfc\n");        
        SX126x_SPI_Init();
        LoRa_sleepMode();
        LoRa_SetTxPower(SX1262_POWER_TX_MAX);
        IWDG_stopWDG( hmIWDG_MACWaitForQuery);
        xTaskRadioNFC = xTaskGetCurrentTaskHandle();
//        xTaskNotifyStateClear( NULL );
        ulNotificationValue = ulTaskNotifyTake( pdTRUE,  portMAX_DELAY);  // wait for
        xTaskRadioNFC = NULL;
        IWDG_startWDG( hmIWDG_MACWaitForQuery);
      }
      if(globalDeviceOperatinStatus  == 0x55)
      {     
        printMAC("\n********************\nRadio turned on again by nfc\n");        
      }

    }
		IWDG_refreshWDG(hmIWDG_MACWaitForQuery);
    MAC_QueryUnixTime = RTC_getUnixTime();
		currSlot = ( MAC_QueryUnixTime % MAC_REQUERY_TIME ) / MAC_SLOT_DURATION;
    
    printMAC("\n Current Slot : %d ", currSlot);
    printMAC("Num Sensor Packets : %d\tNum Log Packets : %d\n", Sensors_get_PacketCount(), LOG_get_Count());
    if(MAC_isQuerySlot(currSlot))
    { 
      printMAC("\n This is my Slot : %d ", currSlot);
    }
        //Do not go into sleep if the operations have overflowed into the next slot 
        //which is yet to be processed.
		
		if (MAC_isQuerySlot(currSlot) || MAC_QueryUnixTime < 1483228800 || MAC_status.firsRun)
    { 
      MAC_QueryUnixTimeLast = MAC_QueryUnixTime;
      secondsToNextSlot = RTC_milliSecondsTillMark( MAC_SLOT_DURATION ); 
      if(batteryVoltage > 2.9)
      {
        SX126x_SPI_Init();
        ret = MAC_QuerySlot( secondsToNextSlot );
//        SX126x_SPI_DeInit();
        MAC_status.firsRun = 0; 
        printMAC("After MAC_QuerySlot   [%d]", ret);
      }
      else
      {
        printMAC("Battery Too Low %.3fv", batteryVoltage);
        ret = MAC_OK;
      }
      if (ret != MAC_OK)
      {
          MAC_QuerySlot_Error_Callback(currSlot);
      }
      else
      {
          MAC_QuerySlot_Success_Callback();
      }    
      
      printMAC("Radio Status %d", Radio.GetStatus()); 
      MAC_QueryUnixTime = RTC_getUnixTime();
      if(MAC_QueryUnixTime > MAC_QueryUnixTimeLast + MAC_SLOT_DURATION)
      {
        // overflowed into the next slot
      }
      else
      {
        RTC_waitTillMark(MAC_SLOT_DURATION, MAC_SLOT_DURATION, hmIWDG_MACWaitForQuery); 
      }
    }
//    else
    {
      MAC_QueryUnixTime = RTC_getUnixTime();
      mySlotTime = MAC_getAssignedSlot(deviceSettings_crc.networkID) * MAC_SLOT_DURATION;
      printMAC("mySlotTime %d - unixtime = %d\n", mySlotTime, MAC_QueryUnixTime);
      MAC_querySleepTime = mySlotTime - ( MAC_QueryUnixTime % MAC_REQUERY_TIME );
      printMAC("MAC_querySleepTime %d\n", MAC_querySleepTime);
      
      if( MAC_querySleepTime <= (int32_t)(-1 * (int32_t)MAC_SLOT_DURATION))
      {
        MAC_querySleepTime += MAC_REQUERY_TIME;
        printMAC("Adjusted MAC_querySleepTime %d\n", MAC_querySleepTime);
      }
      if( MAC_querySleepTime > 1)
      {
      }
      else 
      { // MAC_querySleepTime is less than zero and slot is not passed yet  
        printMAC("Already in slot");
        continue;
      }
      if( MAC_querySleepTime > 60)  // this due to systick inaccuracy over longer period of time
      {
        osDelay((MAC_querySleepTime - 60)*1000);
      }   
      else
      {
        osDelay(RTC_milliSecondsTillMark(MAC_SLOT_DURATION));
      }
    }
    
    
	}
	osThreadTerminate(NULL);

}
void thread_MAC_RangeTest(const void* param)
{
  
  MAC_Return_t ret = MAC_ERROR;
  LoRa_Init();
  LoRa_sleepMode();
  MAC_Init();
  LoRa_setGain(LORA_BW_500, LORA_SF12);
 
	for (;;)
  {
    LoRa_setGain(LORA_BW_500, LORA_SF12);
    ret = MAC_sendACK(deviceSettings_crc.DcuID, MAC_CNTRL_ACKNOWLEDGED);       //send ACK.
    MAC_transmitFrame(MAC_RX1_DELAY);
    osDelay(1000);
    (void) ret;
  }
}


void thread_MAC_UnitTest(const void* param)
{
  int32_t secondsToNextSlot;

  MAC_Return_t ret = MAC_ERROR;
  MAC_QueryUnixTime = RTC_getUnixTime();
  currSlot = ( MAC_QueryUnixTime % MAC_REQUERY_TIME ) / MAC_SLOT_DURATION;
  prevSlot = currSlot - 1;
  //the watchdog is set to 1 minute and 5 seconds, because the thread
  //wakes up every minute to check whether the assigned time slot has arrived.
  hmIWDG_MACWaitForQuery = IWDG_initWDG((MAC_REQUERY_TIME + MAC_SLOT_DURATION)*1000);
  IWDG_startWDG(hmIWDG_MACWaitForQuery);

  osMutexWait(hmutex_MACRadio, osWaitForever);
  LoRa_Init();
  MAC_Init();
  LoRa_setGain(MAC_RADIO_CONFIG_FW_CMD(deviceSettings_crc.DcuID));
  MAC_status.firsRun = true;

  LoRa_sleepMode();
  LoRa_SetTxPower(SX1262_POWER_TX_MAX);
  osMutexRelease(hmutex_MACRadio);
  uint32_t ulNotificationValue;


	for (;;)
  {
    IWDG_refreshWDG(hmIWDG_MACWaitForQuery);
    osDelay(10*1000);
    /*
    LoRa_setGain(LORA_BW_500, LORA_SF12);
    ret = MAC_sendACK(deviceSettings_crc.DcuID, MAC_CNTRL_ACKNOWLEDGED);       //send ACK.
    MAC_transmitFrame(MAC_RX1_DELAY);

    uint32_t ticks = 0;
    uint32_t waitPeriod = 60*1000;
        
    // if there is old packet in the buffer discard those
    LoRa_IrqHandler();
    LoRa_clearIRQFlags( IRQ_RADIO_ALL );
        
    MAC_status.isWaitingForReply = true;
    ///The data query is done at 500 KHz bandwidth and spread factor of 11.
    LoRa_setGain(MAC_RADIO_CONFIG_QUERY(deviceSettings_crc.DcuID));
    //wait for query from DCU
    MAC_Port_closeAll();
    MAC_Port_open(MAC_PORT_COMMAND);
    MAC_Port_open(MAC_PORT_NONE);
    MAC_Port_open(MAC_PORT_FIRMWARE_UPDATE);  
    ret = MAC_WaitForPacket(waitPeriod, true);
    */
    
  }
}
/** @}*/
