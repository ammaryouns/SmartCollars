/**
 * @file   	MAC_DCU_Application.c
 * @author 	Hassan
 * @version	
 * @date	May 19, 2016
 * 
 * @brief   
 */

#include "MAC.h"
#include "MAC_DCU_Application.h"
#include "MAC_DCU_FirmwareApp.h"
#include "mQueue.h"
#include "log.h"
#include "mIWDG.h"
#include "Device_Params.h"
#include "mRTC.h"
#include "SensorDefs.h"

#include "DEBUG_UART.h"




#define MAC_get_Slot( nodeInArray )         ( ((ptrdiff_t)nodeInArray - (ptrdiff_t)MAC_status.connectedNodes[0]) / sizeof(MAC_status.connectedNodes[0]))



/**
 * @defgroup MAC_DCU_Application_Public_Members			MAC DCU Application Public Members
 * @{
 */

osMutexId       hmutex_MACRadio;


osThreadId      hthread_MACSendQuery    = NULL;

/** @}*/


/**
 * @defgroup MAC_DCU_Application_Private_Members		MAC DCU Application Private Members
 * @{
 */

mIWDG_HandleTypeDef* hmIWDG_beacon;


/** @}*/

/**
 * @defgroup MAC_DCU_Application_Private_Functions		MAC DCU Application Private Functions
 * @{
 */

void MAC_pullNetworkAddresses(void)
{
    MAC_Node_t* node = NULL;
    for (int i = 0 ; i < MAC_NODES_MAX ; i++)
    {
        node = &MAC_status.connectedNodes[i / MAC_NODES_IN_SLOT_MAX][i % MAC_NODES_IN_SLOT_MAX];
        if(LoRaNAT_isValidRecord(i))
        {
            if (node->addr == 0)
            {
                node->queryErrorCount = MAC_QUERY_ERRORS_MAX;
                node->querySlotErrorCount = MAC_QUERY_SLOT_ERRORS_MAX;
            }
            node->bandwidth = MAC_RADIO_cofigQuery[deviceSettings.DCU_LoRa_Params.networkID].bandwidth;
            node->spreadFactor = MAC_RADIO_cofigQuery[deviceSettings.DCU_LoRa_Params.networkID].spreadFactor;            
            node->addr = MAC_getNetworkID(i) ;
        }
        else
        {
            node->addr = 0;
        }
    }
}

void MAC_log_SNR(uint32_t nodeID, SNR_Log_Struct* snrLog)
{
    static LOG_ErrorPacket_t errorPacket;
    
    errorPacket.timeStamp = RTC_getUnixTime();
    errorPacket.source    = LOG_ErrorSource_Radio;
    errorPacket.error     = LOG_Radio_NodeSNR;
    
    errorPacket.message.value = 0;
    
    if (snrLog != NULL && snrLog->numOfPackets)
    {
        errorPacket.message.value = (snrLog->RSSIsum / snrLog->numOfPackets);
        errorPacket.message.value <<= 16;
        errorPacket.message.value |= (snrLog->SNRsum / snrLog->numOfPackets) & 0xFFFF;
    }
    LOG_NodeError(nodeID, &errorPacket);
}

/**
 * @brief 	Check if any nodes have not responded for sometime and have exceeded
 * 			the specified time interval.
 * @return
 * 		@arg true	If a node has not responded for the specified time slots.
 * 		@arg false	If all nodes are responding.
 */
bool MAC_isNodeMissing()
{
	for (int i = 0 ; i < MAC_NODES_IN_SLOT_MAX ; i++)
	{
		for (int j = (MAC_TIME_SLOTS_MAX/2) - 1 ; j >= 0 ; j--)
		{
			if (MAC_status.connectedNodes[j][i].addr != 0 &&
				MAC_status.connectedNodes[j][i].querySlotErrorCount >= MAC_QUERY_SLOT_ERRORS_MAX)
			{
				return true;
			}
		}
	}

	return false;
}
/**
 * @brief				Prepare a query packet and load it into the transmit frame
 * @param node			Node to query.
 * @param queryType		Type of Query
 * @param acknowledged	Signals if this query packet is acknowledging a previously received packet.
 * @return				#MAC_Return_t
 */
MAC_Return_t MAC_sendQuery(MAC_Node_t* node,  MAC_CMD_Query_t queryType, bool acknowledged)
{
	static MAC_CMD_t queryCmd;
  uint32_t unixSeconds;
  uint32_t milliSeconds;

	queryCmd.hdr.cmdType = MAC_CMD_QUERY;
	queryCmd.hdr.subCmdType = queryType;

	printMAC("Node : %u, Sending Query", node->addr);

	switch (queryType)
	{
    	case MAC_CMD_QUERY_FIRST:
        RTC_getUnixTimeMs(&unixSeconds, &milliSeconds);
        queryCmd.query.first.rtc = unixSeconds;
        queryCmd.query.first.rtcMiliSeconds = milliSeconds;
        queryCmd.query.first.nodeParamCRC = HAL_CRC_Calculate_ByteStream(&hcrc, (uint8_t*)&deviceSettings.DCU_LoRa_Params.node_Params, sizeof(deviceSettings.DCU_LoRa_Params.node_Params), false);
        break;

	case MAC_CMD_QUERY_SETTINGS:
		memcpy(&queryCmd.query.settings, &deviceSettings.DCU_LoRa_Params.node_Params, sizeof(queryCmd.query.settings));
		break;
    
    default:
        break;
	}

	if (acknowledged)
		return MAC_sendCommand(node->addr, MAC_CNTRL_ACKNOWLEDGE | MAC_CNTRL_ACKNOWLEDGED, &queryCmd);

	return MAC_sendCommand(node->addr, MAC_CNTRL_ACKNOWLEDGE, &queryCmd);
}

  uint32_t RTC_UpdateunixSeconds;
  uint32_t RTC_UpdatemilliSeconds;

/**
 * @brief		Load the RTC update command in the transmit frame
 * @details		The RTC command is broadcast to all nodes and the nodes are not
 * 				to respond to this command.
 * @return		#MAC_Return_t
 */
MAC_Return_t MAC_send_RTC_Update_Command()
{
	static MAC_CMD_t rtcCommand;
  
	rtcCommand.hdr.cmdType = MAC_CMD_RTC_UPDATE;
  RTC_getUnixTimeMs(&RTC_UpdateunixSeconds, &RTC_UpdatemilliSeconds);

	rtcCommand.rtc_Update.rtc = RTC_UpdateunixSeconds;
	rtcCommand.rtc_Update.rtcMiliSeconds = RTC_UpdatemilliSeconds;

	return MAC_sendCommand(0, 0, &rtcCommand);
}





/**
 * @brief	Callback for data packet.
 * @return	#MAC_Return_t
 * 
 * @note This function only loads the response into the MAC frame. 
 *       The packet is transmitted by the #MAC_Query_Node() function.
 */
MAC_Return_t MAC_Data_Callback(MAC_Node_t* node, uint8_t dataPort)
{
	
	MAC_Return_t ret = MAC_ERROR;
	bool settingsRequested = false;
	static Node_SensorsData_t sensorNodeData;

	__packed union MAC_Data_t{

		MAC_SensorData_t sensor;
		MAC_ErrorLog_t  errorLog;

	} *dataPacket = (union MAC_Data_t*)MAC_rxPayload;

    if (dataPort == MAC_PORT_SENSOR_DATA)
    {
    	if (dataPacket->sensor.header.hasPayload)
    	{
    		Sensors_decompressData(dataPacket->sensor.data, &sensorNodeData.sensorData);
    		//memcpy(&sensorNodeData.sensorData, &dataPacket->sensor.data, sizeof(sensorNodeData.sensorData));
            sensorNodeData.nodeID = node->addr;
            Sensors_savePacket(&sensorNodeData);  //.sensorData
    	}

    	settingsRequested = dataPacket->sensor.header.requestSettings;
    }
    else if(dataPort == MAC_PORT_ERROR_LOG)
    {
    	if (dataPacket->errorLog.header.hasPayload)
    		LOG_NodeError(MAC_rxHeader->senderAddr, (LOG_ErrorPacket_t*)&dataPacket->errorLog.log);

    	settingsRequested = dataPacket->errorLog.header.requestSettings;
    }
    else
        return MAC_ERROR;
    

    if (settingsRequested)
    {
    	ret = MAC_sendQuery(node, MAC_CMD_QUERY_SETTINGS, true);
    }
    else if (MAC_rxHeader->frameHeader.controlFlags & MAC_CNTRL_MORE_DATA)
	{
		ret = MAC_sendQuery(node, MAC_CMD_QUERY_NO_PAYLOAD, true);
	}
	else if (MAC_rxHeader->frameHeader.controlFlags & MAC_CNTRL_ACKNOWLEDGE)
	{
		ret = MAC_sendACK(node->addr, MAC_CNTRL_ACKNOWLEDGED);
	}
	//node has more data. send query again.
	return ret;
}


/**
 *
 * @param node
 * @return
 */
MAC_Return_t MAC_ACK_Callback(MAC_Node_t* node)
{
    MAC_Return_t ret = MAC_ERROR;
    
    if (MAC_rxHeader->frameHeader.controlFlags & MAC_CNTRL_MORE_DATA)
    {
        ret = MAC_sendQuery(node, MAC_CMD_QUERY_NO_PAYLOAD, false);
    }
    else if (MAC_rxHeader->frameHeader.controlFlags & MAC_CNTRL_ACKNOWLEDGE)
        ret = MAC_sendACK(node->addr, MAC_CNTRL_ACKNOWLEDGED);

    return ret;
}

/**
 * @brief		Check if the DCU should query the specified node in the given slot.
 * @details		The DCU should query the node if:
 * 				@li	The current time slot is the time slot assigned to the node
 * 				@li And The current time slot is not a firmware update slot
 * 				@li Or if the slot is spaced 20 slots apart and the node has not responded
 * 					for the last two scheduled slots.
 * @param node	Node to query
 * @param slot	Time slot
 * @return
 * 		@arg true	if the DCU should query the node in the given slot
 * 		@arg false  if the DCU should not query the node.
 */
bool MAC_isNodeQuerySlot(MAC_Node_t* node, uint32_t slot)
{
    uint32_t assignedSlot = MAC_getAssignedSlot(node->addr);
    if (assignedSlot == slot)
    {
        return true;
    }
    else if (MAC_isFirmwareUpdateSlot(slot))
    {
        return false;
    }
//    else if ((node->querySlotErrorCount >= MAC_QUERY_SLOT_ERRORS_MAX) &&
//             ((abs((int32_t)assignedSlot - (int32_t)slot)%MAC_ERROR_REQUERY_SLOTS) == 0))
//    {
//        return true;
//    }
        
    return false;
}

/**
 *
 * @param node
 * @return
 */
MAC_Return_t MAC_Query_Node_Error_Callback(MAC_Node_t* node)
{
    
    if (node->querySlotErrorCount < MAC_QUERY_SLOT_ERRORS_MAX)
    {
        node->querySlotErrorCount++;
    }
    
    return MAC_OK;
}

/**
 *
 * @param node
 * @param snrLog
 * @return
 */
MAC_Return_t MAC_Query_Node_Success_Callback(MAC_Node_t* node, SNR_Log_Struct* snrLog)
{
    node->querySlotErrorCount = 0;
    MAC_log_SNR(node->addr, snrLog);
    return MAC_OK;
}


/**
 * @brief 		Query the specified node for any data.
 * @param node	Pointer to node in table that is to be queried.
 * @return
 */
MAC_Return_t MAC_Query_Node(MAC_Node_t* node, uint32_t timeLeftInSlot, SNR_Log_Struct* nodesSNR)
{
	MAC_Return_t ret = MAC_ERROR;
    uint32_t ticks = 0;
    
	//Clear transmit buffer 
    MAC_status.hasFrameInBuffer = false;

    MAC_Port_closeAll();
    MAC_Port_open(MAC_PORT_SENSOR_DATA);
    MAC_Port_open(MAC_PORT_ERROR_LOG);
    MAC_Port_open(MAC_PORT_FIRMWARE_UPDATE);
    MAC_Port_open(MAC_PORT_NONE);
    
    LoRa_setGain(MAC_RADIO_CONFIG_QUERY(deviceSettings.DCU_LoRa_Params.networkID));
	//transmit frame and wait for reply from node.
    ret = MAC_sendQuery(node, MAC_CMD_QUERY_FIRST, false);
	if (ret != MAC_OK) goto error;

	while (timeLeftInSlot > 0)
	{
        ticks = osKernelSysTick();
        //Incase no frame has been loaded into the transmit buffer by 
        //any of the callbacks, but the node requires an acknowledgment, 
        //load a query packet into the tx buffer. 
        //This condition will arise only when the node has more data to send
        //We need not worry about old packets in the rx buffer, because, 
        //before the start of the loop, a frame will always be loaded into the 
        //tx Buffer and subsequent executions of this condition will occur only
        //in case a packet was received successfully.
        if (!MAC_status.hasFrameInBuffer && 
            (MAC_rxHeader->frameHeader.controlFlags & MAC_CNTRL_ACKNOWLEDGE))
        {
            ret = MAC_sendQuery(node, MAC_CMD_QUERY_NO_PAYLOAD, true);
            if (ret != MAC_OK) goto error;
        }
		//Check if any of the callbacks have loaded a new packet in to the
		//frame buffer.
		if (MAC_status.hasFrameInBuffer)
		{
            if (timeLeftInSlot <= MAC_RX1_WINDOW + 2*MAC_RX1_DELAY + 1000)
            {
                MAC_status.hasFrameInBuffer = false;
                ret = MAC_sendACK(node->addr, MAC_CNTRL_ACKNOWLEDGED);
            }
            
      printTask("MAC transmiting Frame, Node : %u,\n", node->addr);
			ret = MAC_transmitFrame(MAC_RX1_DELAY);
			if (ret != MAC_OK) goto error;
		}
		else
		{
			ret = MAC_OK;
            break;
		}

		//If the device is not expecting a response to the
		//transmitted packet, the query cycle is complete.
		if (!MAC_status.isWaitingForReply)
		{
			ret = MAC_OK;
            break;
		}

    printTask("Waiting for reply\n");
		ret = MAC_WaitForPacket(MAC_RX1_WINDOW + MAC_RX1_DELAY, true) ;
    printTask("Waiting End\n");

		if (ret != MAC_OK) goto error;
        nodesSNR->SNRsum += LoRa_getSNR();
        nodesSNR->RSSIsum += LoRa_getRSSI();
        nodesSNR->numOfPackets++;

        node->querySlotErrorCount = 0;
            
    osDelay(1000);

    printTask("Reply Received, Node : %u,\n", node->addr);
		switch (MAC_rxHeader->frameHeader.port)
		{

		case MAC_PORT_SENSOR_DATA:

			ret = MAC_Data_Callback(node, MAC_PORT_SENSOR_DATA);
			break;

		case MAC_PORT_ERROR_LOG:

			ret = MAC_Data_Callback(node, MAC_PORT_ERROR_LOG);
			break;

		case MAC_PORT_FIRMWARE_UPDATE:

			ret = MAC_Firmware_Update_Callback(node, (MAC_Firmware_Packet_t*)MAC_rxPayload, 0);
			break;

    case MAC_PORT_NONE:
      if (MAC_rxHeader->frameHeader.controlFlags & MAC_CNTRL_ACKNOWLEDGED)
      {
          if (MAC_rxHeader->frameHeader.controlFlags & MAC_CNTRL_ACKNOWLEDGE)
          {
            ret = MAC_ACK_Callback(node);
          }
          break;
      }
      else
      {
          ret = MAC_ERROR;
          goto error;
      }
    default:
			ret = MAC_ERROR;
          goto error;

		}
        
		if(ret != MAC_OK) goto error;
        
        timeLeftInSlot -= osKernelSysTick() - ticks;
  }
    
    error:
    MAC_Port_closeAll();
    if(ret != MAC_OK)
        node->queryErrorCount++;
    
    return ret;
}



mQueue_Def( queryNodesQueue, 
               MAC_Node_t*, 
               MAC_NODES_IN_SLOT_MAX * (MAC_SLOT_DURATION / MAC_ERROR_REQUERY_SLOTS), 
               mQueue_ifc);

void thread_MACSendQuery(const void* param)
{

    //TODO : Add watchdog for this thread.

    static uint32_t MAC_unixTime       = 0;
    static uint32_t currSlot           = 0;
    static uint32_t prevSlot           = 0;
    static uint32_t secondsToNextSlot  = 0;
    static int32_t  timeLeftInSlot     = 0;
    static uint32_t ticks              = 0;
    MAC_Node_t* 	node			   = NULL;
    MAC_Return_t	ret;
    SNR_Log_Struct  snrLog;

    MAC_Init();

    /*
        find the current time slot and check if there are any nodes alloted to that time slot.
        query those nodes and keep track of time slot so that if it changes during query, nodes in
        in the next slot should be queried.


    */

    /*
    Query protocol : send query and wait for reply. Expected reply from node is FRAME_TYPE_DATA
                     The node will reply with FRAME_TYPE_DATA if it has data to send
                     or FRAME_TYPE_ACK if there is no data in the queue right now.
                     If MAC_CNTRL_MORE_DATA set in header, query for more data else send ACK to node and move to next one.

    */

    for (;;)
    {
        
        MAC_pullNetworkAddresses();
        
        MAC_unixTime = RTC_getUnixTime();
        secondsToNextSlot = ((MAC_unixTime/MAC_SLOT_DURATION) + 1)*MAC_SLOT_DURATION;
        secondsToNextSlot = secondsToNextSlot - MAC_unixTime;
        printMAC("currSlot : %d \n Unix Time : %d", currSlot, RTC_getUnixTime());

        //Wait for radio mutex. If the mutex is not received in this time interval,
    	//there is no point waiting,
        if (osMutexWait(hmutex_MACRadio, secondsToNextSlot*900) != osOK)
            continue;

        printMAC("Mutex : MACRadio Taken");


        currSlot = ((RTC_getUnixTime())%(MAC_REQUERY_TIME))/MAC_SLOT_DURATION;
        

        if (currSlot != prevSlot)
        {
            prevSlot = currSlot;
           
            memset(&snrLog, 0, sizeof(snrLog));

            //Add nodes in the current slot to the query queue.
            for (int i = 0 ; i < MAC_NODES_IN_SLOT_MAX ; i++)
            {   
                if (MAC_status.connectedNodes[currSlot][i].addr != 0)
                {
                    node = &MAC_status.connectedNodes[currSlot][i];
                    //insert nodes in the current slot first into the queue so they are 
                    //processed first. 
                    node->queryErrorCount = 0;
                    node->timeoutLog = false;
                    mQueue_push(&queryNodesQueue, (void*)&node);
                    
                }
            }
            
            //Add all nodes in the query queue except those whose actual slot this is.
            for (int i = 0 ; i < MAC_NODES_IN_SLOT_MAX ; i++)
        	{
        		for (int j = (MAC_TIME_SLOTS_MAX/2) - 1 ; j >= 0 ; j--)
        		{
        			if (MAC_status.connectedNodes[j][i].addr != 0 &&
                        MAC_getAssignedSlot(MAC_status.connectedNodes[j][i].addr) != currSlot &&
        				MAC_isNodeQuerySlot(&MAC_status.connectedNodes[j][i], currSlot))
        			{
                        node = &MAC_status.connectedNodes[j][i];
                        //Nodes that are not responding will only be queried once by the DCU
                        //if this is not their assigned time slot.
                        node->queryErrorCount = MAC_QUERY_ERRORS_MAX - 1;       
                        mQueue_push(&queryNodesQueue, (void*)&node);
        			}
        		}
        	}

            osDelay(1000);
            //compute the time left in the slot for queries.
            timeLeftInSlot = RTC_milliSecondsTillMark( MAC_SLOT_DURATION );

            

            //Do not start communication if less then 5 seconds remaining in slot,
            //so that it does not carry over to the next window.
            while (mQueue_get_Count(&queryNodesQueue) > 0 && timeLeftInSlot > 5*1000)
            {
            	mQueue_pop(&queryNodesQueue, &node);
                
                //If the node is not to be queried in the current slot, 
                //removed the node from the queue. Count it as an error, since
                //the node should not have been in the queue in the first place.
                if (!MAC_isNodeQuerySlot(node, currSlot))
                {
                    MAC_Query_Node_Error_Callback(node);
                    continue;
                }
                
                
                ticks = osKernelSysTick();
                //wait until communication has ended.
               
                ret = MAC_Query_Node(node, timeLeftInSlot, &snrLog);
                if (ret == MAC_OK)
                {
                    printDEBUG("Node : %u, Query Complete\n", node->addr);
                    MAC_Query_Node_Success_Callback(node, &snrLog);
                }
                else 
                {
                  if (snrLog.numOfPackets > 0)
                  {                
                    MAC_log_SNR(node->addr, &snrLog);
                  }

                    
                    if ( node->queryErrorCount >= MAC_QUERY_ERRORS_MAX )
                    {
                        //Move to next node and do not add this node back into the query queue.
                        MAC_Query_Node_Error_Callback(node);           
                    }
                    else
                        mQueue_push(&queryNodesQueue, &node);
                    
                    if (ret == MAC_TIMEOUT)
                    {
                        printTask("\t>> No Reply From       Node : %u,\n", node->addr);
                        if(node->timeoutLog != true){
                          LOG_Error(LOG_ErrorSource_Radio, LOG_RadioError_QueryRxTimeout, node->addr);
                          node->timeoutLog = true;
                        }
                    }
                    else
                    {
                        printTask("\t>> Error querying      Node : %u\n",node->addr);
                        //LOG_Error(LOG_ErrorSource_Radio, LOG_RadioError_QueryTxFailed, 0);
                    } 
                }

                
                node->bandwidth =  MAC_RADIO_cofigQuery[deviceSettings.DCU_LoRa_Params.networkID].bandwidth; 
                node->spreadFactor = MAC_RADIO_cofigQuery[deviceSettings.DCU_LoRa_Params.networkID].spreadFactor;  

                osDelay(10);
                timeLeftInSlot -= osKernelSysTick() - ticks;
                ticks = osKernelSysTick();

            }
            
        }

        //go to sleep until beginning of the next slot.
        osMutexRelease(hmutex_MACRadio);
        printMAC("Mutex : MACRadio Released");
        
        currSlot = ((RTC_getUnixTime())%(MAC_REQUERY_TIME))/MAC_SLOT_DURATION;
        if (currSlot != prevSlot)
            continue;
        

        osMutexWait(hmutex_MACRadio, osWaitForever);

        if (MAC_isNodeMissing())
        {             
            uint32_t OldSeconds = 0;
            uint32_t OldMilliSeconds = 0;
            RTC_getUnixTimeMs(&OldSeconds, &OldMilliSeconds);
            struct tm* dateTimeOld = localtime(&OldSeconds); 
            printDEBUG("Time Brodcasting on RF %02d:%02d:%02d.%03d \n", dateTimeOld->tm_hour, dateTimeOld->tm_min, dateTimeOld->tm_sec, OldMilliSeconds);

            //The RTC update command is to be transmitted using the default
            //bandwidth and spread factor settings.
            LoRa_setGain(MAC_RADIO_CONFIG_BEACON(deviceSettings.DCU_LoRa_Params.networkID));
            ret = MAC_send_RTC_Update_Command();
            if (ret == MAC_OK)
              MAC_transmitFrame(MAC_RX1_DELAY);
        }

        osMutexRelease(hmutex_MACRadio);


        RTC_waitTillMark( MAC_SLOT_DURATION, MAC_SLOT_DURATION, NULL);

    }

}




/** @}*/
