/**
 * @file   	MAC.c
 * @author 	Hassan
 * @version	
 * @date	May 12, 2016
 * 
 * @brief   
 */

#include "MAC.h"
#include "mRTC.h"
#include "log.h"

#include "DEBUG_UART.h"

/**
 * @defgroup MAC_Private_Defines			MAC Private Defines
 * @{
 */

#define MAC_FRAME_SIZE_MAX					((uint32_t)256)


/** @}*/

/**
 * @defgroup MAC_Private_Macros				MAC Private Macros
 * @{
 */


/** @}*/



/**
 * @defgroup MAC_Private_Members			MAC Private Members
 * @{
 */
MAC_Status_t MAC_status;							/*!< MAC status handle*/

uint8_t MAC_frame[MAC_FRAME_SIZE_MAX];		/*!< Buffer for storing data to be transmitted.*/

extern uint8_t LoRa_Buffer[];       /*!< LoRa Rx buffer defined in LoRa.c */

MAC_Header_t* MAC_txHeader = (MAC_Header_t*)MAC_frame;									/*!< Pointer to header of packet to be transmitted.*/
MAC_Header_t* MAC_rxHeader = (MAC_Header_t*)(LoRa_Buffer);					/*!< Pointer to header of recieved packet*/
uint8_t* MAC_txPayload     = (uint8_t*)(MAC_frame + sizeof(MAC_Header_t));				/*!< Pointer to transmit payload*/
uint8_t* MAC_rxPayload     = (uint8_t*)(LoRa_Buffer + sizeof(MAC_Header_t));	/*!< Pointer to receive payload*/


/* Stores the handle of the task that will be notified when the
transmission is complete. */
TaskHandle_t xTaskRadioIRQ = NULL;


uint8_t MAC_portStatus[1 + MAC_PORT_COUNT()/8];


/** @}*/


/**
 * @defgroup MAC_Public_Members			MAC Public Members
 * @{
 */

MAC_RADIO_CONFIG_PAIR_t MAC_RADIO_cofigBeacon[] = MAC_RADIO_CONFIG_PAIR;
MAC_RADIO_CONFIG_PAIR_t MAC_RADIO_cofigQuery[]  = MAC_RADIO_CONFIG_PAIR;
MAC_RADIO_CONFIG_PAIR_t MAC_RADIO_cofigCmd[]    = MAC_RADIO_CONFIG_PAIR;
MAC_RADIO_CONFIG_PAIR_t MAC_RADIO_cofigData[]   = MAC_RADIO_CONFIG_PAIR;



/** @}*/


/**
 * @defgroup MAC_Private_Functions			MAC Private Functions
 * @{
 */

/**
 *
 * @param cmd
 * @return
 */
uint8_t getQueryCommandSize(MAC_CMD_t* cmd)
{
	switch (cmd->hdr.subCmdType)
	{
	case MAC_CMD_QUERY_NO_PAYLOAD:
		return 0;

	case MAC_CMD_QUERY_FIRST:
		return sizeof(cmd->query.first);

	case MAC_CMD_QUERY_SETTINGS:
		return sizeof(cmd->query.settings);

	default:
		return 0;
	}

	return 0;
}


#ifdef IS_GATEWAY_NODE
/**
 * @brief			Get the node in the DCU list corresponding to the specified address
 * @param nodeAddr	Address of the node to search for
 * @return			Pointer to node in list if exists
 * 					NULL other wise
 */
MAC_Node_t* getNode(uint8_t nodeAddr)
{
	for (int i = 0 ; i < MAC_NODES_IN_SLOT_MAX ; i++)
	{
		for (int j = (MAC_TIME_SLOTS_MAX/2) - 1 ; j >= 0 ; j--)
		{
			if (MAC_status.connectedNodes[j][i].addr == nodeAddr)
			{
				return &MAC_status.connectedNodes[j][i];
			}
		}
	}

	return NULL;
}


/**
 * @brief		Check if the packet is intended for this device
 * @return		#MAC_Return_t
 */
bool MAC_acceptPacket()
{
	if (MAC_rxHeader->destAddr != deviceSettings.DCU_LoRa_Params.networkID)
		return false;

	if (getNode(MAC_rxHeader->senderAddr) == NULL)
		return false;

    if (MAC_rxHeader->frameHeader.controlFlags & MAC_CNTRL_DOWNLINK)
        return false;


	return true;
}

#else

/**
 * @brief		Check if the packet is intended for this device
 * @return		#MAC_Return_t
 */
bool MAC_acceptPacket()
{
	if (MAC_rxHeader->nodeAddr != deviceSettings_crc.networkID &&
		 MAC_rxHeader->nodeAddr != 0)
		return false;

	if (MAC_rxHeader->dcuAddr != deviceSettings_crc.DcuID)
		return false;

    if (!(MAC_rxHeader->frameHeader.controlFlags & MAC_CNTRL_DOWNLINK))
        return false;

	return true;
}

#endif

/**
 * @brief		  	Load a frame into the transmit buffer.
 * @param header	Pointer to header structure of the new frame.
 * @param payload	Pointer to payload of the new frame.
 * @param length	Length of payload.
 * @return			#MAC_Return_t
 */
MAC_Return_t MAC_loadFrame(MAC_Header_t * header, uint8_t* payload, uint8_t length)
{

	///The function returns with an error if:
	if (MAC_status.hasFrameInBuffer)
	{
        return MAC_FRAME_BUFFER_IN_USE;		///@li there is already a frame in the buffer.
	}
	else if (MAC_status.isWaitingForReply)
	{
		return MAC_WAITING_FOR_REPLY;		///@li Node is connected to the network and is waiting for a reply.
	}

    
    #ifdef IS_GATEWAY_NODE
    SET_BIT(header->frameHeader.controlFlags, MAC_CNTRL_DOWNLINK);
    header->senderAddr = deviceSettings.DCU_LoRa_Params.networkID;
    #else
    CLEAR_BIT(header->frameHeader.controlFlags, MAC_CNTRL_DOWNLINK);
    header->nodeAddr = deviceSettings_crc.networkID;
    #endif
    
    
	//copy the header into the frame buffer.
	memcpy(MAC_frame, header, sizeof(MAC_Header_t));

	//if payload is not empty
	if (payload != NULL && length)
		memcpy(MAC_frame + sizeof(MAC_Header_t), payload, length);

	MAC_status.hasFrameInBuffer = true;
    MAC_status.txFrameSize = sizeof(MAC_Header_t) + length;
	return MAC_OK;

}

/** @}*/


/**
 * @defgroup MAC_Public_Functions
 * @{
 */

/**
 * @brief	Initialize the MAC interface.
 * @return
 */
MAC_Return_t MAC_Init(void)
{
    #ifdef IS_GATEWAY_NODE
    memset(MAC_status.connectedNodes, 0, sizeof(MAC_status.connectedNodes));
    MAC_status.numConnectedNodes = 0;
    #endif

	if (LoRa_Init() != LoRa_OK)
		return MAC_ERROR;

    LoRa_setGain(MAC_RADIO_CONFIG_BEACON(0));

	return MAC_OK;
}
extern bool IrqFired;

/**
 * @brief	Transmit the frame in the buffer over the radio
 * @return  #MAC_Return_t
 */
MAC_Return_t MAC_transmitFrame(uint32_t delayBeforeTransmit)
{
    MAC_Return_t ret = MAC_ERROR;
    LoRa_Return_t radioReturn;
    int32_t  timeout = 3000;
	///The function checks if the application is awaiting a response
    if (MAC_status.isWaitingForReply)
        return MAC_WAITING_FOR_REPLY;
    
    if (MAC_status.hasFrameInBuffer == false)
        return MAC_ERROR;
    //Delay before transmitting packet.
    osDelay(delayBeforeTransmit);
//    printMAC(" MAC_transmitFrame\n");
    
    //No FSK implementation yet.
//    if (SX127x_handle.modem == SX127x_MODEM_FSK)
//    {
//        radioReturn = SX127x_FSK_TransmitPacket(MAC_frame, MAC_status.txFrameSize);
//    }
//    else if (SX127x_handle.modem == SX127x_MODEM_LORA)
//    {
//        radioReturn = SX127x_TransmitPacket(MAC_frame, MAC_status.txFrameSize);
//    
//    }    
  IrqFired = 0;
  /* Store the handle of the calling task. */
  xTaskRadioIRQ = xTaskGetCurrentTaskHandle();
  xTaskNotifyStateClear( NULL );
  radioReturn = LoRa_TransmitPacket(MAC_frame, MAC_status.txFrameSize);
  if(radioReturn == LoRa_OK)
  {
    ret = MAC_OK;      
  }
  ///@todo : wait for tx done or timeout interrupt before returning.
  uint32_t ulNotificationValue = ulTaskNotifyTake( pdTRUE, timeout );
  
  if( ulNotificationValue == 1 || IrqFired) // intruppt occred
  {
//    printMAC("                              IrqFired\r\n");
    Radio.IrqProcess();
  }
     
  LoRa_sleepMode();  // put lora chip in sleep mode

  if (LoRa_IRQFlagSet( IRQ_TX_DONE ))
  {
    LoRa_clearIRQFlags(  IRQ_TX_DONE );
		///If the frame has the MAC_CNTRL_REPLY_ACK flag set,
		///the radio is set to wait for reply from destination
//    printMAC(" MAC_CNTRL_REPLY_ACK flag %d\n",(MAC_txHeader->frameHeader.controlFlags & MAC_CNTRL_ACKNOWLEDGE));
		if (MAC_txHeader->frameHeader.controlFlags & MAC_CNTRL_ACKNOWLEDGE)
			MAC_status.isWaitingForReply = true;
    else     
      MAC_status.isWaitingForReply = false;


    MAC_status.lastNodeAddr = MAC_txHeader->nodeAddr;
    MAC_status.lastDcuAddr = MAC_txHeader->dcuAddr;
    //Clear transmit buffer.
    MAC_status.hasFrameInBuffer = false;
		ret = MAC_OK;
	}
  else if (LoRa_IRQFlagSet( IRQ_RX_TX_TIMEOUT ))
  {
    printMAC(" Radio Transmission TimeOut\n");
    LoRa_clearIRQFlags( IRQ_RX_TX_TIMEOUT );
		///If the transmission failed, re initialize the radio.
    MAC_Init();
    LoRa_setGain(MAC_RADIO_CONFIG_QUERY(deviceSettings_crc.DcuID));

    ///The transmit buffer is cleared in this case as well, to allow for new frames
    ///to be loaded into the radio.
    MAC_status.hasFrameInBuffer = false;
    MAC_status.isWaitingForReply = false;
    ret =  MAC_ERROR;
  }
	else
	{
    printMAC(" Radio Transmission Failed\n");
    ///If the transmission failed, re initialize the radio.
    MAC_Init();
    LoRa_setGain(MAC_RADIO_CONFIG_QUERY(deviceSettings_crc.DcuID));

    ///The transmit buffer is cleared in this case as well, to allow for new frames
    ///to be loaded into the radio.
    MAC_status.hasFrameInBuffer = false;
    MAC_status.isWaitingForReply = false;
    ret =  MAC_ERROR;
	}
  xTaskRadioIRQ = NULL;
  return ret;
}


/**
 * @brief				Wait for a packet with expected type.
 * @param expectedReply Expected frame type of packet. This can a combination of
 * 						@ref MAC_FrameType_t;
 * @param waitPeriod	Time to wait for response.
 * @param sleepOnExit	Set the radio in sleep mode at the end of operation.
 * @return
 * 		@arg MAC_OK			if a packet was received.
 * 		@arg MAC_ERROR  	if the payload had a CRC error.
 * 		@arg MAC_TIMEOUT	if the receive operation timed out.
 */
MAC_Return_t MAC_WaitForPacket(int32_t waitPeriod, bool sleepOnExit)
{

  //printMAC("\nMAC_WaitForPacket. s\n"); 
  uint32_t ticks; 
  uint32_t ulNotificationValue = 0;
  MAC_Return_t ret = MAC_TIMEOUT;

  MAC_status.rxFrameSize = 0;
  memset(MAC_rxHeader, 0, sizeof(MAC_Header_t));
  //loop until timeout
  IrqFired = 0;
  /* Store the handle of the calling task. */

  while (waitPeriod > 0)
  {

    //if radio is not in receive mode already, set it in receive mode.
    LoRa_receiverMode(0x00, true);

    xTaskRadioIRQ = xTaskGetCurrentTaskHandle();
    xTaskNotifyStateClear( NULL );
    ticks = osKernelSysTick();
    int32_t timeout = waitPeriod;

    /* Wait for the transmission to complete. */
    ulNotificationValue = ulTaskNotifyTake( pdTRUE, timeout );
    if( ulNotificationValue >= 1 || IrqFired) // intruppt occred
    {
      LoRa_IrqHandler();

      if (LoRa_IRQFlagSet(IRQ_CRC_ERROR))
      {
        LoRa_clearIRQFlags(IRQ_CRC_ERROR | IRQ_RX_DONE);

        ret = MAC_ERROR;
        LOG_Error(LOG_ErrorSource_Radio, LOG_RadioError_RxFailed, MAC_txHeader->dcuAddr);
        break;
      }
      else if (LoRa_IRQFlagSet(IRQ_RX_DONE))
      {
        LoRa_clearIRQFlags(IRQ_RX_DONE);

        //                printMAC(" Radio recived\n");
        if (MAC_acceptPacket() && MAC_Port_isOpen((MAC_Port_t)MAC_rxHeader->frameHeader.port))
        {
          ret = MAC_OK;
          ///@todo : get rx frame length.
          MAC_status.rxFrameSize = LoRa_getRxBufferLength();
          break;
        }
        else
        {
          printMAC("d%d, s%d, p%d\n", MAC_rxHeader->nodeAddr, MAC_rxHeader->dcuAddr, MAC_rxHeader->frameHeader.port);
        }

      }
      else
      {
        printMAC("no int flag set %d, %d, %d\n ", LoRa_IRQFlagGet(), ulNotificationValue, IrqFired);
      }
    }
    else
    {
    printMAC("ulNotificationValue %d, IrqFired %d\n ", ulNotificationValue, IrqFired);
    printMAC("RadioStatus.Irq %d\n ", LoRa_IRQFlagGet());
    LoRa_receiverMode(0x00, true);

    }
    waitPeriod -= (osKernelSysTick() - ticks);
    printMAC("remaing waitPeriod %d\n ", waitPeriod);
    continue;

  }
    /* There are no transmissions in progress, so no tasks to notify. */
    xTaskRadioIRQ = NULL;
    
    LoRa_clearIRQFlags( IRQ_RADIO_ALL );
//    printMAC("MAC_WaitForPacket END\n ", waitPeriod);
    MAC_status.isWaitingForReply = false;
    if (sleepOnExit)
    	LoRa_sleepMode();
    return ret;


}


/**
 * @brief		Check if the specified port is open
 * @param port
 * @return
 * 		@arg true	if the port is open
 * 		@arg flags	if the port is closed.
 */
bool MAC_Port_isOpen(MAC_Port_t port)
{
	return (MAC_portStatus[port/8] & (1 << (port%8)));
}

/**
 * @brief			Set the status of the specified port to open
 * @param port
 * @return			MAC_OK
 */
MAC_Return_t MAC_Port_open(MAC_Port_t port)
{
	MAC_portStatus[port/8] |= (1 << (port%8));
	return MAC_OK;
}

/**
 * @brief		Close all ports.
 * @return		MAC_OK
 */
MAC_Return_t MAC_Port_closeAll()
{
	memset(MAC_portStatus, 0, sizeof(MAC_portStatus));
	return MAC_OK;
}


/**
 * @brief				Send the specified data to the DCU in response to query.
 * @param addr			Destination address.
 * @param port			Specifies the application end point. (type of data)
 * @param controlFlags	Control flags for the packet.
 * @param dataBuff		Pointer to data Buffer.
 * @param len			Length of data in bytes.
 * @return
 */
MAC_Return_t MAC_sendData(uint8_t addr, MAC_Port_t port, uint8_t controlFlags, uint8_t* dataBuff, uint8_t len)
{
    MAC_Header_t header;
    
    header.dcuAddr = addr;
    header.frameHeader.port = port;
    header.frameHeader.controlFlags = controlFlags;
    
    return MAC_loadFrame(&header, dataBuff, len);

}

/**
 * @brief				Send command packet to the specified node.
 * @param addr			Destination node address
 * @param controlFlags	Control flags for packet
 * @param cmd			Command structure to be sent as payload. contains command type and data.
 * @return				#MAC_Return_t
 */
MAC_Return_t MAC_sendCommand(uint8_t addr, uint8_t controlFlags, MAC_CMD_t* cmd)
{
    MAC_Header_t header;
    uint8_t length = sizeof(cmd->hdr);

    header.dcuAddr = addr;
    header.frameHeader.port = MAC_PORT_COMMAND;
    header.frameHeader.controlFlags = controlFlags;
    
    switch (cmd->hdr.cmdType)
    {
    case MAC_CMD_RTC_UPDATE:
    	length += sizeof(cmd->rtc_Update);
    	break;

    case MAC_CMD_SET_NETWORK_ADDR:
    	length += sizeof(cmd->set_NetworkAddr);
    	break;

    case MAC_CMD_QUERY:
    	length += getQueryCommandSize(cmd);
    	break;

    default:
    	length += 0;
    	break;
    }

    return MAC_loadFrame(&header, (uint8_t*)cmd, length);
    
}



/**
 * @brief					Load an empty acknowledgment packet into the transmit buffer.
 * @param addr				Destination device network address.
 * @param controlFlags		Control flags for frame header
 * @return					#MAC_Return_t
 */
MAC_Return_t MAC_sendACK(uint8_t addr, uint8_t controlFlags)
{
    MAC_Header_t header;

    header.dcuAddr = addr;
    header.frameHeader.port = MAC_PORT_NONE;
  
    CLEAR_BIT(controlFlags, MAC_CNTRL_ACKNOWLEDGE); // dont need any more packet
    SET_BIT(controlFlags, MAC_CNTRL_ACKNOWLEDGED);
    header.frameHeader.controlFlags = controlFlags;
    
    return MAC_loadFrame(&header, NULL, 0);
}
/** @}*/
