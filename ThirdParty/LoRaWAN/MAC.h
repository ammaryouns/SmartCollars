/**
 * @file   	MAC.h
 * @author 	Hassan
 * @version	
 * @date	May 12, 2016
 * 
 * @brief   
 */

#ifndef MAC_H_
#define MAC_H_

#include <stdbool.h>

#include "SensorDefs.h"
#include "log.h"
//#include "Cloud_Settings_Params.h"
#include "LoRa.h"
#include "sx126x-board.h"

/**
 * @defgroup MAC_Public_Defines			MAC Public Defines
 * @{
 */

#define printMAC(fmt, ...)         printTask(fmt, ##__VA_ARGS__ );
typedef struct {
    RadioLoRaBandwidths_t bandwidth;
    RadioLoRaSpreadingFactors_t spreadFactor;
}MAC_RADIO_CONFIG_PAIR_t;



#define MAC_RADIO_CONFIG_PAIR       {                                                           \
                                    {LORA_BW_500,  LORA_SF12},   \
                                    {LORA_BW_500,  LORA_SF12},   \
                                    {LORA_BW_500,  LORA_SF11},   \
                                    {LORA_BW_500,  LORA_SF10},   \
                                    {LORA_BW_500,  LORA_SF9},    \
                                    {LORA_BW_500,  LORA_SF8},    \
                                    {LORA_BW_500,  LORA_SF7},    \
                                    {LORA_BW_500,  LORA_SF6},    \
                                    }

 
#define MAC_NODES_IN_SLOT_MAX	((uint32_t)1)		/*!< Number of nodes that can assigned a given time slot.*/
#define MAC_TIME_SLOTS_MAX		((uint32_t)360)		/*!< Total number of time slots.*/


#define MAC_NODES_MAX			(MAC_NODES_IN_SLOT_MAX * MAC_TIME_SLOTS_MAX / 2)	/*!< Total number of nodes that can be connected to the network.*/


#define MAC_RX1_DELAY			((uint32_t)200)	/*!< Time delay between loading the frame into buffer and transmitting it.*/
#define MAC_RX1_WINDOW			((uint32_t)3000)	/*!< Time slot for the node to respond after receiving packet.*/


#define MAC_SLOT_DURATION		((uint32_t)10)								/*!< Duration for each time slot in seconds.*/
#define MAC_REQUERY_TIME		(MAC_TIME_SLOTS_MAX * MAC_SLOT_DURATION)	/*!< Total query cycle duration in seconds.*/

#define MAC_ERROR_REQUERY_SLOTS     ((uint32_t)20)
#define MAC_QUERY_SLOT_ERRORS_MAX    ((uint32_t)2)           /*!< Max number of query slots missed before disconnecting from DCU network */
#define MAC_QUERY_ERRORS_MAX        ((uint32_t)8)           /*!< Max number of consecutive communication failures in slot */

#define MAC_BEACON_PERIOD       ((uint32_t)60)          /*!< Normal node registration mode. */
#define MAC_BEACON_PERIOD2      ((uint32_t)5*60)        /*!< To be used when max number of nodes are connected to the DCU. */

#define MAC_ASSERT_RETURN( returnValue, expectedValue )	if (returnValue != expectedValue)\
														{\
															return returnValue;\
														}\

#define MAC_getAssignedSlot(networkID)		((uint32_t)((networkID) - 1))
#define MAC_getNetworkID( NATIndex )        ((uint32_t)((NATIndex) + 1))

                            
                            
#define OVER_THE_AIR_TIME() 3000
#warning "............Implement Over ther Air time calculator                 "
/**
 * @defgroup MAC_Control_Flags		MAC Control Flags
 * @{
 */
                                                        
typedef enum{
 
    MAC_CNTRL_DOWNLINK          = 1<<0,
    MAC_CNTRL_ACKNOWLEDGE       = 1<<1,
    MAC_CNTRL_ACKNOWLEDGED      = 1<<2,
    MAC_CNTRL_MORE_DATA         = 1<<3,   

}MAC_Control_Flags_t;
 
                                                        
/** @}*/




/**
 * @defgroup MAC_Public_Enums			MAC Public Enums
 * @{
 */

/**
 * @brief Frame Port Definitions
 */

#define MAC_PORT_LIST(MAC_PORT) \
MAC_PORT(   NONE,               0x00    )\
MAC_PORT(   COMMAND,            0x01    )\
MAC_PORT(   SENSOR_DATA,        0x02    )\
MAC_PORT(   ERROR_LOG,          0x03    )\
MAC_PORT(   FIRMWARE_UPDATE,    0x04    )\

typedef enum{

    #define MAC_PORT_ENUM_DEF( port_name,   id )    MAC_PORT_##port_name = id,\

    MAC_PORT_LIST(MAC_PORT_ENUM_DEF)

}MAC_Port_t;

#define MAC_PORT_ROLL_CALL(...)		1 +
#define MAC_PORT_COUNT()		(MAC_PORT_LIST(MAC_PORT_ROLL_CALL) + 0)


/**
 * @brief MAC Command Type Definitions
 */
typedef enum{

	MAC_CMD_RTC_UPDATE			= 0x00,
	MAC_CMD_SET_NETWORK_ADDR	= 0x01,
	MAC_CMD_QUERY				= 0x02

}MAC_CMD_Type_t;


/**
 * @brief MAC Command Query Type Definitions
 */
typedef enum{

	MAC_CMD_QUERY_NO_PAYLOAD = 0x00,
	MAC_CMD_QUERY_FIRST		 = 0x01,
	MAC_CMD_QUERY_SETTINGS	 = 0x02

}MAC_CMD_Query_t;



typedef enum{

	MAC_FW_CMD_START					= 0x00,
	MAC_FW_CMD_UPDATE_REQUEST			= 0x01,
	MAC_FW_CMD_AWAIT_FW_UPDATE			= 0x02,
	MAC_FW_CMD_RESTART_FW_UPDATE		= 0x03,
	MAC_FW_CMD_UPDATE_REQUEST_FAILED	= 0x04,
	MAC_FW_CMD_RESEND_LAST_FW_PAGE		= 0x05,
	MAC_FW_DATA							= 0x06

}MAC_Firmware_Packet_Type_t;

/**
 * @brief MAC function return status definitions
 */

/**
 *
 */
typedef enum {

    MAC_OK = 0,               //!< MAC_OK
    MAC_FRAME_SEND_TIMEOUT,   //!< MAC_FRAME_SEND_TIMEOUT
    MAC_FRAME_BUFFER_IN_USE,  //!< MAC_FRAME_BUFFER_IN_USE
    MAC_NETWORK_NOT_JOINED,   //!< MAC_NETWORK_NOT_JOINED
    MAC_MAX_NODES_CONNECTED,  //!< MAC_MAX_NODES_CONNECTED
    MAC_WAITING_FOR_REPLY,    //!< MAC_WAITING_FOR_REPLY
    MAC_FRAME_BUFFER_OVERFLOW,//!< MAC_FRAME_BUFFER_OVERFLOW
    MAC_ERROR,                //!< MAC_ERROR
	MAC_TIMEOUT                  //!< MAC_TIMEOUT

}MAC_Return_t;


/** @}*/

/**
 * @defgroup MAC_Public_Structs 		MAC Public Structures
 * @{
 */

/**
 * @brief
 */
typedef __packed struct 
{
	uint8_t port			: 3,
            controlFlags    : 5;

}MAC_FrameHeader_t;


/**
 * @brief Structure definition for packet header in LoRaWAN.
 */
typedef __packed struct 
{
	uint16_t nodeAddr;
	uint8_t dcuAddr;
	MAC_FrameHeader_t frameHeader;

}MAC_Header_t;


/**
 *
 */
typedef __packed struct 
{
	uint8_t cmdType 	: 5,
			subCmdType	: 3;

}MAC_CMD_Header_t;

/**
 *
 */
typedef __packed struct 
{
	MAC_CMD_Header_t hdr;

	__packed union {

		/**
		*
		*/
		__packed struct 
		{
			uint32_t rtc;
			uint32_t rtcMiliSeconds;
		}
		rtc_Update;

		/**
		*
		*/
		__packed struct {

			uint8_t devUID[12];
			uint8_t nodeNetworkID;
			uint8_t DCUNetworkID;
		}
		set_NetworkAddr;

		/**
		*
		*/
		__packed union {

			__packed struct {

				uint32_t rtc;
				uint32_t rtcMiliSeconds;
				uint32_t nodeParamCRC;

			}
			first;

			__packed struct {

				CLOUD_SETTINGS_NODE_PARAM_LIST(CLOUD_SETTINGS_PARAM_DEF)
			}
			settings;
		}
		query;

	};
}
MAC_CMD_t;

/**
 *
 */
typedef __packed struct{

	__packed union
	{
		__packed struct{
			uint8_t requestSettings			: 1,
					hasPayload				: 1;
		};
		uint8_t flags;
	};

}MAC_SensorData_Header_t, MAC_ErrorLog_Header_t;

/**
 *
 */
typedef __packed struct{

	MAC_SensorData_Header_t header;
	uint8_t data[sizeof(SensorsData_t)];

}
MAC_SensorData_t;

/**
 *
 */
typedef __packed struct{

	MAC_ErrorLog_Header_t header;
	LOG_ErrorPacket_t log;
}
MAC_ErrorLog_t;

typedef __packed struct{

    MAC_Firmware_Packet_Type_t type;
}
MAC_Firmware_Packet_Header_t;

/**
 * @brief
 */
typedef __packed struct MAC_Firmware_Packet_s MAC_Firmware_Packet_t;

typedef __packed struct MAC_Firmware_Packet_s
{
  MAC_Firmware_Packet_Header_t header;

  __packed union{

    __packed struct{

      uint32_t fileCRC;
      uint32_t pageCRC;
      uint16_t version;
      uint32_t fwSize;
      uint32_t offset;
      uint32_t unixTime;
      uint32_t unixTimeMs;

    }
    fwStart, fwRequest;

    __packed struct{

      uint32_t offset;
    }
    fwResendRequest;

    __packed struct{
  
      uint32_t offset;
      uint8_t data[255 - sizeof(MAC_Header_t) - sizeof(MAC_Firmware_Packet_Header_t) - 4];
    }
    fwData;
  };

}
MAC_Firmware_Packet_t;



/**
 * @brief 	Node network information structure.
 */
typedef struct {

  uint16_t    addr;
  uint16_t  	bandwidth;		/*!< SX1276 bandwidth at which the node is operating.*/
  uint16_t  	spreadFactor;	/*!< SX1276 spread factor at which the node is operating.*/
  uint32_t    querySlotErrorCount; /*!< Count of consecutive query cycles missed by node */
  uint32_t    queryErrorCount;     /*!< Count of consecutive communication failures in the current slot. */
  uint8_t     timeoutLog;     /*!< Count of consecutive communication failures in the current slot. */
}MAC_Node_t;





/**
 * @brief MAC status structure definition
 */
typedef struct {

	bool     isNetworkJoined;		/*!< Indicates whether node is connected to the network.
	 	 	 	 	 	 	 	 	 	 @note For DCU, this should always be true.*/
	bool     hasFrameInBuffer;		/*!< There is data present in the tx Buffer waiting to be transmitted.*/
	bool     isWaitingForReply;		/*!< Node is waiting for reply from another node */
	bool     hasDataInBuffer;		/*!< a packet has been received.*/
	bool     firsRun;		/*!< a packet has been received.*/
    
    uint8_t   rxFrameSize;          /*!< Size of the packet received*/
    uint8_t   txFrameSize;  		/*!< Size of the packet in the transmit frame*/

    #ifdef IS_GATEWAY_NODE
    uint8_t    	numConnectedNodes;		/*!< Number of nodes connected. DCU param.*/
    MAC_Node_t 	connectedNodes[MAC_TIME_SLOTS_MAX][MAC_NODES_IN_SLOT_MAX];	/*!< Table for connected nodes. DCU param.*/

    #else
    MAC_Node_t 	thisNode;	        /*!< TNode MAC param.*/
    uint8_t  timeSlot;				/*!< Time slot assigned to node. Sensor Node param.*/
    uint8_t  querySlotErrorCount;
	#endif

  uint32_t lastNodeAddr;	/*!< ID of the destination of last packet transmitted.*/
  uint32_t lastDcuAddr;	/*!< ID of the destination of last packet transmitted.*/
}MAC_Status_t;
/** @}*/

extern TaskHandle_t xTaskRadioIRQ;
extern MAC_Status_t MAC_status;
extern MAC_Header_t* MAC_txHeader;
extern MAC_Header_t* MAC_rxHeader;
extern uint8_t* MAC_txPayload;
extern uint8_t* MAC_rxPayload;
extern MAC_RADIO_CONFIG_PAIR_t MAC_RADIO_cofigBeacon[];
extern MAC_RADIO_CONFIG_PAIR_t MAC_RADIO_cofigQuery[];
extern MAC_RADIO_CONFIG_PAIR_t MAC_RADIO_cofigCmd[];
extern MAC_RADIO_CONFIG_PAIR_t MAC_RADIO_cofigData[];

#define MAC_RADIO_CONFIG_BEACON(id)     MAC_RADIO_cofigBeacon[id % 10].bandwidth,  \
                                        MAC_RADIO_cofigBeacon[id % 10].spreadFactor
#define MAC_RADIO_CONFIG_QUERY(id)      MAC_RADIO_cofigQuery[id % 10].bandwidth,   \
                                        MAC_RADIO_cofigQuery[id % 10].spreadFactor                                  
#define MAC_RADIO_CONFIG_FW_CMD(id)     MAC_RADIO_cofigCmd[id % 10].bandwidth,     \
                                        MAC_RADIO_cofigCmd[id % 10].spreadFactor                                      
#define MAC_RADIO_CONFIG_FW_DATA(id)    MAC_RADIO_cofigData[id % 10].bandwidth,    \
                                        MAC_RADIO_cofigData[id % 10].spreadFactor


#ifdef __cplusplus
extern "C"{
#endif 

MAC_Return_t MAC_Init(void);
    
MAC_Return_t MAC_transmitFrame      (uint32_t delay);
MAC_Return_t MAC_WaitForPacket      (int32_t waitPeriod, bool sleepOnExit);

MAC_Return_t MAC_sendData           (uint8_t addr, MAC_Port_t port, uint8_t controlFlags, uint8_t* dataBuff, uint8_t len);
MAC_Return_t MAC_sendCommand        (uint8_t addr, uint8_t controlFlags, MAC_CMD_t* cmd);
MAC_Return_t MAC_sendACK			(uint8_t addr, uint8_t controlFlags);

bool 		 MAC_Port_isOpen	(MAC_Port_t port);
MAC_Return_t MAC_Port_open		(MAC_Port_t port);
MAC_Return_t MAC_Port_closeAll	(void);

#ifdef __cplusplus
}
#endif

#endif /* MAC_H_ */
