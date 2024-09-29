/**
 * @file   	SensorDefs.c
 * @author 	Hassan
 * @version	
 * @date	Mar 23, 2016
 * 
 * @brief   
 */

#include <SensorDefs.h>
#include "DEBUG_UART.h"
#include "Device_Params.h"

#define SAMPLE_TIME( index, sample_time )       [index] = sample_time*60,\

static const uint32_t sample_time_lut[] = { SAMPLE_TIME_LIST };
#undef SAMPLE_TIME


mQueue_List_Def( sensors_Queue_List );

uint32_t Sensors_compressNodeData( const Node_SensorsData_t* nodeSensorData, uint8_t* buffer);


/**
 * @brief	Get total number of data packets currently stored in queue.
 * @return	Numer of data packet.
 */
uint32_t Sensors_get_PacketCount()
{
    return mQueue_List_get_elementCount(&sensors_Queue_List);
}

/**
 * @brief			Read first packet in data queue.
 * @param packet[out]	Buffer to store the read data packet.
 * @return			#SensorsReturn_t
 */
__inline SensorsReturn_t Sensors_readPacket(void* packet)
{
	return Sensors_readPacketAt(0, packet);
}

/**
 * @brief			Read sensor data packet at the specified offset in the queue.
 * @param index			Offset of packet from start of queue.
 * @param packet[out]	Buffer to store the read data packet.
 * @return			#SensorsReturn_t
 */
__inline SensorsReturn_t Sensors_readPacketAt(uint32_t index, void* packet)
{
	if (mQueue_List_peekAt_Back(&sensors_Queue_List, index, packet) != mQueue_OK)
        return SENSORS_ERROR;
    
    return SENSORS_OK;
}


uint32_t Sensors_getPostString(uint8_t* buffer, const Node_SensorsData_t* packet, uint8_t* numPackets)
{
	//const uint8_t* iter = buffer;

    //SENSORS_LIST(SENSOR_POST_STRING, packet, iter, numPackets);
    //return (iter - buffer);
    
    uint8_t len = Sensors_compressNodeData(packet, buffer);
    (*numPackets)++;
    
    return len;
}


/**
 * @brief		Save the data packet in the queue.
 * @param data	Pointer to data packet to be saved.
 * @return		#SensorsReturn_t
 */
__weak SensorsReturn_t Sensors_savePacket(void* packet)
{
    SensorsData_t* sensorData = (SensorsData_t*)packet;
    Node_SensorsData_t* nodeSensorData = (Node_SensorsData_t*)packet;
    
    if(sensorData->flags.all)
    {
        printSensorPacket("*******************************************\n\n");
        printSensorPacket("Unix Time %d\t Node %u\n", sensorData->timeStamp, nodeSensorData->nodeID);
        if(sensorData->flags.battery)
        {
            printSensorPacket("battery %.02f%%\n", ((float)sensorData->battery)/100);
        }
        if(sensorData->flags.temperature)
        {
            printSensorPacket("temperature %.02f\n", ((float)sensorData->temperature)/100);
        }
        if(sensorData->flags.activity)
        {
            printSensorPacket("activity %d %d %d %d \n", sensorData->activity[0], 
                                                  sensorData->activity[1], 
                                                  sensorData->activity[2], 
                                                  sensorData->activity[3]);
        }
        if( sensorData->flags.SDCARD)
        {
          printSensorPacket("SDCARD ID ->%d - %dV( %d%%) - DataPoints %d \n",  
                                                  sensorData->SDCard.DviceID, 
                                                  sensorData->SDCard.BatteryVoltage, 
                                                  sensorData->SDCard.BatterySOC,
                                                  sensorData->SDCard.DataPoints);
        }  
        if( sensorData->flags.BEHAVIOUR)
        {
          printSensorPacket("walkingBehavior ID ->%d -\n%d\n%d\n%d\n%d\n%d\n%d \n", nodeSensorData->nodeID,
                                                  sensorData->BEHAVIOUR[0].resting, 
                                                  sensorData->BEHAVIOUR[0].rumination, 
                                                  sensorData->BEHAVIOUR[0].eating,
                                                  sensorData->BEHAVIOUR[0].moving,
                                                  sensorData->BEHAVIOUR[0].walking,
                                                  sensorData->BEHAVIOUR[0].steps);
        }  
        if( sensorData->flags.BEHAVIOUR & 1<<1)
        {
          printSensorPacket("walkingBehavior ID ->%d -\n%d\n%d\n%d\n%d\n%d\n%d \n", nodeSensorData->nodeID,
                                                  sensorData->BEHAVIOUR[1].resting, 
                                                  sensorData->BEHAVIOUR[1].rumination, 
                                                  sensorData->BEHAVIOUR[1].eating,
                                                  sensorData->BEHAVIOUR[1].moving,
                                                  sensorData->BEHAVIOUR[1].walking,
                                                  sensorData->BEHAVIOUR[1].steps);
        }    
        if( sensorData->flags.BEHAVIOUR & 1<<2)
        {
          printSensorPacket("walkingBehavior ID ->%d -\n%d\n%d\n%d\n%d\n%d\n%d \n", nodeSensorData->nodeID,
                                                  sensorData->BEHAVIOUR[2].resting, 
                                                  sensorData->BEHAVIOUR[2].rumination, 
                                                  sensorData->BEHAVIOUR[2].eating,
                                                  sensorData->BEHAVIOUR[2].moving,
                                                  sensorData->BEHAVIOUR[2].walking,
                                                  sensorData->BEHAVIOUR[2].steps);
        }   
        if( sensorData->flags.BEHAVIOUR & 1<<3)
        {
          printSensorPacket("walkingBehavior ID ->%d -\n%d\n%d\n%d\n%d\n%d\n%d \n", nodeSensorData->nodeID,
                                                  sensorData->BEHAVIOUR[3].resting, 
                                                  sensorData->BEHAVIOUR[3].rumination, 
                                                  sensorData->BEHAVIOUR[3].eating,
                                                  sensorData->BEHAVIOUR[3].moving,
                                                  sensorData->BEHAVIOUR[3].walking,
                                                  sensorData->BEHAVIOUR[3].steps);
        }  
        if(sensorData->flags.DevID)
        {
          printSensorPacket("DevID:%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n", 
                                sensorData->Data[0], sensorData->Data[1], sensorData->Data[2], sensorData->Data[3],
                                sensorData->Data[4], sensorData->Data[5], sensorData->Data[6], sensorData->Data[7],
                                sensorData->Data[8], sensorData->Data[9], sensorData->Data[10], sensorData->Data[11]);
        }
    }
    printSensorPacket("\n\n\n\n");

    if (mQueue_List_push(&sensors_Queue_List, packet) != mQueue_OK)
        return SENSORS_ERROR;
    
    return SENSORS_OK;
}


/**
 * @brief				Save the sensor data with the node ID in the queue.
 * @param sensorsData	Pointer to data packet to be saved.
 * @param nodeID		node network ID
 * @return				#SensorsReturn_t
 *
 * @note	The queues in the sensors_Queue_List should be initialized with
 * 			the element type Node_SensorsData_t if this function is to be used.
 */
__weak SensorsReturn_t Sensors_saveNodePacket(SensorsData_t* sensorsData, uint8_t nodeID)
{
    static Node_SensorsData_t nodeSensorsData = {0};
    
    memcpy(&nodeSensorsData.sensorData, sensorsData, sizeof(SensorsData_t));
    nodeSensorsData.nodeID = nodeID;
    
    return Sensors_savePacket(&nodeSensorsData);
}



/**
 * @brief 		Remove the specified number of packets from the queue.
 * @param count Number of packets to be removed from the queue.
 * @return		#SensorsReturn_t
 */
__inline SensorsReturn_t Sensors_clearPackets(uint32_t count)
{
	if (mQueue_List_PopMultiple_Back(&sensors_Queue_List, count) != mQueue_OK)
        return SENSORS_ERROR;
    
    return SENSORS_OK;
}

uint32_t Sensors_compressNodeData( const Node_SensorsData_t* nodeSensorData, uint8_t* buffer)
{
	*((uint8_t*)buffer) = nodeSensorData->nodeID;
	
    return Sensors_compressData(&nodeSensorData->sensorData, buffer + 1) + 1;
}

/**
 * @brief				Compresses the sensor data structure by removing
 * 						sensor data values that are not present.
 *
 * @param sensorData	Sensor data structure to be compressed
 * @param buffer[out]	Byte buffer to store the compressed data.
 * @return				Size of structure after compression in buffer.
 */
uint32_t Sensors_compressData(const SensorsData_t* sensorData, uint8_t* buffer)
{

  /**
	 * @details The compressed structure format is as follows:
	 * 		Feild			|	Size in Bytes
	 * 		Time Stamp		|		4
	 * 		Flags			|		2
	 * 		Battery			|		2	(if Battery flag set)
	 * 		Temperature		|		2	(if temperature flag set)
	 * 		Activity		|		2*4 ( 4 values of 2 bytes, if activity flag set)
	 */
    uint8_t* iter = buffer;
	*((uint32_t*)iter) = sensorData->timeStamp;
	iter += 4;

	*((uint16_t*)iter) = sensorData->flags.all;
  iter += 2;

  if(sensorData->flags.DevID == true)
  {
    for(int i = 0; i < 12; i++)
    {
      iter[i] = sensorData->Data[i];
    }
    iter+=12;
  }
  else if (sensorData->flags.SDCARD == true)
  {
    for(int i = 0; i < sizeof(SDCard_datalog_t); i++)
    {
      iter[i] = sensorData->Data[i];
    }
    iter+=sizeof(SDCard_datalog_t);
  }  
//  else if (sensorData->flags.walkingBehavior == true)
//  {
//    for(int i = 0; i < sizeof(WalkingBehavior_t); i++)
//    {
//      iter[i] = sensorData->Data[i];
//    }
//    iter+=sizeof(WalkingBehavior_t);
//  }
  else
  {
    SENSORS_LIST(SENSORS_COMPRESS_PACKET, sensorData, iter);
  }

	return (iter - buffer);

}


/**
 * @brief					Decompress the sensor data from buffer in to the structure that was
 * 							compressed by #Sensors_compressData() function.
 * @param buffer			Buffer containing packed Sensor data structure. See #Sensors_compressData()
 * 							for details.
 * @param sensorData[out]	Structure for storing decompressed data.
 * @return					SENSORS_OK
 */
SensorsReturn_t Sensors_decompressData(uint8_t* buffer, SensorsData_t* sensorData)
{
  uint8_t* iter = buffer;
  sensorData->timeStamp = *((uint32_t*)iter);
  iter += 4;

  sensorData->flags.all = *((uint16_t*)iter);
  iter += 2;


  if(sensorData->flags.DevID == true)
  {
    for(int i = 0; i < 12; i++)
    {
      sensorData->Data[i] = iter[i];
    }
    iter+=12;
    }
  else if(sensorData->flags.SDCARD == true)
  {
    for(int i = 0; i < sizeof(SDCard_datalog_t); i++)
    {
      sensorData->Data[i] = iter[i];
    }
    iter+=sizeof(SDCard_datalog_t);
  }
//  else if (sensorData->flags.walkingBehavior == true)
//  {
//    for(int i = 0; i < sizeof(WalkingBehavior_t); i++)
//    {
//      sensorData->Data[i] = iter[i];
//    }
//    iter+=sizeof(WalkingBehavior_t);
//  }
  else
  {
    SENSORS_LIST(SENSORS_DECOMPRESS_PACKET, sensorData, iter);
  }
  return (iter - buffer);

  return SENSORS_OK;
}
