/**
******************************************************************************
* @file    SensorDefs.h
* @author  Cowlar Firmware Team
* @version V1.0.3
* @date    08-April-2015
* @brief   Sensor Communication protocols Defs Source File.
*
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

/** @addtogroup CMSIS
* @{
*/

/** @addtogroup stm32f0xx_system
* @{
*/

/**
* @brief Define to prevent recursive inclusion
*/
#ifndef __SENSOR_DEFS_H__
#define __SENSOR_DEFS_H__


   /* Includes ------------------------------------------------------------------*/
#include "SensorDefs_XMacros.h"
#include "mQueue_List.h"

/**
 * @defgroup
 * @{
 */

/**
 * @brief Sample time X macro list
 * Format : SAMPLE_TIME( index, sample_time_in_minutes )
 */
#define SAMPLE_TIME_LIST	\
SAMPLE_TIME(0,   1)\
SAMPLE_TIME(1,   2)\
SAMPLE_TIME(2,   3)\
SAMPLE_TIME(3,   4)\
SAMPLE_TIME(4,   5)\
SAMPLE_TIME(5,   6)\
SAMPLE_TIME(6,  10)\
SAMPLE_TIME(7,  12)\
SAMPLE_TIME(8,  15)\
SAMPLE_TIME(9,  20)\
SAMPLE_TIME(10, 30)\
SAMPLE_TIME(11, 60)\

typedef enum
{
	#define SAMPLE_TIME(index, sample_time)		T_##sample_time##_min = index,\

	SAMPLE_TIME_LIST
    #undef SAMPLE_TIME
    
} SampleTime_e;
#pragma anon_unions

typedef __packed struct{

    uint32_t    timeStamp;
	__packed union{
        __packed struct{
    
            ///@todo move sample time to the left and sensor flags 
            /// to the right so that sample time is not affected when a 
            /// new flag is added to the packet.
            SENSORS_LIST(SENSOR_FLAGS)
            uint8_t SampleTime          : 4;
            uint8_t DevID               : 1;
            uint8_t SDCARD              : 1;
        };
        uint16_t all;
    }flags;
    __packed union
    {
        __packed struct{SENSORS_LIST(SENSOR_DEF)};
        uint8_t Data[12];
        SDCard_datalog_t SDCard;
    };

}SensorsData_t;

typedef __packed struct{

    SensorsData_t sensorData;
    uint16_t nodeID;
    
}Node_SensorsData_t;


/**
* @}
*/

   /** @addtogroup SI4432_System_Exported_Constants
   * @{
   */

#define PACKET_FW_UPDATE_SIZE                  ((uint32_t)0x09)
#define PACKET_FW_SIZE_QUERY                   ((uint32_t)0x10)
#define PACKET_FW_BYTES_QUERY                  ((uint32_t)0x11)
#define PACKET_FW_BYTES                        ((uint32_t)0x12)
#define PACKET_FW_END_OF_FILE                  ((uint32_t)0x13)


#define NODE_FW_VER_NFV					((uint32_t)0x4969)

#define ACTIVITY_DATA_A                 ((uint32_t)0x41)
#define ACTIVITY_DATA_B                 ((uint32_t)0x42)

#define RUMINATION_DATA_RMNT            ((uint32_t)0x544E4D52)
#define EATING_DATA_EAT                 ((uint32_t)0x544145)

#define RUMNATION_ANN_RMNA              ((uint32_t)0x414E4D52)
#define EATING_ANN_EATA                 ((uint32_t)0x41544145)

#define TEMPERATURE_DATA_T              ((uint32_t)0x54)

#define PRESSURE_SOIL_DATA_SPC		    ((uint32_t) 0x434D53)
#define PRESSURE_SOIL_DATA_SPC1         ((uint32_t) 0x434D5301)
#define PRESSURE_SOIL_DATA_SPC2         ((uint32_t) 0x434D5302)


#define PRESSURE_SOIL_DATA_SPR		    ((uint32_t) 0x524D53)

#define PRESSURE_SOIL_DATA_SPT		    ((uint32_t) 0x544D53)

#define TEMPERATURE_SOIL_DATA_ST        ((uint32_t)0x5453)

#define AMBIENT_TEMP_DATA_AT            ((uint32_t)0x5441)

#define HUMIDITY_DATA_H                 ((uint32_t)0x4841)

#define PACKET_RTC_UPDATE				((uint32_t)0x55)

#define BATTERY_DATA_B		            ((uint32_t)0x424E)

#define BATTERY_DCU_DATA_DB				((uint32_t) 0x4244)

#define BATTERY_CURRENT                 ((uint32_t) 0x4342)

#define GSM_LOCATION                    ((uint32_t)0x5969)

#define WATERMARK_RESISTOR              (6780u)

#define CORETEMP_NODE_NCT				((uint32_t)	0x544E)
#define CORETEMP_DCU_DCT				((uint32_t) 0x5444)

#define MESSAGE_HELLO					((uint32_t) 0x01020304)


#define ANN_DETAIL_RESULT  			    ((uint32_t)0x61)
#define ANN_RESULT_UNKNOWN 			    ((uint32_t)0x62)

   /**	
   * @}
   */

typedef enum{

	SENSORS_OK = 0,
	SENSORS_ERROR

}SensorsReturn_t;


extern mQueue_List_t sensors_Queue_List;


#ifdef __cplusplus
extern "C" {
#endif


uint32_t Sensors_get_PacketCount(void);

SensorsReturn_t	Sensors_readPacket(void* data);

SensorsReturn_t Sensors_readPacketAt(uint32_t index, void* data);

SensorsReturn_t Sensors_savePacket(void* data);

SensorsReturn_t	Sensors_clearPackets(uint32_t count);

uint32_t Sensors_getPostString(uint8_t* buffer, const Node_SensorsData_t* packet, uint8_t* numPackets);

uint32_t Sensors_compressData(const SensorsData_t* sensorData, uint8_t* buffer);

SensorsReturn_t Sensors_decompressData(uint8_t* buffer, SensorsData_t* sensorData);

#ifdef __cplusplus
}
#endif

#endif /*__SYSTEM_STM32F0XX_H */

/**
* @}
*/

/**
* @}
*/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
