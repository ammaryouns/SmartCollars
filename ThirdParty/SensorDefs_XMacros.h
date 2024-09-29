/**
 * @file   	SensorDefs_XMacros.h
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Oct 31, 2016
 * 
 * @brief   
 */

#ifndef SENSORDEFS_XMACROS_H_
#define SENSORDEFS_XMACROS_H_

#include "Sensors_Types_XMacros.h"

#pragma anon_unions

typedef __packed struct{
  int16_t BatterySOC;
  int16_t BatteryVoltage;
  uint32_t DataPoints;
  uint16_t DviceID;
  uint8_t counter;
  uint8_t DataIsbiengsaved  : 1;
  uint8_t FiFoFul           : 1;
  uint8_t RamFul            : 1;    
  uint8_t Reset             : 1;    
}SDCard_datalog_t;

typedef __packed struct{
  uint8_t rumination;
  uint8_t eating;
  uint8_t resting;
  uint8_t moving;
  uint8_t walking;
  uint16_t steps;
}WalkingBehavior_t;

#define BATTERY_DATA        ((packet->nodeID == 0xFF) ? BATTERY_DCU_DATA_DB : BATTERY_DATA_B )

#define POST_STRING_STRUCT_DEF_TEMPLATE( dataMembers ) \
__packed struct{\
\
    uint8_t size;\
    uint32_t dataType;\
    uint32_t deviceID;\
    uint32_t timeStamp;\
    dataMembers\
\
}

#define SENSOR_POST_STRING_PREPARE(name, dataTypeStr, packet, buffer, numPackets) \
 \
    postString_##name->size        = sizeof(Sensor_PostString_##name##_t) - sizeof(postString_##name->size);\
    postString_##name->dataType    = dataTypeStr;\
    postString_##name->deviceID    = packet->nodeID;\
    postString_##name->timeStamp   = packet->sensorData.timeStamp;\
    \
    buffer += sizeof(Sensor_PostString_##name##_t);\
    (*numPackets)++;\



#define SENSOR_FLAGS(type, modifier, array, name, ...) \
	SENSOR_FLAGS_##modifier(name, array)

#define SENSOR_DEF(type, modifier, array, name, ...) \
		SENSOR_DEF_##modifier(type, array, name)

#define SENSOR_POST_STRING_STRUCT_DEF(type, modifier, array, name, dataTypeStr, postVarType, ...) \
		SENSOR_POST_STRING_STRUCT_DEF_##modifier(postVarType, name)\

#define SENSOR_POST_STRING(type, modifier, array, name, dataTypeStr, postVarType, packet, buffer, numPackets, ...) \
        SENSOR_POST_STRING_##modifier(name, array, dataTypeStr, packet, buffer, numPackets);

#define SENSOR_POST_STRING_MAX_SIZE(type, modifier, array, name, ...) \
		SENSOR_POST_STRING_MAX_SIZE_##modifier(array, name) +

#define SENSORS_POST_STRING_COUNT(type, modifier, array, name, ...)  \
		SENSORS_POST_STRING_COUNT_##modifier(name, array)

#define SENSORS_COMPRESS_PACKET(type, modifier, array, name, dataTypeStr, postVarType, packet, buffer, ...)    \
        SENSORS_COMPRESS_PACKET_##modifier(type, name, array, dataTypeStr, packet, buffer)


#define SENSORS_DECOMPRESS_PACKET(type, modifier, array, name, dataTypeStr, postVarType, packet, buffer, ...)    \
        SENSORS_DECOMPRESS_PACKET_##modifier(type, name, array, dataTypeStr, packet, buffer)
 


//#define SENSORS_STRUCT_BEHAVIOUR(SENSOR, ...)	\
//	SENSOR(uint16_t,    DEFAULT,  ,      unknown,     ANN_RESULT_UNKNOWN, uint16_t, __VA_ARGS__ ) \
//	SENSOR(uint16_t,    DEFAULT,  ,      Reserve2,    ANN_RESULT_UNKNOWN, uint16_t, __VA_ARGS__ ) \
//	SENSOR(uint16_t,    DEFAULT,  ,      Reserve3,    ANN_RESULT_UNKNOWN, uint16_t, __VA_ARGS__ ) \
//	SENSOR(uint16_t,    DEFAULT,  ,      Reserve4,    ANN_RESULT_UNKNOWN, uint16_t, __VA_ARGS__ ) 


#define SENSORS_UNION_ACTIVITY(SENSOR, ...) \
  SENSOR(WalkingBehavior_t,   ARRAY, 4,    BEHAVIOUR,       ACTIVITY_DATA_B,  WalkingBehavior_t, __VA_ARGS__ ) \
  SENSOR(uint16_t,            ARRAY, 4,     activity,       ACTIVITY_DATA_A,           uint16_t, __VA_ARGS__ ) 


//SENSOR(uint16_t,            ARRAY, 4, DetailResult,     ANN_DETAIL_RESULT,           uint16_t, __VA_ARGS__ )


/**
 * @brief
 */
#define SENSORS_LIST(SENSOR, ...)	\
	SENSOR(uint16_t,      UNION, 1,    ACTIVITY,                      ,         , __VA_ARGS__ ) \
	SENSOR(uint16_t,    DEFAULT, 1,     battery,          BATTERY_DATA, uint16_t, __VA_ARGS__ ) \
	SENSOR( int16_t,    DEFAULT, 1, temperature,    TEMPERATURE_DATA_T,  int16_t, __VA_ARGS__ ) \

SENSORS_LIST(SENSOR_POST_STRING_STRUCT_DEF)


//#define Sensors_get_MaxPostStringSize()			(SENSORS_LIST(SENSOR_POST_STRING_MAX_SIZE) 0)
//#define Sensors_get_PostStringCount()           (SENSORS_LIST(SENSORS_POST_STRING_COUNT) 0)

#define Sensors_get_MaxPostStringSize()             (sizeof(SensorsData_t))
#define Sensors_get_PostStringCount()               (1)

#endif /* SENSORDEFS_XMACROS_H_ */
