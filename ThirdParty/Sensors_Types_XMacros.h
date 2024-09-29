/**
 * @file   	Sensors_Types_XMacros.h
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Mar 14, 2017
 * 
 * @brief   
 */

#ifndef SENSORS_TYPES_XMACROS_H_
#define SENSORS_TYPES_XMACROS_H_

#include <stdint.h>

#define SENSOR_FLAGS_BEHAVIOUR(name, array)                 uint8_t name : 1;
#define SENSOR_DEF_BEHAVIOUR(type, array, name)             type name;
#define SENSOR_POST_STRING_MAX_SIZE_BEHAVIOUR(array, name)  sizeof(Sensor_PostString_##name##_t)
#define SENSORS_POST_STRING_COUNT_BEHAVIOUR(name, array)			1 +
#define SENSOR_POST_STRING_STRUCT_DEF_BEHAVIOUR(postVarType, name) \
		typedef POST_STRING_STRUCT_DEF_TEMPLATE(postVarType name;) Sensor_PostString_##name##_t;\

#define SENSOR_POST_STRING_BEHAVIOUR(name, array, dataTypeStr, packet, buffer, numPackets) \
\
Sensor_PostString_##name##_t* postString_##name = (Sensor_PostString_##name##_t*)buffer;\
if (packet->sensorData.flags.name)\
{\
	SENSOR_POST_STRING_PREPARE(name, dataTypeStr, packet, buffer, numPackets)\
	postString_##name->name       = packet->sensorData.name;\
}\

#define SENSORS_COMPRESS_PACKET_BEHAVIOUR(type, name, array, dataTypeStr, packet, buffer)    \
\
if (packet->flags.name)\
{\
    *(type*)buffer = packet->name;\
    buffer += sizeof(type);\
}\

#define SENSORS_DECOMPRESS_PACKET_BEHAVIOUR(type, name, array, dataTypeStr, packet, buffer) \
\
if (packet->flags.name)\
{\
    packet->name = *(type*)buffer;\
    buffer += sizeof(type);\
}\


#define SENSOR_FLAGS_DEFAULT(name, array)                   uint8_t name : 1;
#define SENSOR_DEF_DEFAULT(type, array, name)               type name;
#define SENSOR_POST_STRING_MAX_SIZE_DEFAULT(array, name)    sizeof(Sensor_PostString_##name##_t)
#define SENSORS_POST_STRING_COUNT_DEFAULT(name, array)			1 +
#define SENSOR_POST_STRING_STRUCT_DEF_DEFAULT(postVarType, name) \
		typedef POST_STRING_STRUCT_DEF_TEMPLATE(postVarType name;) Sensor_PostString_##name##_t;\

#define SENSOR_POST_STRING_DEFAULT(name, array, dataTypeStr, packet, buffer, numPackets) \
\
Sensor_PostString_##name##_t* postString_##name = (Sensor_PostString_##name##_t*)buffer;\
if (packet->sensorData.flags.name)\
{\
	SENSOR_POST_STRING_PREPARE(name, dataTypeStr, packet, buffer, numPackets)\
	postString_##name->name       = packet->sensorData.name;\
}\

#define SENSORS_COMPRESS_PACKET_DEFAULT(type, name, array, dataTypeStr, packet, buffer)    \
\
if (packet->flags.name)\
{\
    *(type*)buffer = packet->name;\
    buffer += sizeof(type);\
}\

#define SENSORS_DECOMPRESS_PACKET_DEFAULT(type, name, array, dataTypeStr, packet, buffer) \
\
if (packet->flags.name)\
{\
    packet->name = *(type*)buffer;\
    buffer += sizeof(type);\
}\


#define SENSOR_FLAGS_ARRAY(name, array) 							      uint8_t name : array;
#define SENSOR_DEF_ARRAY(type, array, name)      			      type name[array];
#define SENSOR_POST_STRING_MAX_SIZE_ARRAY(array, name)		  (array * sizeof(Sensor_PostString_##name##_t))
#define SENSORS_POST_STRING_COUNT_ARRAY(name, array)		    array +
#define SENSOR_POST_STRING_STRUCT_DEF_ARRAY(postVarType, name) \
		SENSOR_POST_STRING_STRUCT_DEF_DEFAULT(postVarType, name)

#define SENSOR_POST_STRING_ARRAY(name, array, dataTypeStr, packet, buffer, numPackets) \
\
Sensor_PostString_##name##_t* postString_##name;\
if (packet->sensorData.flags.name)\
{\
    for (int i = 0 ; i < array ; i++)\
    {\
        postString_##name = (Sensor_PostString_##name##_t*)buffer;\
        SENSOR_POST_STRING_PREPARE(name, dataTypeStr, packet, buffer, numPackets)\
        postString_##name->name = packet->sensorData.name[i];\
        postString_##name->timeStamp = (packet->sensorData.timeStamp) - ((sample_time_lut[packet->sensorData.flags.SampleTime] / 4)*(4-1-i));\
    }\
}\

#define SENSORS_COMPRESS_PACKET_ARRAY(type, name, array, dataTypeStr, packet, buffer)  \
\
if(packet->flags.name)\
{\
    for (int i = 0 ; i < array ; i++)\
    {\
        if( packet->flags.name & 1<< i){\
        *(type*)buffer = packet->name[i];\
        buffer += sizeof(type);}\
    }\
}\

#define SENSORS_DECOMPRESS_PACKET_ARRAY(type, name, array, dataTypeStr, packet, buffer)  \
\
if(packet->flags.name)\
{\
    for (int i = 0 ; i < array ; i++)\
    {\
        if( packet->flags.name & 1<< i){\
        packet->name[i] = *(type*)buffer;\
        buffer += sizeof(type);}\
    }\
}\

  

#define SENSOR_FLAGS_UNION(name, array)    							      SENSORS_UNION_##name(SENSORS_UNION_FLAGS)
#define SENSOR_DEF_UNION(type, array, name)						      __packed union { SENSORS_UNION_##name(SENSORS_UNION_DEF) };
#define SENSOR_POST_STRING_MAX_SIZE_UNION(array, name)      (4*sizeof(Sensor_PostString_activity_t))
#define SENSORS_POST_STRING_COUNT_UNION(name, array)			  4 +
#define SENSOR_POST_STRING_STRUCT_DEF_UNION(postVarType, name)	\
		SENSORS_UNION_##name(SENSORS_UNION_POST_STRING_STRUCT_DEF)

#define SENSOR_POST_STRING_UNION(name, array, dataTypeStr, packet, buffer, numPackets)	\
		SENSORS_UNION_##name(SENSORS_UNION_POST_STRING, packet, buffer, numPackets)

#define SENSORS_COMPRESS_PACKET_UNION(type, name, array, dataTypeStr, packet, buffer) \
        SENSORS_UNION_##name(SENSORS_UNION_COMPRESS_PACKET, packet, buffer)

#define SENSORS_DECOMPRESS_PACKET_UNION(type, name, array, dataTypeStr, packet, buffer) \
        SENSORS_UNION_##name(SENSORS_UNION_DECOMPRESS_PACKET, packet, buffer)



#define SENSOR_FLAGS_STRUCT(name, array)                      SENSORS_STRUCT_##name(SENSORS_STRUCT_FLAGS)
#define SENSOR_DEF_STRUCT(type, array, name)					      __packed struct { SENSORS_STRUCT_##name(SENSORS_STRUCT_DEF) };
#define SENSOR_POST_STRING_MAX_SIZE_STRUCT(array, name)     SENSORS_STRUCT_##name( SENSORS_STRUCT_POST_STRING_MAX_SIZE )
#define SENSORS_POST_STRING_COUNT_STRUCT(name, array)       SENSORS_STRUCT_##name( SENSORS_STRUCT_POST_STRING_COUNT )
#define SENSOR_POST_STRING_STRUCT_DEF_STRUCT(postVarType, name)	\
		SENSORS_STRUCT_##name(SENSORS_STRUCT_POST_STRING_STRUCT_DEF)

#define SENSOR_POST_STRING_STRUCT(name, array, dataTypeStr, packet, buffer, numPackets)	\
		SENSORS_STRUCT_##name(SENSORS_STRUCT_POST_STRING, packet, buffer, numPackets)

#define SENSORS_COMPRESS_PACKET_STRUCT(type, name, array, dataTypeStr, packet, buffer)  \
        SENSORS_STRUCT_##name(SENSORS_STRUCT_COMPRESS_PACKET, packet, buffer)

#define SENSORS_DECOMPRESS_PACKET_STRUCT(type, name, array, dataTypeStr, packet, buffer)  \
        SENSORS_STRUCT_##name(SENSORS_STRUCT_DECOMPRESS_PACKET, packet, buffer)



#define SENSOR_FLAGS_FW(name)
#define SENSOR_DEF_FW(type, array, name)            		    SENSOR_DEF_DEFAULT(type, array, name)
#define SENSOR_POST_STRING_MAX_SIZE_FW(array, name)			    SENSOR_POST_STRING_MAX_SIZE_DEFAULT(array, name)
#define SENSORS_POST_STRING_COUNT_FW(name, array)			      1 +

#define SENSOR_POST_STRING_STRUCT_DEF_FW(postVarType, name) \
		SENSOR_POST_STRING_STRUCT_DEF_DEFAULT(postVarType, name)

#define SENSOR_POST_STRING_FW(name, array, dataTypeStr, packet, buffer, numPackets) \
    Sensor_PostString_##name##_t* postString_##name = (Sensor_PostString_##name##_t*)buffer;\
    SENSOR_POST_STRING_PREPARE(name, dataTypeStr, packet, buffer, numPackets)\
    postString_##name->name = packet->sensorData.name;\

#define SENSORS_COMPRESS_PACKET_FW(type, name, array, dataTypeStr, packet, buffer)   \
{\
    *(type*)buffer = packet->name;\
    buffer += sizeof(type);\
}\

#define SENSORS_DECOMPRESS_PACKET_FW(type, name, array, dataTypeStr, packet, buffer)   \
{\
    packet->name = *(type*)buffer;\
    buffer += sizeof(type);\
}\


#define SENSORS_UNION_FLAGS(type, modifier, array, name, ...) \
	SENSOR_FLAGS_##modifier(name, array)

#define SENSORS_UNION_DEF(type, modifier, array, name, ...) \
		SENSOR_DEF_##modifier(type, array, name)\

#define SENSORS_UNION_POST_STRING_STRUCT_DEF(type, modifier, array, name, dataTypeStr, postVarType, ...) \
		SENSOR_POST_STRING_STRUCT_DEF_##modifier(postVarType, name)\

#define SENSORS_UNION_POST_STRING(type, modifier, array, name, dataTypeStr, postVarType, packet, buffer, numPackets, ...) \
        SENSOR_POST_STRING_##modifier(name, array, dataTypeStr, packet, buffer, numPackets);

#define SENSORS_UNION_POST_STRING_MAX_SIZE(type, modifier, array, name, ...) \
		SENSOR_POST_STRING_MAX_SIZE_##modifier(array, name) +

#define SENSORS_UNION_POST_STRING_COUNT(type, modifier, array, name, ...)  \
		SENSORS_POST_STRING_COUNT_##modifier(name, array)

#define SENSORS_UNION_COMPRESS_PACKET(type, modifier, array, name, dataTypeStr, postVarType, packet, buffer, ...) \
        SENSORS_COMPRESS_PACKET_##modifier(type, name, array, dataTypeStr, packet, buffer)

#define SENSORS_UNION_DECOMPRESS_PACKET(type, modifier, array, name, dataTypeStr, postVarType, packet, buffer, ...) \
        SENSORS_DECOMPRESS_PACKET_##modifier(type, name, array, dataTypeStr, packet, buffer)




#define SENSORS_STRUCT_FLAGS(type, modifier, array, name, ...) \
        SENSOR_FLAGS_##modifier(name, array)

#define SENSORS_STRUCT_DEF(type, modifier, array, name, ...) \
		SENSOR_DEF_##modifier(type, array, name)\

#define SENSORS_STRUCT_POST_STRING_STRUCT_DEF(type, modifier, array, name, dataTypeStr, postVarType, ...) \
		SENSOR_POST_STRING_STRUCT_DEF_##modifier(postVarType, name)\

#define SENSORS_STRUCT_POST_STRING(type, modifier, array, name, dataTypeStr, postVarType, packet, buffer, numPackets, ...) \
        SENSOR_POST_STRING_##modifier(name, array, dataTypeStr, packet, buffer, numPackets);

#define SENSORS_STRUCT_POST_STRING_MAX_SIZE(type, modifier, array, name, ...) \
		SENSOR_POST_STRING_MAX_SIZE_##modifier(array, name) +

#define SENSORS_STRUCT_POST_STRING_COUNT(type, modifier, array, name, ...)  \
		SENSORS_POST_STRING_COUNT_##modifier(name, array)

#define SENSORS_STRUCT_COMPRESS_PACKET(type, modifier, array, name, dataTypeStr, postVarType, packet, buffer, ...) \
        SENSORS_COMPRESS_PACKET_##modifier(type, name, array, dataTypeStr, packet, buffer)

#define SENSORS_STRUCT_DECOMPRESS_PACKET(type, modifier, array, name, dataTypeStr, postVarType, packet, buffer, ...) \
        SENSORS_DECOMPRESS_PACKET_##modifier(type, name, array, dataTypeStr, packet, buffer)

#endif /* SENSORS_TYPES_XMACROS_H_ */
