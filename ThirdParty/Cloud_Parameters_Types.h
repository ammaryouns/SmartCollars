/**
 * @file   	Cloud_Parameters_Types.h
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Feb 14, 2017
 * 
 * @brief   
 */

#ifndef CLOUD_PARAMETERS_TYPES_H_
#define CLOUD_PARAMETERS_TYPES_H_

#include <stdint.h>


#define PARAM_DECLARE_UI8(varName, arraySize)					uint8_t varName;
#define PARAM_GET_CODE_UI8(id)                   				id,
#define PARAM_PRINT_UI8(buffer, object, varName, length)		do{ length += sprintf((char*)(buffer + length), "%u",  (object)->varName); }while(0)
#define PARAM_PRINT_DEBUG_UI8( object, varName )				do{ printDEBUG(#varName" : %u\n",  (object)->varName); }while(0)
#define PARAM_UPDATE_UI8(object, varName, newValue, length)		do{ (object)->varName = *((uint32_t*)newValue); }while(0)
#define PARAM_COMPARE_UI8(object1, object2, varName)			(object1)->varName == (object2)->varName


#define PARAM_DECLARE_SI8(varName, arraySize)					int8_t varName;
#define PARAM_GET_CODE_SI8(id)                   				PARAM_GET_CODE_UI8(id)
#define PARAM_PRINT_SI8(buffer, object, varName, length)		PARAM_PRINT_UI8(buffer, object, varName, length)
#define PARAM_PRINT_DEBUG_SI8( object, varName )				PARAM_PRINT_DEBUG_UI8(object, varName)
#define PARAM_UPDATE_SI8(object, varName, newValue, length)		PARAM_UPDATE_UI8(object, varName, newValue, length)
#define PARAM_COMPRAE_SI8(object1, object2, varName)			PARAM_COMPARE_UI8(object1, object2, varName)




#define PARAM_DECLARE_UI32(varName, arraySize)					uint32_t varName;
#define PARAM_GET_CODE_UI32(id)                   				PARAM_GET_CODE_UI8(id)
#define PARAM_PRINT_UI32(buffer, object, varName, length)		PARAM_PRINT_UI8(buffer, object, varName, length)
#define PARAM_PRINT_DEBUG_UI32( object, varName )				PARAM_PRINT_DEBUG_UI8(object, varName)
#define PARAM_UPDATE_UI32(object, varName, newValue, length)	PARAM_UPDATE_UI8(object, varName, newValue, length)
#define PARAM_COMPARE_UI32(object1, object2, varName)			PARAM_COMPARE_UI8(object1, object2, varName)




#define PARAM_DECLARE_FW_VER(varName, arraySize)            	uint32_t varName;
#define PARAM_GET_CODE_FW_VER(id)                 				id,
#define PARAM_PRINT_FW_VER(buffer, object, varName, length)    	do{ length += sprintf((char*)(buffer + length), "%.2f",(float)((object)->varName)/100); }while(0)
#define PARAM_PRINT_DEBUG_FW_VER( object, varName )         	do{ printDEBUG(#varName" : %.2f\n",(float)((object)->varName)/100); }while(0)
#define PARAM_UPDATE_FW_VER(object, varName, newValue, length) \
																do{ \
																	float fwTemp;\
																	sscanf((char*)newValue, "%f", &fwTemp); \
																	(object)->varName = (fwTemp + 0.001f)*100; \
																}while(0)
#define PARAM_COMPARE_FW_VER(object1, object2, varName)			PARAM_COMPARE_UI8(object1, object2, varName)





#define PARAM_DECLARE_STRING(varName, arraySize)				char  varName[arraySize];
#define PARAM_GET_CODE_STRING(id)                 				id,
#define PARAM_PRINT_STRING(buffer, object, varName, length)		do{ length += sprintf((char*)(buffer + length), "%s",  (object)->varName); }while(0)
#define PARAM_PRINT_DEBUG_STRING( object, varName)				do{ printDEBUG(#varName" : %s\n",  (object)->varName); }while(0)
#define PARAM_UPDATE_STRING(object, varName, newValue, length)	do{ strncpy((char*)(object)->varName, (char*)newValue, sizeof((object)->varName) - 1); }while(0)
#define PARAM_COMPARE_STRING(object1, object2, varName)			strncmp((object1)->varName, (object2)->varName, sizeof((object1)->varName)) == 0






#define PARAM_DECLARE_DATA_SEQUENCE(varName, arraySize)			Sensor_Acq_Seq_t varName;
#define PARAM_GET_CODE_DATA_SEQUENCE(id)          				id,
#define PARAM_PRINT_DATA_SEQUENCE(buffer, object, varName, length)	\
																for (int i = 0 ; i < sizeof((object)->varName.acquireData) ; i++)\
																{\
																	length += sprintf((char*)(buffer + length),  "%2hhx", (object)->varName.acquireData[i]);\
																}
#define PARAM_PRINT_DEBUG_DATA_SEQUENCE( object, varName )	\
																printDEBUG(#varName " : ");\
																for (int i = 0 ; i < sizeof((object)->varName.acquireData) ; i++)\
																{\
																	printDEBUG("%2hhx", (object)->varName.acquireData[i]);\
																}\
																printDEBUG("\n");
#define PARAM_UPDATE_DATA_SEQUENCE(object, varName, newValue, length) 	\
																do{ memcpy(&(object)->varName.acquireData, newValue, sizeof((object)->varName.acquireData)); }while(0)
#define PARAM_COMPARE_DATA_SEQUENCE(object1, object2, varName)	\
														memcmp(&(object1)->varName, &(object2)->varName, sizeof((object1)->varName)))



#define PARAM_INIT( varType, varName, arraySize, initValue, ...) \
	.varName = initValue,\


#define PARAM_COUNT(...)         1 +

#endif /* CLOUD_PARAMETERS_TYPES_H_ */
