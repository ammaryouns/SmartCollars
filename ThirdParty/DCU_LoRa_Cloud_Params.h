/**
 * @file   	DCU_LoRa_Cloud_Params.h
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Feb 14, 2017
 * 
 * @brief   
 */

#ifndef DCU_LORA_CLOUD_PARAMS_H_
#define DCU_LORA_CLOUD_PARAMS_H_

#include "Node_Cloud_Params.h"


#define DCU_LORA_PARAMS_DEFAULT	\
		{ CLOUD_SETTINGS_DCU_LORA_PARAM_LIST(PARAM_INIT) }

#define NUM_DCU_LORA_PARAM()        (CLOUD_SETTINGS_DCU_LORA_PARAM_LIST(PARAM_COUNT) 0)




#define PARAM_DECLARE_DCU_LORA_PARAMS(varName, arraySize)		DCU_LoRa_Params_t varName;

#define PARAM_GET_CODE_DCU_LORA_PARAMS(id)  \
										PARAM_GET_CODE_UI32(id) \
										PARAM_GET_CODE_UI32(NUM_DCU_LORA_PARAM()) \
										CLOUD_SETTINGS_DCU_LORA_PARAM_LIST(CLOUD_SETTINGS_DCU_LORA_PARAM_GET_CODE_LIST)

#define PARAM_PRINT_DCU_LORA_PARAMS(buffer, object, varName, length) \
        								CLOUD_SETTINGS_DCU_LORA_PARAM_LIST(CLOUD_SETTINGS_DCU_LORA_PARAM_PRINT, &(object)->varName, buffer, length)

#define PARAM_PRINT_DEBUG_DCU_LORA_PARAMS( object, varName )\
										printDEBUG(#varName"\n");\
										CLOUD_SETTINGS_DCU_LORA_PARAM_LIST(CLOUD_SETTINGS_DCU_LORA_PARAM_PRINT_DEBUG, &(object)->varName)

#define PARAM_UPDATE_DCU_LORA_PARAMS(object, varName, newValue, length)   \
										do{ parse_DCU_LoRa_Params(&(object)->varName, newValue, length); }while(0)






#define CLOUD_SETTINGS_DCU_LORA_PARAM_GET_CODE_LIST( varType, varName, arraySize, initValue, id, ...) \
    PARAM_GET_CODE_##varType(id) \

#define CLOUD_SETTINGS_DCU_LORA_PARAM_PRINT( varType, varName, arraySize, initValue, id, object, buffer, length) \
	PARAM_PRINT_##varType(buffer, object, varName, length);\

#define CLOUD_SETTINGS_DCU_LORA_PARAM_PRINT_DEBUG( varType, varName, arraySize, initValue, id, object)\
    PARAM_PRINT_DEBUG_##varType(object, varName);\



/**
 * @defgroup Node_Settings_cloud_params			Node Settings Cloud parameters
 * @details This is an X macro list of the parameters in the node settings that can be configured by the
 * 			user from the web portal.
 *
 * 			The arguments for the parameter respectively are:
 * 			@param dataType 		This may of the following types:
 * 									@arg UI32				uint32_t
 * 									@arg FW_VER				firmware version (uint32_t)
 * 									@arg STRING     		string (uint8_t [])
 * 									@arg DATA_SEQUENCE		Sensor data acquisition sequence(Sensor_Acq_Seq_t)
 *
 *			@param varName			name of the variable
 *			@param arraySize		size of the array. 0 if single variable.
 *			@param initValue		initial value of the variable
 *			@param code				update code for the parameter. Used when updating setting from the cloud.
 *			@param __var_args__		may contain additional arguments depending upon the usage of the X macro list.
 * @{
 */

#define CLOUD_SETTINGS_DCU_LORA_PARAM_LIST(CLOUD_SETTINGS_PARAM, ...) \
CLOUD_SETTINGS_PARAM( FW_VER, 	        fwVersion,		  0, 	  0,		                0xFF,  __VA_ARGS__)\
CLOUD_SETTINGS_PARAM( NODE_PARAMS,	    node_Params,    0,    NODE_PARAMS_DEFAULT,  0xFE,  __VA_ARGS__)\
CLOUD_SETTINGS_PARAM( UI8, 			        networkID, 			0, 	  0x01,		              0xFD,  __VA_ARGS__)\
CLOUD_SETTINGS_PARAM( UI8, 			        NodeReset, 			0, 	  0x00,		              0xFC,  __VA_ARGS__)

/** @}*/




#endif /* DCU_LORA_CLOUD_PARAMS_H_ */
