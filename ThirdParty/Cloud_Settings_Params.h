/**
 * @file   	Cloud_Settings_Params.h
 * @author 	Hassan
 * @version	
 * @date	Aug 26, 2016
 * 
 * @brief	X macro definitions file for settings parameters that can be updated from the cloud
 * 			The format of #define is :
 *			CLOUD_SETTINGS_PARAM( type, VarName, count   
 */

#ifndef __CLOUD_SETTINGS_PARAMS_H_
#define __CLOUD_SETTINGS_PARAMS_H_


#include "DCU_LoRa_Cloud_Params.h"

        
/**
 * @defgroup DCU_param_macro_list_usage			DCU Parameter Macro list Usages
 * @details		These macros are passed as arguments to the DCU parameter X macro list to generate code
 * 				for the specific usage.
 * @{
 */

#define CLOUD_SETTINGS_PARAM_COUNT(...)         1 + 


#define NUM_DCU_PARAM()         (CLOUD_SETTINGS_DCU_PARAM_LIST(CLOUD_SETTINGS_PARAM_LIST) 0)

/**
 * @brief Define the DCU parameters list variables.
 */
#define CLOUD_SETTINGS_PARAM_DEF( varType, varName, arraySize, ...) \
	PARAM_DECLARE_##varType(varName, arraySize)\
    


/**
 * @brief Initialize the parameter list inside the struct.
 */
#define CLOUD_SETTINGS_PARAM_INIT( varType, varName, arraySize, initValue, ...) \
	.varName = initValue,\
	\

#define CLOUD_SETTING_PARAM_GET_CODE_LIST( varType, varName, arraySize, initValue, id, ...) \
	PARAM_GET_CODE_##varType(id) \

/**
 * @brief Print the parameter list in order serially.
 */
#define CLOUD_SETTINGS_PARAM_PRINT( varType, varName, arraySize, initValue, id, object, buffer, length) \
	PARAM_PRINT_##varType(buffer, object, varName, length);\

/**
 * @brief Print the parameter list for the node parameters serially.
 */


#define CLOUD_SETTINGS_PARAM_PRINT_DEBUG( varType, varName,  arraySize, initValue, id, object )\
    PARAM_PRINT_DEBUG_##varType(object, varName);\


/**
 * @brief case statements for updating the variable once the new settings have been downloaded from the server.
 */
#define CLOUD_SETTINGS_PARAM_UPDATE_CASE( varType, varName, arraySize, initValue, id, object, newValue, length)	\
	case id: \
		PARAM_UPDATE_##varType(object, varName, newValue, length);\
	break;\

#define CLOUD_SETTINGS_PARAM_COMPARE( varType, varName, arraySize, initValue, id, object1, object2, branch_false) \
    if ((PARAM_COMPARE_##varType( object1, object2, varName)) == false)\
    {\
        branch_false\
    }\
    
/** @}*/




/**
 * @defgroup Gateway_Settings_cloud_params			Gateway Settings Cloud parameters
 * @details This is an X macro list of the parameters in the node settings that can be configured by the
 * 			user from the web portal.
 *
 * 			The arguments for the parameter repectively are:
 * 			@param dataType 		This may of the following types:
 * 									@arg UI32				uint32_t
 * 									@arg FW_VER				firmware version (uint32_t)
 * 									@arg STRING     		string (uint8_t [])
 * 									@arg DATA_SEQUENCE		Sensor data acquistion sequence(Sensor_Acq_Seq_t)
 * 									@arg NODE_SETTINGS		Node Settings struct (Node_Settings_t)
 *
 *			@param varName			name of the variable
 *			@param arraySize		size of the array. 0 if single variable. if not zero,
 *									the datatype must of type #dataType_ARRAY or STRING
 *			@param initValue		initial value of the variable
 *			@param code				update code for the parameter. Used when updating setting from the cloud.
 *			@param __var_args__		may contain additional arguments depending upon the usage of the X macro list.
 * @{
 */
#define CLOUD_SETTINGS_DCU_PARAM_LIST(CLOUD_SETTINGS_PARAM, ...) \
CLOUD_SETTINGS_PARAM( UI32, 		        UploadTime, 			    0,    60*60,		  				          0x01,  __VA_ARGS__)\
CLOUD_SETTINGS_PARAM( STRING, 			    dataPostAddress,		  128,  HTTPPARA_DATA_URL,			      0x03,  __VA_ARGS__)\
CLOUD_SETTINGS_PARAM( STRING, 			    logPostAddress,		    128,  HTTPPARA_LOG_URL, 			      0x04,  __VA_ARGS__)\
CLOUD_SETTINGS_PARAM( STRING, 			    firmwareUpdateAddress,128,  HTTPPARA_FW_UPDATE_URL,		    0x05,  __VA_ARGS__)\
CLOUD_SETTINGS_PARAM( STRING, 			    creditCheckCode, 	    12,   GSM_CREDIT_CHECK_CUSD,  		  0x06,  __VA_ARGS__)\
CLOUD_SETTINGS_PARAM( STRING, 			    packageCheckCode,	    12,   GSM_PACKAGE_CHECK_CUSD, 		  0x07,  __VA_ARGS__)\
CLOUD_SETTINGS_PARAM( STRING, 			    dataSMSAddress, 	    15,   GSM_DATA_SMS_ADDRESS,   		  0x08,  __VA_ARGS__)\
CLOUD_SETTINGS_PARAM( STRING, 			    logSMSAddress, 		    15,   GSM_LOG_SMS_ADDRESS, 	 	      0x09,  __VA_ARGS__)\
CLOUD_SETTINGS_PARAM( FW_VER,           fwVersion,            0,    0,                            0x0A,  __VA_ARGS__)\
CLOUD_SETTINGS_PARAM( DATA_SEQUENCE,    dataSequence,         0,    SENSOR_DATA_SEQUENCE_DEFAULT, 0x0B,  __VA_ARGS__)\
CLOUD_SETTINGS_PARAM( DCU_LORA_PARAMS,  DCU_LoRa_Params, 	    0,    DCU_LORA_PARAMS_DEFAULT, 	    0x0C,  __VA_ARGS__)\
CLOUD_SETTINGS_PARAM( STRING,           rtcUpdateAddress,     128,  HTTPPARA_RTC_URL,             0x0D,  __VA_ARGS__)\
CLOUD_SETTINGS_PARAM( STRING,           paramUpdateAddress,   128,  HTTPPARA_PARAMS_URL,          0x0E,  __VA_ARGS__)\
CLOUD_SETTINGS_PARAM( STRING,           LoRaNATUpdateAddress, 128,  HTTPPARA_LORA_NAT_URL,        0x0F,  __VA_ARGS__)\
CLOUD_SETTINGS_PARAM( UI8, 			        NodeReset, 			      0, 	  0x00,		                      0xFC,  __VA_ARGS__)

//CLOUD_SETTINGS_PARAM( NODE_PARAMS,	    node_Params,          0,    NODE_PARAMS_DEFAULT,          0xFE,  __VA_ARGS__)\
//CLOUD_SETTINGS_PARAM( UI8, 			        networkID, 			      0, 	  0x01,		                      0xFD,  __VA_ARGS__)\


/** @}*/


static const uint8_t DCU_PARAM_CODE_LIST[] = { CLOUD_SETTINGS_DCU_PARAM_LIST(CLOUD_SETTING_PARAM_GET_CODE_LIST) };

#endif
