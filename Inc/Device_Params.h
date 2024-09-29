/**
 * @file   	Device_Params.h
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Oct 26, 2016
 * 
 * @brief   
 */

#ifndef DEVICE_PARAMS_H_
#define DEVICE_PARAMS_H_

#include "Cloud_Settings_Params.h"
#include "crc32.h"
#include "FwFile.h"


#ifdef __cplusplus
extern "C"{
#endif 

/**
 * @defgroup Device_Parameters_Defines			Device Parameters Defines
 * @{
 */
#define SENSOR_DATA_SEQUENCE_DEFAULT   { .acquireData = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}}

#define deviceParams_computeCRC( pFlashParams )     HAL_CRC_Calculate_ByteStream(&hcrc, (uint8_t*)(pFlashParams), offsetof(Device_Flash_Params_t, crc), false)

/** @}*/


/**
 * @defgroup Device_Parameter_Enums				Device Parameters Enum Definitions
 * @{
 */

typedef enum{

    DEV_GSM         = 3,
    DEV_DCU_LORA    = 4,
    DEV_NODE        = 5
    
}Device_Type_t;


typedef enum{
  DCU_v8_L452       = 0x2D78756E,
  DCU_v6_Electron   = 0x1D79756E,
  DCU_v6_LoRa       = 0x1D79756E   
}App_it_t;
/** @}*/


/**
 * @defgroup Device_Parameter_Structs			Device Parameters Structure Definitions
 * @{
 */

typedef __packed struct{
    
    uint8_t uid[12];
    
}Device_UID_t;

/**
 * @brief    Node Settings parameters that can be configured through the DCU from the cloud.
 *           This struct is only used for sizeof purposes and is not to be instantiated anywhere.
 */
typedef __packed struct{

	///The cloud settings must be the first element in the struct.
	/// updating the node settings through LoRa WAN depends on this.
    CLOUD_SETTINGS_NODE_PARAM_LIST(CLOUD_SETTINGS_PARAM_DEF)

}Node_Cloud_Params_t;


/**
 * @brief  Device settings parameters structure.
 */

typedef struct{

    Node_Cloud_Params_t params;
    uint16_t networkID;		
    uint8_t DcuID;			///UID of the DCU of whose network this node is a part of.  
    uint32_t crc;

}Device_Flash_Params_t;

/** @}*/


extern Device_Flash_Params_t  deviceSettings_crc;

#define deviceSettings        deviceSettings_crc.params

extern mFile_t fwFile;


uint32_t generate_Device_ID (void);
void deviceParams_getUID(Device_UID_t* devUID);

const Device_Flash_Params_t* deviceParams_pullFromFlash(Device_Flash_Params_t* params);
HAL_StatusTypeDef deviceParams_commitToFlash(const Device_Flash_Params_t* params);
uint32_t deviceParams_getCRC(void);
void deviceParams_reset(void);
bool deviceParam_isFlashUpToDate(const Device_Flash_Params_t* params);
   


#ifdef __cplusplus
}
#endif

#endif /* DEVICE_PARAMS_H_ */
