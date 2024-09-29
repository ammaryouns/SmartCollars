/**
 * @file   	Device_Params.c
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Oct 26, 2016
 * 
 * @brief   
 */


#include "Device_Params.h"
#include "mFlash.h"

#define NETWORK_ID_DEFAULT              0
#define DCU_ID_DEFAULT                  3


#define NODE_PARAMS_STRUCT_DEFAULT         \
    .params = { CLOUD_SETTINGS_NODE_PARAM_LIST( CLOUD_SETTINGS_PARAM_INIT ) },  \
    .networkID              = NETWORK_ID_DEFAULT,\
    .DcuID                  = DCU_ID_DEFAULT,

    


/**
 * @defgroup Device_Params_Private_Structs			Device Parameters Private Structures
 * @{
 */



/** @}*/

/**
 * @defgroup Device_Params_Private_Members			Device Parameters Private Members
 * @{
 */

const Device_Flash_Params_t __attribute__((at(FLASH_FW_PARAM_ADDR)))        mainFlashParams = { NODE_PARAMS_STRUCT_DEFAULT };
const Device_Flash_Params_t __attribute__((at(FLASH_FW_BKUP_PARAM_ADDR)))   bkupFlashParams = { NODE_PARAMS_STRUCT_DEFAULT };


/** @}*/

/**
 * @defgroup Device_Params_Exported_Members			Device Params Exported Members
 * @{
 */

mFile_Def( fwFile, FLASH_FW2_BaseAddr, FLASH_FW2_MAX_SIZE, &mFile_Flash_ifc );

Device_Flash_Params_t deviceSettings_crc =
{
    NODE_PARAMS_STRUCT_DEFAULT

};



/** @}*/


/**
 * @defgroup Device_Params_Public_Functions			Device Params Public Functions
 * @{
 */


uint32_t generate_Device_ID(){
  Device_UID_t devUID;
  deviceParams_getUID(&devUID);
  return HAL_CRC_Calculate_ByteStream(&hcrc, (uint8_t*)&devUID, sizeof(Device_UID_t), false);
}

/**
 *
 * @return
 */
void deviceParams_getUID(Device_UID_t* devUID)
{
    memcpy(devUID, (uint8_t*)UID_BASE, sizeof(Device_UID_t));
}


/**
 *
 * @return
 */
bool deviceParam_isFlashUpToDate(const Device_Flash_Params_t* params)
{
    return (memcmp( &mainFlashParams.params, 
                    &params->params,
                    sizeof(mainFlashParams.params)) == 0);        
}


/**
 *
 * @param flashParams
 * @return
 */
HAL_StatusTypeDef deviceParams_updateFlash(const Device_Flash_Params_t* const flashParams,
                                                 Device_Flash_Params_t* const params)
{
    HAL_StatusTypeDef ret = HAL_ERROR;
    
    
    ret = HAL_FLASH_ErasePages((uint32_t)flashParams, sizeof(Device_Flash_Params_t));
    if (ret != HAL_OK)return ret;
  
    params->crc = HAL_CRC_Calculate_ByteStream(&hcrc, (uint8_t*)&params->params, offsetof(Device_Flash_Params_t, crc), false);
    
    ret = HAL_FLASH_Write((uint32_t)flashParams, (uint8_t*)params, sizeof(Device_Flash_Params_t));
    if(ret != HAL_OK)return ret;
    
    return ret;
}


/**
 *
 * @return
 */
HAL_StatusTypeDef deviceParams_commitToFlash(const Device_Flash_Params_t* params)
{
    HAL_StatusTypeDef ret = HAL_ERROR;
    
    //deviceParams_pullFromFlash();
    
    //Check if the new parameters are the same as those in the flash
    //and the crc of the main and backup flash params is correct. 
    //The crc check is for the case when the input params may be the same
    //as the flash parameters but the crc might not be up to date.
    
    //The main params crc is used for both comparisons because the main and
    //the backup parameters should have the same crc.
    if(deviceParam_isFlashUpToDate(params) && 
        mainFlashParams.crc == deviceParams_computeCRC(&mainFlashParams) &&
        mainFlashParams.crc == deviceParams_computeCRC(&bkupFlashParams))
        return HAL_OK;
//    printTask("Flash parameters need to update ");
    ret = deviceParams_updateFlash(&mainFlashParams, ( Device_Flash_Params_t*)params);
    if (ret != HAL_OK) return ret;
    
    ret = deviceParams_updateFlash(&bkupFlashParams, ( Device_Flash_Params_t*)params);
    return ret;
}
/**
 *
 */
void deviceParams_reset()
{
//    printTask("deviceParams_reset");
    static Device_Flash_Params_t resetParams = { NODE_PARAMS_STRUCT_DEFAULT };
    deviceParams_commitToFlash(&resetParams);
}


/**
 *
 */
const Device_Flash_Params_t* deviceParams_pullFromFlash(Device_Flash_Params_t* params)
{
    uint32_t mainCRC = deviceParams_computeCRC( &mainFlashParams );
    uint32_t bkupCRC = deviceParams_computeCRC( &bkupFlashParams );
    
    if (mainCRC == mainFlashParams.crc)
    {
        if (mainCRC != bkupCRC)
        {
            memcpy(params, &mainFlashParams, sizeof(Device_Flash_Params_t));
            deviceParams_updateFlash(&bkupFlashParams, params);
        }   
    }
    else if (bkupCRC == bkupFlashParams.crc)
    {
        if (mainCRC != bkupCRC)
        {
            memcpy(params, &bkupFlashParams, sizeof(Device_Flash_Params_t));
            deviceParams_updateFlash(&mainFlashParams, params);
        }
    }
    else
    {
        deviceParams_reset();   
        memcpy(params, &mainFlashParams, sizeof(Device_Flash_Params_t));
    }
    
    if (params != NULL)
    { 
      memcpy(params, &mainFlashParams, sizeof(Device_Flash_Params_t));
      params->crc = deviceParams_computeCRC(&mainFlashParams);
      deviceParams_commitToFlash(params);
    }

    return &mainFlashParams;
}



uint32_t deviceParams_getCRC(void)
{
    return HAL_CRC_Calculate_ByteStream(&hcrc, (uint8_t*)&deviceSettings, sizeof(deviceSettings), false);
}
/** @}*/
