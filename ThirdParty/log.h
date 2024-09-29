
#ifndef __LOG_H_
#define __LOG_H_


#include "mQueue_List.h"


#define ERROR_LOG_TIMEPERIOD        30*60

typedef enum{

    LOG_OK          = 0,
    LOG_ERROR       = 1,
    LOG_TIMEOUT     = 2
    
}LOG_Return_t;


typedef enum{

    LOG_ErrorSource_Radio   = 0x00,
    LOG_ErrorSource_Sensor  = 0x01,
    LOG_ErrorSource_GSM     = 0x02, 
    LOG_ErrorSource_System  = 0x03,
    LOG_ErrorSource_SecondarySystem  = 0x07
 
}LOG_ErrorSource_t;


typedef enum{

    LOG_Error_FuelGaugeFailed       = 0x00,
    LOG_Error_DS18B20Failed         = 0x01,
    LOG_Error_MPUFailed             = 0x02,
    LOG_Error_MLX90615Failed        = 0x07
    
}LOG_SensorError_t;


typedef enum{

    LOG_RadioError_BeaconTxFailed  = 0x00,
    LOG_RadioError_QueryTxFailed   = 0x01,
    LOG_RadioError_QueryRxTimeout  = 0x02,
    LOG_RadioError_RxFailed        = 0x03,
    LOG_RadioError_NodeRegFailed   = 0x04,
    LOG_RadioError_NodeDropped     = 0x05,
    LOG_RadioError_NodeRegistered  = 0x06,
    LOG_Radio_NodeSNR              = 0x07,
    LOG_RadioError_FwUpdateFailed  = 0x08,
    LOG_RadioError_RegisteredNodes = 0x09,
    LOG_RadioError_MutexTimeout    = 0x0A,
    
}LOG_RadioError_t;


typedef enum{

    LOG_Error_GSMPowerUpFailed      = 0x00,
    LOG_Error_GSMPowerDownFailed    = 0x01,
    LOG_Error_NetworkRegTimeout  = 0x02,
    LOG_Error_SMSReadFailed      = 0x03,
    LOG_Error_SAPBROpenFailed    = 0x04,
    LOG_Error_SAPBRCloseFailed   = 0x05,
    LOG_Error_HTTPPostTimeout    = 0x06,
    LOG_Error_HTTPPostError      = 0x07,
    LOG_Error_FWDownloadTimeout  = 0x08,
    LOG_Error_FWDownloadError    = 0x09,
    LOG_Error_RTCUpdateTimeout   = 0x0A,
    LOG_Error_RTCUpdateFailed    = 0x0B,
    LOG_Error_SendBatchError     = 0x0C,
    LOG_Error_SetURLError        = 0x0D,
    LOG_Error_CoAPPostError      = 0x0E
         
}LOG_GSMError_t;

typedef enum{

    LOG_Error_I2CFailed     	    = 0x00,
    LOG_Error_IWDGOverflow  	    = 0x01,
    LOG_Error_SystemReset   	    = 0x02,
    LOG_Error_HardFaultPC   	    = 0x03,
    LOG_Error_HardFaultLR   	    = 0x04,
	  LOG_Error_ThreadTerminate 	    = 0x05,
    LOG_Error_EEPROM_Settings_CRC   = 0x06,
    LOG_Error_Stack_Overflow        = 0x07,
    LOG_Error_FirmwareDownloaded    = 0x08,
    LOG_Error_FirmwareCRCFailed     = 0x09,
    LOG_Error_FirmwareVersion       = 0x0A,
    LOG_Error_RTCreSync             = 0x0B,
    LOG_Error_HardFaultSP           = 0x0C,
    
}LOG_SystemError_t;


typedef enum{

    LOG_FwUpdateError_InitialCommandNotReceived = 0x00,
    LOG_FwUpdateError_InvalidCommand,
    LOG_FwUpdateError_OffsetTooFarBehind,
    LOG_FwUpdateError_RxTimeout
    
}LOG_FWError;



typedef  uint8_t LOG_ErrorDef_t;         


typedef union __attribute__((packed)){
    
    uint8_t  buff[4];
    uint32_t value;
    
}LOG_ErrorMessage_t;


#pragma anon_unions
typedef struct __attribute__((packed)){

    struct __attribute__((packed)){
        
        uint32_t            timeStamp;
        LOG_ErrorDef_t      error       :   4;
        LOG_ErrorSource_t   source      :   4;  
          
    };
    
    LOG_ErrorMessage_t message;     
    
}LOG_ErrorPacket_t;


typedef struct __attribute__((packed)){

    LOG_ErrorPacket_t errorPacket;
    uint16_t deviceID;
    
}LOG_NodeErrorPacket_t;

typedef struct {
    int16_t SNRsum;
    int16_t RSSIsum;
    uint8_t numOfPackets;
    
}SNR_Log_Struct;

extern mQueue_List_t logs_Queue_List;


LOG_Return_t LOG_Error              (LOG_ErrorSource_t source, LOG_ErrorDef_t error, uint32_t message);
LOG_Return_t LOG_NodeError          (uint16_t id, LOG_ErrorPacket_t* errorPacket);
LOG_Return_t LOG_readError          (void* errorPacket);
uint32_t LOG_get_Count              (void);
LOG_Return_t LOG_clear              (uint32_t count );
LOG_Return_t LOG_readErrorAt        (uint32_t index, void* errorPacket);

#endif
