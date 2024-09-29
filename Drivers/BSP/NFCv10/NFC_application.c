/**
 * @file   	NFC_Application.c
 * @author 	Umer Ilyas
 * @version	v1.0.0
 * @date	  Sep 4, 2018
 * 
 * @brief   
 */
 
#include "NFC_Application.h"
#include "st25dv.h"
#include "nfc04a1.h"
#include "DEBUG_UART.h"
#include "Device_Params.h"
#include "i2c.h"

/* Private defines -----------------------------------------------------------*/

/* Private typedefs ----------------------------------------------------------*/

/* Exported Variables --------------------------------------------------------*/
volatile uint32_t RFActivity = 0;


extern osThreadId      hthread_MACWaitForQuery;
extern uint32_t globalDeviceOperatinStatus;
extern TaskHandle_t xTaskMpuIRQ;
extern TaskHandle_t xTaskRadioNFC;


void WriteNfcSettings(uint8_t * settings, int32_t size)
{
     if(ST25DV_IO_MemWrite( settings, ST25DV_ADDR_DATA_I2C, SMARTAG_ADDR_DEVICE_SETTINGS, size )!=NFCTAG_OK) {
     Error_Handler_NFC(NFC_READING_ERROR);     
   }

  
}

void ReadNfcSettings(uint8_t * settings, int32_t size)
{
     if(ST25DV_IO_MemRead( settings, ST25DV_ADDR_DATA_I2C, SMARTAG_ADDR_DEVICE_SETTINGS, size )!=NFCTAG_OK) {
     Error_Handler_NFC(NFC_READING_ERROR);     
   }

  
}

void WriteNfcDevieID(uint8_t * devID)
{

   if(ST25DV_IO_MemWrite( devID, ST25DV_ADDR_DATA_I2C, SMARTAG_ADDR_DEVICE_ID, 12 )!=NFCTAG_OK) {
     Error_Handler_NFC(NFC_READING_ERROR);     
   }
 
}
/* Private variables ---------------------------------------------------------*/
volatile uint8_t NFCStatus = NFC_STATUS_ON;
/* R/F Activity from ST25DV */
static uint32_t RFActivityStatus = FIELD_UNDEF;
static uint32_t deviceOperatinStatus = -1;
static Device_Flash_Params_t deviceSettings_nfc;

/*
check if there is any update of settings in NFC memory
Or save the device settings in NFC memory
*/
void Nfc_DeviceSettings(void)
{
  uint32_t command = 0;
  if(ST25DV_IO_MemRead( (uint8_t*)&command, ST25DV_ADDR_DATA_I2C, SMARTAG_ADDR_DEVICE_COMMAND, 4 )!=NFCTAG_OK) {
     Error_Handler_NFC(NFC_READING_ERROR);   
  }
  if (command == 0x45)  // update the Device settings, NFC has new settings.
  {
    ReadNfcSettings((uint8_t *) &deviceSettings_nfc, sizeof(deviceSettings_nfc));
    if(deviceSettings_nfc.crc == deviceParams_computeCRC(&deviceSettings_nfc)) // write to memmory if CRC is ok
    {
      deviceParams_commitToFlash(&deviceSettings_nfc);
    }
    if(deviceParam_isFlashUpToDate(&deviceSettings_crc) == 0) // update the ram device settings if there is some change
    {
      deviceParams_pullFromFlash(&deviceSettings_crc);
    }
    
    command = 0;  // reset the nfc commnad
    if(ST25DV_IO_MemWrite( (uint8_t*)&command, ST25DV_ADDR_DATA_I2C, SMARTAG_ADDR_DEVICE_COMMAND, 4 )!=NFCTAG_OK) {
     Error_Handler_NFC(NFC_READING_ERROR);   
    }  
  }
  else // No new settings from NFC
  {
    if(deviceSettings_nfc.crc != deviceSettings_crc.crc)
      {    
        WriteNfcSettings((uint8_t *) &deviceSettings_crc, sizeof(deviceSettings_crc));
      }   
  }
}
void NfcMemoryTest(void)
{
  uint32_t commandWrite = 0x5A;
  uint32_t commandRead = 0;
  if(ST25DV_IO_MemWrite( (uint8_t*)&commandWrite, ST25DV_ADDR_DATA_I2C, SMARTAG_ADDR_DEVICE_COMMAND, 4 ) ==NFCTAG_OK) {
    if(ST25DV_IO_MemRead( (uint8_t*)&commandRead, ST25DV_ADDR_DATA_I2C, SMARTAG_ADDR_DEVICE_COMMAND, 4 ) ==NFCTAG_OK) {
        printTask("NFC memory read successful");   
    }
  }
  else
  {        
    printTask("NFC memory read Failed");   

    Error_Handler_NFC(NFC_READING_ERROR);   
  }
 
}
void ReadNfcConfiguration(void)
{
  uint32_t DataBuf32;
  uint8_t *DataBuf8 = (uint8_t *)&DataBuf32;
  uint16_t SampleCounter;
  uint16_t LastSamplePointer;

  /* Read command code */
  if(ST25DV_IO_MemRead( DataBuf8, ST25DV_ADDR_DATA_I2C, SMARTAG_ADDR_DEVICE_OPERATION, 4 )!=NFCTAG_OK) {
     Error_Handler_NFC(NFC_READING_ERROR);
  }
  else
  {
  printTask("SMARTAG_ADDR_DEVICE_OPERATION -> %x", DataBuf32);        
  if(deviceOperatinStatus != DataBuf32)
  {
    deviceOperatinStatus = DataBuf32;
    globalDeviceOperatinStatus = deviceOperatinStatus;
    if(DataBuf32 == 0x55) // enable if this value is in memory
    {  
      if(xTaskMpuIRQ != NULL)
      {
        /* Notify the task that the transmission is complete. */
        xTaskNotifyGive( xTaskMpuIRQ);
      }
      if(xTaskRadioNFC != NULL)
      {
        /* Notify the task that the transmission is complete. */
        xTaskNotifyGive( xTaskRadioNFC);
      }
      //      osThreadResume(hthread_MACWaitForQuery);
    }
    else // disable everythig
    {     
//      osThreadSuspend(hthread_MACWaitForQuery);

    }
  }
  } 
  if(globalDeviceOperatinStatus == 0x55)
  {
    if(xTaskMpuIRQ != NULL)
    {
      /* Notify the task that the transmission is complete. */
      xTaskNotifyGive( xTaskMpuIRQ);
    }    
    if(xTaskRadioNFC != NULL)
    {
      /* Notify the task that the transmission is complete. */
      xTaskNotifyGive( xTaskRadioNFC);
    }
  }

}
/**
  * @brief  Understanding the Interrupt from ST25DV
  * @param  None
  * @retval None
  */
void DetectRFActivity(void)
{
  volatile uint8_t ITStatus;
    

  if(NFCStatus == NFC_STATUS_OFF ) {
    NFCStatus = NFC_STATUS_ON;
    PowerOnNFC();
    /* rise time required by VDD_EEPROM for NFC */
    HAL_Delay(200);
  
    if( HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_RESET )
    {
      printDEBUG("MX_I2C1_Init \r\n");
      MX_I2C1_Init();
    }
  }

  /* Read if the Field is Rising or not */
  if(St25Dv_i2c_ExtDrv.ReadITSTStatus_Dyn(&ITStatus)!=NFCTAG_OK ) {
    Error_Handler_NFC(NFC_CONFIG_ERROR);
  }
    while(ITStatus != 0)
    {
  printDEBUG("ITStatus %02X\r\n", ITStatus);
  if(ITStatus&0x10) {
    /* FIELD RISING */
    RFActivityStatus = FIELD_RISING;
    printDEBUG("Detected NFC FIELD_RISING\r\n");
  } 
  if(ITStatus&0x08) {
    /* FIELD FALLING */
    RFActivityStatus = FIELD_FALLING;
    printDEBUG("Detected NFC FIELD_FALLING\r\n\n");
    /* Control if there is a new configuration or a new command for
     * setting the TimeStamp */
    
  /* Read command code */
    ReadNfcConfiguration();
    Nfc_DeviceSettings();
    
//    NFCStatus = NFC_STATUS_OFF;
//    PowerOffNFC();
  }  
  osDelay(1000);
  if(St25Dv_i2c_ExtDrv.ReadITSTStatus_Dyn(&ITStatus)!=NFCTAG_OK ) {
    Error_Handler_NFC(NFC_CONFIG_ERROR);
  }
  printDEBUG("Detected NFC                  END\r\n\n");

}

}
