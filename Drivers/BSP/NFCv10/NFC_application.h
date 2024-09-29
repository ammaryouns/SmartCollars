/**
 * @file   	NFC_Application.h
 * @author 	Umer Ilyas
 * @version	v1.0.0
 * @date	  Sep 4, 2018
 * 
 * @brief   
 */
 
#ifndef _NFC_Application_H_
#define _NFC_Application_H_
#include "stdint.h"

#define NFC_STATUS_ON  1
#define NFC_STATUS_OFF 0

extern volatile uint32_t RFActivity;

void WriteNfcSettings(uint8_t * settings, int32_t size);
void WriteNfcDevieID(uint8_t * devID);
void DetectRFActivity(void);
void ReadNfcConfiguration(void);
void NfcMemoryTest(void);
void Nfc_DeviceSettings(void);


#endif //_NFC_Application_H_
