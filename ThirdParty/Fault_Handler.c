/**
 * @file   	Fault_Handler.c
 * @author 	Hassan
 * @version	
 * @date	Jun 16, 2016
 * 
 * @brief   
 */


#include <Fault_Handler.h>
#include <mIWDG.h>
#include "DEBUG_UART.h"

/**
 * @defgroup Fault_Handler_Exported_Members		Fault Handler Exported Members
 * @{
 */
Fault_Record_t Fault_Record;
/** @}*/

/**
 * @defgroup Fault_Handler_Exported_Members		Fault Handler Exported Members
 * @{
 */
extern mIWDG_HandleTypeDef* hmIWDG_battery;
extern mIWDG_HandleTypeDef* hmIWDG_fuelGauge;
extern mIWDG_HandleTypeDef* hmIWDG_temperature;

/** @}*/


/**
 * @defgroup Fault_Handler_Public_Functions		Fault Handler Public Functions
 * @{
 */



void FuelGauge_Fault_Handler(STC3115_BatteryData_TypeDef* batteryData, STC3115_ConfigData_TypeDef* configData)
{
    printDEBUG("\nThread : fuelGauge \n Fuel gauge update failed \n");
	Fault_Record.FuelGauge.count++;
	Fault_Record.FuelGauge.lastOccurence = RTC_getUnixTime();

	LOG_Error(LOG_ErrorSource_Sensor, LOG_Error_FuelGaugeFailed, 0);

	if (Fault_Record.FuelGauge.count > MAX_FAULT_TOLERANCE_COUNT)
    {
        IWDG_stopWDG(hmIWDG_fuelGauge);
		osThreadTerminate(NULL);
    }
    
    GasGauge_Initialization(configData, batteryData);
}





void DS18B20_Fault_Handler(void)
{
    printDEBUG("\nThread : Temperature, Error reading temperature\n");
	Fault_Record.DS18B20.count++;
	Fault_Record.DS18B20.lastOccurence = RTC_getUnixTime();

	LOG_Error(LOG_ErrorSource_Sensor, LOG_Error_DS18B20Failed, 0);

	if (Fault_Record.DS18B20.count > MAX_FAULT_TOLERANCE_COUNT)
	{
        IWDG_stopWDG(hmIWDG_temperature);
        osThreadTerminate(NULL);
    }
}

void MLX90615_Fault_Handler(void)
{
    printDEBUG("\nThread : Temperature, Error reading IR temperature\n");
	Fault_Record.MLX90615.count++;
	Fault_Record.MLX90615.lastOccurence = RTC_getUnixTime();

	LOG_Error(LOG_ErrorSource_Sensor, LOG_Error_MLX90615Failed, 0);

	if (Fault_Record.MLX90615.count > MAX_FAULT_TOLERANCE_COUNT)
	{
        IWDG_stopWDG(hmIWDG_temperature);
        osThreadTerminate(NULL);
    }
}





/** @}*/
