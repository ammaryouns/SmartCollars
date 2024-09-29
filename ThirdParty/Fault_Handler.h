/**
 * @file   	Fault_Handler.h
 * @author 	Hassan
 * @version	
 * @date	Jun 16, 2016
 * 
 * @brief   
 */

#ifndef FAULT_HANDLER_H_
#define FAULT_HANDLER_H_

#include "bsp.h"
#include <log.h>

#include <stc3115_Driver.h>
/**
 * @defgroup Fault_Handler_Public_Structs			Fault Handler Public Structs
 * @{
 */
 
#define MAX_FAULT_TOLERANCE_COUNT			((uint32_t)10)

/**
 * @brief	Summary structure for faults.
 */
typedef struct{

	uint32_t count;			/*!< Number of times the particular fault has occured.*/
	uint32_t lastOccurence;	/*!< Unix Time stamp of last occurence of fault*/

}Fault_Summary_t;


/**
 * @brief Record of fault summaries for each sensor.
 */
typedef struct{

	Fault_Summary_t DS18B20,		/*!< temperature sensor*/
					FuelGauge,		/*!< STC3115 Fuel gauge*/
					I2C,			/*!< I2C line*/
					MLX90615;		/*!< MLX90615 IR temperature sensor*/

}Fault_Record_t;
/** @}*/


extern Fault_Record_t Fault_Record;

#ifdef __cplusplus
extern "C"{
#endif 

void DS18B20_Fault_Handler(void);
void MLX90615_Fault_Handler(void);
void FuelGauge_Fault_Handler(STC3115_BatteryData_TypeDef* batteryData, STC3115_ConfigData_TypeDef* configData);

#ifdef __cplusplus
}
#endif

#endif /* FAULT_HANDLER_H_ */
