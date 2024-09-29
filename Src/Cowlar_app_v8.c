/**
 * @file   	Cowlar_app_v8.c
 * @author 	Umer
 * @version	
 * @date	Nov 8, 2017
 * 
 * @brief   
 */
 

#include "Cowlar_app_v8.h"
#include "mRTC.h"
#include "mIWDG.h"
#include "log.h"
#include "SensorDefs.h"
#include "DEBUG_UART.h"

#include "SensorTile_accelero.h"
#include "SensorTile_gyro.h"

#ifdef CLASSIFICATION_APP	  

#include "MPU_ANN.h"
#include "MPU_Filter.h"
#include "behavior_ANN.h"
#include "walking_ANN.h"
#include "MPU_Features.h"
#endif

#include "Device_Params.h"
#include <arm_math.h>



#define SENSOR_ERROR_RETRIES                3u


//#define printMPU(fmt, ...)         printTask(fmt, ##__VA_ARGS__ );
#define printMPU(...)         

//Software watchdogs for threads.
mIWDG_HandleTypeDef* hmIWDG_battery;
mIWDG_HandleTypeDef* hmIWDG_MPU;

extern uint32_t globalDeviceOperatinStatus;

osThreadId   hthread_MPU;
osThreadId   hthread_battery;

//global variables for battery thread.
float batteryVoltage = 3.5f;
float batteryPercent[10];
uint32_t batteryIndex = 0;
uint32_t batterySamples  = 2;


SensorsData_t nodeBehaviorData;
uint32_t      nodeBehaviorDataCount = 0;
SensorsData_t nodeBehaviorUnknownData;
SensorsData_t nodeSensorData;
SensorsData_t SDcardData;

/**
 * @brief  Calculate the Slot index of the current window in sampling cycle.
 * @return Current Sample Slot index in day.
 */
uint32_t getCurrentSampleSlot()
{
    RTC_unixTime = RTC_getUnixTime();
    return (RTC_unixTime % (48*RECIPE_WINDOW_DURATION)) / SENSORS_BEHAVIOUR_LOG_DURATION;
}

/**
 * @brief Check if data is to be saved in the current sample slot.
 * @return 
 *          @arg true   If sensor data is to be saved in the current slot.
 *          @arg false  If sensor data is not to be saved.
 */
bool acquireData()
{
    RTC_unixTime = RTC_getUnixTime();
    uint32_t currSampleSlot = (RTC_unixTime % (48*RECIPE_WINDOW_DURATION)) / RECIPE_WINDOW_DURATION;
    #ifndef IS_GATEWAY_NODE
    uint8_t bitFeild = deviceSettings.dataSequence.acquireData[currSampleSlot/8];
    #else
    uint8_t bitFeild = deviceSettings.DCU.dataSequence.acquireData[currSampleSlot/8];
    #endif
    
    return bitFeild & (1 << ( 7 - (currSampleSlot%8) ));
}
uint8_t SampleTimeBits(uint32_t sensorSampleWindowDuration)
{
  uint8_t SampleTime = 0;
  switch(sensorSampleWindowDuration)
  {
    case 1*60: SampleTime =T_1_min;break;
    case 2*60: SampleTime =T_2_min;break;
    case 3*60: SampleTime =T_3_min;break;
    case 4*60: SampleTime =T_4_min;break;
    case 5*60: SampleTime =T_5_min;break;
    case 6*60: SampleTime =T_6_min;break;
    case 10*60: SampleTime =T_10_min;break;
    case 15*60: SampleTime =T_15_min;break;
    case 20*60: SampleTime =T_20_min;break;
    case 30*60: SampleTime =T_30_min;break;
    case 60*60: SampleTime =T_60_min;break;
    default: SampleTime =T_30_min;break;
  }
  return SampleTime;
}
float batteryPercentAvg(void)
{
    float battery = 0;
    for(int i = 0;i<batterySamples;i++)
    {
        battery += batteryPercent[i];
    }
    return (battery/batterySamples);
}



#define t_BATTERY_SLEEP_TIME    ((SENSORS_BATTERY_SAMPLE_DURATION  + 100)*1000)                    //in mS
//time 
void thread_battery(void const * argument)
{
  hmIWDG_battery = IWDG_initWDG(t_BATTERY_SLEEP_TIME );        
  IWDG_startWDG(hmIWDG_battery);
  static float bat_voltage = 0;
  uint32_t batteryNextSampleSlot = 0;
  uint32_t batteryNextSaveSlot = 0;
  
  uint32_t batteryFirstRun = 0;

//  uint32_t batteryPrevLogSlot = 0;
//  batteryPrevLogSlot = getCurrentSampleSlot();
//  uint32_t batteryCurrLogSlot = 0;
  uint32_t unixTimeBattery = RTC_getUnixTime();
  
#ifdef BATTERY_ADC_UNIT_TEST
    for(int32_t i =0; i< 3; i++)   // run this 10 times with 2 sec for each loop  for power measure 
    {
      uint32_t  vRefAdc;
      osDelay(20);                  //allow time for the capacitor to charge. Time constant is 20 milliseconds.   
      ADC_BATTERY_ENABLE();
      bat_voltage = ADC_GetValue(BATTERY_ADC_CHANNEL, 1, &vRefAdc);
      ADC_BATTERY_DISABLE();                      
      printTask("ADC Voltage  : %f V , vRef = %f", bat_voltage, ((float)vRefAdc / 1000.0f) );
      
      bat_voltage = bat_voltage * (((float)vRefAdc / 1000.0f)) * BATTERY_VOLTAGE_SCALING_FACTOR;
      printTask("Battery Voltage level : %3.4f V", bat_voltage);
      if(bat_voltage > 3.0f) break;
      osDelay(40);                   //allow time for the capacitor to charge. Time constant is 20 milliseconds.
    }   
    
#endif
  /* Infinite loop */
  for(;;)
  {            
    IWDG_refreshWDG(hmIWDG_battery);
    unixTimeBattery = RTC_getUnixTime();
#ifdef BATTERY_ADC_UNIT_TEST
    
    osDelay(10000);   // go to sleep after adc read for 20 sec
#else


#endif
  }
  
  osThreadTerminate(NULL);
}


/*
 *
 *        Motion sensor app
 *        Get Data from LSM6DSM and process it
 *        Apply the ANN on and save the result
 *
 *
 */

#define MPU_ACTIVITY_COUNT   4             //preferably a factor of minutes in refreshtime
#define MPU_ACTIVITY_TIME    ((uint32_t)(SENSORS_BEHAVIOUR_LOG_DURATION))

#define MPU_ENERGY_WINDOW_LENGTH (DEFAULT_MPU_HZ * 60)
#define MPU_ENERGY_THRESHOLD     ((float)0.02f * MPU_ENERGY_WINDOW_LENGTH)
  
typedef struct
{
	int16_t accl[3];
}fifoPattern_t;

#define mpuBufferMaxSize          1800
#define MPU_FIFO_SIZE 	          4096

uint32_t        annBehaviorBuffer_count = 0;
uint32_t        mpuBuffer_count = 0;
float           mpuBuffer[3][mpuBufferMaxSize];
fifoPattern_t   fifoMirror[MPU_FIFO_SIZE / sizeof(fifoPattern_t)];
#ifdef CLASSIFICATION_APP	  

#endif

TaskHandle_t xTaskMpuIRQ = NULL;



int32_t gyro[3];
int32_t accl[3];
int32_t fifo_couter = 0;;
LSM6DSM_ACC_GYRO_FIFO_FULL_t FIFO_FULL;
uint16_t FIFONumOfEntries = 0;  
uint16_t afterreadFIFONumOfEntries = 0;  
uint16_t LastFIFONumOfEntries = 0;  
uint8_t PedometerStatus = 0;
uint16_t PedometerStepCount = 0;
uint32_t FifoFul;
uint32_t FifoFulForRadio = 0;
uint32_t DataPointsThisSample = 0;
uint32_t DataPointsForRadio = 0;

volatile uint8_t MEMSInterrupt = 0;
void *LSM6DSM_X_0_handle;
void *LSM6DSM_G_0_handle;


#ifdef RAW_DATA_COLLECTION 

#endif


uint32_t stepsCount = 0;
uint32_t stepsCountLast = 0;
/**
* @brief  Initialize all sensors
* @param  None
* @retval None
*/
static void initializeAllSensors( void )
{
  if (BSP_ACCELERO_Init( LSM6DSM_X_0, &LSM6DSM_X_0_handle ) != COMPONENT_OK)
  {
    while(1);
  }
  if (BSP_GYRO_Init( LSM6DSM_G_0, &LSM6DSM_G_0_handle ) != COMPONENT_OK)
  {
    while(1);
  }

	if (	BSP_ACCELERO_Enable_Pedometer_Ext(LSM6DSM_X_0_handle)!= COMPONENT_OK)
	{
		while(1);
	}
  BSP_ACCELERO_Reset_Step_Counter_Ext(LSM6DSM_X_0_handle);   // reset the step counter to zero
	if (LSM6DSM_ACC_GYRO_Enable_FIFO_stream( LSM6DSM_X_0_handle, 1, 3276 ) != MEMS_SUCCESS)
	{
		while(1);
	}
  status_t s;
  s = LSM6DSM_ACC_GYRO_W_HP_SLOPE_XL( LSM6DSM_X_0_handle, LSM6DSM_ACC_GYRO_HP_SLOPE_XL_EN);
  s = LSM6DSM_ACC_GYRO_W_LowPassFiltSel_XL( LSM6DSM_X_0_handle, LSM6DSM_ACC_GYRO_LPF2_XL_ENABLE);
//  s = LSM6DSM_ACC_GYRO_W_LowPassFiltSel_XL( LSM6DSM_X_0_handle, LSM6DSM_ACC_GYRO_LPF2_XL_DISABLE);
//  s = LSM6DSM_ACC_GYRO_W_BW_SEL( LSM6DSM_X_0_handle, LSM6DSM_ACC_GYRO_BW_SEL_ODR2);
  s = LSM6DSM_ACC_GYRO_W_InComposit( LSM6DSM_X_0_handle, LSM6DSM_ACC_GYRO_IN_ODR_DIV_4);
  s = LSM6DSM_ACC_GYRO_W_HPCF_XL( LSM6DSM_X_0_handle, LSM6DSM_ACC_GYRO_HPCF_XL_DIV4);


}
/**
* @brief  Initialize all sensors
* @param  None
* @retval None
*/
static void DeInitializeAllSensors( void )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)LSM6DSM_X_0_handle;
  ACCELERO_Drv_t *driver = NULL;  
  if(ctx != NULL)
  {
    driver = ( ACCELERO_Drv_t * )ctx->pVTable;
    if ( driver->DeInit != NULL )
    {
      driver->DeInit((DrvContextTypeDef *)(LSM6DSM_X_0_handle));
    }  
  }
  ctx = (DrvContextTypeDef *)LSM6DSM_G_0_handle;
  driver = NULL;  
  if(ctx != NULL)
  {
    driver = ( ACCELERO_Drv_t * )ctx->pVTable;
    if ( driver->DeInit != NULL )
    {
      driver->DeInit((DrvContextTypeDef *)(LSM6DSM_G_0_handle));
    }  
  } 
}
/**
* @brief  Enable all sensors
* @param  None
* @retval None
*/
void enableAllSensors( void )
{
  BSP_ACCELERO_Sensor_Enable( LSM6DSM_X_0_handle );
//  BSP_GYRO_Sensor_Enable( LSM6DSM_G_0_handle );
}

status_t ConvertToRaw_ACC( int16_t *buff, uint16_t len)
{
	int32_t i = 0;
  if(mpuBuffer_count + len > mpuBufferMaxSize)
  {
    len = mpuBufferMaxSize - mpuBuffer_count;  // donot over flow the fifo and stop when internal ram is full
  }
	for(i = 0; i< len; i++)
	{
		/* Apply proper shift and sensitivity    LSM6DSM_ACC_GYRO_FS_XL_2g */
		/* also Apply rotation vector */
		/* orientation : antenna on the bottom - BQ on the top - component facing towards cow*/
		mpuBuffer[0][i + mpuBuffer_count] = (buff[i*3 + 0] * (float)(-0.000061));
		mpuBuffer[1][i + mpuBuffer_count] = (buff[i*3 + 1] * (float)( 0.000061));
		mpuBuffer[2][i + mpuBuffer_count] = (buff[i*3 + 2] * (float)(-0.000061));
	}
  mpuBuffer_count += len; 
  return MEMS_SUCCESS;
}

int New_Accel_results(float* accel)
{       
  if (mpuBuffer_count < sizeof(mpuBuffer[0]) / sizeof(mpuBuffer[0][0]))
  {
    mpuBuffer[0][mpuBuffer_count] = accel[0];
    mpuBuffer[1][mpuBuffer_count] = accel[1];
    mpuBuffer[2][mpuBuffer_count] = accel[2];        
    mpuBuffer_count++;
  }

  return 0;
} 
/**
 * @brief Proccess the fifo data
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Proccess( void *handle )
{
  DrvStatusTypeDef status;
  status = BSP_ACCELERO_Get_Pedometer_Status_Ext(handle, &PedometerStatus);
  status = BSP_ACCELERO_Get_Step_Count_Ext(handle, &PedometerStepCount);
  
#ifdef RAW_DATA_COLLECTION  
  steps = PedometerStepCount;
#endif
//  stepsCount = PedometerStepCount;
  if(status != COMPONENT_OK)
  {
    printMPU("Step Count [%d]  \n", PedometerStepCount);
    return COMPONENT_ERROR;
  }
  /// something to do with PedometerStatus and PedometerStepCount
  printMPU("Step Count [%d]  \n", PedometerStepCount);
  
  LSM6DSM_ACC_GYRO_R_FIFOFull(handle, &FIFO_FULL);
  if(FIFO_FULL == LSM6DSM_ACC_GYRO_FIFO_FULL_FIFO_FULL)
  {
    FifoFul = 1;FifoFulForRadio = 1;
    LSM6DSM_ACC_GYRO_Disable_FIFO_stream(LSM6DSM_X_0_handle);
    LSM6DSM_ACC_GYRO_Enable_FIFO_stream( LSM6DSM_X_0_handle, 1, 3120 );
  }
  uint16_t FIFOPattern = 0;
  uint8_t raw_data_tmp[2];
  LSM6DSM_ACC_GYRO_R_FIFONumOfEntries(handle, &FIFONumOfEntries);
  
  LastFIFONumOfEntries = FIFONumOfEntries;
  while(FIFONumOfEntries > 2)
  {
    
    fifo_couter++;
    status_t status_mems = LSM6DSM_ACC_GYRO_R_FIFOPattern(handle, &FIFOPattern);	
    if(status_mems != MEMS_SUCCESS)
    {
      printMPU("Step Count [%d]  \n", PedometerStepCount);
      return COMPONENT_ERROR;
    }		

    /*   Only Accl  */
    if(	FIFOPattern == 0)
    {
      uint16_t samples = (FIFONumOfEntries / 3);   // three axis 
      status_mems = LSM6DSM_ACC_GYRO_Get_GetFIFOnData(handle, (uint8_t*)&fifoMirror[0], samples);  /// read from fifo @todo: DMA

#ifdef CLASSIFICATION_APP	  
      status_mems = ConvertToRaw_ACC((int16_t*)&fifoMirror[0], samples);						
#endif
#ifdef RAW_DATA_COLLECTION  
      SDmpuBuffer_count += samples; 
      NextSampleTimeSeconds       = LastSampleTimeSeconds;            // use this time now for SD card write;
      NextSampleTimeMilliSeconds  = LastSampleTimeMilliSeconds;


      RTC_getUnixTimeMs(&LastSampleTimeSeconds, &LastSampleTimeMilliSeconds);   // read current time to use for next time Sd card write
#endif
      FIFONumOfEntries -= (samples * 3);
      status_t status_mems = LSM6DSM_ACC_GYRO_R_FIFONumOfEntries(handle, &afterreadFIFONumOfEntries);	
//      status_mems = LSM6DSM_ACC_Get_Acceleration(handle, accl, 1);	
//      FIFONumOfEntries -=  3;
      if(status_mems != MEMS_SUCCESS)
      {
        printMPU("Step Count [%d]  \n", PedometerStepCount);
        return COMPONENT_ERROR;
      }		
    }
    else if(	FIFOPattern == 1) // in case pointer got out of sync
    {
      LSM6DSM_ACC_GYRO_Get_GetFIFOData(handle, raw_data_tmp); // read two bytes 
      FIFONumOfEntries -= 1;
      printMPU("FiFo Pattern mismath \n");

    }  
    else if(	FIFOPattern == 2) // in case pointer got out of sync
    {
      LSM6DSM_ACC_GYRO_Get_GetFIFOData(handle, raw_data_tmp); // read two bytes 
      FIFONumOfEntries -= 1;
      LSM6DSM_ACC_GYRO_Get_GetFIFOData(handle, raw_data_tmp); // read two bytes 
      FIFONumOfEntries -= 1;
      printMPU("FiFo Pattern mismath \n");

    }
    else
    {
      LSM6DSM_ACC_GYRO_Get_GetFIFOData(handle, raw_data_tmp); // read two bytes 
      FIFONumOfEntries -= 1;
      LSM6DSM_ACC_GYRO_Get_GetFIFOData(handle, raw_data_tmp); // read two bytes 
      FIFONumOfEntries -= 1;
    }
  }
  
  return COMPONENT_OK;
		
}


#ifdef MPU_UNIT_TEST

/**
 * @brief Proccess the fifo data
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef MPU_UnitTestGravityAndFifo( void *handle )
{
  DrvStatusTypeDef status;
  status = BSP_ACCELERO_Get_Pedometer_Status_Ext(handle, &PedometerStatus);
  status = BSP_ACCELERO_Get_Step_Count_Ext(handle, &PedometerStepCount);

//  stepsCount = PedometerStepCount;
  if(status != COMPONENT_OK)
  {
    printMPU("Step Count [%d]  \n", PedometerStepCount);
    return COMPONENT_ERROR;
  }
  /// something to do with PedometerStatus and PedometerStepCount
  printMPU("Step Count [%d]  \n", PedometerStepCount);
  
  LSM6DSM_ACC_GYRO_R_FIFOFull(handle, &FIFO_FULL);
  if(FIFO_FULL == LSM6DSM_ACC_GYRO_FIFO_FULL_FIFO_FULL)
  {
    FifoFul = 1;FifoFulForRadio = 1;
    LSM6DSM_ACC_GYRO_Disable_FIFO_stream(LSM6DSM_X_0_handle);
    LSM6DSM_ACC_GYRO_Enable_FIFO_stream( LSM6DSM_X_0_handle, 1, 3120 );
  }
  uint16_t FIFOPattern = 0;
  uint8_t raw_data_tmp[2];
  LSM6DSM_ACC_GYRO_R_FIFONumOfEntries(handle, &FIFONumOfEntries);
  uint32_t currentTicks = osKernelSysTick();
  
  LastFIFONumOfEntries = FIFONumOfEntries;
  {
    
    fifo_couter++;
    status_t status_mems = LSM6DSM_ACC_GYRO_R_FIFOPattern(handle, &FIFOPattern);	
    if(status_mems != MEMS_SUCCESS)
    {
      printMPU("Step Count [%d]  \n", PedometerStepCount);
      return COMPONENT_ERROR;
    }		

    /*   Only Accl  */
    if(	FIFOPattern == 0)
    {
      uint16_t samples = (FIFONumOfEntries / 3);   // three axis 
      status_mems = LSM6DSM_ACC_GYRO_Get_GetFIFOnData(handle, (uint8_t*)&fifoMirror[0], samples);  /// read from fifo @todo: DMA
      uint32_t samplesi= 0;
      #define offsetInData 40
      printTask("LSM samples : %d\n", samples);
      float gravity_mean = 0;
      for(samplesi = offsetInData; samplesi< samples && samplesi< offsetInData + 5; samplesi++)
      {
        float x = (float)fifoMirror[samplesi].accl[0] * ( 0.000061f);
        float y = (float)fifoMirror[samplesi].accl[1] * ( 0.000061f);
        float z = (float)fifoMirror[samplesi].accl[2] * ( 0.000061f);
        float gravity2 = (x*x) + (y*y) + (z*z);
        float gravity;
        arm_sqrt_f32(gravity2, &gravity);
        printTask("LSM [%2.3f][%2.3f][%2.3f][%2.3f]", 
                                (float)fifoMirror[samplesi].accl[0] * ( 0.000061f),
                                (float)fifoMirror[samplesi].accl[1] * ( 0.000061f),
                                (float)fifoMirror[samplesi].accl[2] * ( 0.000061f),
                                 gravity);
        gravity_mean += gravity;
      }
      gravity_mean /= 5;  
      if ( 0.97 < gravity_mean && gravity_mean < 1.03 )
      {
        printTask("gravity vector verified");
      }
      
      #define MPU_UNITTEST_FIFO_TEST_DELAY    20000
      osDelay(MPU_UNITTEST_FIFO_TEST_DELAY);
      uint32_t totalTicks = osKernelSysTick() - currentTicks;
      status_mems = LSM6DSM_ACC_GYRO_R_FIFONumOfEntries(handle, &FIFONumOfEntries);
      if (status_mems != MEMS_SUCCESS)
        return COMPONENT_ERROR;
      status_mems = LSM6DSM_ACC_GYRO_R_FIFOPattern(handle, &FIFOPattern);	
      if (status_mems != MEMS_SUCCESS)
        return COMPONENT_ERROR;
      /*   Only Accl  */
      if(	FIFOPattern == 0)
      {
        uint16_t samples = (FIFONumOfEntries / 3);   // three axis 
        status_mems = LSM6DSM_ACC_GYRO_Get_GetFIFOnData(handle, (uint8_t*)&fifoMirror[0], samples);  /// read from fifo @todo: DMA
        if (status_mems != MEMS_SUCCESS)
          return COMPONENT_ERROR;
        float freq = ((float)samples * 1000.0f ) / (float) (totalTicks);
        printTask("Number of samples  [%d] time [%d]  freq  [%3.2f]", samples, totalTicks, freq);
        if (25 <= freq && freq <= 27)
        {
          printTask("Fifo OK");
        }
        else
        {
          printTask("Fifo Not OK");
        }
      }


      FIFONumOfEntries -= (samples * 3);
      status_t status_mems = LSM6DSM_ACC_GYRO_R_FIFONumOfEntries(handle, &afterreadFIFONumOfEntries);	
//      status_mems = LSM6DSM_ACC_Get_Acceleration(handle, accl, 1);	
//      FIFONumOfEntries -=  3;
      if(status_mems != MEMS_SUCCESS)
      {
        printMPU("Step Count [%d]  \n", PedometerStepCount);
        return COMPONENT_ERROR;
      }		
    }
    else if(	FIFOPattern == 1) // in case pointer got out of sync
    {
      LSM6DSM_ACC_GYRO_Get_GetFIFOData(handle, raw_data_tmp); // read two bytes 
      FIFONumOfEntries -= 1;
      printMPU("FiFo Pattern mismath \n");

    }  
    else if(	FIFOPattern == 2) // in case pointer got out of sync
    {
      LSM6DSM_ACC_GYRO_Get_GetFIFOData(handle, raw_data_tmp); // read two bytes 
      FIFONumOfEntries -= 1;
      LSM6DSM_ACC_GYRO_Get_GetFIFOData(handle, raw_data_tmp); // read two bytes 
      FIFONumOfEntries -= 1;
      printMPU("FiFo Pattern mismath \n");

    }
    else
    {
      LSM6DSM_ACC_GYRO_Get_GetFIFOData(handle, raw_data_tmp); // read two bytes 
      FIFONumOfEntries -= 1;
      LSM6DSM_ACC_GYRO_Get_GetFIFOData(handle, raw_data_tmp); // read two bytes 
      FIFONumOfEntries -= 1;
    }
  }
  
  return COMPONENT_OK;
		
}

#define MPU_SELFTEST_SAMPLE_UNIT  5
#define MPU_SELFTEST_MIN_ACC      90
#define MPU_SELFTEST_MAX_ACC      1700

DrvStatusTypeDef MPU_SelfTest(void *handle)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;
  SensorAxes_t  acceleration_TEMP[1];     
  SensorAxes_t  acceleration_NOST[MPU_SELFTEST_SAMPLE_UNIT];     // 5 samples without selftest
  SensorAxes_t  acceleration_ST[MPU_SELFTEST_SAMPLE_UNIT];       // 5 samples with selftest
  SensorAxes_t  acceleration_NOST_mean;    
  SensorAxes_t  acceleration_ST_mean;      
  SensorAxes_t  acceleration_ST_diff;      
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  driver = ( ACCELERO_Drv_t * )ctx->pVTable;

  if ( driver->Get_Axes == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  printTask("MPU unit test satarted Positive sign self-test\n");

  
  if( !LSM6DSM_ACC_GYRO_WriteReg(handle, LSM6DSM_ACC_GYRO_CTRL1_XL, 0x38) )   //0x38 > 52Hz & 4G
    return COMPONENT_ERROR;
  
  if( !LSM6DSM_ACC_GYRO_WriteReg(handle, LSM6DSM_ACC_GYRO_CTRL2_G, 0x00) )
    return COMPONENT_ERROR;

  if( !LSM6DSM_ACC_GYRO_WriteReg(handle, LSM6DSM_ACC_GYRO_CTRL3_C, 0x44) )    // BDU =1
    return COMPONENT_ERROR;
 
  if( !LSM6DSM_ACC_GYRO_WriteReg(handle, LSM6DSM_ACC_GYRO_CTRL4_C, 0x00) )
    return COMPONENT_ERROR;
 
  if( !LSM6DSM_ACC_GYRO_WriteReg(handle, LSM6DSM_ACC_GYRO_CTRL5_C, 0x00) )
    return COMPONENT_ERROR;
 
  if( !LSM6DSM_ACC_GYRO_WriteReg(handle, LSM6DSM_ACC_GYRO_CTRL6_G, 0x00) )
    return COMPONENT_ERROR;
 
  if( !LSM6DSM_ACC_GYRO_WriteReg(handle, LSM6DSM_ACC_GYRO_CTRL7_G, 0x00) )
    return COMPONENT_ERROR;
 
  if( !LSM6DSM_ACC_GYRO_WriteReg(handle, LSM6DSM_ACC_GYRO_CTRL8_XL, 0x00) )
    return COMPONENT_ERROR;
 
  if( !LSM6DSM_ACC_GYRO_WriteReg(handle, LSM6DSM_ACC_GYRO_CTRL9_XL, 0x00) )
    return COMPONENT_ERROR;
 
  if( !LSM6DSM_ACC_GYRO_WriteReg(handle, LSM6DSM_ACC_GYRO_CTRL10_C, 0x00) )
    return COMPONENT_ERROR;
  //Initialize Sensor, Turn on Sensor, enable X/Y/Z axes.
  //Set BDU=1, FS=4G, ODR=52Hz
  osDelay(100);
  
  uint8_t status = 0;
  LSM6DSM_ACC_GYRO_XLDA_t status_raw;

  while( status  == 0)
  { 
    if ( driver->Get_DRDY_Status( ctx, &status) != COMPONENT_OK )
      return COMPONENT_ERROR;
    osDelay(10);
  }
  if ( driver->Get_Axes( ctx, acceleration_TEMP ) == COMPONENT_ERROR )   // discard first smaple
  {
    return COMPONENT_ERROR;
  }
  osDelay(2);

  
  for(int i = 0; i < MPU_SELFTEST_SAMPLE_UNIT; i++)
  {
    while(status == 0)
    {
      if ( driver->Get_DRDY_Status( ctx, &status) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      osDelay(1);
    }
    if ( driver->Get_Axes( ctx, &acceleration_NOST[i] ) == COMPONENT_ERROR )
    {
      return COMPONENT_ERROR;
    }
    osDelay(1);
  }

  
  
  
  if( !LSM6DSM_ACC_GYRO_WriteReg(handle, LSM6DSM_ACC_GYRO_CTRL5_C, 0x01) )        // Enable postive Acc SelfTest
    return COMPONENT_ERROR;
 
  osDelay(100);
  status = 0;
  
  while( status  == 0)
  { 
    if ( driver->Get_DRDY_Status( ctx, &status) != COMPONENT_OK )
      return COMPONENT_ERROR;
    osDelay(10);
  }
  if ( driver->Get_Axes( ctx, acceleration_TEMP ) == COMPONENT_ERROR )   // discard first smaple
  {
    return COMPONENT_ERROR;
  }
  osDelay(2);

  
  for(int i = 0; i < MPU_SELFTEST_SAMPLE_UNIT; i++)
  {
    while(status == 0)
    {
      if ( driver->Get_DRDY_Status( ctx, &status) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      osDelay(1);
    }
    if ( driver->Get_Axes( ctx, &acceleration_ST[i] ) == COMPONENT_ERROR )
    {
      return COMPONENT_ERROR;
    }
    osDelay(1);
  } 
//  printTask(" No selfTest");
//  for(int i = 0; i < MPU_SELFTEST_SAMPLE_UNIT; i++)
//  {
//    printTask(" [%d][%d][%d]" , acceleration_NOST[i].AXIS_X, acceleration_NOST[i].AXIS_Y, acceleration_NOST[i].AXIS_Z); 
//  }
//  
//  printTask(" with selfTest");
//  for(int i = 0; i < MPU_SELFTEST_SAMPLE_UNIT; i++)
//  {
//    printTask(" [%d][%d][%d]" , acceleration_ST[i].AXIS_X, acceleration_ST[i].AXIS_Y, acceleration_ST[i].AXIS_Z); 
//  }  
//  printTask(" Difference");
//  for(int i = 0; i < MPU_SELFTEST_SAMPLE_UNIT; i++)
//  {
//    printTask(" [%d][%d][%d]" ,
//    (int16_t)(acceleration_ST[i].AXIS_X - acceleration_NOST[i].AXIS_X),
//    (int16_t)(acceleration_ST[i].AXIS_Y - acceleration_NOST[i].AXIS_Y),
//    (int16_t)(acceleration_ST[i].AXIS_Z - acceleration_NOST[i].AXIS_Z)); 
//  }  

  acceleration_NOST_mean.AXIS_X = 0;
  acceleration_NOST_mean.AXIS_Y = 0;
  acceleration_NOST_mean.AXIS_Z = 0;
  
  acceleration_ST_mean.AXIS_X = 0;
  acceleration_ST_mean.AXIS_Y = 0;
  acceleration_ST_mean.AXIS_Z = 0;
  
  //take average
  for(int i = 0; i < MPU_SELFTEST_SAMPLE_UNIT; i++)
  {
    acceleration_NOST_mean.AXIS_X += acceleration_NOST[i].AXIS_X;
    acceleration_NOST_mean.AXIS_Y += acceleration_NOST[i].AXIS_Y;
    acceleration_NOST_mean.AXIS_Z += acceleration_NOST[i].AXIS_Z;
    
    acceleration_ST_mean.AXIS_X += acceleration_ST[i].AXIS_X;
    acceleration_ST_mean.AXIS_Y += acceleration_ST[i].AXIS_Y;
    acceleration_ST_mean.AXIS_Z += acceleration_ST[i].AXIS_Z;
  }    
  acceleration_ST_diff.AXIS_X = (acceleration_ST_mean.AXIS_X - acceleration_NOST_mean.AXIS_X) / MPU_SELFTEST_SAMPLE_UNIT;
  acceleration_ST_diff.AXIS_Y = (acceleration_ST_mean.AXIS_Y - acceleration_NOST_mean.AXIS_Y) / MPU_SELFTEST_SAMPLE_UNIT;
  acceleration_ST_diff.AXIS_Z = (acceleration_ST_mean.AXIS_Z - acceleration_NOST_mean.AXIS_Z) / MPU_SELFTEST_SAMPLE_UNIT;
  
  
  // take mod
  acceleration_ST_diff.AXIS_X =  (acceleration_ST_diff.AXIS_X < 0)?(-1 *  acceleration_ST_diff.AXIS_X):  acceleration_ST_diff.AXIS_X;
  acceleration_ST_diff.AXIS_Y =  (acceleration_ST_diff.AXIS_Y < 0)?(-1 *  acceleration_ST_diff.AXIS_Y):  acceleration_ST_diff.AXIS_Y;
  acceleration_ST_diff.AXIS_Z =  (acceleration_ST_diff.AXIS_Z < 0)?(-1 *  acceleration_ST_diff.AXIS_Z):  acceleration_ST_diff.AXIS_Z;
  
  printTask("Postive Self Test result [%d][%d][%d] Should be between 90 and 1700 mg" , acceleration_ST_diff.AXIS_X, acceleration_ST_diff.AXIS_Y, acceleration_ST_diff.AXIS_Z); 
   
  if (MPU_SELFTEST_MIN_ACC <= acceleration_ST_diff.AXIS_X && acceleration_ST_diff.AXIS_X <= MPU_SELFTEST_MAX_ACC &&
      MPU_SELFTEST_MIN_ACC <= acceleration_ST_diff.AXIS_Y && acceleration_ST_diff.AXIS_Y <= MPU_SELFTEST_MAX_ACC &&
      MPU_SELFTEST_MIN_ACC <= acceleration_ST_diff.AXIS_Z && acceleration_ST_diff.AXIS_Z <= MPU_SELFTEST_MAX_ACC)
  {
    printTask("Postive Self Test result PASSED");
  }
  else
  {
    printTask("Postive Self Test result Failed");
  }
  
  printTask(" MPU unit test satarted Negative sign self-test");
  
  if( !LSM6DSM_ACC_GYRO_WriteReg(handle, LSM6DSM_ACC_GYRO_CTRL5_C, 0x00) )
    return COMPONENT_ERROR;
  osDelay(100);
  status = 0;
  
  while( status  == 0)
  { 
    if ( driver->Get_DRDY_Status( ctx, &status) != COMPONENT_OK )
      return COMPONENT_ERROR;
    osDelay(10);
  }
  if ( driver->Get_Axes( ctx, acceleration_TEMP ) == COMPONENT_ERROR )   // discard first smaple
  {
    return COMPONENT_ERROR;
  }
  osDelay(2);

  
  for(int i = 0; i < MPU_SELFTEST_SAMPLE_UNIT; i++)
  {
    while(status == 0)
    {
      if ( driver->Get_DRDY_Status( ctx, &status) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      osDelay(1);
    }
    if ( driver->Get_Axes( ctx, &acceleration_NOST[i] ) == COMPONENT_ERROR )
    {
      return COMPONENT_ERROR;
    }
    osDelay(1);
  }

  
  
  
  if( !LSM6DSM_ACC_GYRO_WriteReg(handle, LSM6DSM_ACC_GYRO_CTRL5_C, 0x02) )        // Enable Negtive Acc SelfTest
    return COMPONENT_ERROR;
 
    osDelay(100);
  
  while( status  == 0)
  { 
    if ( driver->Get_DRDY_Status( ctx, &status) != COMPONENT_OK )
      return COMPONENT_ERROR;
    osDelay(10);
  }
  if ( driver->Get_Axes( ctx, acceleration_TEMP ) == COMPONENT_ERROR )   // discard first smaple
  {
    return COMPONENT_ERROR;
  }
  osDelay(2);

  
  for(int i = 0; i < MPU_SELFTEST_SAMPLE_UNIT; i++)
  {
    while(status == 0)
    {
      if ( driver->Get_DRDY_Status( ctx, &status) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      osDelay(1);
    }
    if ( driver->Get_Axes( ctx, &acceleration_ST[i] ) == COMPONENT_ERROR )
    {
      return COMPONENT_ERROR;
    }
    osDelay(1);
  } 
//  printTask(" No selfTest");
//  for(int i = 0; i < MPU_SELFTEST_SAMPLE_UNIT; i++)
//  {
//    printTask(" [%d][%d][%d]" , acceleration_NOST[i].AXIS_X, acceleration_NOST[i].AXIS_Y, acceleration_NOST[i].AXIS_Z); 
//  }
//  
//  printTask(" with selfTest");
//  for(int i = 0; i < MPU_SELFTEST_SAMPLE_UNIT; i++)
//  {
//    printTask(" [%d][%d][%d]" , acceleration_ST[i].AXIS_X, acceleration_ST[i].AXIS_Y, acceleration_ST[i].AXIS_Z); 
//  }  
//   printTask(" Difference");
//  for(int i = 0; i < MPU_SELFTEST_SAMPLE_UNIT; i++)
//  {
//    printTask(" [%d][%d][%d]" ,
//    (int16_t)(acceleration_ST[i].AXIS_X - acceleration_NOST[i].AXIS_X),
//    (int16_t)(acceleration_ST[i].AXIS_Y - acceleration_NOST[i].AXIS_Y),
//    (int16_t)(acceleration_ST[i].AXIS_Z - acceleration_NOST[i].AXIS_Z)); 
//  }  
   
  acceleration_NOST_mean.AXIS_X = 0;
  acceleration_NOST_mean.AXIS_Y = 0;
  acceleration_NOST_mean.AXIS_Z = 0;
  
  acceleration_ST_mean.AXIS_X = 0;
  acceleration_ST_mean.AXIS_Y = 0;
  acceleration_ST_mean.AXIS_Z = 0;
  
  //take average
  for(int i = 0; i < MPU_SELFTEST_SAMPLE_UNIT; i++)
  {
    acceleration_NOST_mean.AXIS_X += acceleration_NOST[i].AXIS_X;
    acceleration_NOST_mean.AXIS_Y += acceleration_NOST[i].AXIS_Y;
    acceleration_NOST_mean.AXIS_Z += acceleration_NOST[i].AXIS_Z;
    
    acceleration_ST_mean.AXIS_X += acceleration_ST[i].AXIS_X;
    acceleration_ST_mean.AXIS_Y += acceleration_ST[i].AXIS_Y;
    acceleration_ST_mean.AXIS_Z += acceleration_ST[i].AXIS_Z;
  }    
  acceleration_ST_diff.AXIS_X = (acceleration_ST_mean.AXIS_X - acceleration_NOST_mean.AXIS_X) / MPU_SELFTEST_SAMPLE_UNIT;
  acceleration_ST_diff.AXIS_Y = (acceleration_ST_mean.AXIS_Y - acceleration_NOST_mean.AXIS_Y) / MPU_SELFTEST_SAMPLE_UNIT;
  acceleration_ST_diff.AXIS_Z = (acceleration_ST_mean.AXIS_Z - acceleration_NOST_mean.AXIS_Z) / MPU_SELFTEST_SAMPLE_UNIT;
  
  // take mod
  acceleration_ST_diff.AXIS_X =  (acceleration_ST_diff.AXIS_X < 0)?(-1 *  acceleration_ST_diff.AXIS_X):  acceleration_ST_diff.AXIS_X;
  acceleration_ST_diff.AXIS_Y =  (acceleration_ST_diff.AXIS_Y < 0)?(-1 *  acceleration_ST_diff.AXIS_Y):  acceleration_ST_diff.AXIS_Y;
  acceleration_ST_diff.AXIS_Z =  (acceleration_ST_diff.AXIS_Z < 0)?(-1 *  acceleration_ST_diff.AXIS_Z):  acceleration_ST_diff.AXIS_Z;
  
  printTask("Negtive Self Test result [%d][%d][%d] Should be between 90 and 1700 mg" , acceleration_ST_diff.AXIS_X, acceleration_ST_diff.AXIS_Y, acceleration_ST_diff.AXIS_Z); 
 
  if (MPU_SELFTEST_MIN_ACC <= acceleration_ST_diff.AXIS_X && acceleration_ST_diff.AXIS_X <= MPU_SELFTEST_MAX_ACC &&
      MPU_SELFTEST_MIN_ACC <= acceleration_ST_diff.AXIS_Y && acceleration_ST_diff.AXIS_Y <= MPU_SELFTEST_MAX_ACC &&
      MPU_SELFTEST_MIN_ACC <= acceleration_ST_diff.AXIS_Z && acceleration_ST_diff.AXIS_Z <= MPU_SELFTEST_MAX_ACC)
  {
    printTask("Negtive Self Test result PASSED");
  }
  else
  {
    printTask("Negtive Self Test result Failed");
  }  
  
  //Turn off the SelfTest
  if( !LSM6DSM_ACC_GYRO_WriteReg(handle, LSM6DSM_ACC_GYRO_CTRL1_XL, 0x00) )   //0x38 > 52Hz & 4G
    return COMPONENT_ERROR;
  if( !LSM6DSM_ACC_GYRO_WriteReg(handle, LSM6DSM_ACC_GYRO_CTRL5_C, 0x00) )
    return COMPONENT_ERROR;
 
    return COMPONENT_OK;

}
DrvStatusTypeDef MPU_UnitTest_InternalTemp(void *handle)
{

#define LSM6DSM_ACC_GYRO_OUT_TEMP_L  	0X20
#define LSM6DSM_ACC_GYRO_OUT_TEMP_H  	0X21
  LSM6DSM_ACC_GYRO_TDA_t TDA = LSM6DSM_ACC_GYRO_TDA_NO_DATA_AVAIL;
  while (LSM6DSM_ACC_GYRO_TDA_NO_DATA_AVAIL == TDA)
  {
    if(MEMS_SUCCESS !=  LSM6DSM_ACC_GYRO_R_TDA(handle, &TDA))
    {  
      return COMPONENT_ERROR;
    }
  }
  int16_t tempValue = 0;
  if ( MEMS_SUCCESS ==  LSM6DSM_ACC_GYRO_ReadMem(handle, LSM6DSM_ACC_GYRO_OUT_TEMP_L, (uint8_t*)&tempValue, 2))
  {
    float LSM_temp = (float)(tempValue);
    LSM_temp /= 256.0f;
    LSM_temp += 25.0f;
    printTask("internal Temperaure is %3.2f C", LSM_temp);
  }
  return COMPONENT_OK;

}

#endif

#define t_MPU_IWDG_TIME   	5*60*1000                    //5 minute
#define t_MPU_SLEEP_TIME    20*1000                      //20 minute

uint32_t annCalcStartTime = 0;  
uint32_t annCalcENDTime = 0;    
uint32_t annCalcLastTime = 0;    
uint32_t annCalcMaxTime = 0; 


  uint32_t ulNotificationMMpu  = 0;   

//Takes ? ms per iteration.
void thread_MPU(const void* param){
    
  hmIWDG_MPU = IWDG_initWDG(t_MPU_IWDG_TIME);   // 5 mitues
  IWDG_startWDG(hmIWDG_MPU);
  
  xTaskMpuIRQ = xTaskGetCurrentTaskHandle();
	/* Initialize and Enable the available sensors */
#if defined(MPU_UNIT_TEST)
  osDelay(10);  // so that main thread can run first
	initializeAllSensors();
	enableAllSensors();
  {
    osDelay(1500); // wait 1500 secs so that acclero data become usable.
    Sensor_IO_SPI_Init();
    DrvStatusTypeDef status = MPU_UnitTestGravityAndFifo(LSM6DSM_X_0_handle);

    status = MPU_SelfTest(LSM6DSM_X_0_handle);
    
    status = MPU_UnitTest_InternalTemp(LSM6DSM_X_0_handle);
  }
  IWDG_refreshWDG(hmIWDG_MPU);
  DeInitializeAllSensors();
	//initializeAllSensors();

#endif
  
#ifdef CLASSIFICATION_APP	
	initializeAllSensors();
	enableAllSensors();
  
  mpu_filter_init(hpFirWalking, fir_coefficients, firStateWalking);
  mpu_filter_init(hpFirBehavior, fir_coefficients, firStateBehavior);
  
  ANN_init(&annBehavior);
  ANN_Walking_init(&annWalking);
  uint32_t annNextSlotBehavior = 0;    
  uint32_t annNextSlotWalking = 0;    
  annNextSlotBehavior = RTC_secondsAtMark(ANN_BEHAVIOR_SAMPLE_TIME);
  annNextSlotWalking = RTC_secondsAtMark(ANN_WALKING_SAMPLE_TIME);
#endif
  
  
  uint32_t MPU_Process_counter = 0;
  int8_t errorRetries = 5;
  uint32_t unixTimeMPU = RTC_getUnixTime();

    
       
  uint32_t mpuPrevLogSlot = 0;
  mpuPrevLogSlot = getCurrentSampleSlot();
  uint32_t mpuCurrLogSlot = 0;


  float walkingCount = 0;
  float notwalkingCount = 0;
  float walkingUnknownCount = 0;
  float ruminationCount = 0;
  float eatingCount = 0;
  float movingCount = 0;
  float restingCount = 0;
  float walking2Count = 0;
  float unknownCount = 0;
  uint32_t subWindowCount = 0;

//  float ruminationCountANN = 0;
//  float eatingCountANN = 0;
//  float restingCountANN = 0;

  uint32_t unixTimeBattery = 0;
  uint32_t batteryNextSaveSlot = 0;

#ifndef RAW_DATA_COLLECTION  

#endif
  
  
  uint32_t activityIndex = 0;
  activityIndex = (unixTimeMPU %MPU_ACTIVITY_TIME) ;
  activityIndex /= (MPU_ACTIVITY_TIME / MPU_ACTIVITY_COUNT);  
  activityIndex +=(MPU_ACTIVITY_COUNT-1);
  activityIndex %=(MPU_ACTIVITY_COUNT);
        
#ifdef RAW_DATA_COLLECTION  
  
  uint8_t threadMPUReset = 1;
  uint32_t SDcardNextBatteryLog = 0;
  uint32_t unixSeconds = 0;
  uint32_t milliSeconds = 0; 
  char fileName[50];
  uint32_t lastFileNameTime = 0;
  int32_t stepsLastMinute = 0;               
  int32_t stepsThisMinute = 0;               
  int32_t stepsLastHour = 0;               
  int32_t stepsThisHour = 0;               
  uint32_t lastMinute = (RTC_getUnixTime() / 60);
  uint32_t lastHour = (RTC_getUnixTime() / 3600);
  stepsLastMinute = steps; 
  stepsLastHour = steps; 

  int32_t sampleCounter = 0;
  RTC_getUnixTimeMs(&LastSampleTimeSeconds, &LastSampleTimeMilliSeconds);   // read current time to use for next time Sd card write
  SDcardNextBatteryLog = LastSampleTimeSeconds;
  SDCardSPI_config(   C_SPI_SCK_Pin, C_SPI_SCK_GPIO_Port, 
                      C_SPI_MOSI_Pin, C_SPI_MOSI_GPIO_Port,
                      C_SPI_MISO_Pin, C_SPI_MISO_GPIO_Port,
                      C_SPI_NSS_Pin, C_SPI_NSS_GPIO_Port);
  uint8_t HighByte[2] = {0xFF, 0xFF};
          
  osMutexWait(hmutex_SDCard, osWaitForever);
  while(writeTestFile())
  {
      osDelay(10);  
  }  
  osMutexRelease(hmutex_SDCard);

#endif
 
  for(;;)
  {
    IWDG_refreshWDG(hmIWDG_MPU);
    printMPU("MPU thread\n");        
        
#if defined(MPU_UNIT_TEST)
    osDelay(30*1000);
#else        
   
#endif
  }
  osThreadTerminate(NULL);    
}
