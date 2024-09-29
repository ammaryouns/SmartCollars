/**
 * @file   	Cowlar_app_v8.h
 * @author 	Umer
 * @version	
 * @date	Nov 8, 2017
 * 
 * @brief   
 */
 
 
#ifndef COWLAR_APP_V8_H_
#define COWLAR_APP_V8_H_


#include "bsp.h"
#ifndef RAW_DATA_COLLECTION
  #define DEFAULT_MPU_HZ          26      // Hz
#endif
#define ANN_BEHAVIOR_SAMPLE_TIME  60      // sec
#define ANN_WALKING_SAMPLE_TIME   10      // sec

#define RECIPE_WINDOW_DURATION          ((uint32_t)30*60)   //  30 Minutes (in seconds) this is fixed as in the cloud
#define SENSORS_BATTERY_SAMPLE_DURATION (6 * 60*60)   ///  this should be factors of @RECIPE_WINDOW_DURATION
#define SENSORS_BATTERY_LOG_DURATION    (6 * 3600)                    // six hour
#define SENSORS_BEHAVIOUR_LOG_DURATION  (15 * 60)                    // 


#define SENSOR_ERROR_RETRIES                3u

#endif /* COWLAR_APP_V8_H_ */
