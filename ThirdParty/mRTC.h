/**
 * @file   	mRTC.h
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Oct 26, 2016
 * 
 * @brief   This file provides functions for updating the RTC and setting alarm in the future.
 */

#ifndef MRTC_H_
#define MRTC_H_

#include "bsp.h"
#include "mIWDG.h"
#include <time.h>

/**
 * @defgroup RTC_Public_Macros				RTC Public Macros
 * @{
 */



/** @}*/

extern uint32_t RTC_unixTime;

#ifdef __cplusplus
extern "C"{
#endif 

uint32_t RTC_secondsTillMark2     (uint32_t unixSeconds, uint32_t mark);
uint32_t RTC_secondsTillMark	    (uint32_t mark);
uint32_t RTC_milliSecondsTillMark (uint32_t mark);
uint32_t RTC_secondsAtMark        (uint32_t mark);

HAL_StatusTypeDef RTC_setTime		  (uint32_t unixTime);
HAL_StatusTypeDef RTC_setTimeMs   (uint32_t unixSeconds, uint32_t milliSeconds);
HAL_StatusTypeDef RTC_setDateTime (struct tm *dateTime, uint32_t milliSeconds);
HAL_StatusTypeDef RTC_setAlarm		(uint32_t seconds);
HAL_StatusTypeDef RTC_setAlarmMs  (uint32_t millisSconds);

uint32_t RTC_getUnixTime          (void);
uint32_t RTC_getUnixTimeMs        (uint32_t *unixSeconds, uint32_t *milliSeconds);
uint32_t RTC_getUnixTimeMsGPS     (uint32_t *unixSeconds, uint32_t *milliSeconds);

void 	   RTC_waitTillMark 	      (uint32_t mark, uint32_t step, mIWDG_HandleTypeDef* hmIWDG);
void     RTC_ApplyTempCompansation(uint32_t temprature);

#ifdef __cplusplus
}
#endif

#endif /* MRTC_H_ */
