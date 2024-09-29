/**
 * @file   	mRTC.c
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Oct 26, 2016
 * 
 * @brief   
 */

#include "mRTC.h"
#include "rtc.h"
#include "DEBUG_UART.h"

/**
 * @defgroup RTC_Exported_Members			RTC Exported Members
 * @{
 */
uint32_t RTC_unixTime;
static int32_t RTC_MilliSecondOffset = 0;
/** @}*/


/**
 * @defgroup RTC_Public_Functions			RTC Public Functions
 * @{
 */
 __INLINE uint32_t RTC_secondsTillMark2(uint32_t unixSeconds, uint32_t mark)  /// @todo: implement
{
	return (mark - (unixSeconds % mark));
}
// mark in seconds 
__INLINE uint32_t RTC_secondsTillMark( uint32_t mark)
{
	return RTC_secondsTillMark2(RTC_getUnixTime(), mark);
}

// mark in seconds 
__INLINE uint32_t RTC_milliSecondsTillMark( uint32_t mark)
{
  uint32_t Seconds = 0;
  uint32_t MilliSeconds = 0;    
  RTC_getUnixTimeMs(&Seconds, &MilliSeconds);
  int32_t timeTillMark = RTC_secondsTillMark2(Seconds, mark); 
  timeTillMark *=1000;
  timeTillMark -= MilliSeconds;  // MilliSeconds has alread passed compansate
  if(timeTillMark < 0)
  {
    printTask("\t\tRTC underflow for %d ticks\n\n",timeTillMark); 
    timeTillMark = 0;
  }
  if(timeTillMark > (mark *1000))
  {
    printTask("\t\tRTC overflow for %d ticks\n\n",timeTillMark); 
    timeTillMark = 0;
  }
	return timeTillMark;
}
__INLINE uint32_t RTC_secondsAtMark( uint32_t mark)
{
  uint32_t t = RTC_getUnixTime();
	return t + RTC_secondsTillMark2(t, mark);
}

/**
 * @brief			Block the current task untill the specified mark.
 * @param mark		Specifies the slot in seconds. (i.e 20 would ublock the task at start of next
 * 														20 second interval in unix Time)
 * @param step		granularity (quantum) of the delay
 * @param hmIWDG	Pointer to the software watchdog of the thread. 
 * 
 */
void RTC_waitTillMark (uint32_t mark, uint32_t step, mIWDG_HandleTypeDef* hmIWDG)
{
    uint32_t prevMark = 0;
    uint32_t nextMark = 0;
    uint32_t timeTillMark = 0;


    prevMark = RTC_getUnixTime()/mark;
    nextMark = prevMark;

    while ( prevMark == nextMark )
    {
      if (hmIWDG != NULL)
          IWDG_refreshWDG(hmIWDG);

      timeTillMark = RTC_milliSecondsTillMark(mark);
      printTask("\nTime left : %d\n", timeTillMark);
      if (timeTillMark > step * 1000)
      {
        timeTillMark = step * 1000;
      }
      if (timeTillMark > 60*1000)
      {        
        osDelay((timeTillMark - 60*1000));
      }
      else if (timeTillMark > 1*1000)
      {
        osDelay((timeTillMark - 1*1000));            
      }
      else
      {
        osDelay(timeTillMark);
      }

      nextMark = RTC_getUnixTime() / mark;
    }

     printTask("\nDelay over\n", timeTillMark);

}

/**
 * @brief	Get current Unix epoch time.
 * @return  Epoch time stamp.
 */
uint32_t RTC_getUnixTime()
{
    uint32_t unixSeconds = 0;
    uint32_t milliSeconds = 0;
    RTC_getUnixTimeMs(&unixSeconds, &milliSeconds);
    return unixSeconds;
}
   
/**
 * @brief	Get current millisec of Current Day
 * @return  Epoch time stamp.
 */
uint32_t RTC_getUnixTimeMs(uint32_t *unixSeconds, uint32_t *milliSeconds)
{

    struct tm currTime;
    time_t time = 0;
    uint32_t timems  = 0;
    int32_t ms = 0;

    static RTC_DateTypeDef dateStructure;
    static RTC_TimeTypeDef timeStructure;

    ///@note The HAL_RTC_GetDate function must be called
    /// 	  after HAL_RTC_GetTime to unlock the RTC registers in hardware.
    /* Get the RTC current Time */
    HAL_RTC_GetTime(&hrtc, &timeStructure, FORMAT_BIN);
    /* Get the RTC current Date */
    HAL_RTC_GetDate(&hrtc, &dateStructure, FORMAT_BIN);


    currTime.tm_year = dateStructure.Year + 2000 - 1900;
    currTime.tm_mon  = dateStructure.Month - 1;
    currTime.tm_mday = dateStructure.Date;
    currTime.tm_hour = timeStructure.Hours;
    currTime.tm_min  = timeStructure.Minutes;
    currTime.tm_sec  = timeStructure.Seconds;
    currTime.tm_isdst = false;
    time = mktime(&currTime);
    
    *unixSeconds = time;
    
#if defined(STM32L4) || defined(STM32L100xBA) || defined (STM32L151xBA) || defined (STM32L152xBA) || defined(STM32L100xC) || defined (STM32L151xC) || defined (STM32L152xC) || defined (STM32L162xC) || defined(STM32L151xCA) || defined (STM32L151xD) || defined (STM32L152xCA) || defined (STM32L152xD) || defined (STM32L162xCA) || defined (STM32L162xD) || defined(STM32L151xE) || defined(STM32L151xDX) || defined (STM32L152xE) || defined (STM32L152xDX) || defined (STM32L162xE) || defined (STM32L162xDX) 

    ms = (((int32_t)timeStructure.SecondFraction - (int32_t)timeStructure.SubSeconds) * 1000 / (int32_t)(timeStructure.SecondFraction + 1));    
    
    timems += (timeStructure.Hours * (24*60));
    timems += (timeStructure.Minutes * 60);
    timems += (timeStructure.Seconds);
    timems *= (1000);
    timems += (ms);
    
//    printDEBUG("\t");
//    printDEBUG("time %02d:%02d:%02d.%03d[%03d]", currTime.tm_hour, currTime.tm_min, currTime.tm_sec, ms, timeStructure.SubSeconds );
//    printDEBUG(" - RTC%d.%03d", time, ms);
//    printDEBUG(" - offset %03d", RTC_MilliSecondOffset);

    ms -= RTC_MilliSecondOffset;
    
    if(ms >= 1000)
    {
      ms -= 1000;
      *unixSeconds +=1;
    }
    else if (ms < 0)
    {
      ms += 1000;
      *unixSeconds -=1;
    }
    // milisecond correction
   *milliSeconds = ms;
//    printDEBUG(" - Corrected%d.%03d\n", *unixSeconds, *milliSeconds);
#endif

    return timems;
}
uint32_t RTC_getUnixTimeMsGPS(uint32_t *unixSeconds, uint32_t *milliSeconds)
{

    struct tm currTime;
    time_t time = 0;
    uint32_t timems  = 0;
    int32_t ms = 0;

    static RTC_DateTypeDef dateStructure;
    static RTC_TimeTypeDef timeStructure;

    ///@note The HAL_RTC_GetDate function must be called
    /// 	  after HAL_RTC_GetTime to unlock the RTC registers in hardware.
    /* Get the RTC current Time */
    HAL_RTC_GetTime(&hrtc, &timeStructure, FORMAT_BIN);
    /* Get the RTC current Date */
    HAL_RTC_GetDate(&hrtc, &dateStructure, FORMAT_BIN);


    currTime.tm_year = dateStructure.Year + 2000 - 1900;
    currTime.tm_mon  = dateStructure.Month - 1;
    currTime.tm_mday = dateStructure.Date;
    currTime.tm_hour = timeStructure.Hours;
    currTime.tm_min  = timeStructure.Minutes;
    currTime.tm_sec  = timeStructure.Seconds;
    currTime.tm_isdst = false;
    time = mktime(&currTime);
    
    *unixSeconds = time;
    
#if defined(STM32L4) || defined(STM32L100xBA) || defined (STM32L151xBA) || defined (STM32L152xBA) || defined(STM32L100xC) || defined (STM32L151xC) || defined (STM32L152xC) || defined (STM32L162xC) || defined(STM32L151xCA) || defined (STM32L151xD) || defined (STM32L152xCA) || defined (STM32L152xD) || defined (STM32L162xCA) || defined (STM32L162xD) || defined(STM32L151xE) || defined(STM32L151xDX) || defined (STM32L152xE) || defined (STM32L152xDX) || defined (STM32L162xE) || defined (STM32L162xDX) 

    ms = ((timeStructure.SecondFraction - timeStructure.SubSeconds) * 1000 / (timeStructure.SecondFraction + 1));    
    
    timems += (timeStructure.Hours * (24*60));
    timems += (timeStructure.Minutes * 60);
    timems += (timeStructure.Seconds);
    timems *= (1000);
    timems += (ms);
    
//    printDEBUG("\t");
//    printDEBUG("time %02d:%02d:%02d.%03d[%03d]", currTime.tm_hour, currTime.tm_min, currTime.tm_sec, ms, timeStructure.SubSeconds );
//    printDEBUG(" - RTC%d.%03d", time, ms);
//    printDEBUG(" - offset %03d", RTC_MilliSecondOffset);

    ms -= RTC_MilliSecondOffset;
    
    if(ms >= 1000)
    {
      ms -= 1000;
      *unixSeconds +=1;
    }
    else if (ms < 0)
    {
      ms += 1000;
      *unixSeconds -=1;
    }
    // milisecond correction
   *milliSeconds = ms;
//    printDEBUG(" - Corrected%d.%03d", *unixSeconds, *milliSeconds);
    printDEBUG("\n");
#endif

    return timems;
}
typedef struct{
    uint32_t Second:5; // Second / 2 (0..29, e.g. 25 for 50)
    uint32_t Minute:6; //            (0..59)
    uint32_t Hour:5;   //            (0..23)
    uint32_t Day:5;    //            (1..31) Day of the month
    uint32_t Month:4;  //            (1..12) 
    uint32_t Year:7;   //            (0..127) Year origin from the 1980 (e.g. 37 for 2017)
} Fattime_t;

uint32_t get_fattime (void)
{
    uint32_t returnValue;
    Fattime_t *cTime;
    cTime = (Fattime_t*)&returnValue;

    static RTC_DateTypeDef dateStructure;
    static RTC_TimeTypeDef timeStructure;

    ///@note The HAL_RTC_GetDate function must be called
    /// 	  after HAL_RTC_GetTime to unlock the RTC registers in hardware.
    /* Get the RTC current Time */
    HAL_RTC_GetTime(&hrtc, &timeStructure, FORMAT_BIN);
    /* Get the RTC current Date */
    HAL_RTC_GetDate(&hrtc, &dateStructure, FORMAT_BIN);

 
    cTime->Year = dateStructure.Year + 2000 - 1980;
    cTime->Month  = dateStructure.Month;
    cTime->Day = dateStructure.Date;
    cTime->Hour = timeStructure.Hours;
    cTime->Minute  = timeStructure.Minutes;
    cTime->Second  = timeStructure.Seconds;
    
    return returnValue;
}

/**
 * @brief			Update the RTC to the time specified by the Unix time stamp.
 * @param unixTime	Unix Epoch time
 * @return			#HAL_StatusTypeDef
 */
HAL_StatusTypeDef RTC_setTime(uint32_t unixTime){
    return RTC_setTimeMs(unixTime, 0);
}

/**
 * @brief			Update the RTC to the time specified by the Unix time stamp.
 * @param unixTime	Unix Epoch time
 * @return			#HAL_StatusTypeDef
 */
HAL_StatusTypeDef RTC_setTimeMs(uint32_t unixSeconds, uint32_t milliSeconds){
    struct tm dateTime;
    localtime_r(&unixSeconds, &dateTime);
    return RTC_setDateTime(&dateTime, milliSeconds);
}

HAL_StatusTypeDef RTC_setDateTime(struct tm *dateTime, uint32_t milliSeconds){

    RTC_DateTypeDef dateStructure;
    RTC_TimeTypeDef timeStructure;

#if defined(STM32L4) || defined(STM32L100xBA) || defined (STM32L151xBA) || defined (STM32L152xBA) || defined(STM32L100xC) || defined (STM32L151xC) || defined (STM32L152xC) || defined (STM32L162xC) || defined(STM32L151xCA) || defined (STM32L151xD) || defined (STM32L152xCA) || defined (STM32L152xD) || defined (STM32L162xCA) || defined (STM32L162xD) || defined(STM32L151xE) || defined(STM32L151xDX) || defined (STM32L152xE) || defined (STM32L152xDX) || defined (STM32L162xE) || defined (STM32L162xDX) 
    
    HAL_RTC_GetTime(&hrtc, &timeStructure, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &dateStructure, RTC_FORMAT_BIN);
  
    int32_t SSR_offset = ((milliSeconds *(timeStructure.SecondFraction + 1)) / 1000) - (timeStructure.SecondFraction - timeStructure.SubSeconds);
       
#endif 

    timeStructure.Hours   = dateTime->tm_hour;
    timeStructure.Minutes = dateTime->tm_min;
    timeStructure.Seconds = dateTime->tm_sec;
    timeStructure.TimeFormat = RTC_HOURFORMAT12_AM;
    timeStructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    timeStructure.StoreOperation = RTC_STOREOPERATION_RESET;
  
  
    dateStructure.Year  = dateTime->tm_year - 2000 + 1900;
    dateStructure.Month = dateTime->tm_mon + 1;
    dateStructure.Date  = dateTime->tm_mday;
    dateStructure.WeekDay = dateTime->tm_wday;
    if(dateStructure.WeekDay == 0) dateStructure.WeekDay = 7;

    HAL_RTC_SetTime(&hrtc, &timeStructure, RTC_FORMAT_BIN);
    HAL_StatusTypeDef ret =  HAL_RTC_SetDate(&hrtc, &dateStructure, RTC_FORMAT_BIN);
    
#if defined(STM32L4) || defined(STM32L100xBA) || defined (STM32L151xBA) || defined (STM32L152xBA) || defined(STM32L100xC) || defined (STM32L151xC) || defined (STM32L152xC) || defined (STM32L162xC) || defined(STM32L151xCA) || defined (STM32L151xD) || defined (STM32L152xCA) || defined (STM32L152xD) || defined (STM32L162xCA) || defined (STM32L162xD) || defined(STM32L151xE) || defined(STM32L151xDX) || defined (STM32L152xE) || defined (STM32L152xDX) || defined (STM32L162xE) || defined (STM32L162xDX) 
    if(SSR_offset < 0)
    {
      SSR_offset *= -1;
      HAL_RTCEx_SetSynchroShift(&hrtc, 0, SSR_offset);
    }
    else 
    {
      SSR_offset = timeStructure.SecondFraction + 1 - SSR_offset;
      HAL_RTCEx_SetSynchroShift(&hrtc, 1, SSR_offset);      
    }

#endif
    HAL_RTC_GetTime(&hrtc, &timeStructure, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &dateStructure, RTC_FORMAT_BIN);

    return ret;
}

/**
 * @brief			Set RTC alarm after the specified interval in seconds.
 * @param seconds	Time till alarm in seconds.
 * @return			#HAL_StatusTypeDef
 */
HAL_StatusTypeDef RTC_setAlarm(uint32_t seconds){

	RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
	RTC_AlarmTypeDef sAlarm;

	HAL_RTC_GetTime(&hrtc, &sTime, FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, FORMAT_BIN);

	sAlarm.Alarm = RTC_ALARM_A;
	sAlarm.AlarmTime.Seconds = sTime.Seconds + seconds%60;
    seconds /= 60;

    if (sAlarm.AlarmTime.Seconds >= 60){
        sAlarm.AlarmTime.Seconds -= 60;
        seconds++;
    }

	sAlarm.AlarmTime.Minutes = sTime.Minutes + seconds%60;
    seconds /= 60;
    if (sAlarm.AlarmTime.Minutes >= 60){
        sAlarm.AlarmTime.Minutes -= 60;
        seconds++;
    }

	sAlarm.AlarmTime.Hours = sTime.Hours + seconds%24;
    if (sAlarm.AlarmTime.Hours >= 24){
        sAlarm.AlarmTime.Hours = 0;
    }



    sAlarm.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
    sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
    sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
    sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    sAlarm.AlarmDateWeekDay = 1;
    sAlarm.Alarm = RTC_ALARM_A;

#if defined(STM32L4) || defined(STM32L100xBA) || defined (STM32L151xBA) || defined (STM32L152xBA) || defined(STM32L100xC) || defined (STM32L151xC) || defined (STM32L152xC) || defined (STM32L162xC) || defined(STM32L151xCA) || defined (STM32L151xD) || defined (STM32L152xCA) || defined (STM32L152xD) || defined (STM32L162xCA) || defined (STM32L162xD) || defined(STM32L151xE) || defined(STM32L151xDX) || defined (STM32L152xE) || defined (STM32L152xDX) || defined (STM32L162xE) || defined (STM32L162xDX) 
    sAlarm.AlarmTime.SubSeconds = 0;
    sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
#endif
    

	return HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, FORMAT_BIN);

}


/**
 * @brief			Set RTC alarm after the specified interval in seconds.
 * @param seconds	Time till alarm in seconds.
 * @return			#HAL_StatusTypeDef
 */
HAL_StatusTypeDef RTC_setAlarmMs(uint32_t millisSconds){

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  RTC_AlarmTypeDef sAlarm;
  int32_t ms = 0;

  HAL_RTC_GetTime(&hrtc, &sTime, FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, FORMAT_BIN);

  sAlarm.Alarm = RTC_ALARM_A;
  
  ms = ((sTime.SecondFraction - sTime.SubSeconds) * 1000 / (sTime.SecondFraction + 1));    

//  printDEBUG("curr time %02d:%02d:%02d.%03d[%03d]", sTime.Hours, sTime.Minutes, sTime.Seconds, ms, sTime.SubSeconds );
  uint32_t seconds = millisSconds /1000; 
  millisSconds %= 1000;
  millisSconds *= (hrtc.Init.SynchPrediv + 1);
  millisSconds /= (1000);

#if defined(STM32L4) || defined(STM32L100xBA) || defined (STM32L151xBA) || defined (STM32L152xBA) || defined(STM32L100xC) || defined (STM32L151xC) || defined (STM32L152xC) || defined (STM32L162xC) || defined(STM32L151xCA) || defined (STM32L151xD) || defined (STM32L152xCA) || defined (STM32L152xD) || defined (STM32L162xCA) || defined (STM32L162xD) || defined(STM32L151xE) || defined(STM32L151xDX) || defined (STM32L152xE) || defined (STM32L152xDX) || defined (STM32L162xE) || defined (STM32L162xDX) 
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
  
  ms = (int32_t)sTime.SubSeconds - (int32_t)millisSconds;
  if (ms < 0){
    ms += 256;
    seconds++;        // subsec overflow into seconds
  }
  sAlarm.AlarmTime.SubSeconds = ms;
#endif
  
  sAlarm.AlarmTime.Seconds = sTime.Seconds + (seconds%60);
 
  sAlarm.AlarmTime.Seconds = sTime.Seconds + (seconds%60);
  seconds /= 60;

  if (sAlarm.AlarmTime.Seconds >= 60){
    sAlarm.AlarmTime.Seconds -= 60;
    seconds++;
  }

	sAlarm.AlarmTime.Minutes = sTime.Minutes + (seconds%60);
  seconds /= 60;
  if (sAlarm.AlarmTime.Minutes >= 60){
    sAlarm.AlarmTime.Minutes -= 60;
    seconds++;
  }

	sAlarm.AlarmTime.Hours = sTime.Hours + (seconds%24);
  if (sAlarm.AlarmTime.Hours >= 24){
    sAlarm.AlarmTime.Hours = 0;
  }


  sAlarm.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;    

  ms = ((sTime.SecondFraction - sAlarm.AlarmTime.SubSeconds) * 1000 / (sTime.SecondFraction + 1));    
//  printDEBUG("Alarm time %02d:%02d:%02d.%03d[%03d]", sAlarm.AlarmTime.Hours, sAlarm.AlarmTime.Minutes, sAlarm.AlarmTime.Seconds, ms, sAlarm.AlarmTime.SubSeconds );
  if( sAlarm.AlarmTime.SubSeconds > 255)
    sAlarm.AlarmTime.SubSeconds = 255;
	return HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, FORMAT_BIN);

}

void RTC_ApplyTempCompansation(uint32_t temprature)
{
  float freqError = (temprature - 25.0f);
  float CalibMinusValue = 0;
  uint32_t CalibPlusValue = RTC_SMOOTHCALIB_PLUSPULSES_RESET;
  uint32_t CalibPeriod = RTC_SMOOTHCALIB_PERIOD_32SEC;
  freqError *= freqError;
  freqError *= TEMPERATURE_COEFFICIENT;   // ppm
  freqError /= 1000000.0f;                // divide by million

  if(freqError > 0)
  {
    CalibPlusValue = RTC_SMOOTHCALIB_PLUSPULSES_SET;
    CalibMinusValue = (freqError * (float) ((1<20)  - 512)) - (float)512;
    CalibMinusValue = CalibMinusValue / (float) ((1<20)  +1 );
    CalibMinusValue = -1 * CalibMinusValue;
    if( CalibMinusValue < 255 )
    {
      CalibPeriod = RTC_SMOOTHCALIB_PERIOD_32SEC;
    }
    else
    {
      CalibMinusValue = (freqError * (float) ((1<19)  - 256)) - (float)256;
      CalibMinusValue = CalibMinusValue / (float) ((1<19)  +1 );
      CalibMinusValue = -1 * CalibMinusValue;
      if( CalibMinusValue < 255 )
      {
        CalibPeriod = RTC_SMOOTHCALIB_PERIOD_16SEC;
      }
      else
      {
        CalibMinusValue = (freqError * (float) ((1<18)  - 128)) - (float)128;
        CalibMinusValue = CalibMinusValue / (float) ((1<18)  + 1 );
        CalibMinusValue = -1 * CalibMinusValue;
        CalibPeriod = RTC_SMOOTHCALIB_PERIOD_8SEC;
      }
    }
  }
  else
  {
    CalibPlusValue = RTC_SMOOTHCALIB_PLUSPULSES_RESET;
    CalibMinusValue = freqError * (float) ((1<20));
    CalibMinusValue = CalibMinusValue / (float) ((1<20)  + 1 );
    CalibMinusValue = -1 * CalibMinusValue;
    if( CalibMinusValue < 255 )
    {
      CalibPeriod = RTC_SMOOTHCALIB_PERIOD_32SEC;
    }
    else
    {
      CalibMinusValue = freqError * (float) ((1<19));
      CalibMinusValue = CalibMinusValue / (float) ((1<19)  + 1 );
      CalibMinusValue = -1 * CalibMinusValue;
      if( CalibMinusValue < 255 )
      {
        CalibPeriod = RTC_SMOOTHCALIB_PERIOD_16SEC;
      }
      else
      {
        CalibMinusValue = freqError * (float) ((1<18));
        CalibMinusValue = CalibMinusValue / (float) ((1<18)  + 1 );
        CalibMinusValue = -1 * CalibMinusValue;
        CalibPeriod = RTC_SMOOTHCALIB_PERIOD_8SEC;
      }
    }
  }
  HAL_RTCEx_SetSmoothCalib(&hrtc, CalibPeriod, CalibPlusValue, (uint32_t)CalibMinusValue);
}
/** @}*/
