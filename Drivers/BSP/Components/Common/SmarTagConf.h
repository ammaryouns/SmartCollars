/**
  ******************************************************************************
  * @file    SmarTagConf.h
  * @author  Central LAB
  * @version V1.1.2
  * @date    06-Jul-2018
  * @brief   Configuration file for SmarTag application
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SMARTAGCONF_H
#define __SMARTAGCONF_H

//#include "DEBUG_UART.h"

/* Uncomment the following define for disabling the LowPowerMode for Debug */
//#define SMARTAG_ENABLE_DEBUG

/* Uncomment the following define for enabling the VCOM printf */
#define SMARTAG_ENABLE_PRINTF

#ifndef SMARTAG_ENABLE_DEBUG
  /* Uncomment the following line for enabling AutoStarting after 16 seconds */
  #define SMARTAG_AUTOSTART_SECONDS 16
#endif /* SMARTAG_ENABLE_DEBUG */

/* Uncomment the following define for adding the password protection for R/F.
 * Only Android will be able to read the data protected */
//#define SMARTAG_ENABLE_PROTECTION_RF

/* Uncomment the following define for adding the password protection for I2C */
//#define SMARTAG_ENABLE_PROTECTION_I2C

/******************  Don't change the following section ***********************/
/* Uncomment the following line if you want to save 2 NDEF header:
  - one for launching the ST Android Application
  - one for storing configuration and log data */
//#define SMARTAG_DOUBLE_NDEF

#ifdef SMARTAG_DOUBLE_NDEF
  #define SMARTAG_FIRST_NDEF_SIZE 36
#else /* SMARTAG_DOUBLE_NDEF */
  #define SMARTAG_FIRST_NDEF_SIZE 0
#endif /* SMARTAG_DOUBLE_NDEF */

/* st25dv tag sizes */
#define NFCTAG_4K_SIZE            ((uint32_t) 0x200)
#define NFCTAG_16K_SIZE           ((uint32_t) 0x800)
#define NFCTAG_64K_SIZE           ((uint32_t) 0x2000)

/* Eval-SmarTag1 includes the st25dv64k */
#define ST25DV_MAX_SIZE           NFCTAG_16K_SIZE
/* Dimension of the CC file in bytes */
#define ST25DV_CC_SIZE            8

/* Address due to extended CC file + NDEF header before payload == (4 or 8) + 24 + SMARTAG_FIRST_NDEF_SIZE*/
#define SMARTAG_START_ADDR_OFFSET (0x18+ST25DV_CC_SIZE + SMARTAG_FIRST_NDEF_SIZE)

/****************************************************************************************
 * SmarTag logging mode functionality                                                   *
 * Active:   read sensor data at regular intervals and store sequentially in NFC EEPROM *
 * One Shot: read sensor data once and store in NFC EEPROM                              *
 ****************************************************************************************/
#define SMARTAG_LOGMODE_INACTIVE    0
#define SMARTAG_LOGMODE_ACTIVE      1
#define SMARTAG_LOGMODE_ACTIVE_THS  3
#define SMARTAG_SAVE_NEXT_SAMPLE    4

/* Package Version only numbers 0->9 */
#define SMARTAG_RECORD_VERSION 1
#define SMARTAG_VERSION_MAJOR  1
#define SMARTAG_VERSION_MINOR  0
#define SMARTAG_VERSION_PATCH  2

/* MSB to LSB: 16bits sampling time, 8bits log mode, 8bits enable flags (last byte, MSB) */
#define SMARTAG_CONFIG_ADDR        (0x0004 + SMARTAG_START_ADDR_OFFSET)

/* NFC EEPROM address for date and time (32 bits compact data: 6 bits year-baseyear, 5 bits day, 4 bits month, 5 bits hour, 6 bits min, 6 bits secs */
#define SMARTAG_ADDR_32BIT_DATE_TIME (0x0008 + SMARTAG_START_ADDR_OFFSET)

/* NFC EEPROM address for Thresholds values */
#define SMARTAG_ADDR_32BIT_THS (0x000C + SMARTAG_START_ADDR_OFFSET)

/* NFC EEPROM address for command (32 bits command to this app or reply from this app) */
#define SMARTAG_ADDR_COMMAND_REPLY  (0x0014 + SMARTAG_START_ADDR_OFFSET)

#define SMARTAG_MAX_T_32BITDATATIME_ADDR   (0x0018 + SMARTAG_START_ADDR_OFFSET)
#define SMARTAG_MIN_T_32BITDATATIME_ADDR   (0x001C + SMARTAG_START_ADDR_OFFSET)
#define SMARTAG_MAX_RH_32BITDATATIME_ADDR  (0x0020 + SMARTAG_START_ADDR_OFFSET)
#define SMARTAG_MIN_RH_32BITDATATIME_ADDR  (0x0024 + SMARTAG_START_ADDR_OFFSET)

/* NFC EEPROM address maximum and minimum value for temperature (min= 8 bits, max= 8 bits) and relative humidity (min= 8 bits, max= 8 bits) */
#define SMARTAG_ADDR_MINMAX_T_RH   (0x0028 + SMARTAG_START_ADDR_OFFSET)

#define SMARTAG_MAX_P_32BITDATATIME_ADDR   (0x002C + SMARTAG_START_ADDR_OFFSET)
#define SMARTAG_MIN_P_32BITDATATIME_ADDR   (0x0030 + SMARTAG_START_ADDR_OFFSET)

#define SMARTAG_MAX_ACC_32BITDATATIME_ADDR  (0x0034 + SMARTAG_START_ADDR_OFFSET)

/* NFC EEPROM address maximum and minimum value for ambient pressure (min= 12 bits, max= 12 bits) and accelerometer vector norm (8 bits) */
#define SMARTAG_ADDR_MINMAX_P_ACCV (0x0038 + SMARTAG_START_ADDR_OFFSET)

/*  16bits counter for sensor and NFC EEPROM address for last sample pointer (16 bits) <- currently used only by firmware, smartphone app uses sample counter instead */
#define SMARTAG_ADDR_LAST_SAMPLE_POINTER   (0x003C + SMARTAG_START_ADDR_OFFSET)

/*  16bits counter for sensor and NFC EEPROM address for last sample pointer (16 bits) <- currently used only by firmware, smartphone app uses sample counter instead */
#define SMARTAG_ADDR_DEVICE_OPERATION      (0x0060)
#define SMARTAG_ADDR_DEVICE_ID             (SMARTAG_ADDR_DEVICE_OPERATION + 4)
#define SMARTAG_ADDR_DEVICE_COMMAND        (SMARTAG_ADDR_DEVICE_ID + 12)
#define SMARTAG_ADDR_DEVICE_SETTINGS       (SMARTAG_ADDR_DEVICE_COMMAND + 4)

/**********************************************************************************************************************
 * Memory locations for Date, time and compact data sample (data written in circular sequence ):                      *
 * - 32 bits for date and time (6 bits year-baseyear, 5 bits day, 4 bits month, 5 bits hour, 6 bits min, 6 bits secs) *
 * - 32 bits for compact data sample (12 bits pressure, 7 bits humidity, 7 bits temperature, 6 bits acc vector norm)  *
 **********************************************************************************************************************/
#define SMARTAG_BEGIN_ADDR_COMPACT_DATA (0x0044 + SMARTAG_START_ADDR_OFFSET)

/* We need to save the TLV termination byte */
#define SMARTAG_END_ADDR_COMPACT_DATA   (ST25DV_MAX_SIZE - ST25DV_CC_SIZE)

#define ST25DV_NFC_MAX_SAMPLE_NUM ((SMARTAG_END_ADDR_COMPACT_DATA - SMARTAG_BEGIN_ADDR_COMPACT_DATA) >> 3)

/* Default value for Watchdog timeout period: range value = [1 ... 32] sec. */
#define WATCHDOG_TIMEOUT_PERIOD 3

/* Default values */
#define DATA_DEFAULT_ENABLE_FLAG 0x0F
#define DATA_DEFAULT_LOG_MODE SMARTAG_LOGMODE_ACTIVE
#define DATA_DEFAULT_SAMPLE_INT 5

#define DATA_DEFAULT_P_MAX 0
#define DATA_DEFAULT_P_MIN 4095

#define DATA_DEFAULT_T_MAX 0
#define DATA_DEFAULT_T_MIN 127

#define DATA_DEFAULT_RH_MAX 0
#define DATA_DEFAULT_RH_MIN 127

#define DATA_DEFAULT_ACC_MAX 0
   
/* Default THS values */
#define DATA_DEFAULT_P_THS_MAX 110000
#define DATA_DEFAULT_P_THS_MIN 90000

#define DATA_DEFAULT_T_THS_MAX 250
#define DATA_DEFAULT_T_THS_MIN 240

#define DATA_DEFAULT_RH_THS_MAX 500
#define DATA_DEFAULT_RH_THS_MIN 400
#define DATA_DEFAULT_ACC_THS_MAX 1024

/* Humidity sensor mask (hygrometer reading from HTS221) */
#define HUMIDITY_ENABLE_MASK            0x02
#define HUMIDITY_IS_ENABLE ((NFC_EEPROM_Data.EnableFlags)&HUMIDITY_ENABLE_MASK)

/* Pressure sensor mask (barometer reading from LPS22HB) */
#define PRESSURE_ENABLE_MASK            0x04
#define PRESSURE_IS_ENABLE  ((NFC_EEPROM_Data.EnableFlags)&PRESSURE_ENABLE_MASK)

/* Temperature sensor mask (termometer reading from HTS221) */
#define TEMPERATURE_ENABLE_MASK         0x01
#define TEMPERATURE_IS_ENABLE ((NFC_EEPROM_Data.EnableFlags)&TEMPERATURE_ENABLE_MASK)

/* Accelerometer sensor mask (accelerometer reading from LSM303AH) */
#define ACCELEROMETER_ENABLE_MASK       0x08
#define ACCELEROMETER_IS_ENABLE ((NFC_EEPROM_Data.EnableFlags)&ACCELEROMETER_ENABLE_MASK)
   
/* 6D Orientation mask (accelerometer reading from LSM303AH) */
#define ORIENTATION6D_ENABLE_MASK       0x10
#define ORIENTATION6D_IS_ENABLE ((NFC_EEPROM_Data.EnableFlags)&ORIENTATION6D_ENABLE_MASK)

/* WakeUp mask (accelerometer reading from LSM303AH) */
#define WAKEUP_ENABLE_MASK       0x20
#define WAKEUP_IS_ENABLE ((NFC_EEPROM_Data.EnableFlags)&WAKEUP_ENABLE_MASK)

/* Set RTC with value stored at SMARTAG_ADDR_32BIT_DATE_TIME */
#define APP_COMMAND_SET_RTC 0x00000001

/* ack: oneshot has been executed */
#define APP_REPLY_ONESHOT   0x00000100

/* ack: RTC has been configurated */
#define APP_REPLY_SET_RTC   0xFF000000

/* NFC field */
#define FIELD_FALLING 0
#define FIELD_RISING  1
#define FIELD_UNDEF   2


#ifdef SMARTAG_ENABLE_PRINTF
  #define SMARTAG_PRINTF(...) //printDEBUG(__VA_ARGS__)
#else /* SMARTAG_ENABLE_PRINTF */
  #define SMARTAG_PRINTF(...)
#endif /* SMARTAG_ENABLE_PRINTF */

#endif /* __SMARTAGCONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
