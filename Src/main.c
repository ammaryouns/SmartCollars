
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "usart.h"
#include "rtc.h"
#include "spi.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include "bsp.h"
#include "mIWDG.h"
#include "log.h"
#include "DEBUG_UART.h"
#include "mRTC.h"
#include "SensorTile_accelero.h"
#include "SensorTile_gyro.h"
#include "crc32.h"
#include "Device_Params.h"
#include "mRTC.h"

#include "SensorDefs.h"


#include "DEBUG_UART.h"

#include "MAC_Application.h"
#include "MAC_Node_FirmwareApp.h"
#include "nfc04a1_nfctag.h"
#include "TagType5.h"
#include "NFC_Application.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


//uint8_t H_DviceID = LSM6DSM_PED7OMETER_THRESHOLD;
uint8_t H_DviceID = 3; //65
uint32_t globalDeviceOperatinStatus = 55;

const uint32_t  NODE_FW_CRC       __attribute__((at(FLASH_FW_CRC_ADDR)))     = (CRC_SEED);
const uint32_t  NODE_FW_VERSION   __attribute__((at(FLASH_FW_VER_ADDR)))     = (100);      
const uint32_t  NODE_FW_SIZE      __attribute__((at(FLASH_FW_SIZE_ADDR)))    = (0x20000); // 128 KB
const uint32_t  NODE_TYPE         __attribute__((at(FLASH_FW_TYPE_ADDR)))    = (DEV_NODE);
const uint32_t  NODE_APP_ID       __attribute__((at(FLASH_FW_APP_ID_ADDR)))  = ( 0x2D78756E );
const uint8_t   CompileTime[25]   __attribute__((at(FLASH_FW_CDT_ADDR)))     = __TIME__ ", " __DATE__ "\n\0";



/* Private variables ---------------------------------------------------------*/
static Node_SensorsData_t    sensors_RAM_Queue_Buffer[100];
static LOG_NodeErrorPacket_t logs_RAM_Queue_Buffer[400];

osThreadId t_IWDGHandle;

osMutexId       hmutex_SDCard;
uint32_t LastRTCsyncPacket = 0;

mQueue_List_Element_Def( sensors_RAM_Queue, Node_SensorsData_t,     sizeof_array(sensors_RAM_Queue_Buffer),  
                                                                    sensors_RAM_Queue_Buffer, 
                                                                    mQueue_ifc); 
                                                                      
mQueue_List_Element_Def( logs_RAM_Queue,    LOG_NodeErrorPacket_t,  sizeof_array(logs_RAM_Queue_Buffer),
                                                                    logs_RAM_Queue_Buffer,
                                                                    mQueue_ifc);


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void thread_iwdg(void const * argument);

void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint32_t GunixSeconds;
uint32_t GmilliSeconds;
extern uint32_t __Vectors[];                             /* vector table ROM  */
extern uint32_t __Vectors_Size;                             /* vector table ROM  */
 
#define VECTORTABLE_SIZE        (0x200)          /* size Cortex-M3 vector table */
#define VECTORTABLE_ALIGNMENT   (0x200ul) /* 16 Cortex + 32 ARMCM3 = 48 words */
                                          /* next power of 2 = 256            */
 
/* new vector table in RAM */
uint32_t vectorTable_RAM[VECTORTABLE_SIZE] __attribute__((at(SRAM1_BASE + 0x2000)));

uint32_t * bbbb = (uint32_t *)( 0x20020A0C);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */



  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
//    HAL_PWREx_EnterSHUTDOWNMode();
  int i;
  for (i = 0; i < VECTORTABLE_SIZE; i++) {
    vectorTable_RAM[i] = __Vectors[i];            /* copy vector table to RAM */
  }
 
  
    /* relocate vector table */ 
  __disable_irq();
    SCB->VTOR = (uint32_t)&vectorTable_RAM;
  __DSB();
  __enable_irq();
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CRC_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  
  
//  HAL_GPIO_WritePin(R_POWER_EN_GPIO_Port, R_POWER_EN_Pin, GPIO_PIN_SET);
//  RTC_getUnixTimeMsGPS(&GunixSeconds, &GmilliSeconds);

#if defined(GPS_DATA_LOGGING)  ||  defined( POWER_DATA_LOGGING) || defined( RAW_DATA_COLLECTION)
  osMutexDef(MUTEX_SD_CARD);
  hmutex_SDCard = osMutexCreate(osMutex(MUTEX_SD_CARD));
#endif

  #ifndef DEBUG_DISABLE_LORA   
  
  osMutexDef(MUTEX_MAC_RADIO);
  hmutex_MACRadio       = osMutexCreate(osMutex(MUTEX_MAC_RADIO));

  osThreadDef(t_MAC_QUERY, thread_MAC_Query, osPriorityRealtime, 0, 2*configMINIMAL_STACK_SIZE);
  hthread_MACWaitForQuery = osThreadCreate(osThread(t_MAC_QUERY), NULL);

  #ifndef DEBUG_DISABLE_OTA_UPDATE
  
  osThreadDef(t_MAC_OTA, thread_FirmwareUpdate, osPriorityRealtime, 0,4* configMINIMAL_STACK_SIZE);
  hthread_FirmwareUpdate = osThreadCreate(osThread(t_MAC_OTA), NULL);
  #endif
  
  #else
  #if defined(RADIO_UNIT_TEST)
  osMutexDef(MUTEX_MAC_RADIO);
  hmutex_MACRadio       = osMutexCreate(osMutex(MUTEX_MAC_RADIO));

  osThreadDef(t_LORA, thread_MAC_UnitTest, osPriorityRealtime, 0, 2*configMINIMAL_STACK_SIZE);
  hthread_MACWaitForQuery = osThreadCreate(osThread(t_LORA), NULL);

  #endif
  #endif

  /* definition and creation of t_IWDG */
  osThreadDef(t_IWDG, thread_iwdg, osPriorityLow, 0, configMINIMAL_STACK_SIZE);
  t_IWDGHandle = osThreadCreate(osThread(t_IWDG), NULL);
 
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  
  printDEBUG_int();

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure LSE Drive Capability 
    */
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 13;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_LPUART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_LSE;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */

void SystemClock_ConfigMSI_2MHz(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure LSE Drive Capability 
    */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_MSI;

  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;

  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */

  
    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Enable MSI Auto calibration 
    */
  HAL_RCCEx_EnableMSIPLLMode();

  /* SysTick_IRQn interrupt configuration */

}
/** System Clock Configuration
*/
void SystemClock_ConfigHSI(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure LSE Drive Capability 
    */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  
  //RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 13;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_LSE;
//  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
//  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}


extern TaskHandle_t xTaskMpuIRQ;
extern volatile uint8_t MEMSInterrupt; 

#ifdef POWER_DATA_LOGGING
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float power_mW= 0;
float loadvoltage = 0;
int writeIntoFile (const char* path, char * buff, unsigned int size );
#endif

extern uint8_t uartBufferRx[];
extern uint32_t uartBufferRxCount;

uint32_t LastGPSTimeSynced = (uint32_t)-1;
uint32_t BSP_NFCTAG_int_count = 0;
uint16_t BSP_NFCTAG_ITStatus = 0;
#if defined(GPS_DATA_LOGGING) 
char GPStimeFileName[] = "GPSTime.gps";
static uint8_t GPStimeSDBuffer[200];
static uint32_t GPStimeSDBufferBytes=0;
uint8_t GPStimeString[512];
uint32_t GPStimeAvailable = 0;
#endif


#if defined(CORE_TEMP_LOGGING)  || defined(LSE_TEMP_COMPENSATION)
float Core_Temperature = 0;        
float Get_CoreTemperature(void)
{
  uint16_t uhADCxConvertedData_VrefAnalog_mVolt = 1800;  

  uint32_t TS_DATA[1];
  ADC_ChannelConfTypeDef sConfig;
  
  MX_ADC1_Init();
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }  
  
  /* Run the ADC calibration in single-ended mode */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK){
    TS_DATA[0]= HAL_ADC_GetValue(&hadc1);
  }
  HAL_ADC_Stop(&hadc1);  
  HAL_ADC_DeInit(&hadc1);  
 
  Core_Temperature = __LL_ADC_CALC_TEMPERATURE(uhADCxConvertedData_VrefAnalog_mVolt, TS_DATA[0], LL_ADC_RESOLUTION_12B);
  return   Core_Temperature;
}
 #endif
TaskHandle_t xTaskNfcIRQ = NULL;
extern volatile uint8_t NFCStatus;

//Eeprom_t Eeprom;
//#define SX_DEMO_APPLICATION
/* thread_Default function */
void thread_Default(void const * argument)
{
  /* USER CODE BEGIN thread_Default */
  deviceParams_pullFromFlash(&deviceSettings_crc);
  
  SensorsData_t deviceID;
  deviceParams_getUID( (Device_UID_t*)deviceID.Data);
  printTask("DevID:%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n", 
  deviceID.Data[0], deviceID.Data[1], deviceID.Data[2], deviceID.Data[3],
  deviceID.Data[4], deviceID.Data[5], deviceID.Data[6], deviceID.Data[7],
  deviceID.Data[8], deviceID.Data[9], deviceID.Data[10], deviceID.Data[11]);
  printTask("Test Firmware : %.2f", (float)NODE_FW_VERSION/100);
  printTask("fimware Compile TimeStamp : %s", CompileTime);      
  printTask("IP : *.*.%u.%u", deviceSettings_crc.DcuID, deviceSettings_crc.networkID);
 
  //The last queue added to list is placed at the start.
  mQueue_List_AddQueue(&sensors_Queue_List, &sensors_RAM_Queue);
  mQueue_List_AddQueue(&logs_Queue_List, &logs_RAM_Queue);

  mQueue_List_get_elementCount(&sensors_Queue_List);
  mQueue_List_get_elementCount(&logs_Queue_List);
  LOG_Error(LOG_ErrorSource_System, LOG_Error_SystemReset, RCC->CSR);
  LOG_Error(LOG_ErrorSource_System, LOG_Error_FirmwareVersion, NODE_FW_VERSION);
  __HAL_RCC_CLEAR_RESET_FLAGS();  
  
  deviceID.flags.all= 0;
  deviceID.timeStamp = RTC_getUnixTime();
  deviceID.flags.DevID = true;
  Sensors_savePacket(&deviceID);

  
  if(BSP_NFCTAG_Init() == NFCTAG_OK) {
    //printTask("NFC chip is present\r\n\n");
    BSP_NFCTAG_GetITStatus(&BSP_NFCTAG_ITStatus);
//      uint16_t ITconfig = 0x80 | 0x40;
//      BSP_NFCTAG_ConfigIT(ITconfig);
      
    NfcType5_NDEFInitHeader((uint8_t *)deviceID.Data);
    ReadNfcConfiguration();
    NfcMemoryTest();
    osDelay(200);
    xTaskNfcIRQ = xTaskGetCurrentTaskHandle();
    WriteNfcDevieID((uint8_t *)deviceID.Data);
    Nfc_DeviceSettings();
  }
  else
  {
    printTask("NFC chip is not present ***********************\n");
  }
//  HAL_I2C_DeInit( &hi2c1 );
  NFCStatus = NFC_STATUS_OFF;
  PowerOffNFC();
  uint32_t ulNotificationValue;
  int32_t poweroff_count = 0;
  int32_t NFC_lastcomm_UnixTime = 0;


#ifdef POWER_DATA_LOGGING

#endif

#if defined(CORE_TEMP_LOGGING)  || defined(LSE_TEMP_COMPENSATION)
  Core_Temperature =  Get_CoreTemperature();
  printTask("Core Temperature %3.2f C", Core_Temperature);

#endif
#if defined(SX_DEMO_APPLICATION)   
  

 
#endif
#if defined(GPS_DATA_LOGGING) || defined(ANN_TEST)

#endif
  for(;;)
  {  

#if defined( POWER_DATA_LOGGING) 

#endif

    
#ifdef GPS_DATA_LOGGING

#endif
    
  

  
  
#if defined(GPS_DATA_LOGGING)   

#else
  {
    xTaskNfcIRQ = xTaskGetCurrentTaskHandle();   
    BaseType_t  statebefore = xTaskNotifyStateClear( NULL );   
  
    int sleepTime = portMAX_DELAY;   
    if( NFCStatus == NFC_STATUS_ON)
    {
      if(NFC_lastcomm_UnixTime + 5 > RTC_getUnixTime())
      {
        sleepTime = 5000;
        
      }
      else
      {
        printDEBUG("turn off NFC \r\n");
        NFCStatus = NFC_STATUS_OFF;
        PowerOffNFC();
//        HAL_I2C_DeInit(&hi2c1);
        poweroff_count = 0;   
//        printDEBUG("turn off I2c \r\n");
        
      }

    }

    ulNotificationValue = ulTaskNotifyTake( pdTRUE, sleepTime );
    xTaskNfcIRQ = 0;
    printTask("NFC int handler  %d,%d,%d\n", RFActivity, BSP_NFCTAG_int_count, ulNotificationValue);        
        /* There are a RF activity */
    if(RFActivity) {
      printDEBUG("INFC int handler       RFActivity\r\n");
      RFActivity = 0;
      DetectRFActivity();
      poweroff_count = 0;
      NFC_lastcomm_UnixTime = RTC_getUnixTime();
    }

  }
#endif
  }
  /* USER CODE END thread_Default */
}


#if defined(GPS_DATA_LOGGING) || defined(ANN_TEST)

int New_Accel_results(float* accel);
extern uint32_t forceEnd; 
extern uint32_t forceWalkingEnd;
/**************** GPS TIME ************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if( uartBufferRx[uartBufferRxCount] == '\n')
    {
        uartBufferRx[uartBufferRxCount+1] = '\0';
      
        if(uartBufferRx[0] == '\0')
        {
        }
#if defined(GPS_DATA_LOGGING)          
        else if(uartBufferRx[0] == '$')
        {
            int32_t Hours = 0;
            int32_t Minutes = 0;
            int32_t Seconds = 0;
            int32_t MiliSeconds = 0;
            int32_t Date = 0;
            int32_t Month = 0;
            int32_t Year = 0;
            int32_t ZoneHour = 0;
            int32_t ZoneMinute = 0;
            uint32_t checkSum = 0;
            int32_t parameters = sscanf((char*)uartBufferRx, "$GPZDA,%02d%02d%02d.%d,%d,%d,%d,%d,%d*%X\r\n",
                    &Hours, &Minutes, &Seconds, &MiliSeconds, &Date, &Month, &Year, &ZoneHour, &ZoneMinute, &checkSum);
            if(parameters == 10)
            { 
              uint32_t unixSeconds = 0;
              uint32_t milliSeconds = 0;    
              RTC_getUnixTimeMsGPS(&unixSeconds, &milliSeconds);
              struct tm currTime;

              currTime.tm_year = Year - 1900;
              currTime.tm_mon  = Month - 1;
              currTime.tm_mday = Date;
              currTime.tm_hour = Hours;
              currTime.tm_min  = Minutes;
              currTime.tm_sec  = Seconds;
              currTime.tm_isdst = false;
              time_t time = mktime(&currTime);
              
              int32_t deltaTime = (int32_t)time - (int32_t)unixSeconds;
              deltaTime *= 1000; // convert to millisecond
              deltaTime += MiliSeconds;
              deltaTime -= milliSeconds;
            
              printDEBUG("\t\tTime synced [%d]\n", deltaTime);
              if(deltaTime < 0) deltaTime *=-1;
//              if ((deltaTime)  > 4000) // if there is time differnen update the time
//              {
//                RTC_setDateTime(&currTime, MiliSeconds);  
//              }
              GPStimeSDBufferBytes = sprintf((char*)GPStimeString, "%d.%03d,%d.%03d,%d,%s \r\n ", unixSeconds, milliSeconds, time, MiliSeconds, Core_Temperature, (char*)uartBufferRx);  
              GPStimeAvailable = true;                      
              
              LastGPSTimeSynced = osKernelSysTick();    
              
              printDEBUG("Time Recived on GPS %02d:%02d:%02d.%03d \n", currTime.tm_hour, currTime.tm_min, currTime.tm_sec, MiliSeconds);
              printDEBUG("Time Recived on GPS %d.%03d,%d.%03d \r\n ", unixSeconds, milliSeconds, time, MiliSeconds);
 //              memset(uartBufferRx, 0,  uartBufferRxCount);  
             
            }
        }   
#endif        
#if defined(ANN_TEST)
        else if(uartBufferRx[0] == 'A')
        {
            float  a[3] = {0,0,0};
            sscanf((char*)uartBufferRx, "A,%f,%f,%f\n", &a[0], &a[1], &a[2]);
            New_Accel_results(a);
        }
        else if(uartBufferRx[0] == 'E')
        {
            if(strncmp((char*)uartBufferRx, "END", 3) == 0)
            {
              forceEnd = 1;
            }
            else if (strncmp((char*)uartBufferRx, "ENW", 3) == 0)
            {
               forceEnd = 1; forceWalkingEnd = 1; 
            }
            printDEBUG("%s" ,uartBufferRx);
        }   
#endif        
        uartBufferRxCount = 0;
        HAL_UART_Receive_IT(&hlpuart1, (uint8_t*)&uartBufferRx[uartBufferRxCount], 1);
    }
    else
    {
        uartBufferRxCount++;
        HAL_UART_Receive_IT(&hlpuart1, (uint8_t*)&uartBufferRx[uartBufferRxCount], 1);
    }

}
#endif

/* thread_iwdg function */
void thread_iwdg(void const * argument)
{ 
  uint32_t ticks = osKernelSysTick();
  uint8_t index = 0;
#ifndef DEBUG_DISABLE_IWDG   // Do not create IWDG thread if disabled 
  MX_IWDG_Init(); 
#else
  #warning "Watchdog is diabled \n"
#endif
  mIWDG_init();

  /* Infinite loop */
  for(;;)
  {
#ifndef DEBUG_DISABLE_IWDG   // Do not create IWDG thread if disabled 
    HAL_IWDG_Refresh(&hiwdg);
#endif
    //printTask("\t\tHAL_IWDG_Refresh\n");        
        
    //incase of overflow.
    index = IWDG_checkOverFlow();

    if (IWDG_checkOverFlow())
    {   
      //back up packets before resetting.
      //HAL_EEPROM_WriteQueue(&eepromPacketQueue, &packetQueue, packetQueue.count);
      LOG_Error(LOG_ErrorSource_System, LOG_Error_IWDGOverflow, index);
      SCB->VTOR = FLASH_BASE;
      HAL_NVIC_SystemReset();
      for(;;);                //manually reset system or wait for IWDG to do it automatically???
    }

    ticks = RTC_milliSecondsTillMark( 20 );
    //printTask("\t\twait for %d ticks\n\n",ticks);        
    if(ticks > 20000)
    {
    }
    osDelay(ticks);    
  }
}

extern void RadioOnDioIrq( void );

/**
* @brief  EXTI line detection callbacks
* @param  GPIO_Pin: Specifies the pins connected EXTI line
* @retval None
*/
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(GPIO_Pin == MPU_INT2_Pin ){
    MEMSInterrupt=1;
    if(xTaskMpuIRQ != NULL)
    {
      /* Notify the task that the transmission is complete. */
      vTaskNotifyGiveFromISR( xTaskMpuIRQ, &xHigherPriorityTaskWoken );
//      xTaskNotifyFromISR( xTaskMpuIRQ,
//                       0x01,
//                       eSetBits,
//                       &xHigherPriorityTaskWoken );
    }
	}
  else if(GPIO_Pin == SX1262_DIO1_Pin)
  {
    RadioOnDioIrq();
    if(xTaskRadioIRQ != NULL)
    {
      /* Notify the task that the transmission is complete. */
      vTaskNotifyGiveFromISR( xTaskRadioIRQ, &xHigherPriorityTaskWoken );
    }

  } 
  else if(GPIO_Pin == NFC_INT_Pin)
  { 
    RFActivity = 1;
    BSP_NFCTAG_int_count++;
    if(xTaskNfcIRQ != NULL)
    {
      vTaskNotifyGiveFromISR( xTaskNfcIRQ, &xHigherPriorityTaskWoken );
    }
  }
  portYIELD_FROM_ISR(    xHigherPriorityTaskWoken);

}
extern  uint32_t ulNotificationMMpu;   

extern void HAL_IncTicks(uint32_t ticks);
#define printSleep(...)

static uint32_t awakeTicks = 0;
extern void PreSleepProcessing(uint32_t* idleTime){    
   
  #ifndef DISABLE_UC_SLEEP 
//  printTask("MPU interupt fired %d,%d\n", ulNotificationMMpu, MEMSInterrupt);        
  printSleep("NFC interupt  %d,%d\n", RFActivity, BSP_NFCTAG_int_count);        

  printSleep("\nAwake Time : %u\n", osKernelSysTick() - awakeTicks);
  printSleep("Expected Idle Time : %d \n\t", *idleTime);

//  for (int i = 0 ; i < ulNumberTasksInList; i++)
//  {
//    printDEBUG("%11s:%7u%7u%7u", ucTaskList[i], 
//                                            ulTimeSpentInTask[i], 
//                                            ulTaskSwitchInTick[i],
//                                            ulTaskDelayedForTicks[i]);
//  }
  
//  HAL_GPIO_WritePin(SX1262_NSS_GPIO_Port, SX1262_NSS_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(LSM6DSM_SPI_CS_Port, LSM6DSM_SPI_CS_Pin, GPIO_PIN_SET);
  if(*idleTime > 60000)
  {
    *idleTime = 60000;
  }
  if (RTC_setAlarmMs(*idleTime ) == HAL_OK)
  {
//    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
//    SystemClock_ConfigMSI_2MHz();
  printSleep("HAL_PWREx_EnterSTOP2Mode\n");
    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
//    HAL_PWREx_EnterSHUTDOWNMode();
  }
//  HAL_GPIO_WritePin(SX1262_NSS_GPIO_Port, SX1262_NSS_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(LSM6DSM_SPI_CS_Port, LSM6DSM_SPI_CS_Pin, GPIO_PIN_RESET);
 

//  memset(ulTimeSpentInTask, 0, sizeof(ulTimeSpentInTask));

  printSleep("Woke up from sleep\n");
  SystemClock_ConfigHSI();
  
  printSleep("SystemClock_ConfigHSI configured\n");

  *idleTime = 0;
  #endif
   
} 

extern void PostSleepProcessing(uint32_t* idleTime)
{
    
   
    
}


void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime ){
    
  TickType_t xModifiableIdleTime;
  int32_t prevRTCTicks;
  int32_t currRTCTicks;
  uint32_t ticksToAdd;
  uint32_t unixSeconds;
  uint32_t milliSeconds;

  /* Enter a critical section but don't use the taskENTER_CRITICAL()
  method as that will mask interrupts that should exit sleep mode. */
  HAL_SuspendTick();
  __disable_irq();

  prevRTCTicks = RTC_getUnixTimeMs(&unixSeconds, &milliSeconds);
  /* If a context switch is pending or a task is waiting for the scheduler
  to be unsuspended then abandon the low power entry. */
  if( eTaskConfirmSleepModeStatus() == eAbortSleep )
  {
    __enable_irq();
    HAL_ResumeTick();
    return ;
  }
  else
  {
    /* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
    set its parameter to 0 to indicate that its implementation contains
    its own wait for interrupt or wait for event instruction, and so wfi
    should not be executed again.  However, the original expected idle
    time variable must remain unmodified, so a copy is taken. */
    xModifiableIdleTime = xExpectedIdleTime;
    configPRE_SLEEP_PROCESSING( &xModifiableIdleTime );
    if( xModifiableIdleTime > 0 )
    {
        __DSB();
        __WFI();
        __ISB();
    }
    configPOST_SLEEP_PROCESSING( &xExpectedIdleTime );


    /* Re-enable interrupts - see comments above __disable_irq() call
    above. */

    //woke up from something other than RTC
    if(__HAL_RTC_ALARM_GET_FLAG(&hrtc, RTC_FLAG_ALRAF) == (uint32_t)RESET)
    {
      __HAL_RTC_ALARM_EXTI_CLEAR_FLAG();            
    }
    
    currRTCTicks = RTC_getUnixTimeMs(&unixSeconds, &milliSeconds);
    ticksToAdd = currRTCTicks - prevRTCTicks;
    
    printSleep("\nThread : SuppressTickAndSleep \n Time spent in stop mode : %d \n", ticksToAdd);
    
    //to account for assert in vTaskStepTick()
    if (ticksToAdd > xExpectedIdleTime)
        ticksToAdd = xExpectedIdleTime - 1;
    
    printSleep("\nThread : SuppressTickAndSleep \n Time spent in stop mode : %d \n", ticksToAdd);
    portENTER_CRITICAL();
    {
        HAL_IncTicks( ticksToAdd );
        vTaskStepTick( ticksToAdd );
    }
    portEXIT_CRITICAL();
//        printDEBUG("xTickCount after sleep : %d \n", osKernelSysTick());
    
    __enable_irq();
    
    HAL_ResumeTick();
    awakeTicks = osKernelSysTick();
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
////  /* User can add his own implementation to report the HAL error return state */
////  while(1) 
////  {
////  }
  /* USER CODE END Error_Handler_Debug */
}
void Error_Handler_NFC(uint32_t ErroCode)
{
  switch(ErroCode) {
  case NFC_CONFIG_ERROR:
    /* Error on Configuration triple Led blinking */
  case NFC_WRITING_ERROR:
    /* Error on NFC Writing double Led blinking */
  case NFC_READING_ERROR:
    /* Error on NFC Reading single Led blinking */
  break;
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
