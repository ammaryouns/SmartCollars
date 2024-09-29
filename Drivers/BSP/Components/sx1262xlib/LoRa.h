/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: Display driver header

Maintainer: Gregory Cristian & Gilbert Menth
*/

#ifndef _LORA_H_
#define _LORA_H_

#include "bsp.h"
#include "radio.h"
#include "sx126x.h"




/*!
 * \brief Define range of central frequency [Hz]
 */
#define DEMO_CENTRAL_FREQ_MIN       850000000UL
#define DEMO_CENTRAL_FREQ_MAX       930000000UL

/*!
 * \brief Define 3 preset central frequencies [Hz]
 */
#define DEMO_CENTRAL_FREQ_PRESET1   868000000UL
#define DEMO_CENTRAL_FREQ_PRESET2   915200000UL
#define DEMO_CENTRAL_FREQ_PRESET3   930000000UL

/*!
 * \brief Define min and max Tx power [dBm]
 */
#define SX1261_POWER_TX_MIN           -17
#define SX1261_POWER_TX_MAX           15

#define SX1262_POWER_TX_MIN           -10
#define SX1262_POWER_TX_MAX           22
#define LORA_POWER_TX_MAX             20
#define LORA_POWER_TX_MIN             5


typedef enum{
  
  LoRa_OK = 0,
  LoRa_ERROR,
  LoRa_BUSY,
  LoRa_TIMEOUT
  
}LoRa_Return_t;

/*!
 * \brief Define GFSK bitrate
 */
typedef enum
{
    DEMO_BR_100         = 100,
    DEMO_BR_600         = 600,
    DEMO_BR_4800        = 4800,
    DEMO_BR_9600        = 9600,
    DEMO_BR_19200       = 19200,
    DEMO_BR_57600       = 57600,
    DEMO_BR_100000      = 100000,
    DEMO_BR_250000      = 250000,
}DemoBitrate_t;

/*!
 * \brief Define GFSK frequency deviation
 */
typedef enum
{
    DEMO_FDEV_5000      = 5000,
    DEMO_FDEV_10000     = 10000,
    DEMO_FDEV_25000     = 25000,
    DEMO_FDEV_50000     = 50000,
    DEMO_FDEV_75000     = 75000,
    DEMO_FDEV_100000    = 100000,
    DEMO_FDEV_150000    = 150000,
}DemoFrequencyDev_t;

/*!
 * \brief List of states for demo state machine
 */
enum DemoInternalStates
{
    APP_IDLE = 0,               // nothing to do (or wait a radio interrupt)
    SEND_PING_MSG,
    SEND_PONG_MSG,
    APP_RX,                     // Rx done
    APP_RX_TIMEOUT,             // Rx timeout
    APP_RX_ERROR,               // Rx error
    APP_TX,                     // Tx done
    APP_TX_TIMEOUT,             // Tx error
    PER_TX_START,               // PER master
    PER_RX_START,               // PER slave
    CAD_DONE,                   // CAD Done
    CAD_DONE_CHANNEL_DETECTED   // Channel Detected following a CAD
};


/*!
 * \brief Define freq offset for config central freq in "Radio Config Freq" menu
 */
enum FreqBase
{
    FB1     = 1,            //   1 Hz
    FB10    = 10,           //  10 Hz
    FB100   = 100,          // 100 Hz
    FB1K    = 1000,         //   1 kHz
    FB10K   = 10000,        //  10 kHz
    FB100K  = 100000,       // 100 kHz
    FB1M    = 1000000,      //   1 MHz
    FB10M   = 10000000      //  10 MHz
};



/*!
 * \brief Init RAM copy of Eeprom structure and init radio with it.
 *
 */
LoRa_Return_t LoRa_Init( void );

uint8_t LoRa_TransmitPacket(uint8_t* data, uint32_t length);

uint8_t LoRa_receiverMode(uint32_t timeout, bool boostedRx);

uint8_t LoRa_standbyMode(void);

uint8_t LoRa_sleepMode(void);

uint32_t LoRa_getTimeOnAir( uint8_t modulation );

int8_t LoRa_getSNR( void );

int8_t LoRa_getRSSI( void );

RadioState_t LoRa_getOperatingMode(void);

uint8_t LoRa_IrqHandler(void);


void LoRa_clearIRQFlags( RadioIrqMasks_t flags);

bool LoRa_IRQFlagSet( RadioIrqMasks_t flags );

RadioIrqMasks_t LoRa_IRQFlagGet( void );

LoRa_Return_t LoRa_setGain(RadioLoRaBandwidths_t bw, RadioLoRaSpreadingFactors_t sf);
LoRa_Return_t LoRa_SetTxPower(int8_t txPower);
LoRa_Return_t LoRa_calcAndSetTxPower(int8_t rxPower);

uint32_t LoRa_getRxBufferLength(void);

#endif // _LORA_H_
