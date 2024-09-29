/**
 * @file   	SX127x.h
 * @author 	Hassan
 * @version	
 * @date	Mar 17, 2016
 * 
 * @brief   
 */

#ifndef SX127x_H_
#define SX127x_H_


#include <SX127x_spi.h>

#if defined(SX1272)

    #include "SX1272Regs-LoRa.h"
    #include "SX1272Regs-Fsk.h"

#elif defined(SX1276)

    #include <SX1276Regs-LoRa.h>
    #include <SX1276Regs-Fsk.h>

#endif

/**
 * @defgroup SX127x_Public_Macros		SX127x Public Macros
 * @{
 */

#define SX127x_IRQFlagSetLora(flag)			((SX127x_handle.LoRa.irqFlags & (flag)) == (flag))
#define SX127x_clearIRQFlagLoRa(flag)		(SX127x_handle.LoRa.irqFlags &= ~(flag))

/** @}*/


/**
 * @defgroup SX127x_enum 	SX127x Enumeration Definitions
 * @{
 */


/**
 * @brief 	 SX127x Modem configuration type define
 * @warning  For the time being, the driver only implements the LoRa configuration
 * 		  	 and as such will not work in FSK mode.
 */
typedef enum{

	SX127x_MODEM_FSK = 0,	/*!< SX127x in FSK mode*/
	SX127x_MODEM_LORA		/*!< SX127x in LoRa mode*/

}SX127xModem_t;

/**
 * @brief Different operating mode definitions for SX127x
 *
 * @note  These values have been defined according the register values
 * 		  as specified in the SX127x datasheet.
 */
typedef enum{

	SX127x_SLEEP 		= 0x00,	/*!< SX127x is in sleep mode
	 	 	 	 	 	 	 	 	 @note this mode has the lowest power consumption*/
	SX127x_STANDBY		= 0x01,	/*!< SX127x is in standby*/
	SX127x_RECEIVER		= 0x05,	/*!< SX127x is waiting to receive signal*/
	SX127x_TRANSMITTER	= 0x03,	/*!< SX127x is transmitting packet*/

}SX127xMode_t;


typedef enum{

	SX127x_PASELECT_RFO  	= RF_PACONFIG_PASELECT_RFO,
	SX127x_PASELECT_PABOOST = RF_PACONFIG_PASELECT_PABOOST

}SX127x_PASelect_t;

/** @}*/

/**
 * @defgroup SX127x_struct	SX127x Structure Definitions
 * @{
 */

/**
 * @brief
 */
typedef struct{
	union{
		struct{
			uint16_t SyncAddressMatch   : 1;
			uint16_t PreambleDetect     : 1;
			uint16_t Timeout		    : 1;
			uint16_t Rssi		        : 1;
			uint16_t PllLock		    : 1;
			uint16_t TxReady		    : 1;
			uint16_t RxReady		    : 1;
			uint16_t ModeReady		    : 1;
			uint16_t LowBat		        : 1;
			uint16_t CrcOk		        : 1;
			uint16_t PayloadReady		: 1;
			uint16_t PacketSent		    : 1;
			uint16_t FifoOverrun		: 1;
			uint16_t FifoLevel		    : 1;
			uint16_t FifoEmpty		    : 1;
			uint16_t FifoFull           : 1;
		}irqFlags;
		struct{
            uint8_t RegIrqFlags1;
            uint8_t RegIrqFlags2;
        }IrqReg;
    };

}SX127x_FSK_t;

/**
 * @brief
 */
typedef struct{

	uint8_t 	 	bandwidth;			/*!< Current bandwidth set in SX127x*/
	uint8_t 	 	spreadFactor;		/*!< Current spread factor set in SX127x*/
	uint8_t		 	irqFlags;			/*!< status of the interrupt status read from SX127x*/

}SX127x_LoRa_t;

/**
 * @brief Handler struct to keep track of SX127x status and configuration
 * @details this is a detailed description
 */
typedef struct{

	SX127xMode_t 	mode;				/*!< Current operating mode*/
	SX127xModem_t	modem;				/*!< Current modem selected*/

	SX127x_LoRa_t	LoRa;
	SX127x_FSK_t	FSK;

	union{
		struct{

			uint8_t DIO0		: 1;
			uint8_t DIO1		: 1;
			uint8_t DIO2		: 1;
			uint8_t DIO3		: 1;
			uint8_t	DIO4		: 1;
			uint8_t DIO5		: 1;

		}nIRQFlags;

		uint8_t nIRQ;
	};

	int16_t		 SNR;				/*!< SNR of the last transmission received*/
	int16_t		 RSSI;				/*!< RSSI of the last transmission received*/
	uint32_t 	 lastRxTick;		/*!< Time of last packet receive*/
	uint32_t 	 lastTxTick;		/*!< Time of last packet successfully transmitted.*/
	uint32_t	 frequencyChannel;	/*!< Current frequency channel*/
	uint8_t		 rxBuffer[256];		/*!< character buffer to store received packets*/
	uint8_t 	 rxLength;			/*!< Size of the last received packet in bytes.*/

}SX127xHandle_t;

/** @}*/

#define RF_FREQUENCY                                868000000 // Hz
#define TX_OUTPUT_POWER                             14        // dBm


#define FSK_FDEV                                    50e3      // Hz
#define FSK_DATARATE                                100e3      // bps
#define FSK_BANDWIDTH                               100e3      // Hz
#define FSK_AFC_BANDWIDTH                           83.333e3  // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false


extern SX127xHandle_t SX127x_handle;




#ifdef __cplusplus
extern "C"{
#endif


SX127xReturn_t	SX127x_Init		(void);
SX127xReturn_t  SX127x_config   (SX127xModem_t modem);
SX127xReturn_t  SX127x_nIRQHandler(void);
SX127xReturn_t  SX127x_receiverMode(void);
SX127xReturn_t  SX127x_TransmitPacket(uint8_t* packet, uint8_t len);
SX127xReturn_t  SX127x_Transmit_IT	 (uint8_t* packet, uint8_t len);
SX127xReturn_t  SX127x_sleepMode(void);
SX127xReturn_t  SX127x_setGain(uint8_t bandwidth, uint8_t spreadFactor);
SX127xReturn_t  SX127x_verifyConfig(void);
SX127xReturn_t  SX127x_FSK_TransmitPacket(uint8_t* packet, uint8_t len);

bool SX127x_isRxDone(void);
bool SX127x_isTxDone(void);

#ifdef __cplusplus
}
#endif

#endif /* SX127x_H_ */
