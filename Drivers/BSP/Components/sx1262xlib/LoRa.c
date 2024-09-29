/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: PingPong, PER demo implementation.

Maintainer: Gregory Cristian & Gilbert Menth
*/

#include "bsp.h"
#include "sx126x.h"
#include "bsp.h"
#include "radio.h"
#include "LoRa.h"
#include <math.h>
#include "DEBUG_UART.h"

/*!
 * \brief Defines the local payload buffer size
 */
#define BUFFER_SIZE                     255


/*!
 * \brief Define time used in PingPong demo to synch with cycle
 * RX_TIMEOUT_MARGIN is the free time between each cycle (time reserve)
 */
#define RX_TIMEOUT_MARGIN               150  // ms
#define RX_TX_TRANSITION_WAIT           5    // ms






/*!
 * \brief Buffer and its size
 */
uint8_t BufferSize = BUFFER_SIZE;
uint8_t LoRa_Buffer[BUFFER_SIZE];




/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( void );

/*!
 * \brief Function executed on Radio CAD Done event
 */
void OnCadDone( bool channelActivityDetected );

/*!
 * \brief All the callbacks are stored in a structure
 */
RadioEvents_t RadioEvents =
{
    .TxDone = &OnTxDone,        // txDone
    .RxDone = &OnRxDone,        // rxDone
    .TxTimeout = &OnTxTimeout,     // txTimeout
    .RxTimeout =  &OnRxTimeout,     // rxTimeout
    .RxError = &OnRxError,       // rxError
    .CadDone = &OnCadDone,       // cadDone
};

struct {
	
	uint32_t Irq;
	
	uint32_t lastTxTick;
	uint32_t lastRxTick;
	
	int8_t RSSI;
	int8_t SNR;

} RadioStatus;



/*!
 * \brief Mask of IRQs
 */
uint16_t IrqMask = 0x0000;

/*!
 * \brief Locals parameters and status for radio API
 * NEED TO BE OPTIMIZED, COPY OF STUCTURE ALREADY EXISTING
 */
PacketParams_t PacketParams;
PacketStatus_t PacketStatus;
ModulationParams_t ModulationParams;


/*!
 * \brief Frequency Error (only LSB)
 */
static double FreErrorLsb = 0.0;




void LoRa_config( uint8_t modulation );

void LoRa_clearIRQFlags( RadioIrqMasks_t flags)
{
  RadioStatus.Irq &= ~flags;
}

bool LoRa_IRQFlagSet( RadioIrqMasks_t flags )
{
  return RadioStatus.Irq & flags;
}

RadioIrqMasks_t LoRa_IRQFlagGet( void )
{
  return RadioStatus.Irq;
}

uint32_t LoRa_getRxBufferLength(void)
{
  return BufferSize;
}

LoRa_Return_t LoRa_setGain(RadioLoRaBandwidths_t bw, RadioLoRaSpreadingFactors_t sf)
{
  ModulationParams.Params.LoRa.Bandwidth = bw;
  ModulationParams.Params.LoRa.SpreadingFactor = sf;
  
  return LoRa_OK;
}
LoRa_Return_t LoRa_SetTxPower(int8_t txPower)
{
  if(txPower > LORA_POWER_TX_MAX) txPower = LORA_POWER_TX_MAX;
  ModulationParams.Params.LoRa.TxPower = txPower; 
  return LoRa_OK;
}

LoRa_Return_t LoRa_calcAndSetTxPower(int8_t rxPower)
{
  int8_t txPower = (-11/53)*rxPower +(242/53);
  printDEBUG("Rx%d,Tx%d\n", rxPower, txPower);
  if(txPower < 5) txPower = 5;
  LoRa_SetTxPower(txPower);
  
  return LoRa_OK;
}

uint8_t LoRa_sleepMode(void)
{
	SleepParams_t sleepParams;
	sleepParams.Value = 0;
	sleepParams.Fields.WarmStart = true;
	
	Radio.Sleep();
	return 0;
}

uint8_t LoRa_standbyMode(void)
{
	Radio.Standby();
	return 0;
}


uint8_t LoRa_receiverMode(uint32_t timeout, bool boostedRx)
{
  if (Radio.GetStatus() == RF_RX_RUNNING)
    return 0;
  
	LoRa_standbyMode();
	
  Radio.SetRxConfig( MODEM_LORA, 
                     ModulationParams.Params.LoRa.Bandwidth,
                     ModulationParams.Params.LoRa.SpreadingFactor,
                     ModulationParams.Params.LoRa.CodingRate,
                     0,
                     PacketParams.Params.LoRa.PreambleLength,
                     0,
                     PacketParams.Params.LoRa.HeaderType,
                     PacketParams.Params.LoRa.PayloadLength,
                     PacketParams.Params.LoRa.CrcMode,
                     0, 0, 
                     PacketParams.Params.LoRa.InvertIQ,
                     true );
                     
  
                      
  
	RadioStatus.Irq = 0;
	
	if( boostedRx == false )
	{
			Radio.Rx( timeout );
	}
	else
	{
			Radio.RxBoosted( timeout );
	}

  return LoRa_OK;
}

extern bool IrqFired;
__inline uint8_t LoRa_IrqHandler(void)
{
  Radio.IrqProcess();
	return 0;
}


uint8_t LoRa_TransmitPacket(uint8_t* data, uint32_t length)
{
  int32_t timeout = 3000;
  uint32_t ToA;
	if (length > 255)
		return -1;
	
	LoRa_standbyMode();
  
  Radio.SetTxConfig( MODEM_LORA, ModulationParams.Params.LoRa.TxPower, 0,
                      ModulationParams.Params.LoRa.Bandwidth,
                      ModulationParams.Params.LoRa.SpreadingFactor,
                      ModulationParams.Params.LoRa.CodingRate,
                      PacketParams.Params.LoRa.PreambleLength,
                      PacketParams.Params.LoRa.HeaderType,
                      PacketParams.Params.LoRa.CrcMode,
                      0, 0, 
                      PacketParams.Params.LoRa.InvertIQ,
                      3000);
  
	ToA = Radio.TimeOnAir( MODEM_LORA, length );
  
	RadioStatus.Irq = 0;
	
	Radio.Send( data, length );


  
	return LoRa_OK;
}


LoRa_Return_t LoRa_Init( void )
{ 
  ///@todo : uncomment 
   //HAL_GPIO_WritePin(SX1262_POWER_EN_GPIO_Port, SX1262_POWER_EN_Pin, GPIO_PIN_SET);
    SX126xIoInit();
    SX126x_SPI_Init();

    Radio.Init(&RadioEvents);
  

    memset( &LoRa_Buffer, 0x00, BUFFER_SIZE );
        
    
    ModulationParams.PacketType = PACKET_TYPE_LORA;
    PacketParams.PacketType     = PACKET_TYPE_LORA;
    

		LoRa_config( PACKET_TYPE_LORA );

  return LoRa_OK;
}


/*
 * Function still being implemented >>> To be completed 
 * WARNING: Computation is in float and his really slow
 * LongInterLeaving vs LegacyInterLeaving has no influence on TimeOnAir.
 */
uint32_t LoRa_getTimeOnAir( uint8_t modulation )
{
   return Radio.TimeOnAir( MODEM_LORA, 0xFF );
}

void LoRa_config( uint8_t modulation )
{
    Radio.Standby(  );
    
//    printTask("> InitializeDemoParameters\n\r");
    if( modulation == PACKET_TYPE_LORA )
    {
        printTask("set param LORA for demo\n\r");
        ModulationParams.PacketType = PACKET_TYPE_LORA;
        PacketParams.PacketType     = PACKET_TYPE_LORA;

        ModulationParams.Params.LoRa.SpreadingFactor = LORA_SF10;
        ModulationParams.Params.LoRa.Bandwidth       = LORA_BW_500;
        ModulationParams.Params.LoRa.CodingRate      = LORA_CR_4_5;

        PacketParams.Params.LoRa.PreambleLength      = 8;
        PacketParams.Params.LoRa.HeaderType          = LORA_PACKET_VARIABLE_LENGTH;
        PacketParams.Params.LoRa.PayloadLength       = 0xFF;
        PacketParams.Params.LoRa.CrcMode             = LORA_CRC_ON;
        PacketParams.Params.LoRa.InvertIQ            = LORA_IQ_NORMAL;

        
        if( ( ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF6 ) || ( ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF5 ) )
        {
            if( PacketParams.Params.LoRa.PreambleLength < 12 )
            {
                PacketParams.Params.LoRa.PreambleLength = 12;
            }
        }
    }
    else// if( modulation == PACKET_TYPE_GFSK )
    {
        printd("set param GFSK for demo\n\r");
				return ;
    }


    Radio.SetChannel( 915200000UL );    
}





int8_t LoRa_getSNR(void)
{
  return RadioStatus.SNR;
}

int8_t LoRa_getRSSI(void)
{
  return RadioStatus.RSSI;
}

RadioState_t LoRa_getOperatingMode(void)
{
  return Radio.GetStatus();
}
// ************************     Radio Callbacks     ****************************
// *                                                                           *
// * These functions are called through function pointer by the Radio low      *
// * level drivers                                                             *
// *                                                                           *
// *****************************************************************************
void OnTxDone( void )
{
  printTask("TxDone");
	//on tx done, the radio automatically goes into
	//standby mode. The explicit call to LoRa_standby is
	//to set the OperatingMode variable.
	LoRa_standbyMode();
	
	RadioStatus.Irq |= IRQ_TX_DONE;
	RadioStatus.lastTxTick = HAL_GetTick();
	
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{


	RadioStatus.Irq |= IRQ_RX_DONE;
  
  memcpy( LoRa_Buffer, payload, size );
	BufferSize = size;
	RadioStatus.RSSI = rssi;
	RadioStatus.SNR = snr;
  
  printTask("RXDone\nPL = %d, Rssi/SNR %d/%d\n\r", size, rssi, snr);
}

void OnTxTimeout( void )
{
	//on tx timeout, the radio automatically goes into
	//standby mode. The explicit call to LoRa_standby is
	//to set the OperatingMode variable.
	LoRa_standbyMode();
	
	RadioStatus.Irq |= IRQ_RX_TX_TIMEOUT;
}

void OnRxTimeout( void )
{
	//on rx done, the radio automatically goes into
	//standby mode. The explicit call to LoRa_standby is
	//to set the OperatingMode variable.
	LoRa_standbyMode();
	
	RadioStatus.Irq |= IRQ_RX_TX_TIMEOUT;
}

void OnRxError( void )
{
		int8_t RSSI, SNR;
  ///@todo : read error code
    IrqErrorCode_t errorCode;
    RadioStatus.Irq |= IRQ_CRC_ERROR;
    if( errorCode == IRQ_HEADER_ERROR_CODE )
    {
#ifdef ADV_DEBUG
        printd( ">> IRQ_HEADER_ERROR_CODE\n\r" );
#endif
    }
    else if( errorCode == IRQ_SYNCWORD_ERROR_CODE )
    {
#ifdef ADV_DEBUG
        printd( ">> IRQ_SYNCWORD_ERROR_CODE\n\r" );
#endif
    }
    else if( errorCode == IRQ_CRC_ERROR_CODE )
    {
#ifdef ADV_DEBUG
        printd( ">> IRQ_CRC_ERROR_CODE\n\r" );
#endif
    }
    else
    {
#ifdef ADV_DEBUG
        printd( "unknown error\n\r" );
#endif
    }
			
		//clear FIFO by going into the sleep mode.
		LoRa_sleepMode();
		LoRa_standbyMode();

}

void OnCadDone( bool channelActivityDetected )
{
	LoRa_standbyMode();
	
	RadioStatus.Irq |= IRQ_CAD_DONE;
    if( channelActivityDetected == true )
    {
			RadioStatus.Irq |= IRQ_CAD_ACTIVITY_DETECTED;
    }
    
}
