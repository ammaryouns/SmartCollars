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
#include "radio.h"
#include "sx126x-hal.h"
#include "DemoApplication.h"
#include "DEBUG_UART.h"
#include <math.h>

/*!
 * \brief Defines the local payload buffer size
 */
#define BUFFER_SIZE                     255

/*!
 * \brief Defines the size of the token defining message type in the payload
 *        cf. above.
 */
#define PINGPONG_SIZE                   4
#define PER_SIZE                        3

/*!
 * \brief Define time used in PingPong demo to synch with cycle
 * RX_TIMEOUT_MARGIN is the free time between each cycle (time reserve)
 */
#define RX_TIMEOUT_MARGIN               150  // ms
#define RX_TX_TRANSITION_WAIT           5    // ms


/*!
 * \brief Define the possible message type for the Ping-Pong and PER apps
 */
const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";
const uint8_t PerMsg[]  = "PER";

/*!
 * \brief Buffer and its size
 */
uint8_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( void );

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
void OnRxError( IrqErrorCode_t );

/*!
 * \brief Function executed on Radio CAD Done event
 */
void OnCadDone( bool channelActivityDetected );

/*!
 * \brief All the callbacks are stored in a structure
 */
RadioCallbacks_t RadioEvents =
{
    &OnTxDone,        // txDone
    &OnRxDone,        // rxDone
    NULL,             // rxPreambleDetect
    NULL,             // rxSyncWordDone
    NULL,             // rxHeaderDone
    &OnTxTimeout,     // txTimeout
    &OnRxTimeout,     // rxTimeout
    &OnRxError,       // rxError
    &OnCadDone,       // cadDone
};

// SPI
// mosi, miso, sclk, nss, dio0, dio1, dio2, dio3, rst, freqSel, deviceSel, antSwPower, callbacks...
SX126xInterface_t *Radio = 0;

/*!
 * \brief Tx LED toggling on transmition success
 */
uint32_t TX_LED; //( A4 );

/*!
 * \brief Rx LED toggling on reception success
 */
uint32_t RX_LED; //( A5 );

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
 * \brief Flag to indicate if the demo is already running
 */
static bool DemoRunning = false;

/*!
 * \brief Frequency Error (only LSB)
 */
static double FreErrorLsb = 0.0;

/*!
 * \brief Flag holding the current internal state of the demo application
 */
static uint8_t DemoInternalState = APP_IDLE;

/*!
 * \brief Ticker for master to synch Tx frames. Flags for PER and PingPong demo
 * for Synch TX in cycle.
 */
//Ticker SendNextPacket;
static bool SendNext = false;

/*!
 * \brief Hold last Rx packet number to compute PER in PER and PingPong demo
 */
static uint32_t PacketRxSequence = 0;
static uint32_t PacketRxSequencePrev = 0;

void LedBlink( void );
void InitializeDemoParameters( uint8_t modulation );
uint16_t GetTimeOnAir( uint8_t modulation );
void SendNextPacketEvent( void );



// ************************     Ping Pong Demo     *****************************
// *                                                                           *
// *                                                                           *
// *                                                                           *
// *****************************************************************************
uint8_t RunDemoApplicationPingPong( void )
{
    uint8_t i = 0;
    uint8_t refreshDisplay = 0;

    if( Eeprom.EepromData.DemoSettings.HoldDemo == true )
    {
        return 0;   // quit without refresh display
    }

    if( DemoRunning == false )
    {
        DemoRunning = true;
        TX_LED = 0;
        RX_LED = 0;
        Eeprom.EepromData.DemoSettings.CntPacketTx        = 0;
        Eeprom.EepromData.DemoSettings.CntPacketRxOK      = 0;
        Eeprom.EepromData.DemoSettings.CntPacketRxOKSlave = 0;
        Eeprom.EepromData.DemoSettings.CntPacketRxKO      = 0;
        Eeprom.EepromData.DemoSettings.CntPacketRxKOSlave = 0;
        Eeprom.EepromData.DemoSettings.RxTimeOutCount     = 0;
      
        Eeprom.EepromData.DemoSettings.ModulationType = PACKET_TYPE_LORA;
        InitializeDemoParameters( Eeprom.EepromData.DemoSettings.ModulationType );

        Eeprom.EepromData.DemoSettings.InterPacketDelay = GetTimeOnAir( Eeprom.EepromData.DemoSettings.ModulationType ) + RX_TX_TRANSITION_WAIT;

        printd( "Start RunDemoApplicationPingPong.\n\r" );

        if( Eeprom.EepromData.DemoSettings.Entity == MASTER )
        {
            DemoInternalState = SEND_PING_MSG;
              SendNext = true;
//            uint32_t temp = ( Eeprom.EepromData.DemoSettings.InterPacketDelay << 1 ) + RX_TIMEOUT_MARGIN;
//            float val = ( float )temp / 1000.0;
//            SendNextPacket.attach( &SendNextPacketEvent, val );
        }
        else
        {
            IrqMask = IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT;
            Radio->SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
            // Rx Single without timeout for the start
            RX_LED = !RX_LED;
            
            if( Eeprom.EepromData.DemoSettings.BoostedRx == false )
            {
                Radio->SetRx( 0x0000 );
            }
            else
            {
                Radio->SetRxBoosted( 0x0000 );
            }
            DemoInternalState = APP_IDLE;
        }
    }

    Radio->ProcessIrqs( );

    if( Eeprom.EepromData.DemoSettings.Entity == MASTER )
    {
        switch( DemoInternalState )
        {
            case SEND_PING_MSG:
                if( ( Eeprom.EepromData.DemoSettings.MaxNumPacket != 0 ) \
                    && ( Eeprom.EepromData.DemoSettings.CntPacketTx >= Eeprom.EepromData.DemoSettings.MaxNumPacket ) )
                {
//                    SendNextPacket.detach( );
                    SendNext = false;
                    RX_LED = 0;
                    TX_LED = 0;
                    DemoInternalState = APP_IDLE;
                    Radio->SetStandby( STDBY_RC );
                    Eeprom.EepromData.DemoSettings.HoldDemo = true;
                    refreshDisplay = 1;
                }
                else
                {
                    if( SendNext == true )
                    {
                        SendNext = false;
                        Radio->SetStandby( STDBY_RC );
                        DemoInternalState = APP_IDLE;
                        Eeprom.EepromData.DemoSettings.CntPacketTx++;
                        // Send the next PING frame
                        Buffer[0] = ( Eeprom.EepromData.DemoSettings.CntPacketTx >> 24 ) & 0xFF;
                        Buffer[1] = ( Eeprom.EepromData.DemoSettings.CntPacketTx >> 16 ) & 0xFF;
                        Buffer[2] = ( Eeprom.EepromData.DemoSettings.CntPacketTx >> 8 )  & 0xFF;
                        Buffer[3] = ( Eeprom.EepromData.DemoSettings.CntPacketTx & 0xFF );
                        Buffer[4] = PingMsg[0];
                        Buffer[5] = PingMsg[1];
                        Buffer[6] = PingMsg[2];
                        Buffer[7] = PingMsg[3];
                        for( i = 8; i < Eeprom.EepromData.DemoSettings.PayloadLength; i++ )
                        {
                            Buffer[i] = i;
                        }
                        TX_LED = !TX_LED;
                        IrqMask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;
                        Radio->SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                        Radio->SendPayload( Buffer, Eeprom.EepromData.DemoSettings.PayloadLength, \
                                           Eeprom.EepromData.DemoSettings.InterPacketDelay << 6 );
                    }
                }
                break;

            case APP_TX:
                DemoInternalState = APP_IDLE;
                TX_LED = !TX_LED;
                RX_LED = !RX_LED;
                IrqMask = IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT;
                Radio->SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );

                if( Eeprom.EepromData.DemoSettings.BoostedRx == false )
                {
                    Radio->SetRx( ( ( Eeprom.EepromData.DemoSettings.InterPacketDelay << 1 ) + RX_TIMEOUT_MARGIN ) << 6 );
                }
                else
                {
                    Radio->SetRxBoosted( ( ( Eeprom.EepromData.DemoSettings.InterPacketDelay << 1 ) + RX_TIMEOUT_MARGIN ) << 6 );
                }
                printd("\t\tMASTER: Packet Recived -> OK TX/RX %3d/%2d\n\r", Eeprom.EepromData.DemoSettings.CntPacketTx, Eeprom.EepromData.DemoSettings.CntPacketRxOK);
                printd("\t\tMASTER: Packet Recived -> KO TX/RX %3d/%2d\n\r", Eeprom.EepromData.DemoSettings.CntPacketRxKO, Eeprom.EepromData.DemoSettings.CntPacketRxKOSlave);
                break;

            case APP_RX:
                RX_LED = !RX_LED;
                Radio->GetPayload( Buffer, &BufferSize, BUFFER_SIZE );
                Radio->GetPacketStatus( &PacketStatus );
                if( Eeprom.EepromData.ModulationParams.PacketType == PACKET_TYPE_LORA )
                {
                    Eeprom.EepromData.DemoSettings.RssiValue = PacketStatus.Params.LoRa.RssiPkt;
                    Eeprom.EepromData.DemoSettings.SnrValue = PacketStatus.Params.LoRa.SnrPkt;
                    printd("\t\tMASTER: Packet Recived -> RSSI/SNR %3d/%2d\n\r", PacketStatus.Params.LoRa.RssiPkt, PacketStatus.Params.LoRa.SnrPkt);
                }
                else // Eeprom.EepromData.ModulationParams.PacketType == PACKET_TYPE_GFSK
                {
                    Eeprom.EepromData.DemoSettings.RssiValue = PacketStatus.Params.Gfsk.RssiAvg;
                    Eeprom.EepromData.DemoSettings.SnrValue = 0;
                }

                if( ( BufferSize >= PINGPONG_SIZE ) && ( strncmp( ( const char* )( Buffer + 8 ), ( const char* )PongMsg, PINGPONG_SIZE ) == 0 ) )
                {
                    ComputePingPongPayload( Buffer, BufferSize );
                }
                else
                {
                    Eeprom.EepromData.DemoSettings.CntPacketRxKO++;
                }
                DemoInternalState = SEND_PING_MSG;
                refreshDisplay = 1;
                SendNext = true;
                break;

            case APP_RX_TIMEOUT:
            case APP_RX_ERROR:
                RX_LED = !RX_LED;  
                SendNext = true;
                Eeprom.EepromData.DemoSettings.CntPacketRxKO++;
                DemoInternalState = SEND_PING_MSG;
                refreshDisplay = 1;
                break;

            case APP_TX_TIMEOUT:
                printd( "Failure: timeout in Tx is shorter than the packet time on air\n\r" );
                DemoInternalState = APP_IDLE;
                Eeprom.EepromData.DemoSettings.HoldDemo = true;
                refreshDisplay = 1;
                break;

            case APP_IDLE: // do nothing
                break;

            default:
                break;
        }
    }
    else // SLAVE
    {
        switch( DemoInternalState )
        {
            case SEND_PONG_MSG:
                printd("DemoInternalState -> SEND_PONG_MSG\n\r" );
                wait_ms( RX_TX_TRANSITION_WAIT );

                DemoInternalState = APP_IDLE;
                // Send the next PING frame
                Buffer[0]  = ( Eeprom.EepromData.DemoSettings.CntPacketTx >> 24 ) & 0xFF;
                Buffer[1]  = ( Eeprom.EepromData.DemoSettings.CntPacketTx >> 16 ) & 0xFF;
                Buffer[2]  = ( Eeprom.EepromData.DemoSettings.CntPacketTx >>  8 ) & 0xFF;
                Buffer[3]  = ( Eeprom.EepromData.DemoSettings.CntPacketTx & 0xFF );
                Buffer[4]  = ( ( Eeprom.EepromData.DemoSettings.CntPacketRxKO + \
                                 Eeprom.EepromData.DemoSettings.RxTimeOutCount ) >> 24 ) & 0xFF;
                Buffer[5]  = ( ( Eeprom.EepromData.DemoSettings.CntPacketRxKO + \
                                 Eeprom.EepromData.DemoSettings.RxTimeOutCount ) >> 16 ) & 0xFF;
                Buffer[6]  = ( ( Eeprom.EepromData.DemoSettings.CntPacketRxKO + \
                                 Eeprom.EepromData.DemoSettings.RxTimeOutCount ) >> 8 ) & 0xFF;
                Buffer[7]  = ( ( Eeprom.EepromData.DemoSettings.CntPacketRxKO + \
                                 Eeprom.EepromData.DemoSettings.RxTimeOutCount ) & 0xFF );
                Buffer[8]  = PongMsg[0];
                Buffer[9]  = PongMsg[1];
                Buffer[10] = PongMsg[2];
                Buffer[11] = PongMsg[3];
                for( i = 12; i < Eeprom.EepromData.DemoSettings.PayloadLength; i++ )
                {
                    Buffer[i] = i;
                }
                TX_LED = !TX_LED;
                IrqMask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;
                Radio->SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                Radio->SendPayload( Buffer, Eeprom.EepromData.DemoSettings.PayloadLength, \
                                   ( Eeprom.EepromData.DemoSettings.InterPacketDelay ) << 6 );
                break;

            case APP_TX:
                printd("DemoInternalState -> APP_TX\n\r" );
                if( ( Eeprom.EepromData.DemoSettings.MaxNumPacket != 0 ) \
                    && ( ( Eeprom.EepromData.DemoSettings.CntPacketRxOK + Eeprom.EepromData.DemoSettings.CntPacketRxKO + \
                           Eeprom.EepromData.DemoSettings.RxTimeOutCount ) >= Eeprom.EepromData.DemoSettings.MaxNumPacket ) )
                {
//                    SendNextPacket.detach( ); 
                    printd("SendNext = false\n\r" );
                    SendNext = false;
                    RX_LED = 0;
                    TX_LED = 0;
                    DemoInternalState = APP_IDLE;
                    Radio->SetStandby( STDBY_RC );
                    Eeprom.EepromData.DemoSettings.HoldDemo = true;
                    refreshDisplay = 1;
                }
                else
                {
                    printd("SendNext = true\n\r" );
                    DemoInternalState = APP_IDLE;
                    TX_LED = !TX_LED;
                    RX_LED = !RX_LED;
                    IrqMask = IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT;
                    Radio->SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                    if( Eeprom.EepromData.DemoSettings.BoostedRx == false )
                    {
                        Radio->SetRx( ( ( Eeprom.EepromData.DemoSettings.InterPacketDelay << 1 ) + RX_TIMEOUT_MARGIN ) << 6 );
                    }
                    else
                    {
                        Radio->SetRxBoosted( ( ( Eeprom.EepromData.DemoSettings.InterPacketDelay << 1 ) + RX_TIMEOUT_MARGIN) << 6 );
                    }
                    refreshDisplay = 1;
                }
                break;

            case APP_RX:
                printd("DemoInternalState -> APP_RX\n\r" );
                DemoInternalState = APP_IDLE;
                RX_LED = !RX_LED;
                Radio->GetPayload( Buffer, &BufferSize, BUFFER_SIZE );
                Radio->GetPacketStatus( &PacketStatus );
                if( Eeprom.EepromData.ModulationParams.PacketType == PACKET_TYPE_LORA )
                {
                    Eeprom.EepromData.DemoSettings.RssiValue = PacketStatus.Params.LoRa.RssiPkt;
                    Eeprom.EepromData.DemoSettings.SnrValue = PacketStatus.Params.LoRa.SnrPkt;
                    printd("\t\tSLAVE:  Packet Recived -> RSSI/SNR %3d/%2d\n\r", PacketStatus.Params.LoRa.RssiPkt, PacketStatus.Params.LoRa.SnrPkt);
                }
                else // Eeprom.EepromData.ModulationParams.PacketType == PACKET_TYPE_GFSK
                {
                    Eeprom.EepromData.DemoSettings.RssiValue = PacketStatus.Params.Gfsk.RssiAvg;
                    Eeprom.EepromData.DemoSettings.SnrValue = 0;
                }

                if( ( BufferSize >= PINGPONG_SIZE ) && ( strncmp( ( const char* )( Buffer + 4 ), ( const char* )PingMsg, PINGPONG_SIZE ) == 0 ) )
                {
                    ComputePingPongPayload( Buffer, BufferSize );
                    DemoInternalState = SEND_PONG_MSG;
                }
                else
                {
                    Eeprom.EepromData.DemoSettings.CntPacketRxKO++;
                    RX_LED = !RX_LED;
                    IrqMask = IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT;
                    Radio->SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                    if( Eeprom.EepromData.DemoSettings.BoostedRx == false )
                    {
                        Radio->SetRx( ( Eeprom.EepromData.DemoSettings.InterPacketDelay ) << 6 );
                    }
                    else
                    {
                        Radio->SetRxBoosted( ( Eeprom.EepromData.DemoSettings.InterPacketDelay ) << 6 );
                    }
                    refreshDisplay = 1;
                }
                break;

            case APP_RX_TIMEOUT:
            case APP_RX_ERROR:
                printd("DemoInternalState -> APP_RX_TIMEOUT\n\r" );
                DemoInternalState = APP_IDLE;
                Eeprom.EepromData.DemoSettings.RxTimeOutCount++;
                IrqMask = IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT;
                Radio->SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                if( Eeprom.EepromData.DemoSettings.BoostedRx == false )
                {
                    Radio->SetRx( ( Eeprom.EepromData.DemoSettings.InterPacketDelay ) << 6 );
                }
                else
                {
                    Radio->SetRxBoosted( ( Eeprom.EepromData.DemoSettings.InterPacketDelay ) << 6 );
                }
                refreshDisplay = 1;
                break;

            case APP_TX_TIMEOUT:
                printd("DemoInternalState -> APP_TX_TIMEOUT\n\r" );
                printd( "Failure: timeout in Tx is shorter than the packet time on air\n\r" );
                DemoInternalState = APP_IDLE;
                Eeprom.EepromData.DemoSettings.HoldDemo = true;
                refreshDisplay = 1;
                break;

            case APP_IDLE: // do nothing
                //printd("DemoInternalState -> APP_IDLE\n\r" );
                break;

            default:
                break;
        }
    }
    return refreshDisplay;
}

void ComputePingPongPayload( uint8_t *buffer, uint8_t bufferSize )
{
    uint32_t i = 0;

    PacketRxSequence = ( ( uint32_t )buffer[0] << 24 ) | \
                       ( ( uint32_t )buffer[1] << 16 ) | \
                       ( ( uint32_t )buffer[2] << 8 )  | \
                                     buffer[3];

    if( Eeprom.EepromData.DemoSettings.Entity == MASTER )
    {
        Eeprom.EepromData.DemoSettings.CntPacketRxKOSlave = 
                       ( ( uint32_t )buffer[4] << 24 ) | \
                       ( ( uint32_t )buffer[5] << 16 ) | \
                       ( ( uint32_t )buffer[6] << 8 )  | \
                                     buffer[7];
        if( PacketRxSequence > Eeprom.EepromData.DemoSettings.CntPacketRxKOSlave )
        {
            Eeprom.EepromData.DemoSettings.CntPacketRxOKSlave = PacketRxSequence - \
                Eeprom.EepromData.DemoSettings.CntPacketRxKOSlave;
        }
        else
        {
            Eeprom.EepromData.DemoSettings.CntPacketRxOKSlave = 0;
        }
        
        if( PacketRxSequence == Eeprom.EepromData.DemoSettings.CntPacketTx )
        {
            Eeprom.EepromData.DemoSettings.CntPacketRxOK += 1;
        }
        else
        {
            Eeprom.EepromData.DemoSettings.CntPacketRxKO += 1;
        }
    }
    else
    {
        Eeprom.EepromData.DemoSettings.CntPacketRxOK += 1;
        if( ( PacketRxSequence <= PacketRxSequencePrev ) || \
            ( PacketRxSequencePrev == 0 ) )
        {
            // Sequence went back => resynchronization
            // Don't count missed packets this time
            i = 0;
        }
        else
        {
            // Determine number of missed packets
            i = PacketRxSequence - PacketRxSequencePrev - 1;
        }
        // Be ready for the next
        PacketRxSequencePrev = PacketRxSequence;
        Eeprom.EepromData.DemoSettings.CntPacketTx = PacketRxSequence;
        // increment 'missed' counter for the RX session
        Eeprom.EepromData.DemoSettings.CntPacketRxKO += i;
        Eeprom.EepromData.DemoSettings.RxTimeOutCount = 0;
    }
}

// ************************        Utils            ****************************
// *                                                                           *
// *                                                                           *
// *                                                                           *
// *****************************************************************************

uint8_t GetConnectedDevice( void )
{
    return( SX1262 );
}

uint8_t GetMatchingFrequency( void )
{
    return( FS915 );
}
const SX126xHalInterface_t RadioHal = {

    .hspiSX         =   &SPI_SX1262_Handle,
    .RadioNss       =   {.port =  GPIOB, .pin  = GPIO_PIN_12 },
    .RadioReset     =   {.port =  GPIOA, .pin  = GPIO_PIN_3 },
    .BUSY           =   {.port =  GPIOA, .pin  = GPIO_PIN_9 },
    .DIO1           =   {.port =  GPIOA, .pin  = GPIO_PIN_8,  .IRQn = EXTI9_5_IRQn},
    .DIO2           =   {.port =  NULL,  .pin  = NULL },
    .DIO3           =   {.port =  NULL,  .pin  = NULL },
    .MISO           =   {.port =  GPIOB, .pin  = GPIO_PIN_14 },
    .MOSI           =   {.port =  GPIOB, .pin  = GPIO_PIN_15 },
    .SCK            =   {.port =  GPIOB, .pin  = GPIO_PIN_13 },
    .antSwitchPower =   {.port =  GPIOA, .pin  = GPIO_PIN_1 }
};
void InitDemoApplication( void )
{
    RX_LED = 1;
    TX_LED = 1;
    
    Eeprom.EepromData.DemoSettings.RadioPowerMode =  USE_DCDC;
    Radio = SX126xHalInit(&RadioHal, &RadioEvents);

    // Can also be set in LDO mode but consume more power
    Radio->SetRegulatorMode( ( RadioRegulatorMode_t )Eeprom.EepromData.DemoSettings.RadioPowerMode);
    Radio->SetStandby( STDBY_RC );

    memset( &Buffer, 0x00, BufferSize );
    
    RX_LED = 0;
    TX_LED = 0;

    PacketRxSequence = 0;
    PacketRxSequencePrev = 0;
    Eeprom.EepromData.DemoSettings.CntPacketTx    = 0;
    Eeprom.EepromData.DemoSettings.CntPacketRxOK  = 0;
    Eeprom.EepromData.DemoSettings.CntPacketRxKO  = 0;
    Eeprom.EepromData.DemoSettings.RxTimeOutCount = 0;
    
    Eeprom.EepromData.DemoSettings.Entity = SLAVE;
    
    Eeprom.EepromData.DemoSettings.ModulationType = PACKET_TYPE_LORA;
    Eeprom.EepromData.ModulationParams.PacketType = PACKET_TYPE_LORA;
    Eeprom.EepromData.PacketParams.PacketType     = PACKET_TYPE_LORA;
    
    Eeprom.EepromData.DemoSettings.ModulationParam1 = LORA_SF10;
    Eeprom.EepromData.DemoSettings.ModulationParam2 = LORA_BW_500;
    Eeprom.EepromData.DemoSettings.ModulationParam3 = LORA_CR_4_5;
    Eeprom.EepromData.DemoSettings.ModulationParam4 = 0x00;

    Eeprom.EepromData.DemoSettings.PacketParam1 = 8;
    Eeprom.EepromData.DemoSettings.PacketParam2 = LORA_PACKET_VARIABLE_LENGTH;
    Eeprom.EepromData.DemoSettings.PacketParam3 = 16;
    Eeprom.EepromData.DemoSettings.PacketParam4 = LORA_CRC_ON;
    Eeprom.EepromData.DemoSettings.PacketParam5 = LORA_IQ_NORMAL;
    
    
    Eeprom.EepromData.DemoSettings.BoostedRx      = true;

    Eeprom.EepromData.DemoSettings.LastDeviceConnected = SX1262;
    Eeprom.EepromData.DemoSettings.RadioPowerMode = USE_LDO;
    Eeprom.EepromData.DemoSettings.TxPower        = SX1262_POWER_TX_MAX;
    
    
    Eeprom.EepromData.DemoSettings.Frequency      = DEMO_CENTRAL_FREQ_PRESET2; // 915 MHz
    
    Eeprom.EepromData.DemoSettings.MaxNumPacket   = 0x00;
    Eeprom.EepromData.DemoSettings.ModulationType = PACKET_TYPE_LORA;

}

void StopDemoApplication( void )
{
    if( DemoRunning == true )
    {
        Radio->CheckDeviceReady( );

        RX_LED = 0;
        TX_LED = 0;
        DemoRunning = false;
        SendNext = false;
        PacketRxSequence = 0;
        PacketRxSequencePrev = 0;
        Eeprom.EepromData.DemoSettings.CntPacketTx    = 0;
        Eeprom.EepromData.DemoSettings.CntPacketRxOK  = 0;
        Eeprom.EepromData.DemoSettings.CntPacketRxKO  = 0;
        Eeprom.EepromData.DemoSettings.RxTimeOutCount = 0;
        
        DemoInternalState = APP_IDLE;
        Radio->SetStandby( STDBY_RC );
        Radio->ClearIrqStatus( IRQ_RADIO_ALL );
//        SendNextPacket.detach( ); 
    }
}

/*
 * Function still being implemented >>> To be completed 
 * WARNING: Computation is in float and his really slow
 * LongInterLeaving vs LegacyInterLeaving has no influence on TimeOnAir.
 */
uint16_t GetTimeOnAir( uint8_t modulation )
{
    uint16_t result = 2000;
    uint8_t LowDatarateOptimize = 0;

    if( modulation == PACKET_TYPE_LORA )
    {
        volatile double loraBw = 0.0;
        volatile double FreqErrorUnits = 0.0;
        
        switch( Eeprom.EepromData.ModulationParams.Params.LoRa.Bandwidth )
        {
            case LORA_BW_500:
                loraBw = 500e3;
                break;

            case LORA_BW_250:
                loraBw = 250e3;
                break;

            case LORA_BW_125:
                loraBw = 125e3;
                break;

            case LORA_BW_062:
                loraBw = 62e3;
                break;

            case LORA_BW_041:
                loraBw = 41e3;
                break;

            case LORA_BW_031:
                loraBw = 31e3;
                break;

            case LORA_BW_020:
                loraBw = 20e3;
                break;

            case LORA_BW_015:
                loraBw = 15e3;
                break;

            case LORA_BW_010:
                loraBw = 10e3;
                break;

            case LORA_BW_007:
                loraBw = 7e3;
                break;

            default:
                loraBw = 7e3;
                break;
        }

        /* Used to compute the freq Error */
        FreqErrorUnits = FREQ_ERR;
        FreErrorLsb = FreqErrorUnits * ( ( double )loraBw / 1000 ) / 500;

        float ts = 1 << Eeprom.EepromData.ModulationParams.Params.LoRa.SpreadingFactor; // time for one symbol in ms
              ts = (float)ts / (float)loraBw;
              ts = ts * 1000; // from seconds to miliseconds

        float tPreamble = ( Eeprom.EepromData.PacketParams.Params.LoRa.PreambleLength + 4.25 + ( ( Eeprom.EepromData.ModulationParams.Params.LoRa.SpreadingFactor > 6 ) ? 2 : 0 )) * ts; // time of preamble

        switch( Eeprom.EepromData.ModulationParams.Params.LoRa.Bandwidth )
        {
            case LORA_BW_500:
                break;

            case LORA_BW_250:
                if( Eeprom.EepromData.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF12 )
                {
                    LowDatarateOptimize = 1;
                }
                break;

            case LORA_BW_125:
                if( Eeprom.EepromData.ModulationParams.Params.LoRa.SpreadingFactor >= LORA_SF11 )
                {
                    LowDatarateOptimize = 1;
                }
                break;

            case LORA_BW_062:
                if( Eeprom.EepromData.ModulationParams.Params.LoRa.SpreadingFactor >= LORA_SF10 )
                {
                    LowDatarateOptimize = 1;
                }
                break;

            case LORA_BW_041:
            case LORA_BW_031:
                if( Eeprom.EepromData.ModulationParams.Params.LoRa.SpreadingFactor >= LORA_SF9 )
                {
                    LowDatarateOptimize = 1;
                }
                break;

            case LORA_BW_020:
            case LORA_BW_015:
                if( Eeprom.EepromData.ModulationParams.Params.LoRa.SpreadingFactor >= LORA_SF8 )
                {
                    LowDatarateOptimize = 1;
                }
                break;

            case LORA_BW_010:
            case LORA_BW_007:
                if( Eeprom.EepromData.ModulationParams.Params.LoRa.SpreadingFactor >= LORA_SF7 )
                {
                    LowDatarateOptimize = 1;
                }
                break;
        }

        float nData = ceil( ( float )( ( 8 * Eeprom.EepromData.PacketParams.Params.LoRa.PayloadLength +                                           \
                       16 * ( ( Eeprom.EepromData.PacketParams.Params.LoRa.CrcMode == LORA_CRC_OFF ) ? 0 : 1 ) +                \
                       ( ( Eeprom.EepromData.PacketParams.Params.LoRa.HeaderType == LORA_PACKET_VARIABLE_LENGTH ) ? 20 : 0 ) -  \
                       ( 4 * Eeprom.EepromData.ModulationParams.Params.LoRa.SpreadingFactor ) + 8 -                             \
                       ( 8 *( ( Eeprom.EepromData.ModulationParams.Params.LoRa.SpreadingFactor > 6 ) ? 1 : 0 ) ) ) / 4 ) );

               nData = ceil( ( float )nData / ( ( float )( Eeprom.EepromData.ModulationParams.Params.LoRa.SpreadingFactor - \
                              ( LowDatarateOptimize * 2 ) ) ) * ( ( Eeprom.EepromData.ModulationParams.Params.LoRa.CodingRate % 4 ) + 4 ) );

        float tPayload = nData * ts;

        float tHeader = 8 * ts;
        // Time on air [ms]
        float ToA = ceil( tPreamble + tPayload + tHeader );

        result = ( uint16_t )ToA + ( ( uint16_t )ToA >> 1 );   // Set some margin
    }
    else if( modulation == PACKET_TYPE_GFSK )
    {
        uint16_t packetBitCount = Eeprom.EepromData.PacketParams.Params.Gfsk.PreambleLength;

        packetBitCount += ( Eeprom.EepromData.PacketParams.Params.Gfsk.SyncWordLength + 1 );
        packetBitCount += Eeprom.EepromData.PacketParams.Params.Gfsk.PayloadLength + 3;
        packetBitCount *= 8;
        // 1500 = 1000 * 1.5 : 1000 for translate s in ms and 1.5 is some margin
        result = ( uint16_t )( ceil( 1500 * ( float )packetBitCount / Eeprom.EepromData.ModulationParams.Params.Gfsk.BitRate ) );
    }
    return result;
}

void InitializeDemoParameters( uint8_t modulation )
{
    Radio->SetStandby( STDBY_RC );

    Radio->SetRegulatorMode( ( RadioRegulatorMode_t )Eeprom.EepromData.DemoSettings.RadioPowerMode );

    printd("> InitializeDemoParameters\n\r");
    if( modulation == PACKET_TYPE_LORA )
    {
        printd("set param LORA for demo\n\r");
        ModulationParams.PacketType = PACKET_TYPE_LORA;
        PacketParams.PacketType     = PACKET_TYPE_LORA;

        ModulationParams.Params.LoRa.SpreadingFactor = ( RadioLoRaSpreadingFactors_t ) Eeprom.EepromData.DemoSettings.ModulationParam1;
        ModulationParams.Params.LoRa.Bandwidth       = ( RadioLoRaBandwidths_t )       Eeprom.EepromData.DemoSettings.ModulationParam2;
        ModulationParams.Params.LoRa.CodingRate      = ( RadioLoRaCodingRates_t )      Eeprom.EepromData.DemoSettings.ModulationParam3;

        PacketParams.Params.LoRa.PreambleLength      =                                 Eeprom.EepromData.DemoSettings.PacketParam1;
        PacketParams.Params.LoRa.HeaderType          = ( RadioLoRaPacketLengthsMode_t )Eeprom.EepromData.DemoSettings.PacketParam2;
        PacketParams.Params.LoRa.PayloadLength       =                                 Eeprom.EepromData.DemoSettings.PacketParam3;
        PacketParams.Params.LoRa.CrcMode             = ( RadioLoRaCrcModes_t )         Eeprom.EepromData.DemoSettings.PacketParam4;
        PacketParams.Params.LoRa.InvertIQ            = ( RadioLoRaIQModes_t )          Eeprom.EepromData.DemoSettings.PacketParam5;

        Eeprom.EepromData.DemoSettings.PayloadLength = PacketParams.Params.LoRa.PayloadLength;
        
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
        ModulationParams.PacketType = PACKET_TYPE_GFSK;
        PacketParams.PacketType     = PACKET_TYPE_GFSK;

        ModulationParams.Params.Gfsk.BitRate           =                             Eeprom.EepromData.DemoSettings.ModulationParam1;
        ModulationParams.Params.Gfsk.Fdev              =                             Eeprom.EepromData.DemoSettings.ModulationParam2;
        ModulationParams.Params.Gfsk.ModulationShaping = ( RadioModShapings_t )      Eeprom.EepromData.DemoSettings.ModulationParam3;
        ModulationParams.Params.Gfsk.Bandwidth         = ( RadioRxBandwidth_t )      Eeprom.EepromData.DemoSettings.ModulationParam4;
        PacketParams.Params.Gfsk.PreambleLength        =                             Eeprom.EepromData.DemoSettings.PacketParam1;
        PacketParams.Params.Gfsk.PreambleMinDetect     = ( RadioPreambleDetection_t )Eeprom.EepromData.DemoSettings.PacketParam2;
        PacketParams.Params.Gfsk.SyncWordLength        =                             Eeprom.EepromData.DemoSettings.PacketParam3;
        PacketParams.Params.Gfsk.AddrComp              = ( RadioAddressComp_t )      Eeprom.EepromData.DemoSettings.PacketParam4;
        PacketParams.Params.Gfsk.HeaderType            = ( RadioPacketLengthModes_t )Eeprom.EepromData.DemoSettings.PacketParam5;
        PacketParams.Params.Gfsk.PayloadLength         =                             Eeprom.EepromData.DemoSettings.PacketParam6;

        PacketParams.Params.Gfsk.CrcLength             = ( RadioCrcTypes_t )         Eeprom.EepromData.DemoSettings.PacketParam7;
        PacketParams.Params.Gfsk.DcFree                = ( RadioDcFree_t )           Eeprom.EepromData.DemoSettings.PacketParam8;

        Eeprom.EepromData.DemoSettings.PayloadLength = PacketParams.Params.Gfsk.PayloadLength;
    }

    Radio->SetStandby( STDBY_RC );
    Radio->ClearIrqStatus( IRQ_RADIO_ALL );
    Radio->SetPacketType( ModulationParams.PacketType );
    Radio->SetModulationParams( &ModulationParams );
    Radio->SetPacketParams( &PacketParams );

    Radio->SetRfFrequency( Eeprom.EepromData.DemoSettings.Frequency );
    Radio->SetBufferBaseAddresses( 0x00, 0x00 );
    
    
    Radio->SetTxParams( Eeprom.EepromData.DemoSettings.TxPower, RADIO_RAMP_200_US );

    // only used in GFSK
    Radio->SetSyncWord( ( uint8_t[] ){ 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0 } );
    Radio->SetWhiteningSeed( 0x01FF );

    RX_LED = 0;
    TX_LED = 0;
}

/*!
 * \brief Callback of ticker PerSendNextPacket
 */
void SendNextPacketEvent( void )
{
    SendNext = true;
}

void LedBlink( void )
{
    if( ( TX_LED == 0 ) && ( RX_LED == 0 ) )
    {
        TX_LED = 1;
    }
    else if( ( TX_LED == 1 ) && ( RX_LED == 0 ) )
    {
        RX_LED = 1;
    }
    else if( ( TX_LED == 1 ) && ( RX_LED == 1 ) )
    {
        TX_LED = 0;
    }
    else
    {
        RX_LED = 0;
    }
}

// ************************     Radio Callbacks     ****************************
// *                                                                           *
// * These functions are called through function pointer by the Radio low      *
// * level drivers                                                             *
// *                                                                           *
// *****************************************************************************
void OnTxDone( void )
{
    DemoInternalState = APP_TX;
}

void OnRxDone( void )
{
    DemoInternalState = APP_RX;
}

void OnTxTimeout( void )
{
    DemoInternalState = APP_TX_TIMEOUT;
}

void OnRxTimeout( void )
{
    DemoInternalState = APP_RX_TIMEOUT;
}

void OnRxError( IrqErrorCode_t errorCode )
{
    DemoInternalState = APP_RX_ERROR;
    
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
    Radio->GetPacketStatus( &PacketStatus );
}

void OnCadDone( bool channelActivityDetected )
{
    if( channelActivityDetected == true )
    {
        DemoInternalState = CAD_DONE_CHANNEL_DETECTED;
    }
    else
    {
        DemoInternalState = CAD_DONE;
    }
}
