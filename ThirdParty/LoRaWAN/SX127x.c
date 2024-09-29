/**
 * @file   	SX127x.c
 * @author 	Hassan
 * @version	
 * @date	Mar 17, 2016
 * 
 * @brief   
 */


#include <SX127x.h>



/**
 * @defgroup SX127x_private_Structs			SX127x Private Structures
 * @{
 */

/*!
 * FSK bandwidth definition
 */
typedef struct
{
    uint32_t bandwidth;
    uint8_t  RegValue;
}FskBandwidth_t;


/** @}*/


/**
 * @defgroup SX127x_private_defines		SX127x Private Defines
 * @{
 */
#define XTAL_FREQ       ((uint32_t)32000000)
#define FREQ_STEP 		((double)61.03515625)		/*!< Frequency step for converting frf register in
 	 	 	 	 	 	 	 	 	 	 	 	 	 	SX127x to actual frequency*/

/** @}*/



/**
 *  @defgroup SX127x_exported_members	SX127x Exported Members
 * @{
 */

SX127xHandle_t SX127x_handle = /*!< handle for SX127x status and configuration*/
{
    .LoRa = {
        .bandwidth      = RFLR_MODEMCONFIG1_BW_250_KHZ,
        .spreadFactor   = RFLR_MODEMCONFIG2_SF_11,
        .irqFlags       = 0},
    
    .nIRQ               = 0,
    .frequencyChannel   = RF_FREQUENCY,
	.rxLength			= 0,
	.rxBuffer			= {0},
};		
/** @}*/


/**
 * @defgroup SX127x_private_members			SX127x Private Members
 * @{
 */

/*!
 * Precomputed FSK bandwidth registers values
 */
const FskBandwidth_t FskBandwidths[] =
{
    { 2600  , 0x17 },
    { 3100  , 0x0F },
    { 3900  , 0x07 },
    { 5200  , 0x16 },
    { 6300  , 0x0E },
    { 7800  , 0x06 },
    { 10400 , 0x15 },
    { 12500 , 0x0D },
    { 15600 , 0x05 },
    { 20800 , 0x14 },
    { 25000 , 0x0C },
    { 31300 , 0x04 },
    { 41700 , 0x13 },
    { 50000 , 0x0B },
    { 62500 , 0x03 },
    { 83333 , 0x12 },
    { 100000, 0x0A },
    { 125000, 0x02 },
    { 166700, 0x11 },
    { 200000, 0x09 },
    { 250000, 0x01 },
    { 300000, 0x00 }, // Invalid Badwidth
};


/** @}*/


/**
 * @defgroup SX127x_private_functions	SX127x Private Functions
 * @{
 */

/**
 * @brief 		Set the SX127x in the specified operating mode
 * @param mode	Operating mode as defined in #SX127xMode_t
 * @return		SX127x_Return
 */

__STATIC_INLINE SX127xReturn_t SX127x_setOperationMode(SX127xMode_t mode)
{
	return SX127x_modifyRegister(REG_OPMODE, RF_OPMODE_MASK, mode);
}


/**
 * @brief 		Change the modem configuration of SX127x and set the DIO mapping accordingly
 * @param modem	Modem mode as defined by #SX127xModem_t
 * @return		SX127x_Return
 */
SX127xReturn_t SX127x_setModem(SX127xModem_t modem)
{
	SX127xReturn_t retStatus;

	SX127x_handle.modem = modem;

	//SX127x must be in sleep mode to modify OPMODE
    switch( modem )
    {
    case SX127x_MODEM_FSK:

        retStatus = SX127x_setOperationMode( SX127x_SLEEP );
        SX127x_ASSERT_RETURN(retStatus, SX127x_OK);


        retStatus = SX127x_modifyRegister(REG_OPMODE, RF_OPMODE_LONGRANGEMODE_MASK, RF_OPMODE_LONGRANGEMODE_OFF);
        SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

		///@todo Add DIO configuration
		retStatus = SX127x_writeRegister( REG_DIOMAPPING1, 0x00 );
        SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

        retStatus = SX127x_writeRegister( REG_DIOMAPPING2, 0x30 ); // DIO5=ModeReady


        break;
    case SX127x_MODEM_LORA:
    	retStatus = SX127x_setOperationMode( SX127x_SLEEP );
    	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

    	retStatus = SX127x_modifyRegister( REG_OPMODE, RFLR_OPMODE_LONGRANGEMODE_MASK, RFLR_OPMODE_LONGRANGEMODE_ON );
        SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

        //See SX127x datasheet
        retStatus = SX127x_writeRegister( REG_LR_DIOMAPPING1, 0x00 );
        SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

        retStatus = SX127x_writeRegister( REG_LR_DIOMAPPING2, 0x00 );

        break;
    default:
    	retStatus = SX127x_ERROR;
    }

    return retStatus;
}


/**
 * @brief			Set the SX127x to use the specified frequency channel
 * @param frequency	Desired frequency channel
 * @return			SX127x_Return
 *
 * @note 	The niceRF LoRa modules can only be configured for frequency
 * 			in the range of 838 to 898 MHz. Typical is 868 MHz
 */
SX127xReturn_t SX127x_setChannel(uint32_t frequency)
{
	SX127xReturn_t retStatus;

	frequency = ( uint32_t )( ( double )frequency / ( double )FREQ_STEP );

	retStatus = SX127x_writeRegister(REG_FRFMSB, ( uint8_t )( ( frequency >> 16 ) & 0xFF ));
    SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

    retStatus = SX127x_writeRegister( REG_FRFMID, ( uint8_t )( ( frequency >> 8 ) & 0xFF ) );
    SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_writeRegister( REG_FRFLSB, ( uint8_t )( frequency & 0xFF ) );

    return retStatus;
}

/**
 * @brief	Callibrate the receive IQ channels.
 * @return  SX127x_Return
 *
 * @note	For this function, the modem must be in FSK mode and chip in Standby mode
 *
 * @note 	This function should be called every time the frequency channel is changed.
 */
SX127xReturn_t SX127x_callibrateRxChain(void)
{
	uint8_t regPaConfigInitVal;		//backup for current value of PaConfig register
	uint8_t regTemp;				//temporary variable to read registers
	uint8_t regFrfInitVal[3];		//backup for current frequency configuration
	SX127xReturn_t	retStatus;		//return value

	retStatus = SX127x_readRegister(REG_PACONFIG, &regPaConfigInitVal);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_readBurst(REG_FRFMSB, regFrfInitVal, 3);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	//cut the PA, RFO output, power = -1 dBm
	retStatus = SX127x_writeRegister(REG_PACONFIG, 0x00);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	//start callibration in LF
	retStatus = SX127x_modifyRegister(REG_IMAGECAL, RF_IMAGECAL_IMAGECAL_MASK, RF_IMAGECAL_IMAGECAL_START);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	//wait until callibration done
	do{

		retStatus = SX127x_readRegister(REG_IMAGECAL, &regTemp);
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	}while ((regTemp & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING);

	//change frequency to desired channel
	retStatus = SX127x_setChannel( SX127x_handle.frequencyChannel);

	//callibrate again
	retStatus = SX127x_modifyRegister(REG_IMAGECAL, RF_IMAGECAL_IMAGECAL_MASK, RF_IMAGECAL_IMAGECAL_START);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	//wait until callibration done
	do{

		retStatus = SX127x_readRegister(REG_IMAGECAL, &regTemp);
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	}while ((regTemp & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING);

	//restore register paConfig and frf
	retStatus = SX127x_writeRegister(REG_PACONFIG, regPaConfigInitVal);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_writeBurst(REG_FRFMSB, regFrfInitVal, 3);

	return retStatus;
}

/**
 * @brief 	Read the interrupt flags from the SX127x and reset them in the chip
 * @return	SX127x_Return
 *
 * @warning This function does not reset any pending IRQ flags in software shadow register.
 * 			They must be reset manually in application.
 *
 */
SX127xReturn_t SX127x_readInterruptFlags(void)
{

	SX127xReturn_t retStatus;
	uint8_t regIRQFlags;


	switch( SX127x_handle.modem)
	{

	case SX127x_MODEM_FSK:

		retStatus = SX127x_readRegister(REG_IRQFLAGS1, &regIRQFlags);
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);
		SX127x_handle.FSK.IrqReg.RegIrqFlags1 = regIRQFlags;


		retStatus = SX127x_readRegister(REG_IRQFLAGS2, &regIRQFlags);
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);
		SX127x_handle.FSK.IrqReg.RegIrqFlags2 = regIRQFlags;

		//writing a 1 to the flag clears it.
        //Only the RSSI, peamble detect and sync address match interrupts allow write operation
		retStatus = SX127x_writeRegister(REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI | 
                                                        RF_IRQFLAGS1_PREAMBLEDETECT |
                                                        RF_IRQFLAGS1_SYNCADDRESSMATCH);
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

        //only the FIFO overrun, and low battery interrupt allow write operation.
        //The FIFO overrun flag is not set, because when set, the FIFO is cleared.
		retStatus = SX127x_writeRegister(REG_IRQFLAGS2, RF_IRQFLAGS2_LOWBAT);
		break;

	case SX127x_MODEM_LORA:

		retStatus = SX127x_readRegister(REG_LR_IRQFLAGS, &regIRQFlags);
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

		//update the shadow register
		SX127x_handle.LoRa.irqFlags = regIRQFlags;

		//writing a 1 to the flag clears it.
		retStatus = SX127x_writeRegister(REG_LR_IRQFLAGS, 0xFF);

		break;
	}
	return retStatus;
}

/**
 *
 * @param fdev
 * @return
 */
__STATIC_INLINE SX127xReturn_t SX127x_FSK_setFreqDeviation(uint32_t fdev)
{
	SX127xReturn_t retStatus = SX127x_ERROR;

	fdev = ( uint16_t )( ( double )fdev / ( double )FREQ_STEP );

	retStatus = SX127x_writeRegister( REG_FDEVMSB, ( uint8_t )( fdev >> 8 ) );
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

    retStatus = SX127x_writeRegister( REG_FDEVLSB, ( uint8_t )( fdev & 0xFF ) );

    return retStatus;
}

/**
 *
 * @param bitRate
 * @return
 */
__STATIC_INLINE SX127xReturn_t SX127x_FSK_setBitRate(uint32_t bitRate)
{
	SX127xReturn_t retStatus = SX127x_ERROR;

	bitRate = ( uint16_t )( ( double )XTAL_FREQ / ( double )bitRate );

	retStatus = SX127x_writeRegister( REG_BITRATEMSB, ( uint8_t )( bitRate >> 8 ) );
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_writeRegister( REG_BITRATELSB, ( uint8_t )( bitRate & 0xFF ) );

	return retStatus;
}


/**
 *
 * @param preambleLength
 * @return
 */
__STATIC_INLINE SX127xReturn_t SX127x_FSK_setPreamble(uint16_t preambleLength)
{
	SX127xReturn_t retStatus = SX127x_ERROR;

	retStatus = SX127x_writeRegister( REG_PREAMBLEMSB, ( preambleLength >> 8 ) & 0x00FF);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_writeRegister( REG_PREAMBLELSB, preambleLength & 0xFF );

	return retStatus;
}





/**
 * Returns the known FSK bandwidth registers value
 *
 * @param [IN] bandwidth Bandwidth value in Hz
 * @retval regValue Bandwidth register value.
 */
static SX127xReturn_t SX127x_FSK_setBandwidth( uint32_t bandwidth, uint32_t  bandwidthAfc)
{
    uint8_t i;
    uint8_t regValue = 0xFF;
    SX127xReturn_t retStatus = SX127x_ERROR;

    for( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) ) - 1; i++ )
    {
        if( ( bandwidth >= FskBandwidths[i].bandwidth ) && ( bandwidth < FskBandwidths[i + 1].bandwidth ) )
        {
            regValue =  FskBandwidths[i].RegValue;
            break;
        }
    }

    if (regValue == 0xFF)
    	return SX127x_ERROR;

    retStatus = SX127x_writeRegister(REG_RXBW, regValue);
    SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

    for( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) ) - 1; i++ )
    {
        if( ( bandwidthAfc >= FskBandwidths[i].bandwidth ) && ( bandwidthAfc < FskBandwidths[i + 1].bandwidth ) )
        {
            regValue =  FskBandwidths[i].RegValue;
            break;
        }
    }

    if (regValue == 0xFF)
        return SX127x_ERROR;

     return SX127x_writeRegister(REG_AFCBW, regValue);
}


/**
 *
 * @param power
 * @return
 *
 * @Warning 	Does not work with RFO : probably the RFO pin is not connected in the module.
 *
 */
SX127xReturn_t SX127x_setTxPower(int8_t power, SX127x_PASelect_t paSelect)
{

	SX127xReturn_t retStatus = SX127x_ERROR;
	uint8_t paConfig = 0;
	uint8_t paDac = 0;


	retStatus = SX127x_readRegister( REG_PACONFIG , &paConfig);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_readRegister( REG_PADAC , &paDac);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);


	paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK )  | paSelect;

	if (paSelect == SX127x_PASELECT_PABOOST)
	{
		if( power > 17 )
		{
			paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
		}
		else
		{
			paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
		}

		if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
		{
			if( power < 5 )
			{
				power = 5;
			}
			if( power > 20 )
			{
				power = 20;
			}
			paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
		}
		else
		{
			if( power < 2 )
			{
				power = 2;
			}
			if( power > 17 )
			{
				power = 17;
			}
			paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
		}
	}
	else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }

	///@todo add tx power configuration for SX1276 chip
//    #if defined(SX1276)
//    paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x7F;			//set max gain
//    #endif


	retStatus = SX127x_writeRegister( REG_PACONFIG, paConfig );
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_writeRegister( REG_PADAC, paDac );
	return retStatus;
}

/**
 * @brief	Configure the modem registers for use in LoRa mode
 * @return	SX127x_Return
 */
SX127xReturn_t SX127x_config(SX127xModem_t modem)
{

	SX127xReturn_t retStatus;

	uint16_t preambleLength = FSK_PREAMBLE_LENGTH;  ///@todo set preamble length
	uint32_t fdev = FSK_FDEV;			            ///@todo set frequency deviation
	uint32_t bitRate = FSK_DATARATE;	            ///@todo set bit rate


	retStatus = SX127x_setModem( modem );
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_setTxPower(TX_OUTPUT_POWER, SX127x_PASELECT_PABOOST);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

    #if defined(SX1276)
	retStatus = SX127x_modifyRegister( REG_LR_OPMODE, RFLR_OPMODE_FREQMODE_ACCESS_MASK ,RFLR_OPMODE_FREQMODE_ACCESS_HF );
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);
    #endif
    

	retStatus = SX127x_setChannel( SX127x_handle.frequencyChannel );
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);


	switch (SX127x_handle.modem)
	{

	case SX127x_MODEM_FSK:

		#if defined(SX1276)
			///@todo Implement configuration for SX1276 chip for FSK modem
			return SX127x_ERROR;

		#elif defined(SX1272)

		retStatus = SX127x_FSK_setFreqDeviation(fdev);
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

		retStatus = SX127x_FSK_setBitRate(bitRate);
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

		retStatus = SX127x_FSK_setBandwidth(FSK_BANDWIDTH, FSK_AFC_BANDWIDTH);

		retStatus = SX127x_FSK_setPreamble(preambleLength);
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);



		retStatus = SX127x_modifyRegister(REG_PACKETCONFIG1, RF_PACKETCONFIG1_CRC_MASK &
                                                             RF_PACKETCONFIG1_CRCAUTOCLEAR_MASK &
															 RF_PACKETCONFIG1_PACKETFORMAT_MASK,
															 	 	 	 	 	 	 	 RF_PACKETCONFIG1_CRC_ON |
                                                                                         RF_PACKETCONFIG1_CRCAUTOCLEAR_OFF |
															 	 	 	 	 	 	 	 RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE);
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);
		
        retStatus = SX127x_modifyRegister(REG_PACKETCONFIG2, RF_PACKETCONFIG2_DATAMODE_MASK, RF_PACKETCONFIG2_DATAMODE_PACKET);
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);
        
//        retStatus = SX127x_modifyRegister(REG_SYNCCONFIG, RF_SYNCCONFIG_AUTORESTARTRXMODE_MASK & 
//                                                          RF_SYNCCONFIG_SYNC_MASK,
//                                                                                        RF_SYNCCONFIG_AUTORESTARTRXMODE_WAITPLL_OFF |
//                                                                                        RF_SYNCCONFIG_SYNC_OFF);
                                                                                        
        
		#endif

		break;
	case SX127x_MODEM_LORA:

		#if defined(SX1276)
		retStatus = SX127x_modifyRegister( 	REG_LR_MODEMCONFIG1,
										   (RFLR_MODEMCONFIG1_BW_MASK &
											RFLR_MODEMCONFIG1_CODINGRATE_MASK &
											RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK),
										   (RFLR_MODEMCONFIG1_BW_250_KHZ  |
											RFLR_MODEMCONFIG1_CODINGRATE_4_5  |
											RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF) );
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

		retStatus = SX127x_modifyRegister( 	REG_LR_MODEMCONFIG2,
										   (RFLR_MODEMCONFIG2_SF_MASK &
											RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK),
										   (RFLR_MODEMCONFIG2_SF_12  |
											RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON) );
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);


		retStatus = SX127x_modifyRegister(	REG_LR_MODEMCONFIG3,
										   (RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK &
											RFLR_MODEMCONFIG3_AGCAUTO_MASK),
										   (RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON |
											RFLR_MODEMCONFIG3_AGCAUTO_ON) );
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

		#elif defined(SX1272)
		retStatus = SX127x_modifyRegister( 	REG_LR_MODEMCONFIG1,
										   (RFLR_MODEMCONFIG1_BW_MASK &
											RFLR_MODEMCONFIG1_CODINGRATE_MASK &
											RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK &
											RFLR_MODEMCONFIG1_RXPAYLOADCRC_MASK &
											RFLR_MODEMCONFIG1_LOWDATARATEOPTIMIZE_MASK),
										   (RFLR_MODEMCONFIG1_BW_250_KHZ  |
											RFLR_MODEMCONFIG1_CODINGRATE_4_5  |
											RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF |
											RFLR_MODEMCONFIG1_RXPAYLOADCRC_ON |
											RFLR_MODEMCONFIG1_LOWDATARATEOPTIMIZE_ON) );
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

		retStatus = SX127x_modifyRegister( 	REG_LR_MODEMCONFIG2,
										   (RFLR_MODEMCONFIG2_SF_MASK),
										   (RFLR_MODEMCONFIG2_SF_12 ) );
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);
		#endif




		retStatus = SX127x_writeRegister( 	REG_LR_PREAMBLEMSB, RFLR_PREAMBLELENGTHMSB);
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

		retStatus = SX127x_writeRegister( 	REG_LR_PREAMBLELSB, RFLR_PREAMBLELENGTHLSB );
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);


		retStatus = SX127x_modifyRegister( 	REG_LR_DETECTOPTIMIZE,
										   (RFLR_DETECTIONOPTIMIZE_MASK),
										   (RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12) );
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

		retStatus = SX127x_writeRegister( 	REG_LR_DETECTIONTHRESHOLD,
											RFLR_DETECTIONTHRESH_SF7_TO_SF12 );

		break;

	}

	return retStatus;
}

/**
 * @brief Reset the SX127x chip using the nReset GPIO
 */
SX127xReturn_t SX127x_reset(void)
{
	//Disable TX and RX switches through GPIO.
	SX127x_RESET_TXRX();

	//Reset cycle. Pull reset pin low for 10 milliseconds.
    #if defined(SX1276)
	SX127x_RESET_NRST();
	mDelay(10);
	SX127x_SET_NRST();
    #elif defined(SX1272)
    SX127x_SET_NRST();
	mDelay(10);
	SX127x_RESET_NRST();
	#endif
    
	//allow time for chip to wakeup from reset.
	mDelay(20);

	return SX127x_OK;
}


/**
 * @brief 	Set the SX127x in the transmitter mode.
 * @return	SX127x_Return
 */
SX127xReturn_t SX127x_transmitterMode(void)
{
	SX127xReturn_t retStatus;

	retStatus = SX127x_setOperationMode( SX127x_STANDBY );
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);



	switch (SX127x_handle.modem)
	{

        case SX127x_MODEM_FSK:
        // DIO0=PacketSent
        // DIO1=FifoEmpty
        // DIO2=FifoFull
        // DIO3=FifoEmpty
        // DIO4=LowBat
        // DIO5=ModeReady
		retStatus = SX127x_modifyRegister( REG_DIOMAPPING1,(RF_DIOMAPPING1_DIO0_MASK &
                                                           RF_DIOMAPPING1_DIO1_MASK &
                                                           RF_DIOMAPPING1_DIO2_MASK ),
                                                           	   	   	   	   	   	   	RF_DIOMAPPING1_DIO0_00 |
																					RF_DIOMAPPING1_DIO1_01 |
																					RF_DIOMAPPING1_DIO2_00);
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

		retStatus = SX127x_modifyRegister( REG_DIOMAPPING2,(RF_DIOMAPPING2_DIO4_MASK &
														    RF_DIOMAPPING2_DIO5_MASK &
															RF_DIOMAPPING2_MAP_MASK),
		                                                           	   	   	   	   	RF_DIOMAPPING2_DIO4_00 |
																					RF_DIOMAPPING2_DIO5_00 |
																					RF_DIOMAPPING2_MAP_RSSI);
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

//        retStatus = SX127x_modifyRegister(REG_PACKETCONFIG1, RF_PACKETCONFIG1_PACKETFORMAT_MASK, 
//                                                             RF_PACKETCONFIG1_PACKETFORMAT_FIXED);
        

//        retStatus = SX127x_writeRegister(REG_PAYLOADLENGTH, 20 + 1);
//        SX127x_ASSERT_RETURN(retStatus, SX127x_OK);
        
        retStatus = SX127x_readInterruptFlags();
        SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

        SX127x_handle.FSK.irqFlags.PacketSent = false;
            
    SX127x_handle.nIRQ = 0;     //Clear DIO interrupts
    SX127x_SET_TX();

		break;

        case SX127x_MODEM_LORA:
        
	SX127x_SET_TX();

		/**
		 * Interrupts are low enable.
		 * Only the TXDONE interrupt is enabled
		 */
		retStatus = SX127x_writeRegister( REG_LR_IRQFLAGSMASK, 	RFLR_IRQFLAGS_RXTIMEOUT |
																RFLR_IRQFLAGS_RXDONE |
																RFLR_IRQFLAGS_PAYLOADCRCERROR |
																RFLR_IRQFLAGS_VALIDHEADER |
																//RFLR_IRQFLAGS_TXDONE |
																RFLR_IRQFLAGS_CADDONE |
																RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
																RFLR_IRQFLAGS_CADDETECTED );
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

				// DIO0=RxDone
		retStatus = SX127x_modifyRegister( 	REG_DIOMAPPING1,
										   (RFLR_DIOMAPPING1_DIO0_MASK),
										   (RFLR_DIOMAPPING1_DIO0_01) );
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_readInterruptFlags();
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	SX127x_handle.nIRQ = 0;
	SX127x_handle.LoRa.irqFlags &= ~(RFLR_IRQFLAGS_TXDONE);


		//reset FIFO to the starting index to utilize entire FIFO
		retStatus = SX127x_writeRegister(REG_LR_FIFOTXBASEADDR, 0);
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

		retStatus = SX127x_writeRegister(REG_LR_FIFOADDRPTR, 	0);
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

        //clear interrupts register
		retStatus = SX127x_readInterruptFlags();
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

        SX127x_handle.LoRa.irqFlags &= ~(RFLR_IRQFLAGS_TXDONE);

		break;
	}

    
    retStatus = SX127x_setOperationMode( SX127x_TRANSMITTER );
    SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

    SX127x_handle.mode = SX127x_TRANSMITTER;

	return retStatus;
}


/**
 * @brief	Set the SX127x in standby mode
 * @return	SX127x_Return
 */
SX127xReturn_t SX127x_standbyMode(void)
{
	SX127xReturn_t retStatus;
	SX127x_RESET_TXRX();

	retStatus = SX127x_setOperationMode(SX127x_STANDBY);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	SX127x_handle.mode = SX127x_STANDBY;
	return retStatus;
}

/**
 * @brief		Read the Signal to noise ratio of the last packet received
 * @param snr	pointer to return variable for SNR value
 * @return		SX127x_Return
 */
SX127xReturn_t SX127x_readSNR(int16_t* snr)
{
	SX127xReturn_t retStatus;
    int8_t regSNR = 0;
	retStatus = SX127x_readRegister( REG_LR_PKTSNRVALUE , (uint8_t*)&regSNR);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

    *snr = regSNR;
    
	if( (*snr) & 0x80 ) // The SNR sign bit is 1
	{
			// Invert and divide by 4
			*snr = ( ( ~(*snr) + 1 ) & 0xFF ) >> 2;
			*snr = -(*snr);
	}
	else
	{
			// Divide by 4
			*snr = ( (*snr) & 0xFF ) >> 2;
	}

	return retStatus;
}

/**
 * @brief		Read the RSSI value for the last packet received
 * @ref			SX127x_Documentation
 * @param rssi	pointer to return variable for RSSI value
 * @return		SX127x_Return
 */
SX127xReturn_t SX127x_readRSSI(int16_t* rssi)
{
	SX127xReturn_t retStatus;
	int16_t snr;		//SNR value must be read before calculating RSSI
	int8_t regRSSI;
    retStatus = SX127x_readSNR(&snr);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_readRegister( REG_LR_PKTRSSIVALUE , (uint8_t*)&regRSSI);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

    *rssi = regRSSI;
    
    #if defined(SX1276)
	*rssi =  -157 + (*rssi);

	if (snr < 0)
		*rssi += snr/4;
    
    #elif defined(SX1272)
        
    if (snr >= 0)
        *rssi =  -139 + (*rssi) + ((*rssi)>>4);
    else if (snr < 0)
		*rssi = -139 + (*rssi) + ((*rssi)>>4) + snr;
    
    #endif
	return retStatus;
}
/** @}*/


/**
 * @defgroup SX127x_public_functions 	SX127x Public Functions
 * @{
 */



/**
 * @brief	Initialize the SX127x module.
 * @note 	The initialization sequence was carried over from the Semtech SX127x drivers.
 * @return	SX127x_Return
 */
SX127xReturn_t SX127x_Init(void)
{
	SX127xReturn_t retStatus;

	//reset chip before initialization
	SX127x_reset();
	//callibrate rx chain
//	retStatus = SX127x_callibrateRxChain();
//	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);
	//SX127x must be set in sleep mode in order to modify certain registers.
	retStatus = SX127x_setOperationMode( SX127x_SLEEP );
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_setModem(SX127x_MODEM_FSK);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_writeRegister( REG_LNA                , RF_LNA_GAIN_G1 | RF_LNA_BOOST_ON );
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_writeRegister( REG_RXCONFIG           , RF_RXCONFIG_RESTARTRXONCOLLISION_OFF |
															   RF_RXCONFIG_AFCAUTO_ON |
															   RF_RXCONFIG_AGCAUTO_ON |
															   RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_writeRegister( REG_RSSICONFIG         , RF_RSSICONFIG_OFFSET_M_06_DB |
    											   	   	   	   RF_RSSICONFIG_SMOOTHING_8);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_writeRegister( REG_PREAMBLEDETECT     , RF_PREAMBLEDETECT_DETECTOR_ON |
															   RF_PREAMBLEDETECT_DETECTORSIZE_2 |
															   RF_PREAMBLEDETECT_DETECTORTOL_10);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	//TODO : change remaining hardcoded byte values to configuration defines
	retStatus = SX127x_writeRegister( REG_OSC                , 0x07 );
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_writeRegister( REG_SYNCCONFIG         , 0x12 );
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_writeRegister( REG_SYNCVALUE1         , 0xC1 );
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_writeRegister( REG_SYNCVALUE2         , 0x94 );
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_writeRegister( REG_SYNCVALUE3         , 0xC1 );
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_writeRegister( REG_PACKETCONFIG1      , 0xD8 );
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_writeRegister( REG_FIFOTHRESH         , 0x8F );
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_writeRegister( REG_IMAGECAL           , 0x02 );
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_writeRegister( REG_DIOMAPPING1        , 0x00 );
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_writeRegister( REG_DIOMAPPING2        , 0x30 );
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	//set SX127x into LoRa mode
	retStatus = SX127x_setModem(SX127x_MODEM_LORA);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

    uint8_t regvalue = 0;
    retStatus = SX127x_readRegister( REG_LR_DETECTOPTIMIZE  , &regvalue);
    retStatus = SX127x_writeRegister( REG_LR_DETECTOPTIMIZE  , 0x43 );
    SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_writeRegister( REG_LR_PAYLOADMAXLENGTH	, RFLR_PAYLOADMAXLENGTH );
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	//turn off over current protection
	retStatus = SX127x_writeRegister( REG_LR_OCP				, RFLR_OCP_OFF);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	//retStatus = SX127x_configLoRa();
	retStatus = SX127x_setModem(SX127x_MODEM_LORA);
	return retStatus;
}


/**
 * @brief	Set the SX127x in sleep mode
 * @return	SX127x_Return
 */
SX127xReturn_t SX127x_sleepMode(void)
{
	SX127xReturn_t retStatus;
	SX127x_RESET_TXRX();

	retStatus = SX127x_setOperationMode(SX127x_SLEEP);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	SX127x_handle.mode = SX127x_SLEEP;
	return retStatus;

}
uint32_t  Transmition = 0;
/**
 * @brief 		  Send data over RF channel.
 * @param packet  Character buffer containing the data to be sent
 * @param len	  Number of bytes to be sent. Must be less than 256
 * @return		  SX127x_Return
 */
SX127xReturn_t SX127x_TransmitPacket(uint8_t* packet, uint8_t len)
{
  Transmition++;
	SX127xReturn_t retStatus;
	int32_t timeout;
	int32_t timeoutStep;

	retStatus = SX127x_standbyMode();
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	mDelay(1);

	retStatus = SX127x_writeRegister(REG_LR_PAYLOADLENGTH, len);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	//start writing packet from the start of FIFO to use the whole 255 byte FIFO
	retStatus = SX127x_writeRegister(REG_LR_FIFOTXBASEADDR, 0);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_writeRegister(REG_LR_FIFOADDRPTR, 0);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	//load packet into the chip FIFO
	retStatus = SX127x_writeBurst(REG_LR_FIFO, packet, len);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	//start transmission
  SX127x_handle.nIRQ = 0;
	retStatus = SX127x_transmitterMode();
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	//wait for TX done interrupt
  timeout = 500;		//wait for 5 seconds for the interrupt to be set
  timeoutStep = 2;
  
	while (!SX127x_handle.nIRQ  && (timeout-=timeoutStep))
	{
		mDelay(timeoutStep);
	}

    if(timeout  <= 0 )
    {
        SX127x_sleepMode();
        return SX127x_ERROR;
    }

	SX127x_handle.nIRQFlags.DIO0 = 0;

	//read interrupt flags to verify that TXDONE interrupt is set
	//and clear interrupt register
	retStatus = SX127x_readInterruptFlags();
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_sleepMode();
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	if (SX127x_IRQFlagSetLora(RFLR_IRQFLAGS_TXDONE) && SX127x_handle.LoRa.irqFlags != 0xFF)
	{
		SX127x_clearIRQFlagLoRa(RFLR_IRQFLAGS_TXDONE);
		SX127x_handle.lastTxTick = mTick();
		retStatus = SX127x_OK;
	}
	else
	{
		retStatus = SX127x_ERROR;
  }
  
  SX127x_sleepMode();
	return retStatus;

}


/**
 * @brief			Transmit packet in FSK mode
 * @param packet	Buffer containing the data to be sent
 * @param len		Number of bytes to be sent. Must be less than 64
 * @return			#SX127x_Return
 */
SX127xReturn_t SX127x_FSK_TransmitPacket(uint8_t* packet, uint8_t len)
{

	SX127xReturn_t retStatus = SX127x_ERROR;
	uint32_t timeout = 0;
    static uint8_t counter = 0;
    
    if (len > 64)
        return SX127x_ERROR;
    
    
    retStatus = SX127x_writeBurst(REG_FIFO, &len, 1);
    SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

    retStatus = SX127x_writeBurst(REG_FIFO, &counter, 1);
    SX127x_ASSERT_RETURN(retStatus, SX127x_OK);
    
    counter++;
    
    retStatus = SX127x_writeBurst(REG_FIFO, packet, len);
    SX127x_ASSERT_RETURN(retStatus, SX127x_OK);


	//start transmission
	retStatus = SX127x_transmitterMode();
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);        
	
    	//wait for TX done interrupt
    SX127x_handle.nIRQFlags.DIO0 = 0;

    timeout = 100*(SystemCoreClock/1000);		//wait for 5 seconds for the interrupt to be set
    
	while (!SX127x_READ_DIO(0) && (timeout--))
	{
		__nop();
	}

    if(timeout  <= 0 )
    {
        SX127x_sleepMode();
        return SX127x_ERROR;
    }
	
    //read interrupt flags to verify that TXDONE interrupt is set
	//and clear interrupt register
    retStatus = SX127x_readInterruptFlags();
    SX127x_ASSERT_RETURN(retStatus, SX127x_OK);


	SX127x_handle.nIRQ = 0;   // clear all nIRQ flags 
	retStatus = SX127x_sleepMode();         // Transmission is done go to sleep mode
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);
    
    if(SX127x_handle.FSK.irqFlags.PacketSent == true)
    {
        SX127x_handle.FSK.irqFlags.PacketSent = false;
        retStatus =  SX127x_OK;
    }
    else
    {
        retStatus = SX127x_ERROR;
    }

    return retStatus;
}


/**
 * @brief 		  Send data over RF channel in non blocking mode.
 * @param packet  Character buffer containing the data to be sent
 * @param len	  Number of bytes to be sent. Must be less than 256
 * @return		  SX127x_Return
 *
 * @note The application should poll the interrupt in a loop and
 * 		 call the callback on time out or interrupt event.
 */
SX127xReturn_t SX127x_Transmit_IT(uint8_t* packet, uint8_t len)
{
	SX127xReturn_t retStatus;

	retStatus = SX127x_standbyMode();
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	mDelay(1);

	retStatus = SX127x_writeRegister(REG_LR_PAYLOADLENGTH, len);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	//start writing packet from the start of FIFO to use the whole 255 byte FIFO
	retStatus = SX127x_writeRegister(REG_LR_FIFOTXBASEADDR, 0);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	retStatus = SX127x_writeRegister(REG_LR_FIFOADDRPTR, 0);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	//load packet into the chip FIFO
	retStatus = SX127x_writeBurst(REG_LR_FIFO, packet, len);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	//start transmission
	retStatus = SX127x_transmitterMode();
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	//wait for TX done interrupt
    SX127x_handle.nIRQFlags.DIO0 = 0;

    ///@todo Implement SX127x_Transmit_IT callback
	return retStatus;

}

/**
 * @brief 	Set the SX127x in receive mode
 * @return  SX127x_Return
 *
 * @note    For the time being, only the packet receive mode is used.
 * 			The chip also supports a continuous receive mode as well.
 */
SX127xReturn_t SX127x_receiverMode(void)
{

	SX127xReturn_t retStatus;

	retStatus = SX127x_setOperationMode( SX127x_STANDBY );
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);


	switch (SX127x_handle.modem)
	{


	case SX127x_MODEM_FSK:

        // DIO0=PayloadReady
        // DIO1=FifoLevel
        // DIO2=SyncAddr
        // DIO3=FifoEmpty
        // DIO4=Preamble
        // DIO5=ModeReady
		retStatus = SX127x_modifyRegister( REG_DIOMAPPING1,(RF_DIOMAPPING1_DIO0_MASK &
															RF_DIOMAPPING1_DIO1_MASK &
															RF_DIOMAPPING1_DIO2_MASK ),
																					RF_DIOMAPPING1_DIO0_00 |
																					RF_DIOMAPPING1_DIO1_00 |
																					RF_DIOMAPPING1_DIO2_11 );
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

		retStatus = SX127x_modifyRegister( REG_DIOMAPPING2,(RF_DIOMAPPING2_DIO4_MASK &
															RF_DIOMAPPING2_DIO5_MASK &
															RF_DIOMAPPING2_MAP_MASK ),
                                                                        			RF_DIOMAPPING2_DIO4_11 |
																					RF_DIOMAPPING2_MAP_PREAMBLEDETECT );
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

//        retStatus = SX127x_modifyRegister(REG_PACKETCONFIG1, RF_PACKETCONFIG1_PACKETFORMAT_MASK, 
//                                                             RF_PACKETCONFIG1_PACKETFORMAT_FIXED);
//        

//        retStatus = SX127x_writeRegister(REG_PAYLOADLENGTH, 20 + 2);
//        SX127x_ASSERT_RETURN(retStatus, SX127x_OK);
        
        retStatus = SX127x_writeRegister( REG_RXCONFIG, RF_RXCONFIG_AFCAUTO_ON |
        												RF_RXCONFIG_AGCAUTO_ON |
														RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT );
        SX127x_ASSERT_RETURN(retStatus, SX127x_OK);
        
        //Read interrupt flags
        retStatus = SX127x_readInterruptFlags();
        SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

        //Clear Rxdone flag and CRC error flag in the shadow register
        SX127x_handle.FSK.IrqReg.RegIrqFlags1 &= ~( RF_IRQFLAGS1_RSSI | 
                                                    RF_IRQFLAGS1_PREAMBLEDETECT |
                                                    RF_IRQFLAGS1_SYNCADDRESSMATCH);
        
        SX127x_handle.FSK.IrqReg.RegIrqFlags2 &= ~( RF_IRQFLAGS2_FIFOOVERRUN |
                                                    RF_IRQFLAGS2_PAYLOADREADY);

		break;


	case SX127x_MODEM_LORA:
  
	SX127x_SET_RX();

		/**
		 * Interrupts are low enable.
		 * Only the RXDONE and PAYLOADCRCERROR interrupts are enabled
		 */
		retStatus = SX127x_writeRegister( REG_LR_IRQFLAGSMASK, 	RFLR_IRQFLAGS_RXTIMEOUT |
																//RFLR_IRQFLAGS_RXDONE |
																//RFLR_IRQFLAGS_PAYLOADCRCERROR |
																RFLR_IRQFLAGS_VALIDHEADER |
																RFLR_IRQFLAGS_TXDONE |
																RFLR_IRQFLAGS_CADDONE |
																RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
																RFLR_IRQFLAGS_CADDETECTED );
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);


		retStatus = SX127x_modifyRegister( 	REG_DIOMAPPING1,
										   (RFLR_DIOMAPPING1_DIO0_MASK &
											RFLR_DIOMAPPING1_DIO3_MASK),
										   (RFLR_DIOMAPPING1_DIO0_00 |
											RFLR_DIOMAPPING1_DIO3_10) );
		SX127x_ASSERT_RETURN(retStatus, SX127x_OK);


        //clear interrupt flags in the chip
        retStatus = SX127x_readInterruptFlags();
        SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

        
        SX127x_handle.LoRa.irqFlags &= ~(RFLR_IRQFLAGS_RXDONE | RFLR_IRQFLAGS_PAYLOADCRCERROR);

        //reset FIFO buffer pointers in the chip
        retStatus = SX127x_writeRegister(REG_LR_FIFORXBASEADDR, 0);
        SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

        retStatus = SX127x_writeRegister(REG_LR_FIFOADDRPTR, 	0);
        SX127x_ASSERT_RETURN(retStatus, SX127x_OK);



		break;
	}

    SX127x_handle.nIRQ = 0;     //Clear DIO interrupt flags
    
    SX127x_SET_RX();
    retStatus = SX127x_setOperationMode( SX127x_RECEIVER );
    SX127x_ASSERT_RETURN(retStatus, SX127x_OK);
	SX127x_handle.mode = SX127x_RECEIVER;
	return retStatus;
}

/**
 * @brief				Set the SX127x operating bandwidth and spreadfactor.
 * @param bandwidth		New operating bandwidth
 * @param spreadFactor	New operating spread factor.
 * @return				#SX127xReturn_t
 *
 * @note The gain is inversely related to bandwidth and directly related to spread factor.
 * 	     Changing these parameters also changes the transmit time of data.
 *
 * @todo Change the TX gain param in this function as well.
 */
SX127xReturn_t SX127x_setGain(uint8_t bandwidth, uint8_t spreadFactor)
{

	SX127xReturn_t ret;
    SX127x_handle.LoRa.bandwidth = bandwidth;
    SX127x_handle.LoRa.spreadFactor = spreadFactor;

    SX127x_handle.LoRa.bandwidth     &= ~RFLR_MODEMCONFIG1_BW_MASK;
    SX127x_handle.LoRa.spreadFactor  &= ~RFLR_MODEMCONFIG2_SF_MASK;

    //limit the radio to operation only between 125 KHz to 500kHz
    if (SX127x_handle.LoRa.bandwidth > RFLR_MODEMCONFIG1_BW_500_KHZ)
    {
    	SX127x_handle.LoRa.bandwidth = RFLR_MODEMCONFIG1_BW_500_KHZ;
    }
    else if(SX127x_handle.LoRa.bandwidth <= RFLR_MODEMCONFIG1_BW_125_KHZ)
    {
    	SX127x_handle.LoRa.bandwidth = RFLR_MODEMCONFIG1_BW_125_KHZ;
    }

    //Max spread factor is 12. Min is set to 7.
    if (SX127x_handle.LoRa.spreadFactor > RFLR_MODEMCONFIG2_SF_12)
    {
    	SX127x_handle.LoRa.spreadFactor = RFLR_MODEMCONFIG2_SF_12;

    }
    else if (SX127x_handle.LoRa.spreadFactor < RFLR_MODEMCONFIG2_SF_7)
    {
    	SX127x_handle.LoRa.spreadFactor = RFLR_MODEMCONFIG2_SF_7;
    }

    ret = SX127x_modifyRegister(REG_LR_MODEMCONFIG1,
    							RFLR_MODEMCONFIG1_BW_MASK ,
								SX127x_handle.LoRa.bandwidth );
    SX127x_ASSERT_RETURN(ret, SX127x_OK);

    ret = SX127x_modifyRegister(REG_LR_MODEMCONFIG2,
    							RFLR_MODEMCONFIG2_SF_MASK,
								SX127x_handle.LoRa.spreadFactor);

    return ret;
}



/**
 * @brief	Process SX127x interrupts.
 * @return	SX127x_Return
 *
 * @warning The radio should not be put in sleep mode before calling this.
 * 			The SX127x FIFO is cleared and is not accessible when in sleep mode and using LoRa
 */
SX127xReturn_t SX127x_nIRQHandler(void)
{
	SX127xReturn_t retStatus = SX127x_ERROR;
	uint8_t regTemp;	//temporary storage for register values read from chip

	retStatus = SX127x_readInterruptFlags();
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

    switch (SX127x_handle.modem)
    {
     
    case SX127x_MODEM_FSK:
        
        if (SX127x_handle.mode == SX127x_SLEEP ||
            SX127x_handle.FSK.IrqReg.RegIrqFlags1 == 0xFF ||
            SX127x_handle.FSK.IrqReg.RegIrqFlags2 == 0xFF)
        {
            retStatus = SX127x_ERROR;
        }
        else if (SX127x_handle.FSK.irqFlags.PayloadReady == true)
        {
            if (SX127x_handle.FSK.irqFlags.CrcOk == false)
            {
                //Setting the FIFO overrun flag clears the FIFO
                SX127x_writeRegister(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);
                retStatus = SX127x_ERROR;
            }
            else
            {
                //read the packet from the chip
                retStatus = SX127x_readBurst(REG_FIFO, SX127x_handle.rxBuffer, 64);
                SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

                SX127x_handle.rxLength = SX127x_handle.rxBuffer[0];
            }
            
            retStatus = SX127x_modifyRegister( REG_RXCONFIG, RF_RXCONFIG_RESTARTRXONCOLLISION_MASK,
                                                             RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
            SX127x_ASSERT_RETURN( retStatus, SX127x_OK );
        
        }
        
        break;
    
    case SX127x_MODEM_LORA:
        if (SX127x_handle.mode == SX127x_SLEEP ||
            SX127x_handle.LoRa.irqFlags == 0xFF)
        {
            retStatus = SX127x_ERROR;
        }
        else if (SX127x_IRQFlagSetLora(RFLR_IRQFLAGS_PAYLOADCRCERROR))
        {
            //Clear FIFO and reset RXDONE interrupt.
            //FIFO is cleared by entering sleep mode
            SX127x_sleepMode();
            SX127x_standbyMode();

            SX127x_readSNR(&(SX127x_handle.SNR));
            SX127x_readRSSI(&(SX127x_handle.RSSI));
            retStatus = SX127x_ERROR;
        }
        else if (SX127x_IRQFlagSetLora(RFLR_IRQFLAGS_RXDONE))
        {

            //read starting address of the last packet received
            retStatus = SX127x_readRegister(REG_LR_FIFORXCURRENTADDR, &regTemp);
            SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

            //set the FIFO pointer to the start of last packet received
            retStatus = SX127x_writeRegister(REG_LR_FIFOADDRPTR, regTemp);
            SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

            //read the number of bytes received
            retStatus = SX127x_readRegister(REG_LR_RXNBBYTES, &regTemp);
            SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

            SX127x_handle.rxLength = regTemp;

            //read the packet from the chip
            retStatus = SX127x_readBurst(REG_LR_FIFO, SX127x_handle.rxBuffer, regTemp);
            SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

            //read last packet's SNR and RSSI values
            SX127x_readSNR(&(SX127x_handle.SNR));
            SX127x_readRSSI(&(SX127x_handle.RSSI));
        }
        else if (SX127x_IRQFlagSetLora(RFLR_IRQFLAGS_TXDONE))
        {
            SX127x_handle.lastTxTick = mTick();
        }
        else
        {
            retStatus = SX127x_ERROR;
        }

     
        break;
    }    
    return retStatus;
}


bool SX127x_isRxDone(void)
{
	return SX127x_IRQFlagSetLora(RFLR_IRQFLAGS_RXDONE);
}

bool SX127x_isTxDone(void)
{
	return SX127x_IRQFlagSetLora(RFLR_IRQFLAGS_TXDONE);
}

SX127xReturn_t SX127x_verifyConfig(void){

	uint8_t reg;
	
	 SX127x_readRegister(REG_LR_OPMODE, &reg);               //0x80 
    //if (reg != 0x80, &reg) return Failed;
	 SX127x_readRegister(REG_LR_FRFMSB, &reg);               //0xD9
	//if (reg != 0xD9, &reg) return Failed;
     SX127x_readRegister(REG_LR_FRFMID, &reg);               //0x00
	//if (reg != 0x00, &reg) return Failed;
     SX127x_readRegister(REG_LR_FRFMSB, &reg);               //0xD9
	//if (reg != 0xD9, &reg) return Failed;
     SX127x_readRegister(REG_LR_PACONFIG, &reg);             //0xFF
	//if (reg != 0xFF, &reg) return Failed;
     SX127x_readRegister(REG_LR_PARAMP, &reg);               //0x09
	//if (reg != 0x09, &reg) return Failed;
     SX127x_readRegister(REG_LR_OCP, &reg);                  //0x1F
	//if (reg != 0x1F, &reg) return Failed;
     SX127x_readRegister(REG_LR_MODEMCONFIG1, &reg);         //0x82
	//if (reg != 0x82, &reg) return Failed;
     SX127x_readRegister(REG_LR_MODEMCONFIG2, &reg);         //0xA4
	//if (reg != 0xA4, &reg) return Failed;
//     SX127x_readRegister(REG_LR_MODEMCONFIG3, &reg);         //0x0F
//	//if (reg != 0x0F, &reg) return Failed;
     SX127x_readRegister(REG_LR_PREAMBLELSB, &reg);          //0x08
	//if (reg != 0x08, &reg) return Failed;
     SX127x_readRegister(REG_LR_PREAMBLEMSB, &reg);          //0x00
	//if (reg != 0x00, &reg) return Failed;
     SX127x_readRegister(REG_LR_TCXO, &reg);                 //0x09
	//if (reg != 0x09, &reg) return Failed;
     SX127x_readRegister(REG_LR_PADAC, &reg);                //0x87
	//if (reg != 0x87, &reg) return Failed;
     SX127x_readRegister(0x70, &reg);
     SX127x_readRegister(0x42, &reg);
	
    return SX127x_OK;
}


/** @}*/

