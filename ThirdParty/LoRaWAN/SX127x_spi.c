/**
 * @file   	SX127x_spi.c
 * @author 	Hassan
 * @version	
 * @date	Mar 17, 2016
 * 
 * @brief   
 */


#include "SX127x_spi.h"


/**
 * @defgroup SX127x_spi_private_members	SX127x SPI Private Members
 * @{
 */


/** @}*/

/**
 * @defgroup SX127x_spi_public_functions SX127x SPI Public Functions
 * @{
 */

/**
 * @brief 			Read the current value of SX127x register.
 * @param regAddr	Register Address
 * @param pRetValue	pointer to return variable
 * @return
 * 		@arg SX127x_OK		if register read was successful
 * 		@arg SX127x_ERROR	if register read failed
 */
__INLINE SX127xReturn_t SX127x_readRegister(uint8_t regAddr, uint8_t* pRetValue)
{
	return SX127x_readBurst(regAddr, pRetValue, 1);
}

/**
 * @brief 		  Write the specified SX127x register
 * @param regAddr Register Address
 * @param value   Value to be written to the register
 * @return
 * 		@arg SX127x_OK		if register write successful
 * 		@arg SX127x_ERROR	if register write failed
 */
__INLINE SX127xReturn_t SX127x_writeRegister(uint8_t regAddr, uint8_t value)
{
	return SX127x_writeBurst(regAddr, &value, 1);
}

/**
 * @brief		  Modify only certain bits of the register
 * @param regAddr Register Address
 * @param mask	  The write bit mask
 * @param value	  Bits to be written
 * @return
 * 		@arg SX127x_OK		if register modify was successful
 * 		@arg SX127x_ERROR	if register modify failed
 */
SX127xReturn_t SX127x_modifyRegister(uint8_t regAddr, uint8_t mask, uint8_t value)
{
	SX127xReturn_t retStatus;	//return value
	uint8_t regTemp;			//temporary storage for original register value

	//read register
	retStatus = SX127x_readRegister(regAddr, &regTemp);
	SX127x_ASSERT_RETURN(retStatus, SX127x_OK);

	//modify register
	regTemp = (regTemp & mask) | value;

	//write register
	retStatus = SX127x_writeRegister(regAddr, regTemp);

	return retStatus;
}

/**
 * @brief 		  Read multiple consecutive registers in succession
 * @param regAddr Starting register address
 * @param buffer  Buffer to store the read values
 * @param len	  number of registers to read
 * @return
 *   	@arg SX127x_OK		if register read successful
 * 		@arg SX127x_ERROR	if register read failed
 */
SX127xReturn_t SX127x_readBurst(uint8_t regAddr, uint8_t* buffer, uint8_t len)
{		//return value
    HAL_StatusTypeDef halStatus;
	//set wnr bit 0 for read access
	regAddr = regAddr & 0x7F;

	SX127x_RESET_NSEL();
    //Send the read command along with the starting register address
	halStatus = SX127x_spiTransmit(&regAddr, 1);

	if (halStatus == HAL_OK){
		//read the registers in succession
		halStatus = SX127x_spiReceive(buffer, len);
	}
    //Deselect the radio by pulling high the nSEL pin
    SX127x_SET_NSEL();
    
    if (halStatus == HAL_OK)
        return SX127x_OK;
    
    return SX127x_ERROR;
}

/**
 * @brief 		  Write the values specified in the buffer to the SX127x registers in succession
 * @param regAddr starting register address
 * @param buffer  register values stored in order of SX127x register
 * @param len     number of registers to write
 * @return
 *    	@arg SX127x_OK		if register read successful
 * 		@arg SX127x_ERROR	if register read failed
 */
SX127xReturn_t SX127x_writeBurst(uint8_t regAddr, uint8_t* buffer, uint8_t len)
{

	HAL_StatusTypeDef halStatus;

	regAddr = regAddr | 0x80;

	SX127x_RESET_NSEL();

	halStatus = SX127x_spiTransmit(&regAddr, 1);

	if (halStatus == HAL_OK){
		//read the registers in succession
		halStatus = SX127x_spiTransmit(buffer, len);
	}

	SX127x_SET_NSEL();

    if (halStatus == HAL_OK)
        return SX127x_OK;
    
    return SX127x_ERROR;
}

/** @}*/
