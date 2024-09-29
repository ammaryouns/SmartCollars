/**
  ******************************************************************************
  * @file    ST25DVExtFunc.c
  * @author  Central LAB
  * @version V1.1.2
  * @date    06-Jul-2018
  * @brief   Extended ST25DV functionalities APIs Implementation
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "st25dv.h"
#include "TagType5.h"
#include "ST25DVExtFunc.h"

/* Exported Functions  -------------------------------------------------------*/

/**
  * @brief  Write GPO configuration:
  *      GPO managed by user             = ST25DV_GPO_ENABLE_MASK | ST25DV_GPO_RFUSERSTATE_MASK
  *      GPO sensible to RF activity     = ST25DV_GPO_ENABLE_MASK | ST25DV_GPO_RFACTIVITY_MASK
  *      GPO sensible to RF Field change = ST25DV_GPO_ENABLE_MASK | ST25DV_GPO_FIELDCHANGE_MASK
  *
  * @param  uint32_t MsbPasswd MSB 32-bit password
  * @param  uint32_t LsbPasswd LSB 32-bit password
  * @param  uint16_t ITConfig Provides the GPO configuration to apply
  * @retval NFCTAG_StatusTypeDef enum status
  */
NFCTAG_StatusTypeDef ST25DV_WriteConfigIT(uint32_t MsbPasswd,uint32_t LsbPasswd,const uint16_t ITConfig)
{
  NFCTAG_StatusTypeDef Ret;
  ST25DV_PASSWD Passwd;
  /* Present password to open session  */
  Passwd.MsbPasswd = MsbPasswd;
  Passwd.LsbPasswd = LsbPasswd;
  Ret = St25Dv_i2c_ExtDrv.PresentI2CPassword( Passwd );

  /* Change the GPO configuration value */
  Ret = St25Dv_i2c_Drv.ConfigIT( ITConfig );

  /* present wrong password for closing the session */
  Passwd.MsbPasswd = ~MsbPasswd;
  Passwd.LsbPasswd = ~LsbPasswd;
  St25Dv_i2c_ExtDrv.PresentI2CPassword( Passwd );

  return Ret;
}

/**
  * @brief  Read the IT status register from the ST25DV
  * @param  uint8_t *pITStatus current IT status register from the ST25DV:
  *                       RFUSERSTATE  = 0x01
  *                       RFBUSY       = 0x02
  *                       RFINTERRUPT  = 0x04
  *                       FIELDFALLING = 0x08
  *                       FIELDRISING  = 0x10
  *                       RFPUTMSG     = 0x20
  *                       RFGETMSG     = 0x40
  *                       RFWRITE      = 0x80
  *   Once read the IT status register is cleared (set to 00h).
  * @retval NFCTAG_StatusTypeDef enum status
  */
NFCTAG_StatusTypeDef ST25DV_ReadITStatus(uint8_t * const pITStatus)
{
  return St25Dv_i2c_ExtDrv.ReadITSTStatus_Dyn(pITStatus);
}

/**
  * @brief  Change the I2C Password protection
  * @param  uint32_t MsbPasswd MSB 32-bit password
  * @param  uint32_t LsbPasswd LSB 32-bit password
  * @retval NFCTAG_StatusTypeDef enum status
  */
NFCTAG_StatusTypeDef ST25DV_ChangeI2CPassword(uint32_t MsbPasswd,uint32_t LsbPasswd)
{
  NFCTAG_StatusTypeDef Ret = NFCTAG_OK;
  ST25DV_I2CSSO_STATUS I2CSS;
  ST25DV_PASSWD Passwd;
  St25Dv_i2c_ExtDrv.ReadI2CSecuritySession_Dyn( &I2CSS );
  if( I2CSS == ST25DV_SESSION_CLOSED ) {
    /* if I2C session is closed, present default password to open session */
    Passwd.MsbPasswd = 0;
    Passwd.LsbPasswd = 0;
    Ret = St25Dv_i2c_ExtDrv.PresentI2CPassword( Passwd );
    if(Ret==NFCTAG_OK) {
      /* Ok we could Change the default Password */
      Passwd.MsbPasswd = MsbPasswd;
      Passwd.LsbPasswd = LsbPasswd;
      Ret = St25Dv_i2c_ExtDrv.WriteI2CPassword(Passwd);
      if(Ret==NFCTAG_OK) {
        /* Present a wrong password for closing the session we have alredy setted the new one here */
        Passwd.MsbPasswd = ~MsbPasswd;
        Passwd.LsbPasswd = ~LsbPasswd;
        St25Dv_i2c_ExtDrv.PresentI2CPassword( Passwd );
      }
    }
  }
  return Ret;
}

#ifdef SMARTAG_ENABLE_PROTECTION_I2C
/**
  * @brief  Set the I2C protection level creating a Single secure zone
  * @param  uint32_t MsbPasswd MSB 32-bit password
  * @param  uint32_t LsbPasswd LSB 32-bit password
  * ST25DV_PROTECTION_CONF ProtectionLevel
  * @retval NFCTAG_StatusTypeDef enum status
  */
NFCTAG_StatusTypeDef ST25DV_SetICPasswordProtectionZone1(uint32_t MsbPasswd,uint32_t LsbPasswd,ST25DV_PROTECTION_CONF ProtectionLevel)
{
  NFCTAG_StatusTypeDef Ret = NFCTAG_OK;
  ST25DV_I2C_PROT_ZONE pProtZone;
  /* Read the Protection levels */
  Ret = St25Dv_i2c_ExtDrv.ReadI2CProtectZone(&pProtZone);
  if(Ret==NFCTAG_OK) {
    /* Check if the Protect Zone 1 is already on Read and Write Protection */
    if(pProtZone.ProtectZone1!=ProtectionLevel) {
      ST25DV_I2CSSO_STATUS I2CSS;

      /* Read the Session status */
      St25Dv_i2c_ExtDrv.ReadI2CSecuritySession_Dyn( &I2CSS );
      if( I2CSS == ST25DV_SESSION_CLOSED ) {
        ST25DV_PASSWD Passwd;
        /* if I2C session is closed, present password to open session */
        Passwd.MsbPasswd = MsbPasswd;
        Passwd.LsbPasswd = LsbPasswd;
        Ret = St25Dv_i2c_ExtDrv.PresentI2CPassword( Passwd );

        if(Ret==NFCTAG_OK) {
          ST25DV_MEM_SIZE st25dvmemsize;
          uint32_t st25dvbmsize;

          St25Dv_i2c_ExtDrv.ReadMemSize( &st25dvmemsize );
          /* st25dvmemsize is composed of Mem_Size (number of blocks) and BlockSize (size of each blocks in bytes) */
          st25dvbmsize = (st25dvmemsize.Mem_Size + 1) * (st25dvmemsize.BlockSize + 1);

          /* We create one Memory size == to the whole memory size */
          Ret = St25Dv_i2c_ExtDrv.CreateUserZone( st25dvbmsize, 0, 0, 0 );

          if(Ret==NFCTAG_OK) {
            /* Set Protection leve for zone 1 for i2c  */
            Ret = St25Dv_i2c_ExtDrv.WriteI2CProtectZonex( ST25DV_PROT_ZONE1, ProtectionLevel );
          }
          /* Present a wrong Password for closing the session */
          Passwd.MsbPasswd = ~MsbPasswd;
          Passwd.LsbPasswd = ~LsbPasswd;
          St25Dv_i2c_ExtDrv.PresentI2CPassword( Passwd );
        }
      }
    }
  }
  return Ret;
}
#endif /* SMARTAG_ENABLE_PROTECTION_I2C */

#ifdef SMARTAG_ENABLE_PROTECTION_RF
/**
  * @brief  Enable the R/F writing protection for Zone 1 using RF_PWD_1 password
  * @param  uint32_t MsbPasswd MSB 32-bit password
  * @param  uint32_t LsbPasswd LSB 32-bit password
  * @retval NFCTAG_StatusTypeDef enum status
  */
NFCTAG_StatusTypeDef ST25DV_EnableRFWritingPasswordProtectionZone1(uint32_t MsbPasswd,uint32_t LsbPasswd)
{
  NFCTAG_StatusTypeDef Ret = NFCTAG_OK;
  ST25DV_RF_PROT_ZONE pRfprotZone;
  /* Read Level of R/F protection */
  Ret = St25Dv_i2c_ExtDrv.ReadRFZxSS(ST25DV_PROT_ZONE1, &pRfprotZone );
  if(Ret==NFCTAG_OK) {
    /* Change the R/F protection level if it's necessary */
    if((pRfprotZone.PasswdCtrl != ST25DV_PROT_PASSWD1 ) & (pRfprotZone.RWprotection != ST25DV_WRITE_PROT)) {
      ST25DV_PASSWD Passwd;
      /* Present password to open session  */
      Passwd.MsbPasswd = MsbPasswd;
      Passwd.LsbPasswd = LsbPasswd;
      Ret = St25Dv_i2c_ExtDrv.PresentI2CPassword( Passwd );

      pRfprotZone.PasswdCtrl = ST25DV_PROT_PASSWD1;
      pRfprotZone.RWprotection = ST25DV_WRITE_PROT;
      Ret = St25Dv_i2c_ExtDrv.WriteRFZxSS(ST25DV_PROT_ZONE1, pRfprotZone);

      /* present wrong password for closing the session */
      Passwd.MsbPasswd = ~MsbPasswd;
      Passwd.LsbPasswd = ~LsbPasswd;
      St25Dv_i2c_ExtDrv.PresentI2CPassword( Passwd );
    }
  }
  return Ret;
}

/**
  * @brief  Disable the R/F writing protection for Zone 1 using RF_PWD_1 password
  * @param  uint32_t MsbPasswd MSB 32-bit password
  * @param  uint32_t LsbPasswd LSB 32-bit password
  * @retval NFCTAG_StatusTypeDef enum status
  */
NFCTAG_StatusTypeDef ST25DV_DisableRFWritingPasswordProtectionZone1(uint32_t MsbPasswd,uint32_t LsbPasswd)
{
  NFCTAG_StatusTypeDef Ret = NFCTAG_OK;
  ST25DV_RF_PROT_ZONE pRfprotZone;
  /* Read Level of R/F protection */
  Ret = St25Dv_i2c_ExtDrv.ReadRFZxSS(ST25DV_PROT_ZONE1, &pRfprotZone );
  if(Ret==NFCTAG_OK) {
    /* Change the R/F protection level if it's necessary */
    if((pRfprotZone.PasswdCtrl != ST25DV_NOT_PROTECTED ) & (pRfprotZone.RWprotection != ST25DV_NO_PROT)) {
      ST25DV_PASSWD Passwd;
      /* Present password to open session  */
      Passwd.MsbPasswd = MsbPasswd;
      Passwd.LsbPasswd = LsbPasswd;
      Ret = St25Dv_i2c_ExtDrv.PresentI2CPassword( Passwd );

      pRfprotZone.PasswdCtrl = ST25DV_NOT_PROTECTED;
      pRfprotZone.RWprotection = ST25DV_NO_PROT;
      Ret = St25Dv_i2c_ExtDrv.WriteRFZxSS(ST25DV_PROT_ZONE1, pRfprotZone);

      /* present wrong password for closing the session */
      Passwd.MsbPasswd = ~MsbPasswd;
      Passwd.LsbPasswd = ~LsbPasswd;
      St25Dv_i2c_ExtDrv.PresentI2CPassword( Passwd );
    }
  }
  return Ret;
}
#endif /* SMARTAG_ENABLE_PROTECTION_RF */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
