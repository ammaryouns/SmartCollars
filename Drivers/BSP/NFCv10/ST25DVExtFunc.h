/**
  ******************************************************************************
  * @file    ST25DVExtFunc.h
  * @author  Central LAB
  * @version V1.1.2
  * @date    06-Jul-2018
  * @brief   Extended ST25DV functionalities APIs
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
#ifndef __ST25DVEXTFUNC_H
#define __ST25DVEXTFUNC_H

/* Exported Functions --------------------------------------------------------*/

extern NFCTAG_StatusTypeDef ST25DV_ChangeI2CPassword(uint32_t MsbPasswd,uint32_t LsbPasswd);
extern NFCTAG_StatusTypeDef ST25DV_WriteConfigIT(uint32_t MsbPasswd,uint32_t LsbPasswd,const uint16_t ITConfig);
#ifdef SMARTAG_ENABLE_PROTECTION_I2C
extern NFCTAG_StatusTypeDef ST25DV_SetICPasswordProtectionZone1(uint32_t MsbPasswd,uint32_t LsbPasswd,ST25DV_PROTECTION_CONF ProtectionLevel);
#endif /* SMARTAG_ENABLE_PROTECTION_I2C */

#ifdef SMARTAG_ENABLE_PROTECTION_RF
extern NFCTAG_StatusTypeDef ST25DV_EnableRFWritingPasswordProtectionZone1(uint32_t MsbPasswd,uint32_t LsbPasswd);
extern NFCTAG_StatusTypeDef ST25DV_DisableRFWritingPasswordProtectionZone1(uint32_t MsbPasswd,uint32_t LsbPasswd);
#endif /* SMARTAG_ENABLE_PROTECTION_RF */

extern NFCTAG_StatusTypeDef ST25DV_ReadITStatus(uint8_t * const pITStatus);

#endif /* __ST25DVEXTFUNC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
