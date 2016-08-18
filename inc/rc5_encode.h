/**
  ******************************************************************************
  * @file    rc5_encode.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    08-March-2016
  * @brief   This file contains all the functions prototypes for the rc5 encode
  *          firmware library.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
#ifndef __RC5_ENCODE_H
#define __RC5_ENCODE_H

/** @addtogroup IR_REMOTE
  * @{
  */

/** @addtogroup RC5_ENCODE
  * @{
  */

/** @addtogroup RC5_Public_Types
  * @{
  */

/**
  * @brief Definition of the RC5 Control bit value.
  */
typedef enum
{
  RC5_CTRL_RESET                        = ((uint16_t)0),
  RC5_CTRL_SET                          = ((uint16_t)0x0800)
}RC5_Ctrl_t;

/**
  * @}
  */

/** @addtogroup RC5_Public_Variables
  * @{
  */
extern const uint8_t* aRC5Commands [];
extern const uint8_t* aRC5Devices[];

/**
  * @}
  */

/** @addtogroup RC5_Exported_Functions
  * @{
  */
void Menu_RC5_Encode_Func(void);
void RC5_Encode_Init(void);
void RC5_Encode_SendFrame(uint8_t RC5_Address, uint8_t RC5_Instruction, RC5_Ctrl_t RC5_Ctrl);
void RC5_Encode_SignalGenerate(void);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif  /*__RC5_ENCODE_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
