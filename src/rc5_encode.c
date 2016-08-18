/**
  ******************************************************************************
  * @file    rc5_encode.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    08-March-2016
  * @brief   This file provides all the rc5 encode firmware functions
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "rc5_encode.h"

/** @addtogroup IR_REMOTE
  * @{
  */

/** @addtogroup RC5_ENCODE
  * @brief RC5_ENCODE driver module
  * @{
  */

/** @addtogroup RC5_Private_Defines
  * @{
  */
#define  RC5HIGHSTATE     ((uint8_t )0x02)   /* RC5 high level definition*/
#define  RC5LOWSTATE      ((uint8_t )0x01)   /* RC5 low level definition*/

/**
  * @}
  */

/** @addtogroup RC5_Private_Function_Prototypes
  * @{
  */
static uint16_t RC5_BinFrameGeneration(uint8_t RC5_Address, uint8_t RC5_Instruction, RC5_Ctrl_t RC5_Ctrl);
static uint32_t RC5_ManchesterConvert(uint16_t RC5_BinaryFrameFormat);
static void RC5_Encode_DeInit(void);
/**
  * @}
  */

/**
  * @addtogroup RC5_Private_Variables
  * @{
  */

uint8_t RC5RealFrameLength = 14;
uint8_t RC5GlobalFrameLength = 64;
uint16_t RC5BinaryFrameFormat = 0;
uint32_t RC5ManchesterFrameFormat = 0;
/*__IO*/ uint32_t RC5SendOpCompleteFlag = 1;
/*__IO*/ uint32_t RC5SendOpReadyFlag = 0;
RC5_Ctrl_t RC5Ctrl1 = RC5_CTRL_RESET;
/**
  * @}
  */

/**
  * @addtogroup RC5_Exported_Functions
  * @{
  */



/**
  * @brief  De-initializes the peripherals (GPIO, TIM)
  * @param  None
  * @retval None
  */
static void RC5_Encode_DeInit(void)
{
  HAL_TIM_OC_DeInit(&TimHandleLF);
  HAL_TIM_OC_DeInit(&TimHandleHF);
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);
}

/**
  * @brief Init Hardware (IPs used) for RC5 generation
  * @param None
  * @retval  None
  */
void RC5_Encode_Init(void)
{
  TIM_OC_InitTypeDef ch_config;
  GPIO_InitTypeDef gpio_init_struct;

  /* TIM16 clock enable */
  __HAL_RCC_TIM2_CLK_ENABLE();

  /* TIM17 clock enable */
  __HAL_RCC_TIM5_CLK_ENABLE();

  TimHandleLF.Instance = TIM2;
  TimHandleHF.Instance = TIM5;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PB9 */
  gpio_init_struct.Pin = GPIO_PIN_9;
  gpio_init_struct.Mode = GPIO_MODE_AF_PP;
  gpio_init_struct.Pull = GPIO_NOPULL;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
  //TODO: check if alternate function is needed
  //gpio_init_struct.Alternate = GPIO_AF6_IR;
  HAL_GPIO_Init(GPIOB, &gpio_init_struct);

  /* DeInit TIM17 */
  HAL_TIM_OC_DeInit(&TimHandleHF);

  /* Elementary period 888us */
  /* Time base configuration for timer 2 */
  TimHandleHF.Init.Period = IR_ENC_HPERIOD_RC5;
  TimHandleHF.Init.Prescaler = 0x00;
  TimHandleHF.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TimHandleHF.Init.CounterMode = TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(&TimHandleHF);
  if (HAL_TIM_OC_Init(&TimHandleHF) != HAL_OK)
  {
    /* Initialization Error */
    while (1);
  }

  /* Output Compare Timing Mode configuration: Channel 1N */
  ch_config.OCMode = TIM_OCMODE_PWM1;
  ch_config.Pulse = IR_ENC_HPERIOD_RC5 / 4; /* To be compatible with RC5 specification */
  ch_config.OCPolarity = TIM_OCPOLARITY_HIGH;
  ch_config.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  ch_config.OCFastMode = TIM_OCFAST_DISABLE;
  ch_config.OCIdleState = TIM_OCIDLESTATE_RESET;
  ch_config.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&TimHandleHF, &ch_config, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    while (1);
  }

  TIM_SET_CAPTUREPOLARITY(&TimHandleHF, TIM_CHANNEL_1, TIM_CCxN_ENABLE | TIM_CCx_ENABLE );

  HAL_TIM_OC_Start(&TimHandleHF, TIM_CHANNEL_1);

  /* DeInit TIM16 */
  HAL_TIM_OC_DeInit(&TimHandleLF);

  /* Time Base configuration for timer 16 */
  TimHandleLF.Init.Prescaler = 0;
  TimHandleLF.Init.CounterMode = TIM_COUNTERMODE_UP;
  TimHandleLF.Init.Period = IR_ENC_LPERIOD_RC5;
  TimHandleLF.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TimHandleLF.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&TimHandleLF);
  if (HAL_TIM_OC_Init(&TimHandleLF) != HAL_OK)
  {
    /* Initialization Error */
    while (1);
  }

  /* Channel 1 Configuration in Timing mode */
  ch_config.OCMode = TIM_OCMODE_TIMING;
  ch_config.Pulse = IR_ENC_LPERIOD_RC5;
  ch_config.OCPolarity = TIM_OCPOLARITY_HIGH;
  ch_config.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  ch_config.OCFastMode = TIM_OCFAST_DISABLE;
  ch_config.OCIdleState = TIM_OCIDLESTATE_RESET;
  ch_config.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&TimHandleLF, &ch_config, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    while (1);
  }

  TIM_SET_CAPTUREPOLARITY(&TimHandleLF, TIM_CHANNEL_1, TIM_CCxN_ENABLE | TIM_CCx_ENABLE );

  /* Peripheral interrupt init*/
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  /* Enable the main output */
  __HAL_TIM_MOE_ENABLE(&TimHandleLF);

  /* TIM Disable */
  __HAL_TIM_DISABLE(&TimHandleLF);
}

/**
  * @brief Generate and Send the RC5 frame.
  * @param RC5_Address : the RC5 Device destination
  * @param RC5_Instruction : the RC5 command instruction
  * @param RC5_Ctrl : the RC5 Control bit.
  * @retval  None
  */
void RC5_Encode_SendFrame(uint8_t RC5_Address, uint8_t RC5_Instruction, RC5_Ctrl_t RC5_Ctrl)
{
  /* Generate a binary format of the Frame */
  RC5BinaryFrameFormat = RC5_BinFrameGeneration(RC5_Address, RC5_Instruction, RC5_Ctrl);

  /* Generate a Manchester format of the Frame */
  RC5ManchesterFrameFormat = RC5_ManchesterConvert(RC5BinaryFrameFormat);

  /* Set the Send operation Ready flag to indicate that the frame is ready to be sent */
  RC5SendOpReadyFlag = 1;

  /* Reset the counter to ensure accurate timing of sync pulse */
  __HAL_TIM_SET_COUNTER( &TimHandleLF, 0);

  /* TIM IT Enable */
  HAL_TIM_Base_Start_IT(&TimHandleLF);
}

/**
  * @brief Send by hardware Manchester Format RC5 Frame.
  * @retval None
  */
void RC5_Encode_SignalGenerate(void)
{
  uint32_t bit_msg = 0;

  if ((RC5SendOpReadyFlag == 1) && (BitsSentCounter <= (RC5GlobalFrameLength * 2)))
  {
    RC5SendOpCompleteFlag = 0x00;
    bit_msg = (uint8_t)((RC5ManchesterFrameFormat >> BitsSentCounter) & 1);

    if (bit_msg == 1)
    {
      TIM_ForcedOC1Config(TIM_FORCED_ACTIVE);
    }
    else
    {
      TIM_ForcedOC1Config(TIM_FORCED_INACTIVE);
    }
    BitsSentCounter++;
  }
  else
  {
    RC5SendOpCompleteFlag = 0x01;

    /* TIM IT Disable */
    HAL_TIM_Base_Stop_IT(&TimHandleLF);
    RC5SendOpReadyFlag = 0;
    BitsSentCounter = 0;
    TIM_ForcedOC1Config(TIM_FORCED_INACTIVE);

    /* TIM Disable */
    __HAL_TIM_DISABLE(&TimHandleLF);
  }
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief Generate the binary format of the RC5 frame.
  * @param RC5_Address : Select the device address.
  * @param RC5_Instruction : Select the device instruction.
  * @param RC5_Ctrl : Select the device control bit status.
  * @retval Binary format of the RC5 Frame.
  */
static uint16_t RC5_BinFrameGeneration(uint8_t RC5_Address, uint8_t RC5_Instruction, RC5_Ctrl_t RC5_Ctrl)
{
  uint16_t star1 = 0x2000;
  uint16_t star2 = 0x1000;
  uint16_t addr = 0;

  while (RC5SendOpCompleteFlag == 0x00)
  {}

  /* Check if Instruction is 128-bit length */
  if (RC5_Instruction >= 64)
  {
    /* Reset field bit: command is 7-bit length */
    star2 = 0;
    /* Keep the lowest 6 bits of the command */
    RC5_Instruction &= 0x003F;
  }
  else /* Instruction is 64-bit length */
  {
    /* Set field bit: command is 6-bit length */
    star2 = 0x1000;
  }

  RC5SendOpReadyFlag = 0;
  RC5ManchesterFrameFormat = 0;
  RC5BinaryFrameFormat = 0;
  addr = ((uint16_t)(RC5_Address)) << 6;
  RC5BinaryFrameFormat =  (star1) | (star2) | (RC5_Ctrl) | (addr) | (RC5_Instruction);
  return (RC5BinaryFrameFormat);
}

/**
  * @brief Convert the RC5 frame from binary to Manchester Format.
  * @param RC5_BinaryFrameFormat : the RC5 frame in binary format.
  * @retval the RC5 frame in Manchester format.
  */
static uint32_t RC5_ManchesterConvert(uint16_t RC5_BinaryFrameFormat)
{
  uint8_t i = 0;
  uint16_t Mask = 1;
  uint16_t bit_format = 0;
  uint32_t ConvertedMsg = 0;

  for (i = 0; i < RC5RealFrameLength; i++)
  {
    bit_format = ((((uint16_t)(RC5_BinaryFrameFormat)) >> i) & Mask) << i;
    ConvertedMsg = ConvertedMsg << 2;

    if (bit_format != 0 ) /* Manchester 1 -|_  */
    {
      ConvertedMsg |= RC5HIGHSTATE;
    }
    else /* Manchester 0 _|-  */
    {
      ConvertedMsg |= RC5LOWSTATE;
    }
  }
  return (ConvertedMsg);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
