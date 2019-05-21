/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define RF2_RESETN_Pin GPIO_PIN_10
#define RF2_RESETN_GPIO_Port GPIOI
#define RF2_TRX_CTRL_Pin GPIO_PIN_11
#define RF2_TRX_CTRL_GPIO_Port GPIOI
#define RF2_DIO5_Pin GPIO_PIN_0
#define RF2_DIO5_GPIO_Port GPIOF
#define RF2_DIO4_Pin GPIO_PIN_1
#define RF2_DIO4_GPIO_Port GPIOF
#define RF2_DIO3_Pin GPIO_PIN_2
#define RF2_DIO3_GPIO_Port GPIOF
#define RF2_DIO2_Pin GPIO_PIN_3
#define RF2_DIO2_GPIO_Port GPIOF
#define RF2_DIO1_Pin GPIO_PIN_4
#define RF2_DIO1_GPIO_Port GPIOF
#define RF2_DIO0_EXIT_Pin GPIO_PIN_5
#define RF2_DIO0_EXIT_GPIO_Port GPIOF
#define RF2_DIO0_EXIT_EXTI_IRQn EXTI9_5_IRQn
#define RF2_NSS_Pin GPIO_PIN_6
#define RF2_NSS_GPIO_Port GPIOF
#define RF2_EN_Pin GPIO_PIN_2
#define RF2_EN_GPIO_Port GPIOC
#define RF1_TRX_CTRL_Pin GPIO_PIN_1
#define RF1_TRX_CTRL_GPIO_Port GPIOA
#define RF1_DIO5_Pin GPIO_PIN_2
#define RF1_DIO5_GPIO_Port GPIOA
#define RF1_DIO4_Pin GPIO_PIN_2
#define RF1_DIO4_GPIO_Port GPIOH
#define RF_DIO3_Pin GPIO_PIN_3
#define RF_DIO3_GPIO_Port GPIOH
#define RF1_DIO2_Pin GPIO_PIN_4
#define RF1_DIO2_GPIO_Port GPIOH
#define RF1_DIO1_Pin GPIO_PIN_5
#define RF1_DIO1_GPIO_Port GPIOH
#define RF1_DIO0_EXIT_Pin GPIO_PIN_3
#define RF1_DIO0_EXIT_GPIO_Port GPIOA
#define RF1_DIO0_EXIT_EXTI_IRQn EXTI3_IRQn
#define RF1_NSS_Pin GPIO_PIN_4
#define RF1_NSS_GPIO_Port GPIOA
#define RF1_RESETN_Pin GPIO_PIN_5
#define RF1_RESETN_GPIO_Port GPIOC
#define RF1_EN_Pin GPIO_PIN_13
#define RF1_EN_GPIO_Port GPIOF
#define WDI_Pin GPIO_PIN_9
#define WDI_GPIO_Port GPIOE
#define RF3_NSS_Pin GPIO_PIN_11
#define RF3_NSS_GPIO_Port GPIOE
#define RF3_DIO5_Pin GPIO_PIN_15
#define RF3_DIO5_GPIO_Port GPIOE
#define RF3_DIO4_Pin GPIO_PIN_10
#define RF3_DIO4_GPIO_Port GPIOB
#define RF3_DIO3_Pin GPIO_PIN_11
#define RF3_DIO3_GPIO_Port GPIOB
#define RF3_DIO2_Pin GPIO_PIN_6
#define RF3_DIO2_GPIO_Port GPIOH
#define RF3_DIO1_Pin GPIO_PIN_7
#define RF3_DIO1_GPIO_Port GPIOH
#define RF3_DIO0_EXIT_Pin GPIO_PIN_8
#define RF3_DIO0_EXIT_GPIO_Port GPIOH
#define RF3_DIO0_EXIT_EXTI_IRQn EXTI9_5_IRQn
#define RF3_RESETN_Pin GPIO_PIN_9
#define RF3_RESETN_GPIO_Port GPIOH
#define RF3_TRX_CTRL_Pin GPIO_PIN_10
#define RF3_TRX_CTRL_GPIO_Port GPIOH
#define RF3_EN_Pin GPIO_PIN_11
#define RF3_EN_GPIO_Port GPIOH
#define PWRKEY_M35_Pin GPIO_PIN_1
#define PWRKEY_M35_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOG
#define LED2_Pin GPIO_PIN_10
#define LED2_GPIO_Port GPIOG
#define LED3_Pin GPIO_PIN_11
#define LED3_GPIO_Port GPIOG
#define LED4_Pin GPIO_PIN_12
#define LED4_GPIO_Port GPIOG
#define LED5_Pin GPIO_PIN_13
#define LED5_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
