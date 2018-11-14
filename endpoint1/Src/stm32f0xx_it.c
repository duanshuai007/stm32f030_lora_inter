/**
******************************************************************************
* @file    stm32f0xx_it.c
* @brief   Interrupt Service Routines.
******************************************************************************
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#include "stm32f0xx_it.h"

/* USER CODE BEGIN 0 */
#include "stm32f0xx_hal_rtc.h"
#include "Lora.h"
#include "motor.h"

extern LoraPacket gLoraPacket;
extern RTC_HandleTypeDef hrtc;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;

/******************************************************************************/
/*            Cortex-M0 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */
  
  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */
  
  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */
  
  /* USER CODE END SVC_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */
  
  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */
  
  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles RTC global interrupt through EXTI lines 17, 19 and 20.
*/
void RTC_IRQHandler(void)
{
  /* USER CODE BEGIN RTC_IRQn 0 */
  
  /* USER CODE END RTC_IRQn 0 */
  HAL_RTC_AlarmIRQHandler(&hrtc);
  /* USER CODE BEGIN RTC_IRQn 1 */
  
  /* USER CODE END RTC_IRQn 1 */
}

/**
* @brief This function handles EXTI line 4 to 15 interrupts.
*/
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */
  
  /* USER CODE END EXTI4_15_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */
  
  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel 2 and 3 interrupts.
*/
void DMA1_Channel2_3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */
  
  /* USER CODE END DMA1_Channel2_3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */
  
  /* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

/**
* @brief This function handles TIM6 global interrupt.
*/
void TIM6_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */
  
  /* USER CODE END TIM6_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_IRQn 1 */
  
  /* USER CODE END TIM6_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  //空闲中断，仅仅接收会触发空闲中断
  if(__HAL_UART_GET_IT(&huart1,UART_IT_IDLE)!=RESET)
  {
    __HAL_UART_CLEAR_IT(&huart1, UART_CLEAR_IDLEF);
    CopyDataFromDMA();
  }
  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

extern Motor motor;

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
  //    motor.xiaodou_timer_cb();
//}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  if ( motor.ctrl_timer_cb ) {
    motor.ctrl_timer_cb();
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  //AUX interrupt
  
  if(GPIO_Lora_AUX_Pin == GPIO_Pin)
  {   //AUX CHANGE
    if ( HAL_GPIO_ReadPin(GPIO_Lora_AUX, GPIO_Lora_AUX_Pin) == GPIO_PIN_SET ) {
      gLoraPacket.isIdle = 1;
    } else {
      gLoraPacket.isIdle = 0;
    }
  } else {
    //gpio回调函数
    if ( motor.gpio_cb ) {
      motor.gpio_cb(GPIO_Pin);
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  CopyDataFromDMA();
  HAL_UART_Receive_DMA(&huart1, gLoraPacket.dma_rbuff, MAX_DMA_LEN);
}

//错误处理
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uint32_t no;
  /*
#define HAL_UART_ERROR_PE        (0x00000001U)     Parity error        
#define HAL_UART_ERROR_NE        (0x00000002U)     Noise error         
#define HAL_UART_ERROR_FE        (0x00000004U)     frame error         
#define HAL_UART_ERROR_ORE       (0x00000008U)     Overrun error       
#define HAL_UART_ERROR_DMA       (0x00000010U)     DMA transfer error  
#define HAL_UART_ERROR_BUSY
  */
  no = HAL_UART_GetError(huart);
  //    printf("usart errno no = %d\r\n", no);
  if( no & HAL_UART_ERROR_FE) {
    //frame error
    __HAL_UART_CLEAR_FEFLAG(huart);
  }
  if (no & HAL_UART_ERROR_PE) {
    //parity error
    __HAL_UART_CLEAR_PEFLAG(huart);
  }
  if (no & HAL_UART_ERROR_NE) {
    __HAL_UART_CLEAR_NEFLAG(huart);
  }
  if ( no & HAL_UART_ERROR_ORE ) {
    __HAL_UART_CLEAR_OREFLAG(huart);
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  uint32_t no;
  /*
#define HAL_ADC_ERROR_NONE        (0x00U)    No error
#define HAL_ADC_ERROR_INTERNAL    (0x01U)    ADC IP internal error: if problem of clocking,
  enable/disable, erroneous state
#define HAL_ADC_ERROR_OVR         (0x02U)    Overrun error 
#define HAL_ADC_ERROR_DMA         (0x04U)    DMA transfer error 
  */ 
  
  no = HAL_ADC_GetError(hadc);
  
  if ( no & HAL_ADC_ERROR_OVR ) {
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_OVR);
  }
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
