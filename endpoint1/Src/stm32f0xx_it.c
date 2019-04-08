/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "rtc.h"
#include "user_config.h"
#include "ultra.h"

extern LoraPacket gLoraPacket;
extern uint8_t uart2_rbuff[4];
extern Motor motor;
extern Device gDevice;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

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
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  /* USER CODE END EXTI4_15_IRQn 1 */
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
  if(__HAL_UART_GET_IT(&huart1,UART_IT_IDLE)!=RESET) {
    __HAL_UART_CLEAR_IT(&huart1, UART_CLEAR_IDLEF);
  }
  /* USER CODE END USART1_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
  if(__HAL_UART_GET_IT(&huart2,UART_IT_IDLE)!=RESET) {
    __HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_IDLEF);
  }
  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  if ( motor.ctrl_timer_cb ) {
    motor.ctrl_timer_cb();
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Lora_AUX_Pin == GPIO_Pin)
  { //AUX interrupt
    //只唤醒，不需要做额外动作。
  } else {
    //gpio回调函数
    if ( motor.gpio_cb ) {
      motor.gpio_cb(GPIO_Pin);
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t i,pos;
  uint8_t buffer[4];
  
  if ( huart->Instance == USART1 ) {
    LoraModuleReceiveHandler();
  } else if ( huart->Instance == USART2 ) {
    //寻找到超声波数据的头0xff
    for(i=0;i<4;i++) {
      if ( uart2_rbuff[i] == 0xff ) {
        pos = i;
      }
    }
    //数据复制到buffer里
    for(i=0;i<4;i++) {
      buffer[i] = uart2_rbuff[pos];
      pos++;
      if(pos == 4) 
        pos = 0;
    }
    //格式化后读取
    UltraSonicType *ust = (UltraSonicType *)buffer;
    if ( ust->head == 0xff ) {
      if ((( ust->head + ust->data_h + ust->data_l ) & 0xff) == ust->sum) {
        gDevice.u16UltraDistance = (((uint16_t)(ust->data_h)) << 8 | ust->data_l);
      }
    } else {
      gDevice.u16UltraDistance = 0xffff;
    }
    
    HAL_UART_Receive_IT(&huart2, uart2_rbuff, 4);
  }
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
