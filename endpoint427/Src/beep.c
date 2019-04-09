#include "beep.h"
#include "stdint.h"
#include "stm32f0xx.h"
#include "user_config.h"

static void beep_gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /*Configure GPIO pins : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

BEEP_RESP beep_ctrl(BEEP_CMD cmd)
{
  GPIO_PinState res;
  
  beep_gpio_init();
    
  if (BEEP_GET != cmd) {
    HAL_GPIO_WritePin(GPIO_BEEP, GPIO_BEEP_Pin, (GPIO_PinState)cmd);
//    HAL_Delay(5);
  }
  
  res = HAL_GPIO_ReadPin(GPIO_BEEP, GPIO_BEEP_Pin);
  
  switch(cmd)
  {
  case BEEP_OFF:
    if(GPIO_PIN_SET == res)
      return BEEP_FAIL;
    else
      return BEEP_OK;
    break;
  case BEEP_ON:
    if(GPIO_PIN_SET == res)
      return BEEP_OK;
    else
      return BEEP_FAIL;
    break;
  case BEEP_GET:
    if(GPIO_PIN_SET == res)
      return BEEP_GET_RESP_ON;
    else
      return BEEP_GET_RESP_OFF;
    break;
  default:
    return BEEP_FAIL;
  }
}
