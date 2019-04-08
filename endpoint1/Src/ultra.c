#include "ultra.h"
#include "lowpower.h"
#include "user_config.h"
#include <stdint.h>

uint8_t uart2_rbuff[4];

extern Device gDevice;
extern UART_HandleTypeDef huart2;

static void uart2_reinit(void)
{
  __HAL_RCC_USART2_CLK_ENABLE();
  HAL_UART_DeInit(&huart2);
  
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);
  
  HAL_UART_Receive_IT(&huart2, uart2_rbuff, 4);
  //使能空闲中断，仅对接收起作用
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}

uint8_t ReadUltraData(void)
{
  uint16_t ret;
  
  uart2_reinit();
  
  gDevice.u16UltraDistance = 0;
  
  HAL_GPIO_WritePin(GPIO_ULTRASONIC, GPIO_ULTRASONIC_PIN, GPIO_PIN_RESET);
  HAL_Delay(500); //测试时所测到的超声波能测到数据的最小时长
  //禁止超声波电源
  HAL_GPIO_WritePin(GPIO_ULTRASONIC, GPIO_ULTRASONIC_PIN, GPIO_PIN_SET);
  
  if ((gDevice.u16UltraDistance == 0xffff) || (gDevice.u16UltraDistance == 0)) {
    ret = 0xff;
  } else {
    ret = (gDevice.u16UltraDistance/10);  //转换为厘米单位
    if (ret > 0xff)   //防止数据溢出
      ret = 200;
  }
  
  return (uint8_t)ret;
}