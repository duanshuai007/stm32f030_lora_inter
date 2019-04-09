#include "ultra.h"
#include "lowpower.h"
#include "user_config.h"
#include "motor.h"
#include <stdint.h>

uint8_t uart2_rbuff[40] = {0};

extern Device gDevice;
extern UART_HandleTypeDef huart2;
extern Motor gMotor;

static uint16_t UltraModuleRecviveHandler(void) 
{
  int i = 0;
  int iLastPos = -1;
  uint16_t au16Result[3] = {0};

  // find the last 0xFF head
  for(i = sizeof(uart2_rbuff); i >=0; i--)
  {
    if (ULTRA_MSG_HEAD == uart2_rbuff[i])
    {
      iLastPos = i;
      if (i - 3 >= 0 && ULTRA_MSG_HEAD == uart2_rbuff[i - 3]) // FF 00 00 FF
      {
        iLastPos = i - 3;
      }
      break;
    }
  }
  
  i = 0;
  while (iLastPos >= 0)
  {
    if (ULTRA_MSG_HEAD != uart2_rbuff[iLastPos])
    {
      break;
    }
    UltraSonicType *ust = (UltraSonicType *)(uart2_rbuff + iLastPos);
    if ((( ust->head + ust->data_h + ust->data_l ) & 0xff) == ust->sum)
    {
      au16Result[i++] = (((uint16_t)(ust->data_h)) << 8 | ust->data_l);
    }
    if (3 == i) // find the last 3 packets
    {
      break;
    }

    iLastPos -= 4;
  }
  
  if (1 == i)
  {
    return au16Result[0];
  }
  else if (2 == i)
  {
    return au16Result[1];
  }
  else // i == 3
  {
    uint16_t u16Min = au16Result[0];
    uint16_t u16Max = au16Result[0];
    uint8_t u8MinPos = 0;
    uint8_t u8MaxPos = 0;
    for (i = 1; i < 3; i ++)
    {
      if (u16Min > au16Result[i])
      {
        u16Min = au16Result[i];
        u8MinPos = i;
      }
      if (u16Max < au16Result[i])
      {
        u16Max = au16Result[i];
        u8MaxPos = i;
      }
    }
    for (i = 0; i < 3; i++)
    {
      if (i != u8MinPos && i != u8MaxPos)
      {
        break;
      }
    }
    return au16Result[i];
  }
}

void UltraUartEnable(void)
{
  
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
}

void UltraUartDisable(void)
{
//  HAL_UART_Abort_IT(&huart2);
  HAL_UART_DeInit(&huart2);
}

/*
*     读取超声波的距离值
*     在读取完之后，关闭超声波电源后需要延时一小段时间，才能关闭串口，否则会
*     产生错误，导致程序卡死复位
*/
uint8_t ReadUltraData(void)
{
  uint16_t ret = 0;
  HAL_StatusTypeDef hStatus;
  
  UltraUartEnable();
  
  //使能超声波电源
  HAL_GPIO_WritePin(GPIO_ULTRASONIC, GPIO_ULTRASONIC_PIN, GPIO_PIN_RESET);

  hStatus = HAL_UART_Receive(&huart2, uart2_rbuff, sizeof(uart2_rbuff), 2000);
  
//  if (HAL_OK != HAL_UART_Receive(&huart2, uart2_rbuff, sizeof(uart2_rbuff), 2000))
//  {
//    //禁止超声波电源
//    HAL_GPIO_WritePin(GPIO_ULTRASONIC, GPIO_ULTRASONIC_PIN, GPIO_PIN_SET);
//    HAL_Delay(5);
//    UltraUartDisable();
//    return ULTRA_READ_ERROR;
//  }

  //禁止超声波电源
  HAL_GPIO_WritePin(GPIO_ULTRASONIC, GPIO_ULTRASONIC_PIN, GPIO_PIN_SET);
  HAL_Delay(5);
  UltraUartDisable();

  if (hStatus != HAL_OK)
    return ULTRA_READ_ERROR;
  
  ret = UltraModuleRecviveHandler();
  
  ret = ret / 10;  //转换为厘米单位
  if (ret > 200)   //防止数据溢出
    ret = 200;
  return (uint8_t)ret;
}

uint8_t UltraCheckTask(void)
{
//  uint8_t result = 0;
//  result = ReadUltraData();
//  
//  return result;
  return ReadUltraData();
}