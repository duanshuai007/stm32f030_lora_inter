#include "lowpower.h"
#include "stdint.h"
#include "stm32f0xx.h"
#include "lora.h"
#include "user_config.h"
#include "hardware.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc;

extern LoraPacket gLoraPacket;
extern Device gDevice;
extern uint8_t uart2_dma_rbuff[4];

//进入stop模式之前要重新根据当时的电平设置上拉下拉
static void set_gpio(GPIO_TypeDef* GPIOx, uint16_t pin, uint32_t mode)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = mode;
  if (HAL_GPIO_ReadPin(GPIOx, pin) == GPIO_PIN_SET) {
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
  } else {
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
  }
}

static void gpio_used_lp_init(void)
{
  /*
  *       GPIOA中涉及到引脚包括
  *       M0(PA8),K1(PA4),K2(PA5),K3传感器使能(PA6)
  *       PA8和PA6 是输出模式
  *       PA4和PA5 是中断模式
  *       在进入休眠前根据电平值设置上拉或下拉
  */
  set_gpio( GPIO_SENSOR_UP, GPIO_SENSOR_UP_PIN, GPIO_MODE_IT_RISING_FALLING);
  set_gpio( GPIO_SENSOR_DOWN, GPIO_SENSOR_DOWN_PIN, GPIO_MODE_IT_RISING_FALLING);
  set_gpio( GPIO_SENSOR_SWITCH, GPIO_SENSOR_SWITCH_PIN, GPIO_MODE_OUTPUT_PP);
  set_gpio( GPIO_Lora_M0, GPIO_Lora_M0_Pin, GPIO_MODE_OUTPUT_PP);
  /*
  *   GPIOB中涉及到的引脚包括
  *   (P)PB4,(N)PB5,(BEEP)PB10,(AUX)PB14,(M1)PB15
  *   PB4,PB5,PB10,PB15都是输出模式
  *   PB14是中断输入
  */
  set_gpio( GPIO_SENSOR_FORWARE, GPIO_SENSOR_FORWARE_PIN, GPIO_MODE_OUTPUT_PP);
  set_gpio( GPIO_SENSOR_BACKWARD, GPIO_SENSOR_BACKWARD_PIN, GPIO_MODE_OUTPUT_PP);
  set_gpio( GPIO_BEEP, GPIO_BEEP_Pin, GPIO_MODE_OUTPUT_PP);
  set_gpio( GPIO_Lora_M1, GPIO_Lora_M1_Pin, GPIO_MODE_OUTPUT_PP);
  set_gpio( GPIO_Lora_AUX, GPIO_Lora_AUX_Pin, GPIO_MODE_IT_RISING_FALLING);
}

//关闭使用的外设
//0 关闭所有
//1 保留adc 
//2 保留rtc lsi， motor指令
void LowPowerInit(bool mode, bool flag)
{   
  //必须停止，否则功耗会在1ma左右
  HAL_UART_Abort_IT(&huart1);
  HAL_UART_Abort_IT(&huart2);
  
  gpio_used_lp_init();
  
  if ((false == mode) && (false == flag)) {
    __HAL_RCC_RTC_DISABLE();
    __HAL_RCC_LSI_DISABLE(); 
  }
  
  __HAL_RCC_ADC1_CLK_DISABLE();
  __HAL_RCC_TIM6_CLK_DISABLE();
  __HAL_RCC_USART1_CLK_DISABLE();
  __HAL_RCC_USART2_CLK_DISABLE();
  __HAL_RCC_DMA1_CLK_DISABLE();
  
  __HAL_RCC_PWR_CLK_ENABLE();
  
  __HAL_RCC_HSI_DISABLE();
}

static void gpio_not_used_lp_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /*
  *       GPIOA中涉及到引脚包括
  *       M0(PA8),K1(PA4),K2(PA5),K3传感器使能(PA6)
  *       PA2,PA3 是串口2的引脚
  *       PA8和PA6 是输出模式
  *       PA4和PA5 是中断模式
  *       在进入休眠前根据电平值设置上拉或下拉
  */
  GPIO_InitStruct.Pin = (GPIO_PIN_All & (~( GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_6 )));
  //| GPIO_PIN_13 | GPIO_PIN_14 )));
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;    //在这种模式下电流最小
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
  //__HAL_RCC_GPIOA_CLK_DISABLE();
  
  /*
  *   GPIOB中涉及到的引脚包括
  *   (P)PB4,(N)PB5,(BEEP)PB10,(AUX)PB14,(M1)PB15
  *   PB4,PB5,PB10,PB15都是输出模式
  *   PB14是中断输入
  */
  GPIO_InitStruct.Pin = (GPIO_PIN_All & (~(GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15)));
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  //__HAL_RCC_GPIOB_CLK_DISABLE();
  
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;    //在这种模式下电流最小
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  __HAL_RCC_GPIOC_CLK_DISABLE();
  
  GPIO_InitStruct.Pin =  GPIO_PIN_All;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;    //在这种模式下电流最小
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  __HAL_RCC_GPIOF_CLK_DISABLE();
}

//关闭未使用的外设,
//仅仅在系统复位启动后调用一次
void CloseNotUsedPeriphClock(void)
{
  gpio_not_used_lp_init();
  
  __HAL_RCC_CRC_CLK_DISABLE();
  __HAL_RCC_I2C1_CLK_DISABLE();
  __HAL_RCC_I2C2_CLK_DISABLE();
  
  __HAL_RCC_SPI1_CLK_DISABLE();
  __HAL_RCC_SPI2_CLK_DISABLE();
  __HAL_RCC_SYSCFG_CLK_DISABLE();
  __HAL_RCC_TIM1_CLK_DISABLE();
  __HAL_RCC_TIM3_CLK_DISABLE();
  __HAL_RCC_TIM6_CLK_DISABLE();
  __HAL_RCC_TIM14_CLK_DISABLE();
  __HAL_RCC_TIM15_CLK_DISABLE();
  __HAL_RCC_TIM16_CLK_DISABLE();
  __HAL_RCC_TIM17_CLK_DISABLE();
  __HAL_RCC_USART2_CLK_DISABLE();
  __HAL_RCC_WWDG_CLK_DISABLE();
  
  __HAL_RCC_SRAM_CLK_DISABLE();
  __HAL_RCC_FLITF_CLK_DISABLE();
  __HAL_RCC_DBGMCU_CLK_DISABLE();
  
  __HAL_RCC_PWR_CLK_ENABLE();
  
  __HAL_RCC_RTC_DISABLE();
  __HAL_RCC_LSI_DISABLE();
  
  __HAL_RCC_HSI_DISABLE();
  __HAL_RCC_HSE_CONFIG(RCC_HSE_OFF);
  __HAL_RCC_LSE_CONFIG(RCC_LSE_OFF);
  __HAL_RCC_HSI14_DISABLE();
  __HAL_RCC_HSI14ADC_DISABLE();
  __HAL_RCC_PLL_DISABLE();
}

//唤醒后再次进入休眠时关闭使用过的外设
static void periph_used_in_motor_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /*Configure GPIO pins : PA4(K1) PA5(K2) */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /*Configure GPIO pins : PA6(K3) */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /*Configure GPIO pins : PB4(P) PB5(N) */
  GPIO_InitStruct.Pin = GPIO_PIN_4 |GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void periph_used_in_beep_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /*Configure GPIO pins : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void periph_used_in_adc_init(ADC_HandleTypeDef *hadc)
{
  ADC_ChannelConfTypeDef sConfig;
  
  __HAL_RCC_ADC1_CLK_ENABLE();
  
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc->Instance = ADC1;
  hadc->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc->Init.Resolution = ADC_RESOLUTION_8B;
  hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc->Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc->Init.LowPowerAutoWait = DISABLE;
  hadc->Init.LowPowerAutoPowerOff = ENABLE;
  hadc->Init.ContinuousConvMode = DISABLE;
  hadc->Init.DiscontinuousConvMode = DISABLE;
  hadc->Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc->Init.DMAContinuousRequests = DISABLE;
  hadc->Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  /**Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

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
  
  HAL_UART_Receive_DMA(&huart2, uart2_dma_rbuff, 4);
  //使能空闲中断，仅对接收起作用
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}

void UART_ReInit(UART_HandleTypeDef *huart)
{
  __HAL_RCC_USART1_CLK_ENABLE();
  HAL_UART_DeInit(huart);
  huart->Instance = USART1;
  huart->Init.BaudRate = 57600;
  huart->Init.WordLength = UART_WORDLENGTH_9B;
  huart->Init.StopBits = UART_STOPBITS_1;
  huart->Init.Parity = UART_PARITY_ODD;
  huart->Init.Mode = UART_MODE_TX_RX;
  huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart->Init.OverSampling = UART_OVERSAMPLING_16;
  huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(huart);
}

void InitPeriphByCMD(void)
{
  switch(gDevice.u8Cmd)
  {
  case HW_MOTOR_UP:
    uart2_reinit();
  case HW_MOTOR_DOWN:
  case HW_MOTOR_GET:
    //驱动电机用到的外设资源包括，gpio(PB4\5,PA4\5\6,TIM6)
    periph_used_in_motor_init();
    break;
  case HW_BEEP_ON:
  case HW_BEEP_OFF:
  case HW_BEEP_GET:
    periph_used_in_beep_init();
    //蜂鸣器用到外设资源是gpio，(PB10)
    break;
  case HW_ADC_GET:
    //adc
    periph_used_in_adc_init(&hadc);
    break;
  default:
    //error
    break;
  }
}

//0 表示空指令，错误状态
//1 异步指令执行完成
//2 指令执行中接收到新指令，繁忙
uint8_t SyncCMDDone(void)
{
  uint8_t ret = 0;
  //异步指令执行完成后发送resp信息
  if (HW_CMD_NONE != gDevice.u8Cmd) {
    
    if ((CMD_RUN == gDevice.u8CmdRunning) && ( 1 == gDevice.u8CmdDone)) {
    
      LoraTransfer(NORMALCMD);
      
      gDevice.u8CmdDone = 0;
      //发送完成，清空状态标志信息，准备再次接收执行指令
      if(gDevice.u8ReCmdFlag)
        gDevice.u8ReCmdFlag = 0;
      
      gDevice.u8Cmd = HW_CMD_NONE;
      gDevice.u8Resp = 0;
      gDevice.u8CmdRunning = CMD_STOP;
      
      ret = 1;
    } else if (gDevice.u8ReCmdFlag) {
      //在指令执行过程中接收到新指令，返回繁忙状态
      gDevice.u8ReCmdFlag = 0;
      gDevice.u8Resp = HW_BUSY;
      
      LoraTransfer(RECMD);
      
      ret = 2;
    }
  }
  
  return ret;
}

bool ExecCMD(void)
{   
  bool mode = false;
  
  if (HW_CMD_NONE != gDevice.u8Cmd) {
    
    //执行指令，在执行指令之前先初始化对应功能所用到的外设资源
    InitPeriphByCMD();
    
    if ((gDevice.u8Cmd != HW_CMD_NONE) && (gDevice.u8CmdRunning == CMD_STOP)) {
      
      gDevice.u8CmdRunning = CMD_RUN;
      
      uint8_t ret = hardware_ctrl((HW_CMD)gDevice.u8Cmd);
      
      if (gDevice.u8Cmd > HW_MOTOR_DOWN ) {
        //give server some response
        gDevice.u8Resp = ret;
        LoraTransfer(NORMALCMD);
        //指令执行结束，清空状态信息
        gDevice.u8Cmd = HW_CMD_NONE;
        gDevice.u8Resp = 0;
        gDevice.u8CmdRunning = CMD_STOP;
      } else if ((gDevice.u8Cmd == HW_MOTOR_UP) || (gDevice.u8Cmd == HW_MOTOR_DOWN)) {
        //返回指令开始执行的resp
        //该段代码用于电机执行动作后返回电机开始动作的resp
        if ( ret == MOTOR_OK ) {
          //正常执行动作后立刻返回一个响应信息
          gDevice.u8Resp = MOTOR_RUN;
          LoraTransfer(NORMALCMD);
        }
        else if (ret == MOTOR_DONTDO) {
          //电机处于目标状态不需要执行动作
          LoraTransfer(NORMALCMD);
          gDevice.u8Cmd = HW_CMD_NONE;
          gDevice.u8Resp = 0;
          gDevice.u8CmdRunning = CMD_STOP;
          gDevice.u8CmdDone = 0;
        } else if (( ret == MOTOR_ERROR) || (ret == MOTOR_ERROR_ULTRA)) {
          if (ret == MOTOR_ERROR)
            gDevice.u8Resp = MOTOR_CANTUP;
          else if (ret == MOTOR_ERROR_ULTRA)
            gDevice.u8Resp = MOTOR_CANTUP_ULTRA;
          
          LoraTransfer(NORMALCMD);
          gDevice.u8Cmd = HW_CMD_NONE;
          gDevice.u8Resp = 0;
          gDevice.u8CmdRunning = CMD_STOP;
          gDevice.u8CmdDone = 0;
        }
      }
    }
    
    if ((gDevice.u8Cmd == HW_MOTOR_UP) || (gDevice.u8Cmd == HW_MOTOR_DOWN)) {
      mode = true;
    } else {
      mode = false;
    }
  }
  
  return mode;
}
