#include "lowpower.h"
#include "stdint.h"
#include "stm32f0xx.h"
#include "lora.h"
#include "user_config.h"
#include "hardware.h"
#include "motor.h"
#include "rtc.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern Device gDevice;

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

static void set_gpio_analog(GPIO_TypeDef* GPIOx, uint16_t pin)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;

  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

static void gpio_used_lp_init(void)
{
  //关闭超声波模块电源
  HAL_GPIO_WritePin(GPIO_ULTRASONIC, GPIO_ULTRASONIC_PIN, GPIO_PIN_SET);
  
  /*
  *       GPIOA中涉及到引脚包括
  *       M0(PA8),K1(PA4),K2(PA5),K3传感器使能(PA6)
  *       PA8和PA6 是输出模式
  *       PA4和PA5 是中断模式
  *       在进入休眠前根据电平值设置上拉或下拉
  */
#if 0
  //位置检测引脚带有上拉电阻，不设置,如果设置了会电流增大到900ua
  set_gpio( GPIO_SENSOR_UP, GPIO_SENSOR_UP_PIN, GPIO_MODE_OUTPUT_PP);
  set_gpio( GPIO_SENSOR_DOWN, GPIO_SENSOR_DOWN_PIN, GPIO_MODE_OUTPUT_PP);
  set_gpio( GPIO_ULTRASONIC, GPIO_ULTRASONIC_PIN, GPIO_MODE_OUTPUT_PP);
#endif

  //lora模块的模式选择引脚，这两个引脚必须设置为输出
//  set_gpio( GPIO_Lora_M0, GPIO_Lora_M0_Pin, GPIO_MODE_OUTPUT_PP);
//  set_gpio( GPIO_Lora_M1, GPIO_Lora_M1_Pin, GPIO_MODE_OUTPUT_PP);
  
  /*
  *   GPIOB中涉及到的引脚包括
  *   (P)PB4,(N)PB5,(BEEP)PB10,(AUX)PB14,(M1)PB15
  *   PB4,PB5,PB10,PB15都是输出模式
  *   PB14是中断输入
  */
  if ((gDevice.u8Cmd == HW_MOTOR_UP) || (gDevice.u8Cmd == HW_MOTOR_DOWN)) {
    set_gpio(GPIO_SENSOR_FORWARE, GPIO_SENSOR_FORWARE_PIN, GPIO_MODE_OUTPUT_PP);
    set_gpio(GPIO_SENSOR_BACKWARD, GPIO_SENSOR_BACKWARD_PIN, GPIO_MODE_OUTPUT_PP);
    //这个引脚不能设置为输入，设置为输入就会导致电机接收到指令后不动
    set_gpio(GPIO_SENSOR_SWITCH, GPIO_SENSOR_SWITCH_PIN, GPIO_MODE_OUTPUT_PP);
  } else {
    set_gpio_analog(GPIO_SENSOR_FORWARE, GPIO_SENSOR_FORWARE_PIN);
    set_gpio_analog(GPIO_SENSOR_BACKWARD, GPIO_SENSOR_BACKWARD_PIN);
    set_gpio_analog(GPIO_SENSOR_SWITCH, GPIO_SENSOR_SWITCH_PIN);
  }
  
//  set_gpio_analog(GPIO_BEEP, GPIO_BEEP_Pin);
  set_gpio( GPIO_BEEP, GPIO_BEEP_Pin, GPIO_MODE_OUTPUT_PP);
  
  set_gpio( GPIO_Lora_AUX, GPIO_Lora_AUX_Pin, GPIO_MODE_IT_RISING_FALLING);
}

//关闭使用的外设
void LowPowerInit(Device *d)
{   
  //必须停止，否则功耗会在1ma左右
  HAL_UART_Abort_IT(&huart1);
  HAL_UART_Abort_IT(&huart2);
  
  gpio_used_lp_init();
  
  __HAL_RCC_ADC1_CLK_DISABLE();
  __HAL_RCC_TIM6_CLK_DISABLE();
  __HAL_RCC_USART1_CLK_DISABLE();
  __HAL_RCC_USART2_CLK_DISABLE();
  __HAL_RCC_DMA1_CLK_DISABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_HSI_DISABLE();
  
  //设置1秒定时唤醒,只有在电机未使用rtc的时候才使用rtc定时
  if (d->bMotorUseRTC == false)
    rtc_set_timer(6);
  
  __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_14);
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
  GPIO_InitStruct.Pin = (GPIO_PIN_All 
                         & (~( GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_6 )));
  //| GPIO_PIN_13 | GPIO_PIN_14 )));
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;    //在这种模式下电流最小
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
  //__HAL_RCC_GPIOA_CLK_DISABLE();
  
  /*
  *   GPIOB中涉及到的引脚包括
  *   (P)PB4,(N)PB5,(BEEP)PB10,(AUX)PB14,(M1)PB15, (ULTRASONIC PIN)PB13
  *   PB4,PB5,PB10,PB15都是输出模式
  *   PB14是中断输入
  */
  GPIO_InitStruct.Pin = (GPIO_PIN_All 
                         & (~(GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_10 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)));
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

void InitPeriphByCMD(uint8_t cmd)
{
  switch(cmd)
  {
  case HW_MOTOR_UP:
  case HW_MOTOR_DOWN:
  case HW_MOTOR_GET:
  case HW_ULTRA_GET:
    GPIO_Motor_Init(cmd);
    GPIO_MotorSenserInit();
    //开启位置传感器的开关
    MotorSenceSwitch(true);
    //驱动电机用到的外设资源包括，gpio(PB4\5,PA4\5\6,TIM6)
    break;
  case HW_BEEP_ON:
  case HW_BEEP_OFF:
  case HW_BEEP_GET:
    periph_used_in_beep_init();
    //蜂鸣器用到外设资源是gpio，(PB10)
    break;
  case HW_ADC_GET:
    //adc
//    periph_used_in_adc_init(&hadc);
    break;
  default:
    //error
    break;
  }
}

void SyncCMDDone(Device *d)
{
  uint8_t sbuff[32];
  uint8_t len;
  MsgDevice md;
  //异步指令执行完成后发送resp信息
  if (HW_CMD_NONE == d->u8Cmd)
    return;
  
  if (CMD_RUN != d->u8CmdRunning)
    return;
  
  if ( CMD_EXEC_DONE == d->u8CmdDone) {
    
    //电机指令执行完成或是超时，都需要将位置传感器关闭
    if ((d->u8Cmd == HW_MOTOR_UP) || (d->u8Cmd == HW_MOTOR_DOWN)) {
      GPIO_MotorSenserInit();  
      MotorSenceSwitch(false);
    }
    
    md.u8Cmd = d->u8Cmd;
    md.u8Resp = d->u8MotorResp;
    md.u32Identify = d->u32Identify;
    
    len = GetSendData(sbuff, &md);
    LoraSend(sbuff, len);
    
    d->u8CmdDone = CMD_EXEC_NONE;
    d->u8Cmd = HW_CMD_NONE;
    d->u8MotorResp = 0;
    d->u8CmdRunning = CMD_STOP;

  } else if ( CMD_EXEC_DOING == d->u8CmdDone) {
    //对应前倾状态先落下后抬起
    d->u8CmdRunning = CMD_STOP;
  } else {  //CMD_EXEC_NONE
    //do nothing
  }
}

void ExecCMD(Device *d)
{   
  uint8_t len;
  uint8_t sbuff[32];
  MsgDevice md;
  
  if (d->u8Cmd == HW_CMD_NONE)
    return;
  
  if (CMD_STOP != d->u8CmdRunning)
    return;
  
  //判断指令是否支持
  if ((d->u8Cmd >= HW_CMD_MAXVALUE) && (d->u8Cmd != HW_DEVICE_HEART)) {
    d->u8Cmd = HW_CMD_NONE;
    return;
  }
  //执行指令，在执行指令之前先初始化对应功能所用到的外设资源
  InitPeriphByCMD(d->u8Cmd);  
  
  d->u8CmdRunning = CMD_RUN;
  
  uint8_t ret = hardware_ctrl(d);
  
  switch(d->u8Cmd) {
  case HW_MOTOR_UP:
  case HW_MOTOR_DOWN:
    //该段代码用于电机执行动作后返回电机开始动作的resp
    switch(ret) {
    case MOTOR_OK:
      //正常执行动作后立刻返回一个响应信息
      if (d->u8CmdDone != CMD_EXEC_DOING) { 
        md.u8Cmd = d->u8Cmd;
        md.u8Resp = MOTOR_RUN;
        md.u32Identify = d->u32Identify;
        len = GetSendData(sbuff, &md);
        LoraSend(sbuff, len);
      }
      break;
    case MOTOR_DONTDO:
      //电机处于目标位置不需要执行动作,直接关闭位置传感器
      MotorSenceSwitch(false);
      md.u8Cmd = d->u8Cmd;
      md.u8Resp = 0;
      md.u32Identify = d->u32Identify;
      len = GetSendData(sbuff, &md);
      LoraSend(sbuff, len);
      d->u8CmdRunning = CMD_STOP;
      d->u8CmdDone = CMD_EXEC_NONE;
      d->u8Cmd = HW_CMD_NONE;
      break;
    case MOTOR_ERROR:
    case MOTOR_ERROR_ULTRA:
      //超声波状态错误，直接关闭位置传感器
      MotorSenceSwitch(false);
      md.u8Cmd = d->u8Cmd;
      md.u8Resp = ret;
      md.u32Identify = d->u32Identify;
      len = GetSendData(sbuff, &md);
      LoraSend(sbuff, len);
      d->u8CmdRunning = CMD_STOP;
      d->u8CmdDone = CMD_EXEC_NONE;
      d->u8Cmd = HW_CMD_NONE;
      break;
    case MOTOR_OK_NOSEND:
      break;
    default:
      break;
    }
    break;
  case HW_MOTOR_GET:
    //电机状态获取完毕，关闭位置传感器
    MotorSenceSwitch(false);
  case HW_BEEP_ON:
  case HW_BEEP_OFF:
  case HW_BEEP_GET:
  case HW_ADC_GET:
  case HW_DEVICE_HEART:
  case HW_ULTRA_GET:
  case HW_ULTRA_SAFE_SET:
  case HW_ULTRA_SAFE_GET:
    //give server some response
    md.u8Cmd = d->u8Cmd;
    md.u8Resp = ret;
    md.u32Identify = d->u32Identify;
    len = GetSendData(sbuff, &md);
    LoraSend(sbuff, len);
    //指令执行结束，清空状态信息
    d->u8CmdRunning = CMD_STOP;
    d->u8Cmd = HW_CMD_NONE;
    break;
  default:
    d->u8CmdRunning = CMD_STOP;
    d->u8Cmd = HW_CMD_NONE;
    break;
  }
}
