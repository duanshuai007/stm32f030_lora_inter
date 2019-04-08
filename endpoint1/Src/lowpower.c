#include "lowpower.h"
#include "stdint.h"
#include "stm32f0xx.h"
#include "lora.h"
#include "user_config.h"
#include "hardware.h"
#include "motor.h"
#include "rtc.h"
#include "beep.h"
#include "ultra.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern Device gDevice;

static void set_gpio_mode(GPIO_TypeDef* GPIOx, uint16_t pin, uint32_t mode, uint32_t pull)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = mode;
  if (GPIO_MODE_ANALOG == mode) {
    //如果是模拟功能引脚，不配置上下拉
  } else {
    if (GPIO_NOPULL == pull) {
      GPIO_InitStruct.Pull = GPIO_NOPULL;
    } else {
      if (HAL_GPIO_ReadPin(GPIOx, pin) == GPIO_PIN_SET) {
        GPIO_InitStruct.Pull = GPIO_PULLUP;
      } else {
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
      }
    }
  }
  
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

static void gpio_used_lp_init(void)
{
  //lora模块的模式选择引脚，这两个引脚必须设置为输出,
  //如果设置了这两个引脚就会引起切换模式出现一些异常，导致数据发送出错
  set_gpio_mode( GPIO_Lora_M0, GPIO_Lora_M0_Pin, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);
  set_gpio_mode( GPIO_Lora_M1, GPIO_Lora_M1_Pin, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);
  
  /*
  *   GPIOB中涉及到的引脚包括
  *   (P)PB4,(N)PB5,(BEEP)PB10,(AUX)PB14,(M1)PB15
  *   PB4,PB5,PB10,PB15都是输出模式
  *   PB14是中断输入
  */
  //超声波使能引脚 低电平使能，外部电路带有上拉电阻
  //关闭超声波模块电源,
  HAL_GPIO_WritePin(GPIO_ULTRASONIC, GPIO_ULTRASONIC_PIN, GPIO_PIN_SET);
  set_gpio_mode(GPIO_ULTRASONIC, GPIO_ULTRASONIC_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
  
//  HAL_GPIO_WritePin(GPIO_BEEP, GPIO_BEEP_Pin, GPIO_PIN_RESET);
  set_gpio_mode(GPIO_BEEP, GPIO_BEEP_Pin, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
  
  set_gpio_mode(GPIO_Lora_AUX, GPIO_Lora_AUX_Pin, GPIO_MODE_IT_RISING_FALLING, GPIO_PULLUP);
  
  if ((gDevice.u8Cmd == HW_MOTOR_UP) || (gDevice.u8Cmd == HW_MOTOR_DOWN)) {
    set_gpio_mode(GPIO_MOTOR_FORWARE, GPIO_MOTOR_FORWARE_PIN, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);
    set_gpio_mode(GPIO_MOTOR_BACKWARD, GPIO_MOTOR_BACKWARD_PIN, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);
    set_gpio_mode(GPIO_SENSOR_SWITCH, GPIO_SENSOR_SWITCH_PIN, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);
  } else {
    //电机控制引脚，在不使用的时候设置为模拟输入
    set_gpio_mode(GPIO_MOTOR_FORWARE, GPIO_MOTOR_FORWARE_PIN, GPIO_MODE_ANALOG, GPIO_NOPULL);
    set_gpio_mode(GPIO_MOTOR_BACKWARD, GPIO_MOTOR_BACKWARD_PIN, GPIO_MODE_ANALOG, GPIO_NOPULL);
    
    //传感器使能引脚，电路上有上拉电阻，设置为无上拉电阻输出模式
    set_gpio_mode(GPIO_SENSOR_SWITCH, GPIO_SENSOR_SWITCH_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    HAL_GPIO_WritePin(GPIO_SENSOR_SWITCH, GPIO_SENSOR_SWITCH_PIN, GPIO_PIN_RESET);
    //位置检测引脚带有上拉电阻，外部电路带有上拉电阻
    set_gpio_mode(GPIO_SENSOR_UP, GPIO_SENSOR_UP_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    HAL_GPIO_WritePin(GPIO_SENSOR_UP, GPIO_SENSOR_UP_PIN, GPIO_PIN_SET);

    set_gpio_mode(GPIO_SENSOR_DOWN, GPIO_SENSOR_DOWN_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    HAL_GPIO_WritePin(GPIO_SENSOR_DOWN, GPIO_SENSOR_DOWN_PIN, GPIO_PIN_SET);
  }
}

extern void SystemClock_Config(void);

void EnterLowPower(void)
{
  gpio_used_lp_init();

  if (gDevice.bMotorUseRTCInter == false) {
    //电机未使用rtc
    if (rtcSetTimer(SYSTEM_RTC_CYC) == false)
      return;
  } else {
    //电机使用rtc，则按照电机设置的rtc时间来设置
    if (gDevice.u8MotorRtcTimerInter != 0) {
      if (rtcSetTimer(gDevice.u8MotorRtcTimerInter) == false)
        return;
      gDevice.u8MotorRtcTimerInter = 0;
    }
    else {
      //这里不能配置rtc，因为电机运动中会持续触发外部
      //中断，如果这里进行配置就会把原来超时定时给覆盖掉
      return;
    }
  }
  
  //设置rtc时钟，低功耗配置成功，进入休眠
  if ((false == gDevice.bHasLoraInter )
      && (false == gDevice.bHasMotorNormalInter )
        && (false == gDevice.bHasRtcInter ))
  {
    //使用到的外设包括，串口1，2，adc，rtc，gpio，iwdg，
    //rtc，iwdg不需要关闭
    //关闭串口1，2的时钟，adc的时钟
    LoraUartDisable();
//    UltraUartDisable();
  
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_HSI_DISABLE();
//    __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);
//  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_14);
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    //如果不成功则重新进入正常工作模式
    //时钟配置清空
    HAL_RCC_DeInit();
    //HSI时钟使能
    __HAL_RCC_DISABLE_IT(RCC_IT_HSIRDY);
    __HAL_RCC_HSI_ENABLE();
    while(!__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY));
    //配置时钟和DMA
    SystemClock_Config();
    //串口重新配置  
    LoraUartEnable();
  }
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
  
  __HAL_RCC_TIM6_CLK_DISABLE();
  __HAL_RCC_DMA1_CLK_DISABLE();
  //rtc全局使能，不关闭
//  __HAL_RCC_RTC_DISABLE();
//  __HAL_RCC_LSI_DISABLE();
  
  __HAL_RCC_HSI_DISABLE();
  __HAL_RCC_HSE_CONFIG(RCC_HSE_OFF);
  __HAL_RCC_LSE_CONFIG(RCC_LSE_OFF);
  __HAL_RCC_HSI14_DISABLE();
  __HAL_RCC_HSI14ADC_DISABLE();
  __HAL_RCC_PLL_DISABLE();
}

void SyncCMDDone(Device *pDev)
{
  //异步指令执行完成后发送resp信息
  if (HW_CMD_NONE == pDev->u8Cmd) 
    return;
  
  if (CMD_RUN != pDev->u8CmdRunning) 
    return;
  
  uint8_t resp = pDev->u8MotorResp;
  
  if ( CMD_EXEC_DONE == pDev->u8CmdDone) {
    
    pDev->u32LastHeartTime = pDev->u32GlobalTimeCount;
    pDev->u32LastCheckUltraTime = pDev->u32GlobalTimeCount;
    //电机指令执行完成或是超时，都需要将位置传感器关闭
    if ((pDev->u8Cmd == HW_MOTOR_UP) || (pDev->u8Cmd == HW_MOTOR_DOWN)) {
      cancelMotorInterrupt();
    }
    
    LoraSendResp(pDev->u8Cmd, resp, pDev->u32Identify);
    
    pDev->u8CmdDone = CMD_EXEC_NONE;
    pDev->u8Cmd = HW_CMD_NONE;
    pDev->u8MotorResp = 0;
    pDev->u8CmdRunning = CMD_STOP;
  } else if ( CMD_EXEC_DOING == pDev->u8CmdDone) {
    //对应前倾状态先落下后抬起
    pDev->u8CmdRunning = CMD_STOP;
  } else {  //CMD_EXEC_NONE
    //do nothing
  }
}

void ExecCMD(Device *pDev)
{   
  if (pDev->u8Cmd == HW_CMD_NONE)
    return;
  
  if (CMD_STOP != pDev->u8CmdRunning)
    return;
  
  //判断指令是否支持
  if ((pDev->u8Cmd >= HW_CMD_MAXVALUE) && (pDev->u8Cmd != HW_DEVICE_HEART)) {
    pDev->u8Cmd = HW_CMD_NONE;
    return;
  }
  
  pDev->u8CmdRunning = CMD_RUN;
  
  //重置心跳计时器
  pDev->u32LastHeartTime = pDev->u32GlobalTimeCount;
  pDev->u32LastCheckUltraTime = pDev->u32GlobalTimeCount;
  
  hardware_ctrl(pDev);
}

void doCheckUltraByTimer(uint8_t u8TimeOut)
{
  uint32_t u32GlobalTime = gDevice.u32GlobalTimeCount; 
  uint32_t u32LastTime = gDevice.u32LastCheckUltraTime;
  
  if (((u32GlobalTime > u32LastTime) && ((u32GlobalTime - u32LastTime) >= u8TimeOut)) || 
      ((u32GlobalTime < u32LastTime) && ((u32GlobalTime + 24 * 3600 - u32LastTime) >= u8TimeOut)))
  {
    //地锁卧倒时，周期检测超声波。
    gDevice.u32LastCheckUltraTime = gDevice.u32GlobalTimeCount;
    LoraSendResp(HW_ULTRA_GET, UltraCheckTask(), 0xffffffff);
  }
}

void doHeartByTimer(uint8_t u8TimeOut, MOTOR_STATUS mStatus)
{
  uint32_t u32GlobalTime = gDevice.u32GlobalTimeCount; 
  uint32_t u32LastTime = gDevice.u32LastHeartTime;

  if (((u32GlobalTime > u32LastTime) && (u32GlobalTime - u32LastTime >= u8TimeOut)) || 
      ((u32GlobalTime < u32LastTime) && (u32GlobalTime + 24 * 3600 - u32LastTime >= u8TimeOut)))
  {
    gDevice.u32LastHeartTime = gDevice.u32GlobalTimeCount;
    //发送心跳包
//    LoraSendMsg(HW_DEVICE_HEART, ((mStatus << 4) | ULTRA_NO_READ), gDevice.u32Identify);
    LoraSendResp(HW_DEVICE_HEART, mStatus, 0xffffffff);
  }
}