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
    //�����ģ�⹦�����ţ�������������
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
  //loraģ���ģʽѡ�����ţ����������ű�������Ϊ���,
  //������������������žͻ������л�ģʽ����һЩ�쳣���������ݷ��ͳ���
  set_gpio_mode( GPIO_Lora_M0, GPIO_Lora_M0_Pin, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);
  set_gpio_mode( GPIO_Lora_M1, GPIO_Lora_M1_Pin, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);
  
  /*
  *   GPIOB���漰�������Ű���
  *   (P)PB4,(N)PB5,(BEEP)PB10,(AUX)PB14,(M1)PB15
  *   PB4,PB5,PB10,PB15�������ģʽ
  *   PB14���ж�����
  */
  //������ʹ������ �͵�ƽʹ�ܣ��ⲿ��·������������
  //�رճ�����ģ���Դ,
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
    //����������ţ��ڲ�ʹ�õ�ʱ������Ϊģ������
    set_gpio_mode(GPIO_MOTOR_FORWARE, GPIO_MOTOR_FORWARE_PIN, GPIO_MODE_ANALOG, GPIO_NOPULL);
    set_gpio_mode(GPIO_MOTOR_BACKWARD, GPIO_MOTOR_BACKWARD_PIN, GPIO_MODE_ANALOG, GPIO_NOPULL);
    
    //������ʹ�����ţ���·�����������裬����Ϊ�������������ģʽ
    set_gpio_mode(GPIO_SENSOR_SWITCH, GPIO_SENSOR_SWITCH_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    HAL_GPIO_WritePin(GPIO_SENSOR_SWITCH, GPIO_SENSOR_SWITCH_PIN, GPIO_PIN_RESET);
    //λ�ü�����Ŵ����������裬�ⲿ��·������������
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
    //���δʹ��rtc
    if (rtcSetTimer(SYSTEM_RTC_CYC) == false)
      return;
  } else {
    //���ʹ��rtc�����յ�����õ�rtcʱ��������
    if (gDevice.u8MotorRtcTimerInter != 0) {
      if (rtcSetTimer(gDevice.u8MotorRtcTimerInter) == false)
        return;
      gDevice.u8MotorRtcTimerInter = 0;
    }
    else {
      //���ﲻ������rtc����Ϊ����˶��л���������ⲿ
      //�жϣ��������������þͻ��ԭ����ʱ��ʱ�����ǵ�
      return;
    }
  }
  
  //����rtcʱ�ӣ��͹������óɹ�����������
  if ((false == gDevice.bHasLoraInter )
      && (false == gDevice.bHasMotorNormalInter )
        && (false == gDevice.bHasRtcInter ))
  {
    //ʹ�õ����������������1��2��adc��rtc��gpio��iwdg��
    //rtc��iwdg����Ҫ�ر�
    //�رմ���1��2��ʱ�ӣ�adc��ʱ��
    LoraUartDisable();
//    UltraUartDisable();
  
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_HSI_DISABLE();
//    __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);
//  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_14);
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    //������ɹ������½�����������ģʽ
    //ʱ���������
    HAL_RCC_DeInit();
    //HSIʱ��ʹ��
    __HAL_RCC_DISABLE_IT(RCC_IT_HSIRDY);
    __HAL_RCC_HSI_ENABLE();
    while(!__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY));
    //����ʱ�Ӻ�DMA
    SystemClock_Config();
    //������������  
    LoraUartEnable();
  }
}

static void gpio_not_used_lp_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /*
  *       GPIOA���漰�����Ű���
  *       M0(PA8),K1(PA4),K2(PA5),K3������ʹ��(PA6)
  *       PA2,PA3 �Ǵ���2������
  *       PA8��PA6 �����ģʽ
  *       PA4��PA5 ���ж�ģʽ
  *       �ڽ�������ǰ���ݵ�ƽֵ��������������
  */
  GPIO_InitStruct.Pin = (GPIO_PIN_All 
                         & (~( GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_6 )));
  //| GPIO_PIN_13 | GPIO_PIN_14 )));
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;    //������ģʽ�µ�����С
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
  //__HAL_RCC_GPIOA_CLK_DISABLE();
  
  /*
  *   GPIOB���漰�������Ű���
  *   (P)PB4,(N)PB5,(BEEP)PB10,(AUX)PB14,(M1)PB15, (ULTRASONIC PIN)PB13
  *   PB4,PB5,PB10,PB15�������ģʽ
  *   PB14���ж�����
  */
  GPIO_InitStruct.Pin = (GPIO_PIN_All 
                         & (~(GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_10 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)));
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  //__HAL_RCC_GPIOB_CLK_DISABLE();
  
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;    //������ģʽ�µ�����С
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  __HAL_RCC_GPIOC_CLK_DISABLE();
  
  GPIO_InitStruct.Pin =  GPIO_PIN_All;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;    //������ģʽ�µ�����С
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  __HAL_RCC_GPIOF_CLK_DISABLE();
}

//�ر�δʹ�õ�����,
//������ϵͳ��λ���������һ��
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
  //rtcȫ��ʹ�ܣ����ر�
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
  //�첽ָ��ִ����ɺ���resp��Ϣ
  if (HW_CMD_NONE == pDev->u8Cmd) 
    return;
  
  if (CMD_RUN != pDev->u8CmdRunning) 
    return;
  
  uint8_t resp = pDev->u8MotorResp;
  
  if ( CMD_EXEC_DONE == pDev->u8CmdDone) {
    
    pDev->u32LastHeartTime = pDev->u32GlobalTimeCount;
    pDev->u32LastCheckUltraTime = pDev->u32GlobalTimeCount;
    //���ָ��ִ����ɻ��ǳ�ʱ������Ҫ��λ�ô������ر�
    if ((pDev->u8Cmd == HW_MOTOR_UP) || (pDev->u8Cmd == HW_MOTOR_DOWN)) {
      cancelMotorInterrupt();
    }
    
    LoraSendResp(pDev->u8Cmd, resp, pDev->u32Identify);
    
    pDev->u8CmdDone = CMD_EXEC_NONE;
    pDev->u8Cmd = HW_CMD_NONE;
    pDev->u8MotorResp = 0;
    pDev->u8CmdRunning = CMD_STOP;
  } else if ( CMD_EXEC_DOING == pDev->u8CmdDone) {
    //��Ӧǰ��״̬�����º�̧��
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
  
  //�ж�ָ���Ƿ�֧��
  if ((pDev->u8Cmd >= HW_CMD_MAXVALUE) && (pDev->u8Cmd != HW_DEVICE_HEART)) {
    pDev->u8Cmd = HW_CMD_NONE;
    return;
  }
  
  pDev->u8CmdRunning = CMD_RUN;
  
  //����������ʱ��
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
    //�����Ե�ʱ�����ڼ�ⳬ������
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
    //����������
//    LoraSendMsg(HW_DEVICE_HEART, ((mStatus << 4) | ULTRA_NO_READ), gDevice.u32Identify);
    LoraSendResp(HW_DEVICE_HEART, mStatus, 0xffffffff);
  }
}