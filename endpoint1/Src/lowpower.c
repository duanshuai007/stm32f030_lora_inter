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

//����stopģʽ֮ǰҪ���¸��ݵ�ʱ�ĵ�ƽ������������
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
  //�رճ�����ģ���Դ
  HAL_GPIO_WritePin(GPIO_ULTRASONIC, GPIO_ULTRASONIC_PIN, GPIO_PIN_SET);
  
  /*
  *       GPIOA���漰�����Ű���
  *       M0(PA8),K1(PA4),K2(PA5),K3������ʹ��(PA6)
  *       PA8��PA6 �����ģʽ
  *       PA4��PA5 ���ж�ģʽ
  *       �ڽ�������ǰ���ݵ�ƽֵ��������������
  */
#if 0
  //λ�ü�����Ŵ����������裬������,��������˻��������900ua
  set_gpio( GPIO_SENSOR_UP, GPIO_SENSOR_UP_PIN, GPIO_MODE_OUTPUT_PP);
  set_gpio( GPIO_SENSOR_DOWN, GPIO_SENSOR_DOWN_PIN, GPIO_MODE_OUTPUT_PP);
  set_gpio( GPIO_ULTRASONIC, GPIO_ULTRASONIC_PIN, GPIO_MODE_OUTPUT_PP);
#endif

  //loraģ���ģʽѡ�����ţ����������ű�������Ϊ���
//  set_gpio( GPIO_Lora_M0, GPIO_Lora_M0_Pin, GPIO_MODE_OUTPUT_PP);
//  set_gpio( GPIO_Lora_M1, GPIO_Lora_M1_Pin, GPIO_MODE_OUTPUT_PP);
  
  /*
  *   GPIOB���漰�������Ű���
  *   (P)PB4,(N)PB5,(BEEP)PB10,(AUX)PB14,(M1)PB15
  *   PB4,PB5,PB10,PB15�������ģʽ
  *   PB14���ж�����
  */
  if ((gDevice.u8Cmd == HW_MOTOR_UP) || (gDevice.u8Cmd == HW_MOTOR_DOWN)) {
    set_gpio(GPIO_SENSOR_FORWARE, GPIO_SENSOR_FORWARE_PIN, GPIO_MODE_OUTPUT_PP);
    set_gpio(GPIO_SENSOR_BACKWARD, GPIO_SENSOR_BACKWARD_PIN, GPIO_MODE_OUTPUT_PP);
    //������Ų�������Ϊ���룬����Ϊ����ͻᵼ�µ�����յ�ָ��󲻶�
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

//�ر�ʹ�õ�����
void LowPowerInit(Device *d)
{   
  //����ֹͣ�����򹦺Ļ���1ma����
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
  
  //����1�붨ʱ����,ֻ���ڵ��δʹ��rtc��ʱ���ʹ��rtc��ʱ
  if (d->bMotorUseRTC == false)
    rtc_set_timer(6);
  
  __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_14);
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
    //����λ�ô������Ŀ���
    MotorSenceSwitch(true);
    //��������õ���������Դ������gpio(PB4\5,PA4\5\6,TIM6)
    break;
  case HW_BEEP_ON:
  case HW_BEEP_OFF:
  case HW_BEEP_GET:
    periph_used_in_beep_init();
    //�������õ�������Դ��gpio��(PB10)
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
  //�첽ָ��ִ����ɺ���resp��Ϣ
  if (HW_CMD_NONE == d->u8Cmd)
    return;
  
  if (CMD_RUN != d->u8CmdRunning)
    return;
  
  if ( CMD_EXEC_DONE == d->u8CmdDone) {
    
    //���ָ��ִ����ɻ��ǳ�ʱ������Ҫ��λ�ô������ر�
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
    //��Ӧǰ��״̬�����º�̧��
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
  
  //�ж�ָ���Ƿ�֧��
  if ((d->u8Cmd >= HW_CMD_MAXVALUE) && (d->u8Cmd != HW_DEVICE_HEART)) {
    d->u8Cmd = HW_CMD_NONE;
    return;
  }
  //ִ��ָ���ִ��ָ��֮ǰ�ȳ�ʼ����Ӧ�������õ���������Դ
  InitPeriphByCMD(d->u8Cmd);  
  
  d->u8CmdRunning = CMD_RUN;
  
  uint8_t ret = hardware_ctrl(d);
  
  switch(d->u8Cmd) {
  case HW_MOTOR_UP:
  case HW_MOTOR_DOWN:
    //�öδ������ڵ��ִ�ж����󷵻ص����ʼ������resp
    switch(ret) {
    case MOTOR_OK:
      //����ִ�ж��������̷���һ����Ӧ��Ϣ
      if (d->u8CmdDone != CMD_EXEC_DOING) { 
        md.u8Cmd = d->u8Cmd;
        md.u8Resp = MOTOR_RUN;
        md.u32Identify = d->u32Identify;
        len = GetSendData(sbuff, &md);
        LoraSend(sbuff, len);
      }
      break;
    case MOTOR_DONTDO:
      //�������Ŀ��λ�ò���Ҫִ�ж���,ֱ�ӹر�λ�ô�����
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
      //������״̬����ֱ�ӹر�λ�ô�����
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
    //���״̬��ȡ��ϣ��ر�λ�ô�����
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
    //ָ��ִ�н��������״̬��Ϣ
    d->u8CmdRunning = CMD_STOP;
    d->u8Cmd = HW_CMD_NONE;
    break;
  default:
    d->u8CmdRunning = CMD_STOP;
    d->u8Cmd = HW_CMD_NONE;
    break;
  }
}
