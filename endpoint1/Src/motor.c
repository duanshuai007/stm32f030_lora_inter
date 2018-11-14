#include "motor.h"
#include "hardware.h"
#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_def.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_hal_tim.h"

#include "rtc.h"
#include "user_config.h"
#include "Lora.h"

Motor motor;

extern Device gDevice;
extern TIM_HandleTypeDef htim6;

//���������ĸ�״̬ �ɰ汾����
//up    down
//1     0       ����״̬
//0     0       ֱ��״̬
//0     1       ǰ��״̬
//1     1       �Ե�״̬

static void motor_stop(void)
{
  HAL_GPIO_WritePin(GPIO_SENSOR_FORWARE, GPIO_SENSOR_FORWARE_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIO_SENSOR_BACKWARD, GPIO_SENSOR_BACKWARD_PIN, GPIO_PIN_RESET);
}

static void sensor_switch(bool b)
{
  if (true == b)
    HAL_GPIO_WritePin(GPIO_SENSOR_SWITCH, GPIO_SENSOR_SWITCH_PIN, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(GPIO_SENSOR_SWITCH, GPIO_SENSOR_SWITCH_PIN, GPIO_PIN_RESET);
}

static MOTOR_STATUS get_status(void)
{
  GPIO_PinState pin1 = HAL_GPIO_ReadPin(GPIO_SENSOR_UP, GPIO_SENSOR_UP_PIN);
  GPIO_PinState pin2 = HAL_GPIO_ReadPin(GPIO_SENSOR_DOWN, GPIO_SENSOR_DOWN_PIN);
  
#if defined(MODE_OLD)
  if (pin1 == GPIO_PIN_RESET) {
    if (pin2 == GPIO_PIN_RESET) {
      return MOTOR_UP;
    } else {
      return MOTOR_QIANQING;
    }
  } else {
    if (pin2 == GPIO_PIN_RESET) {
      return MOTOR_HOUQING;
    } else {
      return MOTOR_DOWN;
    }
  }
#else
  if (GPIO_PIN_RESET == pin1) {   
    if (pin2 == GPIO_PIN_RESET) {
      return MOTOR_HOUQING;
    } else if(pin2 == GPIO_PIN_SET) {
      return MOTOR_UP;
    }
  } else if(pin1 == GPIO_PIN_SET) {   
    if(pin2 == GPIO_PIN_RESET) {
      return MOTOR_DOWN;
    } else if(pin2 == GPIO_PIN_SET) {
      return MOTOR_QIANQING;
    }
  }
#endif
}

bool motor_runrun(uint8_t state, MOTOR_CMD cmd)
{
  motor.action = cmd;
  
  if (MOTOR_CMD_UP == cmd) {
    if (MOTOR_HOUQING == state) {   //������Ҫ��ת
      HAL_GPIO_WritePin(GPIO_SENSOR_FORWARE, GPIO_SENSOR_FORWARE_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIO_SENSOR_BACKWARD, GPIO_SENSOR_BACKWARD_PIN, GPIO_PIN_SET);
      return true;
    } else if (MOTOR_DOWN == state || MOTOR_QIANQING == state) {   //ǰ����Ե���Ҫ��ת
      HAL_GPIO_WritePin(GPIO_SENSOR_FORWARE, GPIO_SENSOR_FORWARE_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIO_SENSOR_BACKWARD, GPIO_SENSOR_BACKWARD_PIN, GPIO_PIN_RESET);
      return true;
    } else {
      motor.action = MOTOR_CMD_IDLE;
      motor.ctrl_cb(ACTION_OK);
      return false;
    }
  } else if (MOTOR_CMD_DOWN == cmd) {
    if (MOTOR_DOWN != state) {      //��ת
      HAL_GPIO_WritePin(GPIO_SENSOR_FORWARE, GPIO_SENSOR_FORWARE_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIO_SENSOR_BACKWARD, GPIO_SENSOR_BACKWARD_PIN, GPIO_PIN_SET);
      return true;
    } else {
      motor.action = MOTOR_CMD_IDLE;
      motor.ctrl_cb(ACTION_OK);
      return false;
    }
  }
  return false;
}

void motor_gpio_cb(uint16_t pin)
{
  MOTOR_STATUS status = MOTOR_OK;
  
  //�ж��Ƿ��Ǽ�����ŵ��ж� 
  if ((GPIO_SENSOR_UP_PIN == pin ) || ( GPIO_SENSOR_DOWN_PIN == pin)) {
    status = get_status();
    //1 = �Ե�
    //2 = ǰ��
    //3 = ֱ��
    //4 = ����
    if (MOTOR_CMD_UP == motor.action){
      if ((MOTOR_UP == status) || (motor.can_stop == status)) {
        motor_stop();
        rtc_disable();
        motor.status = MOTOR_UP;
        motor.action = MOTOR_CMD_IDLE;
        motor.last_status = MOTOR_UP;
        motor.ctrl_cb(ACTION_OK);
      }
    } else if (MOTOR_CMD_DOWN == motor.action){
      if (MOTOR_DOWN == status) {
//        HAL_Delay(1);   //�������ԣ���stm32f103����ģʽ��ʽ����ʱ���ᵼ�µ�������ʱ����������ֹͣ��
        //��ʱʱ���ǵ���������ʱ����ж�����ס�ˣ�����ּ�⵽DOWNλ�õ�״
        //������Ҫ������ʱ�����ж�һ��
        if (get_status() == status) {
          motor_stop();
          rtc_disable();
          motor.status = MOTOR_DOWN;
          motor.action = MOTOR_CMD_IDLE;
          motor.last_status = MOTOR_DOWN;
          motor.ctrl_cb(ACTION_OK);
        }
      }
    } else {   //û���յ�������ǲ����˶���
      
      //������쳣�����������쳣������־
      gDevice.u8InteFlag = 1;
      motor.status = status;
    }
  }
}

static void motor_timer_ctrl_timeout_cb(void)
{
  rtc_disable();
  
  motor_stop();
  
  motor.status = get_status();
  motor.last_status = motor.status;
  
  if ( motor.action != MOTOR_CMD_IDLE ) {         //�����ж�����ʱ��ִ�г�ʱ�ص�
    motor.ctrl_cb((MOTOR_CB_TYPE)motor.status);
  }
  
  motor.action = MOTOR_CMD_IDLE;
}

//������������ִ�к����øú��������������resp
static void motor_ctrl_cb(MOTOR_CB_TYPE m)
{
  gDevice.u8Resp = m;
  gDevice.u8CmdDone = 1;
}

MOTOR_STATUS motor_conctrl(MOTOR_CMD cmd) 
{
  //�����ǰæ
  if(motor.action != MOTOR_CMD_IDLE)
    return MOTOR_RUNNING;
  
  motor.status = get_status();
  
  //�������ʱUP����ô��ǰ״̬�����DOWN��QIANQING�Ļ������ﵽHOUQING��̬�Ϳ���ֹͣ
  //���������UP����ô��ǰ״̬�Ǻ���Ļ������ﵽǰ���ʱ��Ϳ���ֹͣ
  
  if(cmd == MOTOR_CMD_UP) {
    if (( MOTOR_DOWN == motor.status ) || ( MOTOR_QIANQING == motor.status )){
      motor.can_stop = MOTOR_HOUQING;
    } else if ( MOTOR_HOUQING == motor.status) {
      motor.can_stop = MOTOR_QIANQING;
    }
  }

  if (motor_runrun(motor.status, cmd) == true)
  {
    //������Ƴ�ʱ��ʱ��������ʹ��rtc��ʵ��
    if ( motor.status == MOTOR_HOUQING ) {
      rtc_set_timer(8);
    } else {
      rtc_set_timer(5);
    }
    return MOTOR_OK; 
  } else {
    return MOTOR_DONTDO;
  }
}

static void motor_ctrl_cb_register(motor_ctrl_callback cb)
{
  motor.ctrl_cb = cb;
}

static void motor_gpio_cb_register(motor_gpio_callback cb)
{
  motor.gpio_cb = cb;
}

void motor_init(void)
{
  //ʹ�ܴ���������
  sensor_switch(true);
  
  motor_stop();
  
  motor.action = MOTOR_CMD_IDLE; //��ǰ��������
  motor.status = get_status(); //�����ʼ��״̬
  motor.last_status = motor.status;
  //ע���жϿ��ƻص��������������жϻص���������
  motor_ctrl_cb_register(motor_ctrl_cb);
  //ע�������жϻص��������ú���Ϊ�ж��ڵ��á�
  motor_gpio_cb_register(motor_gpio_cb);
  
  //��ֹ��ʱ������  
  motor.xiaodou_timer.enable = false;
  motor.xiaodou_timer.cur_count = 0;
  motor.xiaodou_timer.max_count = 0;
  
  motor.ctrl_timer_cb = motor_timer_ctrl_timeout_cb;
}

MOTOR_STATUS motor_get_status(void)
{
  if (MOTOR_CMD_IDLE != motor.action) {
    return MOTOR_RUNNING;
  }
  return get_status();
}

void motor_input_pin_off_interrupt(bool b)
{
  GPIO_PinState pin1, pin2;
    
  GPIO_InitTypeDef GPIO_InitStruct;
  HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);

  /*Configure GPIO pins : PA4 PA5 */
  if( true == b ) {
    //close interrupt
    pin1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
    pin2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
    //pa4
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    if ( pin1 == GPIO_PIN_SET ) {
      GPIO_InitStruct.Pull = GPIO_PULLUP;
    } else {
      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    }
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    //pa5
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    if ( pin2 == GPIO_PIN_SET ) {
      GPIO_InitStruct.Pull = GPIO_PULLUP;
    } else {
      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    }
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
  } else {
    //�����ʹ���жϣ���Ҫ�ȿ�����Դ��
    sensor_switch(true);
    HAL_Delay(100);
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  //����ǹر��жϣ�����Ҫ���жϵ�Դ
  if ( true == b ) {
    sensor_switch(false);
  }
}

static void LoraSendAbnormalMSG(void)
{
  gDevice.u8Cmd = HW_MOTOR_ABNORMAL;
  gDevice.u8Resp = motor.status;
  
  LoraTransfer(&gDevice, NORMALCMD);
  
  gDevice.u8Cmd = HW_CMD_NONE;
  gDevice.u8Resp = 0;
}

uint8_t motor_abnormal(void)
{
  //����������쳣������־
  if ((gDevice.u8InteFlag == 1) && (gDevice.u8InterDone == 0)) {
    motor_input_pin_off_interrupt(true);
    rtc_set_timer(1);
    gDevice.u8InteFlag = 0;
    gDevice.u8InterDone = 1;
    return 1;
  } else if ( gDevice.u8InterDone ) {
    //���Ѻ�
    gDevice.u8InterDone = 0;
    motor_input_pin_off_interrupt(false);
    //do resp
    
    if ( motor.status != motor.last_status ) {
      LoraSendAbnormalMSG();
      motor.last_status = motor.status;
    }
    return 0;
  }
  
  return 0;
}
