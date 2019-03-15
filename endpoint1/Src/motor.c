#include "motor.h"
#include "hardware.h"
#include "stm32f0xx.h"
#include "rtc.h"
#include "user_config.h"
#include "Lora.h"
#include "lowpower.h"
#include "ultra.h"

Motor motor;

extern Device gDevice;
extern UART_HandleTypeDef huart2;
extern uint8_t uart2_dma_rbuff[4];

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
        //�����־����0��˵������ͨ�ĵ����ɶ����������ж�
        gDevice.bHasMotorNormalInter = true;
      }
    } else if (MOTOR_CMD_DOWN == motor.action) {
      if (MOTOR_DOWN == status) {
        if (get_status() == status) {
          motor_stop();
          rtc_disable();
          motor.status = MOTOR_DOWN;
          motor.action = MOTOR_CMD_IDLE;
          motor.last_status = MOTOR_DOWN;
          motor.ctrl_cb(ACTION_OK);
          //�����־����0��˵������ͨ�ĵ����ɶ����������ж�
          gDevice.bHasMotorNormalInter = true;
        }
      }
    } else if (MOTOR_CMD_DOWN_INTERNAL == motor.action) {
      if(MOTOR_DOWN == status){
        if (get_status() == status) {
          motor_stop();
          rtc_disable();
          motor.status = MOTOR_DOWN;
          motor.last_status = MOTOR_DOWN;
          motor.ctrl_cb(ACTION_OK);
          motor.action = MOTOR_CMD_IDLE;
          gDevice.bHasMotorNormalInter = true;
        }
      }
    } else {
      //������쳣�����������쳣������־
      gDevice.bInteFlag = true;
      motor.status = status;
      gDevice.bHasMotorAbnmalInter = true;
    }
  }
}

static void motor_timer_ctrl_timeout_cb(void)
{
  if ( motor.action == MOTOR_CMD_IDLE ) 
    return;

  motor_stop();
  motor.status = get_status();
  motor.last_status = motor.status;
  motor.ctrl_cb((MOTOR_CB_TYPE)motor.status);
  motor.action = MOTOR_CMD_IDLE;

  gDevice.bHasRtcInter = true;
}

//������������ִ�к����øú��������������resp
static void motor_ctrl_cb(MOTOR_CB_TYPE m)
{
  gDevice.u8MotorResp = m;
  
  if(motor.action == MOTOR_CMD_DOWN_INTERNAL) {
    gDevice.u8CmdDone = CMD_EXEC_DOING;
  } else {
    gDevice.u8CmdDone = CMD_EXEC_DONE;    
  }
}

MOTOR_STATUS motor_conctrl(MOTOR_CMD cmd) 
{
  uint16_t dat = 0;
  uint8_t sleeptime = 0;
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
    
    //��״̬Ϊ�˷�ֹ��������������Ҫ�򽫵������£�Ȼ������̧��
    if ( motor.status == MOTOR_QIANQING ) {
      //����������
      motor.action = MOTOR_CMD_DOWN_INTERNAL;
      HAL_GPIO_WritePin(GPIO_SENSOR_FORWARE, GPIO_SENSOR_FORWARE_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIO_SENSOR_BACKWARD, GPIO_SENSOR_BACKWARD_PIN, GPIO_PIN_SET);
      rtc_set_timer(5);   //����5�붨ʱ����
      return MOTOR_OK_NOSEND;
    }
    //ʹ�ܳ�������Դ
#if 1
    if ( MOTOR_UP != motor.status ) {
      dat = ReadUltraData();
      
      if ( 0xff == dat )
        return MOTOR_ERROR_ULTRA;
      //��λ:����
      if ( dat < gDevice.u8UltraSafeDistance ) {
        //С�ڰ�ȫ�������Ϊ��λ�г�����̧��
        return MOTOR_ERROR;
      }
    }
#endif
  }

  if (motor_runrun(motor.status, cmd) == true)
  {
    //������Ƴ�ʱ��ʱ��������ʹ��rtc��ʵ��
    if ( motor.status == MOTOR_HOUQING ) {
      sleeptime = 10;
    } else {
      sleeptime = 6;
    }
    rtc_set_timer(sleeptime);   //���������ʱ��ʱ
    
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
 
  motor.ctrl_timer_cb = motor_timer_ctrl_timeout_cb;
}

MOTOR_STATUS motor_get_status(void)
{
  if (MOTOR_CMD_IDLE != motor.action) {
    return MOTOR_RUNNING;
  }
  return get_status();
}

/*
*     ����PA4��PA5�������ŵ��жϽ�ֹ
*     ��Ϊû�취�����������������ж�
*     ��PB14����Loraģ����ж����ţ����ܹ���ֹ��
*     ���԰�PA4��PA5����Ϊ��ͨ�������ţ�Ȼ��ر�λ�ü��ĵ�Դ����
*
*     ���ȡ���ر��жϣ��ȴ�λ�ü��Ŀ���,Ȼ����������Ϊ�ж�ģʽ
**/

/*
*   ����gpioΪ��ͨģʽ
*/
void set_motor_gpio_normal(void)
{
  GPIO_PinState pin_state;
  GPIO_InitTypeDef GPIO_InitStruct;
  
  pin_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
  //pa4
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  if ( pin_state == GPIO_PIN_SET ) {
    GPIO_InitStruct.Pull = GPIO_PULLUP;
  } else {
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  }
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  //pa5
  pin_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  if ( pin_state == GPIO_PIN_SET ) {
    GPIO_InitStruct.Pull = GPIO_PULLUP;
  } else {
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  }
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
}

/*
*   ����gpioΪ�ж�ģʽ�����ش���
*/
void set_motor_gpio_interrupt(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void motor_input_pin_close_interrupt(bool b)
{
  HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);

  /*Configure GPIO pins : PA4 PA5 */
  if( true == b ) {
    //close interrupt
    set_motor_gpio_normal();
  } else {
    //�����ʹ���жϣ���Ҫ�ȿ�����Դ��
    sensor_switch(true);
    HAL_Delay(20);
    set_motor_gpio_interrupt();
  }

  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  //����ǹر��жϣ�����Ҫ���жϵ�Դ
  if ( true == b ) {
    sensor_switch(false);
  }
}

void motor_abnormal_step1(Device *d)
{
  //����������쳣������־
  if ((d->bInteFlag == true) && (d->bInterDone == false)) {
    motor_input_pin_close_interrupt(true);
    d->bInteFlag = false;
    d->bInterDone = true;
    rtc_set_timer(2);   //�쳣����������ʱ
  }
}

void motor_abnormal_step2(Device *d)
{
  if ( d->bInterDone  == false)
    return;
  
  uint8_t len;
  uint8_t sbuff[20];
  MsgDevice md;
  
  //���Ѻ�,����ڷ����쳣������1�뻽���ڼ���յ��˿��������ʱ�Ͳ���Ҫ�����쳣��������Ϣ��
  //����ִ�ж���
  d->bInterDone = false;
  motor_input_pin_close_interrupt(false);
  
  if (d->u8Cmd != HW_CMD_NONE)
    return;

  MOTOR_STATUS last = motor.last_status;
  
  if ( motor.status != last ) {
    md.u8Cmd = HW_MOTOR_ABNORMAL;
    md.u8Resp = motor.status;
    md.u32Identify = gDevice.u32Identify;
    
    len = GetSendData(sbuff, &md);
    LoraSend(sbuff, len);

    motor.last_status = motor.status;
  }
}
