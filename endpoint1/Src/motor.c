#include "motor.h"
#include "hardware.h"
#include "stm32f0xx.h"
#include "rtc.h"
#include "user_config.h"
#include "Lora.h"
#include "lowpower.h"
#include "ultra.h"
#include "beep.h"

Motor motor;

extern Device gDevice;

//���������ĸ�״̬ �ɰ汾����
//up    down
//1     0       ����״̬
//0     0       ֱ��״̬
//0     1       ǰ��״̬
//1     1       �Ե�״̬

//���Ѻ��ٴν�������ʱ�ر�ʹ�ù�������
void GPIO_MotorSenserInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /*Configure GPIO pins : PA6(K3) */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void GPIO_Motor_Init(uint8_t cmd)
{
  GPIO_InitTypeDef GPIO_InitStruct;
 
  switch(cmd)
  {
  case HW_MOTOR_UP:
  case HW_MOTOR_DOWN:
    /*Configure GPIO pins : PA4(K1) PA5(K2) */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
//    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /*Configure GPIO pins : PB4(P) PB5(N) PB13(ULTRASONIC)*/
    GPIO_InitStruct.Pin = GPIO_PIN_4 |GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    break;
  case HW_MOTOR_GET:
  case HW_ULTRA_GET:
    /*Configure GPIO pins : PA4(K1) PA5(K2) */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    break;
  default:
    break;
  }
}

static void motor_stop(void)
{
  HAL_GPIO_WritePin(GPIO_SENSOR_FORWARE, GPIO_SENSOR_FORWARE_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIO_SENSOR_BACKWARD, GPIO_SENSOR_BACKWARD_PIN, GPIO_PIN_RESET);
}

void MotorSenceSwitch(bool b)
{
  if (true == b) {
    HAL_GPIO_WritePin(GPIO_SENSOR_SWITCH, GPIO_SENSOR_SWITCH_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
  } else {
    HAL_GPIO_WritePin(GPIO_SENSOR_SWITCH, GPIO_SENSOR_SWITCH_PIN, GPIO_PIN_RESET);
  }
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

static bool motor_runrun(uint8_t state, MOTOR_CMD cmd)
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

static void mydelay(uint8_t delay)
{
  volatile uint32_t count = 0;
  volatile uint8_t i;
  
  for(i=0; i < delay; i++)
  {
    count = 1000;
    while(count--);
  }
}

static void motor_last_process(MOTOR_STATUS status)
{
  motor_stop();
  rtc_disable();
  motor.status = status;
  motor.ctrl_cb(ACTION_OK);
  motor.action = MOTOR_CMD_IDLE;
  gDevice.bMotorUseRTC = false;
  gDevice.bHasMotorNormalInter = true;
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
        motor_last_process(MOTOR_UP);
      }
    } else if (MOTOR_CMD_DOWN == motor.action) {
      if (MOTOR_DOWN == status) {
        //modify 2019-03-18 15:50
        //ϵͳ�����Ѻ�δ��ʼ��ʱ�ӣ����ܵ���HAL_Delay()
        mydelay(20);
        if (get_status() == status) {
          motor_last_process(MOTOR_DOWN);
        }
      }
    } else if (MOTOR_CMD_DOWN_INTERNAL == motor.action) {
      if(MOTOR_DOWN == status){
//        mydelay(20);
        if (get_status() == status) {
//        status = get_status();
          motor_last_process(MOTOR_DOWN);
        }
      }
    } else {
      //none
    }
  }
}

static void motor_timer_ctrl_timeout_cb(void)
{
  if ( motor.action == MOTOR_CMD_IDLE ) 
    return;

  motor_stop();
  motor.status = get_status();
  motor.ctrl_cb((MOTOR_CB_TYPE)motor.status);
  motor.action = MOTOR_CMD_IDLE;
  gDevice.bMotorUseRTC = false;
  gDevice.bHasRtcInter = true;
}

//������������ִ�к����øú��������������resp
static void motor_ctrl_cb(MOTOR_CB_TYPE m)
{
  gDevice.u8MotorResp = m;
  
  if ((motor.action == MOTOR_CMD_DOWN_INTERNAL) && (motor.status == MOTOR_DOWN)) {
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
  
  gDevice.bMotorAbnormal = false;
  
  rtc_disable();
  gDevice.bHasRtcInter = false;
  
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
      gDevice.bMotorUseRTC = true;
//      return MOTOR_OK_NOSEND;
      return MOTOR_OK;
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
    gDevice.bMotorUseRTC = true;
    
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
  MotorSenceSwitch(true);
  
  motor_stop();
  
  motor.action = MOTOR_CMD_IDLE; //��ǰ��������
  motor.status = get_status(); //�����ʼ��״̬
//  motor.last_status = motor.status;
  //ע���жϿ��ƻص��������������жϻص���������
  motor_ctrl_cb_register(motor_ctrl_cb);
  //ע�������жϻص��������ú���Ϊ�ж��ڵ��á�
  motor_gpio_cb_register(motor_gpio_cb);
 
  motor.ctrl_timer_cb = motor_timer_ctrl_timeout_cb;
  
  MotorSenceSwitch(false);
}

MOTOR_STATUS MotorGetStatus(void)
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
  GPIO_InitTypeDef GPIO_InitStruct;
  
  //pa4 pa5
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
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
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void MotorAbnormalCheck(Device *d)
{
  MOTOR_STATUS status;
  //�����ǰ����ִ�е���˶�ָ��
  if ((d->u8Cmd == HW_MOTOR_UP) || (d->u8Cmd == HW_MOTOR_DOWN))
    return;
  //������Դ
  GPIO_Motor_Init(HW_MOTOR_GET);
  GPIO_MotorSenserInit();
  MotorSenceSwitch(true);
  
  status = MotorGetStatus();
  if (status != motor.status)
  {
    d->bMotorAbnormal = true;
  }
  MotorSenceSwitch(false);
}

void MotorAbnormalProcess(Device *d)
{
  if (d->bMotorAbnormal == false)
    return;
  
  d->bMotorAbnormal = false;
  
  if ((d->u8Cmd == HW_MOTOR_UP) || (d->u8Cmd == HW_MOTOR_DOWN) || (d->u8Cmd == HW_MOTOR_GET))
    return;
  
  MOTOR_STATUS status;
  MsgDevice md;
  uint8_t len;
  uint8_t sbuff[20];

  GPIO_Motor_Init(HW_MOTOR_GET);
  GPIO_MotorSenserInit();
  MotorSenceSwitch(true);
  status = MotorGetStatus();
  if ( motor.status != status ) {
    motor.status = status;
    md.u8Cmd = HW_MOTOR_ABNORMAL;
    md.u8Resp = status;
    md.u32Identify = 0xffffffff;
    len = GetSendData(sbuff, &md);
    LoraSend(sbuff, len);
  }
  MotorSenceSwitch(false);
}
