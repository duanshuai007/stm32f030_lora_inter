#include "motor.h"
#include "hardware.h"
#include "stm32f0xx.h"
#include "rtc.h"
#include "user_config.h"
#include "Lora.h"
#include "lowpower.h"
#include "ultra.h"
#include "beep.h"

extern Motor gMotor;
extern Device gDevice;
static bool bMotorBusy = false;

/* 
  * ���������ĸ�״̬ �ɰ汾����
  * up    down
  * 1     0       ����״̬
  * 0     0       ֱ��״̬
  * 0     1       ǰ��״̬
  * 1     1       �Ե�״̬
  */

/*
  * ���Ѻ��ٴν�������ʱ�ر�ʹ�ù�������
  */
void GPIOMotorSenserInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /*Configure GPIO pins : PA6(K3) */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void GPIOMotorInit(uint8_t cmd)
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
  case HW_DEVICE_HEART:
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
  HAL_GPIO_WritePin(GPIO_MOTOR_FORWARE, GPIO_MOTOR_FORWARE_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIO_MOTOR_BACKWARD, GPIO_MOTOR_BACKWARD_PIN, GPIO_PIN_RESET);
}

static void motor_sence_switch(bool b)
{
  if (true == b) {
    HAL_GPIO_WritePin(GPIO_SENSOR_SWITCH, GPIO_SENSOR_SWITCH_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
  } else {
    HAL_GPIO_WritePin(GPIO_SENSOR_SWITCH, GPIO_SENSOR_SWITCH_PIN, GPIO_PIN_RESET);
  }
}

static MOTOR_STATUS motor_get_status(void)
{
  GPIO_PinState pin1 = HAL_GPIO_ReadPin(GPIO_SENSOR_UP, GPIO_SENSOR_UP_PIN);
  GPIO_PinState pin2 = HAL_GPIO_ReadPin(GPIO_SENSOR_DOWN, GPIO_SENSOR_DOWN_PIN);
  
#if defined(MODE_OLD)
  if (pin1 == GPIO_PIN_RESET) {
    if (pin2 == GPIO_PIN_RESET) {
      return MOTOR_UP;  //all low
    } else {
      return MOTOR_QIANQING;  //pin1 reset pin2 set
    }
  } else {
    if (pin2 == GPIO_PIN_RESET) {
      return MOTOR_HOUQING;
    } else {
      return MOTOR_DOWN;  //all high
    }
  }
#else
  if (GPIO_PIN_RESET == pin1) {   
    if (pin2 == GPIO_PIN_RESET) {
      return MOTOR_HOUQING;
    } else {
      return MOTOR_UP;
    }
  } else if(pin1 == GPIO_PIN_SET) {   
    if(pin2 == GPIO_PIN_RESET) {
      return MOTOR_DOWN;
    } else {
      return MOTOR_QIANQING;
    }
  }
#endif
}

static bool motor_runrun(uint8_t state, MOTOR_CMD cmd)
{
  gMotor.action = cmd;
  
  if (MOTOR_CMD_UP == cmd) {
    if (MOTOR_HOUQING == state) {   //������Ҫ��ת
      HAL_GPIO_WritePin(GPIO_MOTOR_FORWARE, GPIO_MOTOR_FORWARE_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIO_MOTOR_BACKWARD, GPIO_MOTOR_BACKWARD_PIN, GPIO_PIN_SET);
      return true;
    } else if (MOTOR_DOWN == state || MOTOR_QIANQING == state) {   //ǰ����Ե���Ҫ��ת
      HAL_GPIO_WritePin(GPIO_MOTOR_FORWARE, GPIO_MOTOR_FORWARE_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIO_MOTOR_BACKWARD, GPIO_MOTOR_BACKWARD_PIN, GPIO_PIN_RESET);
      return true;
    } else {
      gMotor.action = MOTOR_CMD_IDLE;
      gMotor.ctrl_cb(ACTION_OK);
      return false;
    }
  } else if (MOTOR_CMD_DOWN == cmd) {
    if (MOTOR_DOWN != state) {      //��ת
      HAL_GPIO_WritePin(GPIO_MOTOR_FORWARE, GPIO_MOTOR_FORWARE_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIO_MOTOR_BACKWARD, GPIO_MOTOR_BACKWARD_PIN, GPIO_PIN_SET);
      return true;
    } else {
      gMotor.action = MOTOR_CMD_IDLE;
      gMotor.ctrl_cb(ACTION_OK);
      return false;
    }
  } else {
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
  gMotor.status = status;
  gMotor.realStatus = status;
  gMotor.ctrl_cb(ACTION_OK);
  gMotor.action = MOTOR_CMD_IDLE;
  gDevice.bMotorUseRTCInter = false;
  gDevice.bHasMotorNormalInter = true;
}

/* 
1 = �Ե� 2 = ǰ�� 3 = ֱ�� 4 = ����
*/
void motor_gpio_cb(uint16_t pin)
{
  MOTOR_STATUS status = MOTOR_OK;
  
  //λ�ô�������Դʹ�ܺ����ִ�����жϴ�������������ж�
  //�ͻᵼ�»�ȡ�������״̬��Ϣ��
  if (MOTOR_CMD_IDLE == gMotor.action )
    return;
  
  //�ж��Ƿ��Ǽ�����ŵ��ж� 
  if ((GPIO_SENSOR_UP_PIN == pin ) || ( GPIO_SENSOR_DOWN_PIN == pin)) {
    status = getMotorStatusInt();

    if (MOTOR_CMD_UP == gMotor.action){
      if ((MOTOR_UP == status) || (gMotor.can_stop == status)) {
        motor_last_process(MOTOR_UP);
      }
    } else if (MOTOR_CMD_DOWN == gMotor.action) {
      if (MOTOR_DOWN == status) {
        //modify 2019-03-18 15:50
        //ϵͳ�����Ѻ�δ��ʼ��ʱ�ӣ����ܵ���HAL_Delay()
        mydelay(15);
        if (getMotorStatusInt() == status) {
          motor_last_process(MOTOR_DOWN);
        }
      }
    } else if (MOTOR_CMD_DOWN_INTERNAL == gMotor.action) {
      if(MOTOR_DOWN == status){
        mydelay(15);
        if (getMotorStatusInt() == status) {
          motor_last_process(MOTOR_DOWN);
        }
      }
    } else {
      //do nothing
    }
  }
}

static void motor_timer_ctrl_timeout_cb(void)
{
  if ( gMotor.action == MOTOR_CMD_IDLE ) 
    return;

  motor_stop();
  gMotor.status = getMotorStatusInt();
  gMotor.realStatus = gMotor.status;
  gMotor.ctrl_cb((MOTOR_CB_TYPE)gMotor.status);
  gMotor.action = MOTOR_CMD_IDLE;
  gDevice.bHasRtcInter = true;
  gDevice.bMotorUseRTCInter = false;
}

/*
  * ������������ִ�к����øú��������������resp
  */
static void motor_ctrl_cb(MOTOR_CB_TYPE m)
{
  gDevice.u8MotorResp = m;
  
  if ((gMotor.action == MOTOR_CMD_DOWN_INTERNAL) && (gMotor.status == MOTOR_DOWN)) {
      gDevice.u8CmdDone = CMD_EXEC_DOING;
  } else {
    gDevice.u8CmdDone = CMD_EXEC_DONE;
  }
}

MOTOR_STATUS motor_conctrl(MOTOR_CMD cmd) 
{
  uint16_t dat = 0;
  //�����ǰæ
  if(gMotor.action != MOTOR_CMD_IDLE)
    return MOTOR_RUNNING;
  
  gDevice.bHasRtcInter = false;
  
  gMotor.status = getMotorStatus();//gMotor.realStatus;
  
  //�������ʱUP����ô��ǰ״̬�����DOWN��QIANQING�Ļ������ﵽHOUQING��̬�Ϳ���ֹͣ
  //���������UP����ô��ǰ״̬�Ǻ���Ļ������ﵽǰ���ʱ��Ϳ���ֹͣ
  if(cmd == MOTOR_CMD_UP) {
    if (( MOTOR_DOWN == gMotor.status ) || ( MOTOR_QIANQING == gMotor.status )){
      gMotor.can_stop = MOTOR_HOUQING;
    } else if ( MOTOR_HOUQING == gMotor.status) {
      gMotor.can_stop = MOTOR_QIANQING;
    } else {
      //MOTOR_UP
    }
    
    //��״̬Ϊ�˷�ֹ��������������Ҫ�򽫵������£�Ȼ������̧��
    if ( gMotor.status == MOTOR_QIANQING ) {
      //����������
      gMotor.action = MOTOR_CMD_DOWN_INTERNAL;
      HAL_GPIO_WritePin(GPIO_MOTOR_FORWARE, GPIO_MOTOR_FORWARE_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIO_MOTOR_BACKWARD, GPIO_MOTOR_BACKWARD_PIN, GPIO_PIN_SET);

      gDevice.u8MotorRtcTimerInter = 4;
      gDevice.bMotorUseRTCInter = true;
      return MOTOR_OK;
    }
    //ʹ�ܳ�������Դ
#if 1
    if ( MOTOR_UP != gMotor.status ) {
      dat = ReadUltraData();
      
      if ( ULTRA_READ_ERROR == dat )
        return MOTOR_ERROR_ULTRA;
      //��λ:����
      if ( dat < gDevice.u8UltraSafeDistance ) {
        //С�ڰ�ȫ�������Ϊ��λ�г�����̧��
        return MOTOR_ERROR;
      }
    }
#endif
  }

  if (motor_runrun(gMotor.status, cmd) == true)
  {
    //������Ƴ�ʱ��ʱ��������ʹ��rtc��ʵ��
    if ( gMotor.status == MOTOR_HOUQING ) {
      gDevice.u8MotorRtcTimerInter = 8;
    } else {
      gDevice.u8MotorRtcTimerInter = 6;
    }
    gDevice.bMotorUseRTCInter = true;
    
    return MOTOR_OK; 
  } else {
    return MOTOR_DONTDO;
  }
}

void MotorInit(void)
{
  //ʹ�ܴ���������
  motor_sence_switch(true);
  
  motor_stop();
  
  gMotor.action = MOTOR_CMD_IDLE; //��ǰ��������
  gMotor.status = getMotorStatus(); //�����ʼ��״̬
  //gMotor.realStatus = gMotor.status;
  //ע���жϿ��ƻص��������������жϻص���������
//  motor_ctrl_cb_register(motor_ctrl_cb);
  gMotor.ctrl_cb = motor_ctrl_cb;
  //ע�������жϻص��������ú���Ϊ�ж��ڵ��á�
//  motor_gpio_cb_register(motor_gpio_cb);
  gMotor.gpio_cb = motor_gpio_cb;
  gMotor.ctrl_timer_cb = motor_timer_ctrl_timeout_cb;
  
  motor_sence_switch(false);
}

// public
MOTOR_STATUS MotorGetStatus(void)
{
  return motor_get_status();
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
void setMotorGpioNormal(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/*
*   ����gpioΪ�ж�ģʽ�����ش���
*/
void setMotorGpioInterrupt(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

//ÿ�λ�������ѭ����ֻ��ʵ�ʻ�ȡ���״̬һ��
MOTOR_STATUS getMotorStatus(void)
{
  if ((true == gDevice.bHasGetMotorStatus) || (true == bMotorBusy)) {
    return gMotor.realStatus;
  }
  
  GPIOMotorInit(HW_MOTOR_GET);
  GPIOMotorSenserInit();
  motor_sence_switch(true);
  gMotor.realStatus = motor_get_status();
  gDevice.bHasGetMotorStatus = true;
  motor_sence_switch(false);
  
  return gMotor.realStatus;
}

void resetMotorStatus(void)
{
  gDevice.bHasGetMotorStatus = false;
}

//lora up down
void setMotorInterrupt()
{
  //����ǰ��ʱ�������º�̧�𣬲���Ҫ�ظ�ִ�������ʼ��
  if (bMotorBusy == true)
    return;
  
  bMotorBusy = true;
  GPIOMotorSenserInit();
  motor_sence_switch(true);
  GPIOMotorInit(HW_MOTOR_UP);
  gMotor.realStatus = motor_get_status();
  gDevice.bHasGetMotorStatus = true;
}

//main loop start
void cancelMotorInterrupt()
{
  GPIOMotorInit(HW_MOTOR_GET);
  motor_sence_switch(false);
  bMotorBusy = false;
}

//motor interrupt
MOTOR_STATUS getMotorStatusInt(void)
{
  gMotor.realStatus = motor_get_status();
//  gDevice.bHasGetMotorStatus = true;
  return gMotor.realStatus;
}

bool isMotorBusy(void)
{
  return bMotorBusy;
}

bool MotorAbnormalCheck(Device *pDev, bool flag)
{
  MOTOR_STATUS mStatus;
  
  if (flag)
    return true;
  
#if 1
  //�����ǰ����ִ�е���˶�ָ��������쳣�������飬����������ͳ�����
  if (isMotorBusy() == true)
    return true;

  mStatus = getMotorStatus();

  //��������ָ��ʱ��ѭ���жϵ���״̬���������״̬����������쳣
  if (mStatus != gMotor.status)
  {
    pDev->u32LastHeartTime = pDev->u32GlobalTimeCount;
    pDev->u32LastCheckUltraTime = pDev->u32GlobalTimeCount;
    gMotor.status = mStatus;
    LoraSendResp(HW_MOTOR_ABNORMAL, mStatus, 0xffffffff);
    return true;
  }
  
  return false;
#endif
}
