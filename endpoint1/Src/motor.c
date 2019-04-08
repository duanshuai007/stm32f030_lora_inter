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

//传感器的四个状态 旧版本地锁
//up    down
//1     0       后倾状态
//0     0       直立状态
//0     1       前倾状态
//1     1       卧倒状态

//唤醒后再次进入休眠时关闭使用过的外设
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
    if (MOTOR_HOUQING == state) {   //后倾需要反转
      HAL_GPIO_WritePin(GPIO_SENSOR_FORWARE, GPIO_SENSOR_FORWARE_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIO_SENSOR_BACKWARD, GPIO_SENSOR_BACKWARD_PIN, GPIO_PIN_SET);
      return true;
    } else if (MOTOR_DOWN == state || MOTOR_QIANQING == state) {   //前倾和卧倒需要正转
      HAL_GPIO_WritePin(GPIO_SENSOR_FORWARE, GPIO_SENSOR_FORWARE_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIO_SENSOR_BACKWARD, GPIO_SENSOR_BACKWARD_PIN, GPIO_PIN_RESET);
      return true;
    } else {
      motor.action = MOTOR_CMD_IDLE;
      motor.ctrl_cb(ACTION_OK);
      return false;
    }
  } else if (MOTOR_CMD_DOWN == cmd) {
    if (MOTOR_DOWN != state) {      //反转
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
  
  //判断是否是检测引脚的中断 
  if ((GPIO_SENSOR_UP_PIN == pin ) || ( GPIO_SENSOR_DOWN_PIN == pin)) {
    status = get_status();
    //1 = 卧倒
    //2 = 前倾
    //3 = 直立
    //4 = 后倾
    if (MOTOR_CMD_UP == motor.action){
      if ((MOTOR_UP == status) || (motor.can_stop == status)) {
        motor_last_process(MOTOR_UP);
      }
    } else if (MOTOR_CMD_DOWN == motor.action) {
      if (MOTOR_DOWN == status) {
        //modify 2019-03-18 15:50
        //系统被唤醒后还未初始化时钟，不能调用HAL_Delay()
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

//地锁接受命令执行后会调用该函数向服务器发送resp
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
  //电机当前忙
  if(motor.action != MOTOR_CMD_IDLE)
    return MOTOR_RUNNING;
  
  gDevice.bMotorAbnormal = false;
  
  rtc_disable();
  gDevice.bHasRtcInter = false;
  
  motor.status = get_status();
  
  //如果命令时UP，那么当前状态如果是DOWN或QIANQING的话，当达到HOUQING狂态就可以停止
  //如果命令是UP，那么当前状态是后倾的话，当达到前倾的时候就可以停止
  if(cmd == MOTOR_CMD_UP) {
    if (( MOTOR_DOWN == motor.status ) || ( MOTOR_QIANQING == motor.status )){
      motor.can_stop = MOTOR_HOUQING;
    } else if ( MOTOR_HOUQING == motor.status) {
      motor.can_stop = MOTOR_QIANQING;
    }
    
    //该状态为了防止超声波测量误差，需要向将地锁落下，然后重新抬起
    if ( motor.status == MOTOR_QIANQING ) {
      //将地锁落下
      motor.action = MOTOR_CMD_DOWN_INTERNAL;
      HAL_GPIO_WritePin(GPIO_SENSOR_FORWARE, GPIO_SENSOR_FORWARE_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIO_SENSOR_BACKWARD, GPIO_SENSOR_BACKWARD_PIN, GPIO_PIN_SET);
      rtc_set_timer(5);   //设置5秒定时唤醒
      gDevice.bMotorUseRTC = true;
//      return MOTOR_OK_NOSEND;
      return MOTOR_OK;
    }
    //使能超声波电源
#if 1
    if ( MOTOR_UP != motor.status ) {
      dat = ReadUltraData();
      
      if ( 0xff == dat )
        return MOTOR_ERROR_ULTRA;
      //单位:厘米
      if ( dat < gDevice.u8UltraSafeDistance ) {
        //小于安全距离就认为车位有车，不抬杠
        return MOTOR_ERROR;
      }
    }
#endif
  }

  if (motor_runrun(motor.status, cmd) == true)
  {
    //电机控制超时定时器，这里使用rtc来实现
    if ( motor.status == MOTOR_HOUQING ) {
      sleeptime = 10;
    } else {
      sleeptime = 6;
    }
    rtc_set_timer(sleeptime);   //电机动作超时定时
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
  //使能传感器开关
  MotorSenceSwitch(true);
  
  motor_stop();
  
  motor.action = MOTOR_CMD_IDLE; //当前动作空闲
  motor.status = get_status(); //电机初始化状态
//  motor.last_status = motor.status;
  //注册中断控制回调函数，被引脚中断回调函数调用
  motor_ctrl_cb_register(motor_ctrl_cb);
  //注册引脚中断回调函数，该函数为中断内调用。
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
*     设置PA4和PA5两个引脚的中断禁止
*     因为没办法单独设置两个引脚中断
*     而PB14还是Lora模块的中断引脚，不能够禁止。
*     所以把PA4和PA5设置为普通工作引脚，然后关闭位置检测的电源开关
*
*     如果取消关闭中断，先打开位置检测的开关,然后配置引脚为中断模式
**/

/*
*   设置gpio为普通模式
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
*   设置gpio为中断模式，边沿触发
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
  //如果当前正在执行电机运动指令
  if ((d->u8Cmd == HW_MOTOR_UP) || (d->u8Cmd == HW_MOTOR_DOWN))
    return;
  //开启电源
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
