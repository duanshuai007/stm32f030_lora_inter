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

//传感器的四个状态 旧版本地锁
//up    down
//1     0       后倾状态
//0     0       直立状态
//0     1       前倾状态
//1     1       卧倒状态

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
        motor_stop();
        rtc_disable();
        motor.status = MOTOR_UP;
        motor.action = MOTOR_CMD_IDLE;
        motor.last_status = MOTOR_UP;
        motor.ctrl_cb(ACTION_OK);
      }
    } else if (MOTOR_CMD_DOWN == motor.action){
      if (MOTOR_DOWN == status) {
//        HAL_Delay(1);   //经过测试，在stm32f103休眠模式方式工作时，会导致地锁倒下时不能正常的停止。
        //延时时间是当地锁落下时如果有东西挡住了，会出现检测到DOWN位置的状
        //所以需要短暂延时后再判断一下
        if (get_status() == status) {
          motor_stop();
          rtc_disable();
          motor.status = MOTOR_DOWN;
          motor.action = MOTOR_CMD_IDLE;
          motor.last_status = MOTOR_DOWN;
          motor.ctrl_cb(ACTION_OK);
        }
      }
    } else {   //没有收到命令，但是产生了动作
      
      //电机有异常动作，设置异常动作标志
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
  
  if ( motor.action != MOTOR_CMD_IDLE ) {         //避免有动作的时候执行超时回掉
    motor.ctrl_cb((MOTOR_CB_TYPE)motor.status);
  }
  
  motor.action = MOTOR_CMD_IDLE;
}

//地锁接受命令执行后会调用该函数向服务器发送resp
static void motor_ctrl_cb(MOTOR_CB_TYPE m)
{
  gDevice.u8Resp = m;
  gDevice.u8CmdDone = 1;
}

MOTOR_STATUS motor_conctrl(MOTOR_CMD cmd) 
{
  //电机当前忙
  if(motor.action != MOTOR_CMD_IDLE)
    return MOTOR_RUNNING;
  
  motor.status = get_status();
  
  //如果命令时UP，那么当前状态如果是DOWN或QIANQING的话，当达到HOUQING狂态就可以停止
  //如果命令是UP，那么当前状态是后倾的话，当达到前倾的时候就可以停止
  
  if(cmd == MOTOR_CMD_UP) {
    if (( MOTOR_DOWN == motor.status ) || ( MOTOR_QIANQING == motor.status )){
      motor.can_stop = MOTOR_HOUQING;
    } else if ( MOTOR_HOUQING == motor.status) {
      motor.can_stop = MOTOR_QIANQING;
    }
  }

  if (motor_runrun(motor.status, cmd) == true)
  {
    //电机控制超时定时器，这里使用rtc来实现
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
  //使能传感器开关
  sensor_switch(true);
  
  motor_stop();
  
  motor.action = MOTOR_CMD_IDLE; //当前动作空闲
  motor.status = get_status(); //电机初始化状态
  motor.last_status = motor.status;
  //注册中断控制回调函数，被引脚中断回调函数调用
  motor_ctrl_cb_register(motor_ctrl_cb);
  //注册引脚中断回调函数，该函数为中断内调用。
  motor_gpio_cb_register(motor_gpio_cb);
  
  //禁止定时器功能  
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
    //如果是使能中断，就要先开启电源。
    sensor_switch(true);
    HAL_Delay(100);
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  //如果是关闭中断，则需要后切断电源
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
  //如果发现了异常动作标志
  if ((gDevice.u8InteFlag == 1) && (gDevice.u8InterDone == 0)) {
    motor_input_pin_off_interrupt(true);
    rtc_set_timer(1);
    gDevice.u8InteFlag = 0;
    gDevice.u8InterDone = 1;
    return 1;
  } else if ( gDevice.u8InterDone ) {
    //唤醒后
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

