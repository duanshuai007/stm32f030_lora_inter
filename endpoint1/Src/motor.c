#include "motor.h"
#include "hardware.h"
#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_def.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_hal_tim.h"

#include "rtc.h"
#include "user_config.h"

Motor motor;

extern Device gDevice;
extern TIM_HandleTypeDef htim6;

//传感器的四个状态 旧版本地锁
//up    down
//1     0       后倾状态
//0     0       直立状态
//0     1       前倾状态
//1     1       卧倒状态

uint32_t gsu32_time_inter = 0;    //最近两次中断的间隔
uint32_t gsu32_last_time = 0;       //上一次中断发生的时间
uint8_t gsu8_count = 0;             //当中断持续不断的触发时，会隔一段时间允许发一次通知
bool isDelay = false;         //延迟发送标志，当出现了第一次延迟发送时就需要置为true

//定时器周期100ms
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

//定时器最小定时单位100ms
static void timer_disable(motor_timer *mt)
{
  mt->enable = false;
  mt->cur_count = 0;
  mt->max_count = 0;
}

static void timer_set(motor_timer *mt, uint8_t time)
{
  mt->max_count = time;       //延时时间=time×100ms
  mt->enable = true;
  HAL_TIM_Base_Start_IT(&htim6);
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
  return MOTOR_OK;
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
      if ((MOTOR_UP == status) || (motor.can_stop == status)){
        motor_stop();
        //                timer_disable(&motor.ctrl_timer);
        rtc_disable();
        motor.status = MOTOR_UP;
        motor.action = MOTOR_CMD_IDLE;
        motor.last_status = MOTOR_UP;
        motor.ctrl_cb(ACTION_OK);
      }
    } else if (MOTOR_CMD_DOWN == motor.action){
      if (MOTOR_DOWN == status) {
        HAL_Delay(5);       //延时时间是当地锁落下时如果有东西挡住了，会出现检测到DOWN位置的状
        //所以需要短暂延时后再判断一下
        if (get_status() == status) {
          motor_stop();
          //                    timer_disable(&motor.ctrl_timer);
          rtc_disable();
          motor.status = MOTOR_DOWN;
          motor.action = MOTOR_CMD_IDLE;
          motor.last_status = MOTOR_DOWN;
          motor.ctrl_cb(ACTION_OK);
        }
      }
    } else {   //没有收到命令，但是产生了动作
      
      //电机有异常动作，设置异常动作标志，使cpu不进入休眠
//      gInterFlag = 1;
//      gMode = 1;
      
      motor.status = status;
      uint32_t curTime = HAL_GetTick();
      gsu32_time_inter = curTime - gsu32_last_time;
      gsu32_last_time = curTime;
      
      if (( gsu32_time_inter < 200) || (true == isDelay )) {
        gsu8_count++;
        if ( gsu8_count < 10 ) {
          timer_disable(&motor.xiaodou_timer);
          timer_set(&motor.xiaodou_timer, 2);
          isDelay = true;
        }
      } else {
        gsu8_count = 0;
        timer_disable(&motor.xiaodou_timer);
        timer_set(&motor.xiaodou_timer, 1);
      }
    }
  }
}

static void motor_timer_gpio_xiaodou_cb(void)
{
  if (motor.xiaodou_timer.enable == false)
    return;
  
  motor.xiaodou_timer.cur_count++;
  
  if (motor.xiaodou_timer.cur_count < motor.xiaodou_timer.max_count) {
    return;
  } else {
    isDelay = false;
    gsu8_count = 0;
    
    timer_disable(&motor.xiaodou_timer);
    
    if (motor.status != motor.last_status) {
      motor.last_status = motor.status;
      motor.check_cb((MOTOR_CB_TYPE)motor.status);
    }
  }
  
  if (motor.xiaodou_timer.enable == false) {
    HAL_TIM_Base_Stop_IT(&htim6);
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

//地锁产生异常动作时会在gpio_cb内调用该函数
static void motor_check_cb(MOTOR_CB_TYPE m)
{
  //    MOTOR_TaskPost(INTER_RESP_TYPE, hr);
  //触发了电机异常动作后因为此时地锁应该处于无命令时期
  //所以需要设置对应的标志位让程序能够发送异常resp
  gDevice.u8Cmd = HW_MOTOR_ABNORMAL;
  gDevice.u8CmdRunning = CMD_RUN;
  gDevice.u8Resp = m;
  gDevice.u8CmdDone = 1;
  
}

//地锁接受命令执行后会调用该函数向服务器发送resp
static void motor_ctrl_cb(MOTOR_CB_TYPE m)
{
  //    MOTOR_TaskPost(ACTION_RESP_TYPE, hr);
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
    //        timer_disable(&motor.ctrl_timer);
    //电机控制超时定时器，这里使用rtc来实现
    if ( motor.status == MOTOR_HOUQING ) {
      //            timer_set(&motor.ctrl_timer, 100);
      rtc_set_timer(7);
    } else {
      //            timer_set(&motor.ctrl_timer, 50);
      rtc_set_timer(4);
    }
  }
  
  return MOTOR_OK;
}

static void motor_check_cb_register(motor_check_callback cb)
{
  motor.check_cb = cb;
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
  //注册异常动作回调函数，被控制超时回调函数调用
  motor_check_cb_register(motor_check_cb);
  //注册引脚中断回调函数，该函数为中断内调用。
  motor_gpio_cb_register(motor_gpio_cb);
  
  //禁止定时器功能  
  motor.xiaodou_timer.enable = false;
  motor.xiaodou_timer.cur_count = 0;
  motor.xiaodou_timer.max_count = 0;
  
  motor.xiaodou_timer_cb = motor_timer_gpio_xiaodou_cb;
  motor.ctrl_timer_cb = motor_timer_ctrl_timeout_cb;
}

MOTOR_STATUS motor_get_status(void)
{
  if (MOTOR_CMD_IDLE != motor.action) {
    return MOTOR_RUNNING;
  }
  return get_status();
}
