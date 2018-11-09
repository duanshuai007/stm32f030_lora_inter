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

//���������ĸ�״̬ �ɰ汾����
//up    down
//1     0       ����״̬
//0     0       ֱ��״̬
//0     1       ǰ��״̬
//1     1       �Ե�״̬

uint32_t gsu32_time_inter = 0;    //��������жϵļ��
uint32_t gsu32_last_time = 0;       //��һ���жϷ�����ʱ��
uint8_t gsu8_count = 0;             //���жϳ������ϵĴ���ʱ�����һ��ʱ������һ��֪ͨ
bool isDelay = false;         //�ӳٷ��ͱ�־���������˵�һ���ӳٷ���ʱ����Ҫ��Ϊtrue

//��ʱ������100ms
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

//��ʱ����С��ʱ��λ100ms
static void timer_disable(motor_timer *mt)
{
  mt->enable = false;
  mt->cur_count = 0;
  mt->max_count = 0;
}

static void timer_set(motor_timer *mt, uint8_t time)
{
  mt->max_count = time;       //��ʱʱ��=time��100ms
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
        HAL_Delay(5);       //��ʱʱ���ǵ���������ʱ����ж�����ס�ˣ�����ּ�⵽DOWNλ�õ�״
        //������Ҫ������ʱ�����ж�һ��
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
    } else {   //û���յ�������ǲ����˶���
      
      //������쳣�����������쳣������־��ʹcpu����������
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
  
  if ( motor.action != MOTOR_CMD_IDLE ) {         //�����ж�����ʱ��ִ�г�ʱ�ص�
    motor.ctrl_cb((MOTOR_CB_TYPE)motor.status);
  }
  
  motor.action = MOTOR_CMD_IDLE;
}

//���������쳣����ʱ����gpio_cb�ڵ��øú���
static void motor_check_cb(MOTOR_CB_TYPE m)
{
  //    MOTOR_TaskPost(INTER_RESP_TYPE, hr);
  //�����˵���쳣��������Ϊ��ʱ����Ӧ�ô���������ʱ��
  //������Ҫ���ö�Ӧ�ı�־λ�ó����ܹ������쳣resp
  gDevice.u8Cmd = HW_MOTOR_ABNORMAL;
  gDevice.u8CmdRunning = CMD_RUN;
  gDevice.u8Resp = m;
  gDevice.u8CmdDone = 1;
  
}

//������������ִ�к����øú��������������resp
static void motor_ctrl_cb(MOTOR_CB_TYPE m)
{
  //    MOTOR_TaskPost(ACTION_RESP_TYPE, hr);
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
    //        timer_disable(&motor.ctrl_timer);
    //������Ƴ�ʱ��ʱ��������ʹ��rtc��ʵ��
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
  //ʹ�ܴ���������
  sensor_switch(true);
  
  motor_stop();
  
  motor.action = MOTOR_CMD_IDLE; //��ǰ��������
  motor.status = get_status(); //�����ʼ��״̬
  motor.last_status = motor.status;
  //ע���жϿ��ƻص��������������жϻص���������
  motor_ctrl_cb_register(motor_ctrl_cb);
  //ע���쳣�����ص������������Ƴ�ʱ�ص���������
  motor_check_cb_register(motor_check_cb);
  //ע�������жϻص��������ú���Ϊ�ж��ڵ��á�
  motor_gpio_cb_register(motor_gpio_cb);
  
  //��ֹ��ʱ������  
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
