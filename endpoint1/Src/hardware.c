#include "hardware.h"
#include "stdint.h"
#include "stm32f0xx.h"
#include "stm32f0xx_hal_def.h"
#include "stm32f0xx_hal_adc.h"
#include "stm32f0xx_hal_gpio.h"
#include "beep.h"
#include "motor.h"
#include "rtc.h"

extern uint8_t gOnlineFlag;
extern ADC_HandleTypeDef hadc;

uint8_t hardware_ctrl(HW_CMD cmd)
{
  uint8_t ret = 0;
  
  switch(cmd)
  {
  case HW_MOTOR_UP:       //motor up, Asynchronous
    ret = motor_conctrl(MOTOR_CMD_UP);
    break;
  case HW_MOTOR_DOWN:     //motor down, Asynchronous
    ret = motor_conctrl(MOTOR_CMD_DOWN);
    break;
  case HW_MOTOR_GET:      //motor get status, Synchronization
    ret = motor_get_status();
    break;
  case HW_BEEP_ON:        //beep on, Synchronization
    ret = beep_ctrl(BEEP_ON);
    break;
  case HW_BEEP_OFF:       //beep off, Synchronization
    ret = beep_ctrl(BEEP_OFF);
    break;
  case HW_BEEP_GET:       //beep get status, Synchronization
    ret = beep_ctrl(BEEP_GET);
    break;
  case HW_ADC_GET:        //adc get, Asynchronous
    HAL_ADC_Start(&hadc);
    while(!__HAL_ADC_GET_FLAG(&hadc, ADC_FLAG_EOC));
    ret = HAL_ADC_GetValue(&hadc);
    //8位adc分辨率，单位 3.3/256
    //设定6V为满, 采样电压丿3,6V对应6/4=1.5V 是单片机引脚棿?到的朿?电压倿
    //4V为电量空，采样电压为4V/4=1，是单片机检测到电源耗尽时的电压倿
    // 1.5 × 256 / 3.3 = 116
    // 1 × 256 / 3.3 = 78
    //空电
    ret -= 78;
    //转换成为0-100之间的数倿
    //116-78 = 38
    ret *= 10;
    //这里向上取整，用4做除敿
    break;
  case HW_DEVICE_HEART:
    ret = 1;
    break;
  default:
    ret = 0xff;
    break;
  }
  
  return ret;
}

#if 0
void ReadUID(UID *uid)
{
  uint32_t temp;
  
  temp = (*(volatile unsigned int *)(UID_BASEADDRESS));
  uid->u32CoordinatesXY = temp;
  temp = (*(volatile unsigned int *)(UID_BASEADDRESS + 4));
  uid->u8WaferNumber = (uint8_t)(temp & 0xff);
  uid->u64LotNumber = (temp & 0xffffff00) >> 8;
  temp = (*(volatile unsigned int *)(UID_BASEADDRESS + 8));
  uid->u64LotNumber |= ((uint64_t)temp << 24); 
}
#endif
