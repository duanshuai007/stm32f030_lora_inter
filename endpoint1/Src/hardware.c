#include "hardware.h"
#include "stdint.h"
#include "stm32f0xx.h"
#include "adc.h"
#include "beep.h"
#include "motor.h"
#include "rtc.h"
#include "ultra.h"
#include "flash.h"
#include "lowpower.h"
#include "user_config.h"

uint8_t hardware_ctrl(Device *ptrDev)
{
  uint8_t ret = 1;
  
  HW_CMD cmd = (HW_CMD)ptrDev->u8Cmd;
  
  switch(cmd)
  {
  case HW_MOTOR_UP:       //motor up, Asynchronous
    ret = motor_conctrl(MOTOR_CMD_UP);
    break;
  case HW_MOTOR_DOWN:     //motor down, Asynchronous
    ret = motor_conctrl(MOTOR_CMD_DOWN);
    break;
  case HW_MOTOR_GET:      //motor get status, Synchronization
    ret = MotorGetStatus();
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
  case HW_ADC_GET:        //adc get, Synchronization
    ret = get_adc_value();
    break;
  case HW_DEVICE_HEART:
  case HW_ULTRA_GET:
    {
      uint8_t result = 0;
      ret = MotorGetStatus();
      result |= (ret << 4);
      if (MOTOR_DOWN == ret) {
        ret = ReadUltraData();
        if (ret == 0xff) {
          //READ ERROR
          result |= ULTRA_READ_ERR;
        } else if (ret < ptrDev->u8UltraSafeDistance) {
          //小于安全距离，有车
          result |= ULTRA_HAVE_CAR;
        } else {
          //无车
          result |= ULTRA_NO_CAR;
        }
      } else {
        result |= ULTRA_NO_READ;
      }
      ret = result;
    }
    break;
  case HW_ULTRA_SAFE_SET:
    ret = FLASH_Write_UltraSafeDistance(ptrDev->u8UltraSafeDistance);
    break;
  case HW_ULTRA_SAFE_GET:
    ret = ptrDev->u8UltraSafeDistance;
    break;
  default:
    ret = 0xff;
    break;
  }
  
  return ret;
}

