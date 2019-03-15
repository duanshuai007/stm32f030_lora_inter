#include "hardware.h"
#include "stdint.h"
#include "stm32f0xx.h"
#include "beep.h"
#include "motor.h"
#include "rtc.h"
#include "ultra.h"
#include "user_config.h"

extern ADC_HandleTypeDef hadc;

uint8_t hardware_ctrl(Device *d)
{
  uint8_t ret = 1;
  HW_CMD cmd = (HW_CMD)d->u8Cmd;
  
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
  case HW_ADC_GET:        //adc get, Synchronization
    HAL_ADC_Start(&hadc);
    while(!__HAL_ADC_GET_FLAG(&hadc, ADC_FLAG_EOC));
    ret = HAL_ADC_GetValue(&hadc);
    //8λadc�ֱ��ʣ���λ 3.3/256
    //�趨6VΪ��, ������ѹد3,6V��Ӧ6/4=1.5V �ǵ�Ƭ�����ŗ�?���Ėc?��ѹ��
    //4VΪ�����գ�������ѹΪ4V/4=1���ǵ�Ƭ����⵽��Դ�ľ�ʱ�ĵ�ѹ��
    // 1.5 �� 256 / 3.3 = 116
    // 1 �� 256 / 3.3 = 78
    //�յ�
    ret -= 78;
    //ת����Ϊ0-100֮�������
    //116-78 = 38
    ret *= 3;
    //��������ȡ������4������
    break;
  case HW_DEVICE_HEART:
  case HW_ULTRA_GET:
    {
      uint8_t result = 0;
      ret = motor_get_status();
      result |= (ret << 4);
      if (MOTOR_DOWN == ret) {
        ret = ReadUltraData();
        if (ret < d->u8UltraSafeDistance) //С�ڰ�ȫ���룬�г�
          result |= 1;
      } else {
        result |= 0xf;
      }
      ret = result;
    }
    break;
  case HW_ULTRA_SAFE_SET:
    d->u8UltraSafeDistance = (uint8_t)d->u32Identify;
    ret = 0;
    break;
  case HW_ULTRA_SAFE_GET:
    ret = d->u8UltraSafeDistance;
    break;
  default:
    ret = 0xff;
    break;
  }
  
  return ret;
}

