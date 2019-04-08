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
#include "lora.h"

extern Motor gMotor;

void hardware_ctrl(Device *pDev)
{
  uint8_t ret = 1;
  HW_CMD cmd = (HW_CMD)pDev->u8Cmd;
  
  //ִ�й���
  switch(cmd)
  {
  case HW_MOTOR_UP:       //motor up, Asynchronous
    setMotorInterrupt();
    ret = motor_conctrl(MOTOR_CMD_UP);
    break;
  case HW_MOTOR_DOWN:     //motor down, Asynchronous
    setMotorInterrupt();
    ret = motor_conctrl(MOTOR_CMD_DOWN);
    break;
  case HW_MOTOR_GET:      //motor get status, Synchronization
    //�����µ���״̬
    ret = getMotorStatus();
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
    ret = UltraCheckTask();
    break;
  case HW_ULTRA_SAFE_SET:
    ret = FLASH_Write(FLASH_UPDATE_ULTRA_DISTANCE, pDev->u8UltraSafeDistance);
    break;
  case HW_ULTRA_SAFE_GET:
    ret = pDev->u8UltraSafeDistance;
    break;
  case HW_ULTRA_CHECKTIME_SET:
    ret = FLASH_Write(FLASH_UPDATE_ULTRA_TIMEINTERVAL, pDev->u8UltraCheckTimeinterval);
    break;
  case HW_ULTRA_CHECKTIME_GET:
    ret = pDev->u8UltraCheckTimeinterval;
    break;
  case HW_SYSTEM_RESET:
    //�豸��λ
//    LoraSendResp(0x12, 0x00, 0x98760000);
//    HAL_Delay(300);
    NVIC_SystemReset();
    break;
  default:
    ret = 0xff;
    break;
  }
  
  //������Ӧ��Ϣ
  switch(cmd) 
  {
  case HW_MOTOR_UP:
  case HW_MOTOR_DOWN:
    //�öδ������ڵ��ִ�ж����󷵻ص����ʼ������resp
    switch(ret) {
    case MOTOR_OK:
      //����ִ�ж��������̷���һ����Ӧ��Ϣ
      //�������CMD_EXEC_DOING˵����ǰ���ִ�е�ָ���ʱ����Ҫ�ٴη���run״̬
      if (pDev->u8CmdDone != CMD_EXEC_DOING) { 
        LoraSendResp(pDev->u8Cmd, MOTOR_RUN, pDev->u32Identify);
      }
      break;
    case MOTOR_DONTDO:
      //�������Ŀ��λ�ò���Ҫִ�ж���,ֱ�ӹر�λ�ô�����
      ret = 0;
    case MOTOR_ERROR:
    case MOTOR_ERROR_ULTRA:
      //������״̬����ֱ�ӹر�λ�ô�����
      cancelMotorInterrupt();
      LoraSendResp(pDev->u8Cmd, ret, pDev->u32Identify);
      pDev->u8CmdRunning = CMD_STOP;
      pDev->u8CmdDone = CMD_EXEC_NONE;
      pDev->u8Cmd = HW_CMD_NONE;
      break;
    default:
      break;
    }
    break;
  case HW_MOTOR_GET:
  case HW_ULTRA_GET:
  case HW_DEVICE_HEART:
  case HW_BEEP_ON:
  case HW_BEEP_OFF:
  case HW_BEEP_GET:
  case HW_ADC_GET:
  case HW_ULTRA_SAFE_SET:
  case HW_ULTRA_SAFE_GET:
  case HW_ULTRA_CHECKTIME_SET:
  case HW_ULTRA_CHECKTIME_GET:
    //give server some response
    LoraSendResp(pDev->u8Cmd, ret, pDev->u32Identify);
//    uint32_t u32Msg;
//    u32Msg = (((uint32_t)gWakeUpCount << 24) | ((uint32_t)gWakeUpFromLoraCount << 16) | ((uint32_t)gRecvMsgCount << 8) | (uint32_t)gSendMsgCount);
//    LoraSendMsg(pDev->u8Cmd, ret, u32Msg);
    //ָ��ִ�н��������״̬��Ϣ
    pDev->u8CmdRunning = CMD_STOP;
    pDev->u8Cmd = HW_CMD_NONE;
    break;
  default:
    pDev->u8CmdRunning = CMD_STOP;
    pDev->u8Cmd = HW_CMD_NONE;
    break;
  }
//  return ret;
}

