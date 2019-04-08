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
  
  //执行功能
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
    //不更新地锁状态
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
    //设备复位
//    LoraSendResp(0x12, 0x00, 0x98760000);
//    HAL_Delay(300);
    NVIC_SystemReset();
    break;
  default:
    ret = 0xff;
    break;
  }
  
  //发送响应信息
  switch(cmd) 
  {
  case HW_MOTOR_UP:
  case HW_MOTOR_DOWN:
    //该段代码用于电机执行动作后返回电机开始动作的resp
    switch(ret) {
    case MOTOR_OK:
      //正常执行动作后立刻返回一个响应信息
      //如果等于CMD_EXEC_DOING说明是前倾后执行的指令，此时不需要再次发送run状态
      if (pDev->u8CmdDone != CMD_EXEC_DOING) { 
        LoraSendResp(pDev->u8Cmd, MOTOR_RUN, pDev->u32Identify);
      }
      break;
    case MOTOR_DONTDO:
      //电机处于目标位置不需要执行动作,直接关闭位置传感器
      ret = 0;
    case MOTOR_ERROR:
    case MOTOR_ERROR_ULTRA:
      //超声波状态错误，直接关闭位置传感器
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
    //指令执行结束，清空状态信息
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

