#include "reboot.h"
#include "user_config.h"
#include "stm32f1xx.h"
#include "list.h"
#include "maincontrol.h"

static bool ready_reboot = false;

bool isReadyReboot(void)
{
  return ready_reboot;
}

void gotoReBoot(void)
{
  closeF405Communication();   //停止从f405处接受新指令
  ready_reboot = true;
}

void ReBootTask(void)
{
  if (ready_reboot == true) {
    if (waitAllDeviceisIdle() == true) {  //等待所有设备指令直接结束，再次期间不再发送心跳包
      //UpdateFlash();    //更新flash
      //接收到服务器发送来的升级指令，复位，（在这里复位，程序是从bootloader处重新运行吗？？？）
      //经过测试，结论：复位后程序会从0X08000000位置处开始执行
      __set_FAULTMASK(1);
      NVIC_SystemReset();  
    }
  }
}