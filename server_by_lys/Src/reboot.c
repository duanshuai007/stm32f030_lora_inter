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
  closeF405Communication();   //ֹͣ��f405��������ָ��
  ready_reboot = true;
}

void ReBootTask(void)
{
  if (ready_reboot == true) {
    if (waitAllDeviceisIdle() == true) {  //�ȴ������豸ָ��ֱ�ӽ������ٴ��ڼ䲻�ٷ���������
      //UpdateFlash();    //����flash
      //���յ�������������������ָ���λ���������︴λ�������Ǵ�bootloader�����������𣿣�����
      //�������ԣ����ۣ���λ�������0X08000000λ�ô���ʼִ��
      __set_FAULTMASK(1);
      NVIC_SystemReset();  
    }
  }
}