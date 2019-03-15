#include "ultra.h"
#include "lowpower.h"
#include "user_config.h"
#include <stdint.h>

extern Device gDevice;

uint8_t ReadUltraData(void)
{
  uint16_t ret;
  
  uart2_reinit();
  
  gDevice.u16UltraDistance = 0;
  
  HAL_GPIO_WritePin(GPIO_ULTRASONIC, GPIO_ULTRASONIC_PIN, GPIO_PIN_RESET);
  HAL_Delay(500); //����ʱ���⵽�ĳ������ܲ⵽���ݵ���Сʱ��
  //��ֹ��������Դ
  HAL_GPIO_WritePin(GPIO_ULTRASONIC, GPIO_ULTRASONIC_PIN, GPIO_PIN_SET);
  
  if ((gDevice.u16UltraDistance == 0xffff) || (gDevice.u16UltraDistance == 0)) {
    ret = 0xff;
  } else {
    ret = (gDevice.u16UltraDistance/10);  //ת��Ϊ���׵�λ
    if (ret > 0xff)   //��ֹ�������
      ret = 200;
  }
  
  return (uint8_t)ret;
}