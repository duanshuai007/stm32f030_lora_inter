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
  HAL_Delay(500); //测试时所测到的超声波能测到数据的最小时长
  //禁止超声波电源
  HAL_GPIO_WritePin(GPIO_ULTRASONIC, GPIO_ULTRASONIC_PIN, GPIO_PIN_SET);
  
  if ((gDevice.u16UltraDistance == 0xffff) || (gDevice.u16UltraDistance == 0)) {
    ret = 0xff;
  } else {
    ret = (gDevice.u16UltraDistance/10);  //转换为厘米单位
    if (ret > 0xff)   //防止数据溢出
      ret = 200;
  }
  
  return (uint8_t)ret;
}