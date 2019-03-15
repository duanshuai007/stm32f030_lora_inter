#include "rtc.h"
#include "stdint.h"
#include "stm32f1xx.h"

volatile static uint32_t sui32rtctime = 0;
extern RTC_HandleTypeDef hrtc;

void RTCTask(void)
{
  RTC_TimeTypeDef sTime;
  
  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);  
  sui32rtctime = ((sTime.Minutes * 60) + sTime.Seconds);
}

/*
*   ����ϵͳ���������������������23��60��60��һ���������
*/
uint32_t GetRTCTime(void)
{
  return sui32rtctime;
}

/*
*   ��ȡϵͳʱ�䣬���ص��������ʽʱ��
*   0xaabb
*   aa ��16���Ƶķ�����
*   bb ��16���Ƶ�����
*/
uint16_t GetRTCTimeMinAndSec(void)
{
  uint16_t time;
  
  RTC_TimeTypeDef sTime;
	
  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

  time = (((uint16_t)(sTime.Minutes) << 4) | sTime.Seconds);

  return time;
}




