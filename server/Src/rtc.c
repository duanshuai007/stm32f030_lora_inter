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
*   返回系统启动以来的秒数，最大是23×60×60即一天的秒总数
*/
uint32_t GetRTCTime(void)
{
  return sui32rtctime;
}

/*
*   获取系统时间，返回的是特殊格式时间
*   0xaabb
*   aa 是16进制的分钟数
*   bb 是16进制的秒数
*/
uint16_t GetRTCTimeMinAndSec(void)
{
  uint16_t time;
  
  RTC_TimeTypeDef sTime;
	
  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

  time = (((uint16_t)(sTime.Minutes) << 4) | sTime.Seconds);

  return time;
}




