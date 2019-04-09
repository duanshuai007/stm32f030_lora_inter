#include "rtc.h"
#include "stdint.h"
#include "stm32f0xx.h"
#include "user_config.h"
#include "lora.h"
#include "beep.h"

extern RTC_HandleTypeDef hrtc;
extern Device gDevice;

void rtcCounter(void)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  gDevice.u32GlobalTimeCount = 
    ((uint32_t)sTime.Hours * 3600) + ((uint32_t)sTime.Minutes * 60) + ((uint32_t)sTime.Seconds);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
}

static uint32_t g_u32RtcTime = 0; //·ÀÖ¹cpuÎ´ÐÝÃßµ¼ÖÂÖØ¸´ÉèÖÃÄÖÖÓ
bool rtcSetTimer(uint8_t time)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};
  uint32_t counter_time = 0;
  uint32_t counter_alarm = 0;
  uint8_t u8Retry = 0;

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
  counter_time = ((uint32_t)sTime.Hours * 3600) + ((uint32_t)sTime.Minutes * 60) + ((uint32_t)sTime.Seconds);
  if (counter_time == g_u32RtcTime)
  {
    return false;    
  }
  g_u32RtcTime = counter_time;

  counter_alarm = counter_time + time;
  HAL_RTC_GetAlarm(&hrtc, &sAlarm, RTC_ALARM_A, RTC_FORMAT_BIN);
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
  sAlarm.AlarmTime.Hours   = (uint8_t)((counter_alarm / 3600) % 24);
  sAlarm.AlarmTime.Minutes = (uint8_t)((counter_alarm % 3600) / 60);
  sAlarm.AlarmTime.Seconds = (uint8_t)((counter_alarm % 3600) % 60);
  
  while (1)
  {
    if (HAL_OK == HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A))
    {
      if (HAL_OK == HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN))
      { 
        return true;
      }
    }
    HAL_Delay(5);
    u8Retry++;
    if (u8Retry >= 3)
    {
      NVIC_SystemReset();
    }
  }  
}