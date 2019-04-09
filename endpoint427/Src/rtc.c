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
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  
  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
  
  gDevice.u32GlobalTimeCount = 
    ((uint32_t)sTime.Hours * 3600) + ((uint32_t)sTime.Minutes * 60) + ((uint32_t)sTime.Seconds);
}

//static uint32_t g_u32RtcTime = 0; //∑¿÷πcpuŒ¥–›√ﬂµº÷¬÷ÿ∏¥…Ë÷√ƒ÷÷”
bool rtcSetTimer(uint8_t time)
{
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  RTC_AlarmTypeDef sAlarm;
//  uint32_t counter_time = 0;
//  uint32_t counter_alarm = 0;
//  uint8_t u8Retry = 0;
  uint32_t u32SecondCount = 0;
  
  HAL_RTC_GetAlarm(&hrtc, &sAlarm, RTC_ALARM_A, RTC_FORMAT_BIN);
  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
  u32SecondCount = ((uint32_t)sTime.Hours * 3600) + ((uint32_t)sTime.Minutes * 60) + ((uint32_t)sTime.Seconds) + (uint32_t)time;
//  if (counter_time == g_u32RtcTime)
//  {
//    return false;    
//  }
//  g_u32RtcTime = counter_time;
//  counter_alarm = counter_time + time;
  
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
  sAlarm.AlarmTime.Hours   = (uint8_t)((u32SecondCount / 3600) % 24);
  sAlarm.AlarmTime.Minutes = (uint8_t)((u32SecondCount % 3600) / 60);
  sAlarm.AlarmTime.Seconds = (uint8_t)((u32SecondCount % 3600) % 60);

  if (HAL_OK != HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A))
    return false;
    
  if (HAL_OK != HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN))
    return false;

  return true;
}