#ifndef __RTC_H__
#define __RTC_H__

#include "stdint.h"
#include "stm32f0xx_hal.h"

extern RTC_HandleTypeDef hrtc;

void rtc_set_timer(uint8_t time);

inline void rtc_disable(void)
{
    HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
}

#endif
