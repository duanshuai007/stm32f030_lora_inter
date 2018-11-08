#ifndef __RTC_H__
#define __RTC_H__

#include "stdint.h"
#include "stm32f0xx_hal.h"

extern RTC_HandleTypeDef hrtc;

void rtc_set_timer(uint8_t time);

void rtc_disable(void);

#endif
