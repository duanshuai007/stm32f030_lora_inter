#ifndef __RTC_H__
#define __RTC_H__

#include "stdint.h"
#include "stm32f0xx_hal.h"
#include "user_config.h"

extern RTC_HandleTypeDef hrtc;

bool rtcSetTimer(uint8_t time);
void rtcDisable(void);
uint32_t rtcGetTime(void);
void rtcCounter(void);

#endif
