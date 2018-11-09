#ifndef __RTC_H__
#define __RTC_H__

#include "stdint.h"
#include "stm32f1xx_hal.h"

uint32_t GetRTCTime(void);
uint16_t GetRTCTimeMinAndSec(void);

#endif

