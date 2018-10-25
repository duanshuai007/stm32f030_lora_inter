#ifndef __CRC_STM32F0_H__
#define __CRC_STM32F0_H__

#include "stdint.h"
#include "stm32f0xx.h"
#include "stm32f0xx_hal_def.h"
#include "stm32f0xx_hal_crc.h"

extern CRC_HandleTypeDef hcrc;

#define CRC_STM(msg_p, len) HAL_CRC_Calculate(&hcrc, (uint32_t *)msg_p, len)

#endif
