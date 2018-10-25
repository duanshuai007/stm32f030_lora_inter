#ifndef __LOWPOWER_H__
#define __LOWPOWER_H__

#include "stdint.h"
#include "stm32f0xx_hal.h"

void LowPowerInit(uint8_t mode);
void CloseNotUsedPeriphClock(void);
uint8_t ProcessTheData(void);
void UART_ReInit(UART_HandleTypeDef *huart);
uint8_t SyncCMDDone(void);

#endif
