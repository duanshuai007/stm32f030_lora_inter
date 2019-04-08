#ifndef _STM32_FLASH_H_
#define _STM32_FLASH_H_

#include "stdint.h"
#include "user_config.h"

#define FLASH_DATABASE_START  0x0800f000
#define FLASH_MSG_HEAD        0x474f5246        //"FROG"

#define FLASH_UPDATE_ULTRA_DISTANCE       0
#define FLASH_UPDATE_ULTRA_TIMEINTERVAL   1

#define READ_FLASH_WORD(addr) (*(volatile uint32_t *)(addr))

bool FLASH_Init(uint16_t *serverid, uint16_t *localid, uint8_t *sendch, uint8_t *recvch, uint16_t *ultra_safevalue, uint16_t *ultra_timeinterval);
uint8_t FLASH_Write(uint8_t flag, uint8_t value);

#endif