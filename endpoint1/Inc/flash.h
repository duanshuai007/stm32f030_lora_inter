#ifndef _STM32_FLASH_H_
#define _STM32_FLASH_H_

#include "stdint.h"
#include "user_config.h"

#define FLASH_DATABASE_START  0x0800f000
#define FLASH_MSG_HEAD        0x474f5246        //"FROG"

#define READ_FLASH_WORD(addr) (*(volatile uint32_t *)(addr))

//bool FLASH_Init(void);
bool FLASH_Init(uint16_t *serverid, uint16_t *localid, uint8_t *sendch, uint8_t *recvch);

#endif