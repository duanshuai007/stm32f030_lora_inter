#ifndef _ULTRA_MODE_
#define _ULTRA_MODE_

#include <stdint.h>

//·µ»ØÖµ
#define ULTRA_NO_CAR          0
#define ULTRA_HAVE_CAR        1
#define ULTRA_READ_ERR        0xe
#define ULTRA_NO_READ         0xf
//
#define ULTRA_MSG_HEAD        0xff
#define ULTRA_MSG_LEN         4
#define ULTRA_READ_ERROR      0xff
#define ULTRA_MAX_READ_TIMES  10

typedef struct {
  uint8_t head;
  uint8_t data_h;
  uint8_t data_l;
  uint8_t sum;
} UltraSonicType;

uint8_t UltraCheckTask(void);
uint8_t ReadUltraData(void);

void UltraUartEnable(void);
void UltraUartDisable(void);

#endif