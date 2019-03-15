#ifndef _ULTRA_MODE_
#define _ULTRA_MODE_

#include <stdint.h>

typedef struct {
  uint8_t head;
  uint8_t data_h;
  uint8_t data_l;
  uint8_t sum;
} UltraSonicType;

uint8_t ReadUltraData(void);

#endif