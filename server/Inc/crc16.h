#ifndef __CRC16_H__
#define __CRC16_H__

#include "stdint.h"

uint16_t CRC16_IBM(uint8_t *Msg, uint8_t Len);
unsigned char crc8_chk_value(unsigned char *message, unsigned char len);

#endif



