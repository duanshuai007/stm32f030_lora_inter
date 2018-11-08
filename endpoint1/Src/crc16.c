#include "crc16.h"
#include "stdint.h"
#include "stm32f0xx.h"
#include "stm32f0xx_hal_def.h"

static void InvertUint8(uint8_t *dBuf,uint8_t *srcBuf)
{
    uint8_t i;
    uint8_t tmp = 0;
    for(i=0;i< 8;i++)
    {
        if(srcBuf[0] & (1 << i))                //将数据颠倒 0-7，1-6，2-5，3-4互换位置
            tmp |= 1 << (7-i);
    }
    dBuf[0] = tmp;
}

static void InvertUint16(uint16_t *dBuf,uint16_t *srcBuf)
{
    uint8_t i;
    uint16_t tmp = 0;

    for(i=0;i< 16;i++)
    {
        if(srcBuf[0]& (1 << i))
            tmp|=1<<(15 - i);
    }
    dBuf[0] = tmp;
}

#if 0
static void InvertUint32(uint32_t *dBuf,uint32_t *srcBuf)
{
    uint8_t i;
    uint32_t tmp = 0;
    
    for(i=0;i< 32;i++)
    {
        if(srcBuf[0]& (1 << i))
            tmp|=1<<(15 - i);
    }
    dBuf[0] = tmp;
}
#endif

uint16_t CRC16_IBM(uint8_t *puchMsg, uint8_t usDataLen)
{
    uint16_t wCRCin = 0x0000;
    uint16_t wCPoly = 0x8005;
    uint8_t wChar = 0;
    uint8_t i;
    
    while (usDataLen--) 	
    {
        wChar = *(puchMsg++);
        InvertUint8(&wChar,&wChar);
        wCRCin ^= (wChar << 8);
        for(i = 0;i < 8;i++)
        {
            if(wCRCin & 0x8000)
                wCRCin = (wCRCin << 1) ^ wCPoly;
            else
                wCRCin = wCRCin << 1;
        }
    }
    InvertUint16(&wCRCin,&wCRCin);
    
    return wCRCin;
}
