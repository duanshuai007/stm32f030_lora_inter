#include <stdio.h>
#include <stdlib.h>

void InvertUint8(uint8_t *dBuf,uint8_t *srcBuf)
{   
    uint8_t i;
    uint8_t tmp = 0;
    for(i=0;i< 8;i++)
    {   
        if(srcBuf[0] & (1 << i)) //将数据颠倒 0-7，1-6，2-5，3-4互换位置
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

uint16_t CRC16_IBM(uint8_t *puchMsg, uint8_t usDataLen)
{
    uint16_t wCRCin = 0x0000;
    uint16_t wCPoly = 0x8005;
    uint8_t wChar = 0;
    uint8_t i;

    while (usDataLen--)   
    {   
        wChar = *(puchMsg++);
        printf("char1:%02x\r\n", wChar);
        InvertUint8(&wChar,&wChar);
        printf("char2:%02x\r\n", wChar);
        wCRCin ^= (wChar << 8); 
        printf("wCRCin1:%04x\r\n", wCRCin);
        for(i = 0;i < 8;i++)
        {   
            if(wCRCin & 0x8000)
                wCRCin = (wCRCin << 1) ^ wCPoly;
            else
                wCRCin = wCRCin << 1;
        }   
        printf("wCRCin2:%04x\r\n", wCRCin);
    }   
    InvertUint16(&wCRCin,&wCRCin);

    return wCRCin;
}

int main()
{
    //int i;
    //uint16_t dat;
    //uint16_t invert_dat;
    //
    //for(i = 0; i < 0x100; i++)
    //{    
    //    dat = (uint8_t)i;
    //    InvertUint16(&invert_dat, &dat);
    //    printf("0x%02x, 0x%02x\r\n", dat, invert_dat);
    //}


    uint16_t crc;
    uint8_t buff[] = {0xa5, 0x0c, 0x90, 0x00, 0x01, 0x01, 0x02, 0x03, 0x04};
    crc = CRC16_IBM(buff, sizeof(buff));
    printf("%04x\r\n", crc);
            
    return 0;
}
