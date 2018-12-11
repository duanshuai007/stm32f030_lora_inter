#include "flash.h"
#include "stm32f0xx.h"
#include "stdlib.h"
#include "lora.h"

/*
*   FLASH内保存设备的密钥信息
*   密钥以加密方式保存在FLASH被
*   需要用tea解密方式进行解密
*   
*   [head]    [message]
*   AABBCDDC  .......
*
*   [   m   e  s   s   a   g   e   ]
*   [title][加密内容]
*   对加密内容解密后，需要根据title对解密后的内容yyyyyyyyyyyyyyyyyyyyyyyy
*   进行摘除，最后得到实际的密钥
*/

static void strtohex(uint16_t *dst, uint32_t *src, uint8_t srclen)
{
  uint8_t i, j;
  uint8_t dat;
  
  for(i=0; i < srclen; i++) {
    for ( j = 0; j < 4; j++) {
    
      //src是32为地址指针，+1对应一个32位类型，指向下一个数组元素的位置
      dat = (uint8_t)((*(src + i)) >> (j*8));
      
      if ((dat >= 'a') && (dat <= 'z')) {
        dat -= ('a' - 0x0a);
      } else if ((dat >= '0') && (dat <= '9')) {
        dat -= '0';
      } else if ((dat >= 'A') && (dat <= 'Z')) {
        dat -= ('A' - 0x0a);
      }
      
      *dst <<= 4;
      *dst |= dat;
    }
  }
}

//从FLASH地址0X0800F000处读取本机id，server id，信道
//FROG+[32位 serverid]+[32位 本机id]+[32位信道]
bool FLASH_Init(uint16_t *serverid, uint16_t *localid, uint8_t *sendch, uint8_t *recvch)
{
  uint32_t buffer[3];  //解密密钥
  uint32_t dat;
  uint16_t channel;
  
  dat = READ_FLASH_WORD(FLASH_DATABASE_START);
  if( FLASH_MSG_HEAD == dat) {
    //如果flash内已经有了有效的数据
    buffer[0] = READ_FLASH_WORD(FLASH_DATABASE_START + 4);
    buffer[1] = READ_FLASH_WORD(FLASH_DATABASE_START + 8);
    buffer[2] = READ_FLASH_WORD(FLASH_DATABASE_START + 12);
    
    strtohex(serverid,  &buffer[0], 1);
    strtohex(localid,   &buffer[1], 1);
    strtohex(&channel,  &buffer[2], 1);
    //读取的最后一个位置的数据就是通道，高16b是接收，低16b是发送
    
    *sendch = (uint8_t)channel;
    *recvch = (uint8_t)(channel >> 8);
    
    return true;    
  }

  return false;
}
