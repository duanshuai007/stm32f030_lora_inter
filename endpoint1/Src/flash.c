#include "flash.h"
#include "stm32f0xx.h"
#include "stdlib.h"

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

static void u8tostr(uint32_t *dst, uint8_t *src)
{
  uint8_t i;
  uint8_t dat;
  //将32位的指针转换为8位的指针
  uint8_t *pDst = (uint8_t *)dst;
  *(pDst+0) = 0x30;
  *(pDst+1) = 0x30;
  
  for(i=0;i<2;i++)
  {
    dat = ((*src) >> (4 * (1 - i))) & 0x0f;
    
    //十六进制数转换为字符
    if (dat <= 9) {
      dat += '0';
    } else {
      dat += ('a' - 0x0a);
    }
    
    *(pDst + i + 2) = dat;
  }
}

//从FLASH地址0X0800F000处读取本机id，server id，信道
//FROG+[32位 serverid]+[32位 本机id]+[32位信道]
bool FLASH_Init(uint16_t *serverid, uint16_t *localid, uint8_t *sendch, uint8_t *recvch, uint16_t *ultra_safevalue, uint16_t *ultra_checktime)
{
  uint32_t buffer[5];
  uint32_t dat;
  uint16_t channel;
  
  dat = READ_FLASH_WORD(FLASH_DATABASE_START);
  if( FLASH_MSG_HEAD != dat) 
    return false;
  
  //如果flash内已经有了有效的数据
  buffer[0] = READ_FLASH_WORD(FLASH_DATABASE_START + 4);
  buffer[1] = READ_FLASH_WORD(FLASH_DATABASE_START + 8);
  buffer[2] = READ_FLASH_WORD(FLASH_DATABASE_START + 12);
  buffer[3] = READ_FLASH_WORD(FLASH_DATABASE_START + 16);
  buffer[4] = READ_FLASH_WORD(FLASH_DATABASE_START + 20);
  
  strtohex(serverid,  &buffer[0], 1);
  strtohex(localid,   &buffer[1], 1);
  strtohex(&channel,  &buffer[2], 1);
  strtohex(ultra_safevalue, &buffer[3], 1);
  strtohex(ultra_checktime, &buffer[4], 1);
  //读取的最后一个位置的数据就是通道，高16b是接收，低16b是发送
  
  *sendch = (uint8_t)channel;
  *recvch = (uint8_t)(channel >> 8);
  
  return true;
}

//将设置的超声波安全距离写入到flash中,输入十六进制数
//保存在flash中的是字符串形式的十六进制数
//比如distance=0x28
//保存在flash中则为 0x30 0x30 0x32 0x38
//flag: 0 表示写入安全距离
//    : 1 表示写入检测间隔
//返回值:0 success
//      1 failed
uint8_t FLASH_Write(uint8_t flag, uint8_t value)
{
  uint32_t buffer[5];
  uint32_t dat;
  uint32_t pageerror = 0;
  FLASH_EraseInitTypeDef f;
  
  dat = READ_FLASH_WORD(FLASH_DATABASE_START);
  if( FLASH_MSG_HEAD != dat) 
    return 1;
  
  buffer[0] = READ_FLASH_WORD(FLASH_DATABASE_START + 4);
  buffer[1] = READ_FLASH_WORD(FLASH_DATABASE_START + 8);
  buffer[2] = READ_FLASH_WORD(FLASH_DATABASE_START + 12);
  buffer[3] = READ_FLASH_WORD(FLASH_DATABASE_START + 16);
  buffer[4] = READ_FLASH_WORD(FLASH_DATABASE_START + 20);

  HAL_FLASH_Unlock();
  
  f.TypeErase = FLASH_TYPEERASE_PAGES;
  f.PageAddress = FLASH_DATABASE_START;
  f.NbPages = 1;
  
  if ( HAL_OK != HAL_FLASHEx_Erase(&f, &pageerror)) {
    return 1;
  }
  
  if (flag == FLASH_UPDATE_ULTRA_DISTANCE) {
    buffer[3] = 0;
    u8tostr(&buffer[3], &value);
  } else {
    //FLASH_UPDATE_ULTRA_TIMEINTERVAL
    buffer[4] = 0;
    u8tostr(&buffer[4], &value);
  }
  
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START,       FLASH_MSG_HEAD);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START + 4,   buffer[0]);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START + 8,   buffer[1]);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START + 12,  buffer[2]);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START + 16,  buffer[3]);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START + 20,  buffer[4]);

  HAL_FLASH_Lock();
  
  return 0;
}