#include "flash.h"
#include "stm32f0xx.h"
#include "stdlib.h"

static void strtohex(uint16_t *dst, uint32_t *src, uint8_t srclen)
{
  uint8_t i, j;
  uint8_t dat;
  
  for(i=0; i < srclen; i++) {
    for ( j = 0; j < 4; j++) {
    
      //src��32Ϊ��ַָ�룬+1��Ӧһ��32λ���ͣ�ָ����һ������Ԫ�ص�λ��
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
  //��32λ��ָ��ת��Ϊ8λ��ָ��
  uint8_t *pDst = (uint8_t *)dst;
  *(pDst+0) = 0x30;
  *(pDst+1) = 0x30;
  
  for(i=0;i<2;i++)
  {
    dat = ((*src) >> (4 * (1 - i))) & 0x0f;
    
    //ʮ��������ת��Ϊ�ַ�
    if (dat <= 9) {
      dat += '0';
    } else {
      dat += ('a' - 0x0a);
    }
    
    *(pDst + i + 2) = dat;
  }
}

//��FLASH��ַ0X0800F000����ȡ����id��server id���ŵ�
//FROG+[32λ serverid]+[32λ ����id]+[32λ�ŵ�]
bool FLASH_Init(uint16_t *serverid, uint16_t *localid, uint8_t *sendch, uint8_t *recvch, uint16_t *ultra_safevalue, uint16_t *ultra_checktime)
{
  uint32_t buffer[5];
  uint32_t dat;
  uint16_t channel;
  
  dat = READ_FLASH_WORD(FLASH_DATABASE_START);
  if( FLASH_MSG_HEAD != dat) 
    return false;
  
  //���flash���Ѿ�������Ч������
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
  //��ȡ�����һ��λ�õ����ݾ���ͨ������16b�ǽ��գ���16b�Ƿ���
  
  *sendch = (uint8_t)channel;
  *recvch = (uint8_t)(channel >> 8);
  
  return true;
}

//�����õĳ�������ȫ����д�뵽flash��,����ʮ��������
//������flash�е����ַ�����ʽ��ʮ��������
//����distance=0x28
//������flash����Ϊ 0x30 0x30 0x32 0x38
//flag: 0 ��ʾд�밲ȫ����
//    : 1 ��ʾд������
//����ֵ:0 success
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