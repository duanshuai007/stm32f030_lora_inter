#include "flash.h"
#include "stm32f0xx.h"
#include "stdlib.h"
#include "lora.h"

/*
*   FLASH�ڱ����豸����Կ��Ϣ
*   ��Կ�Լ��ܷ�ʽ������FLASH��
*   ��Ҫ��tea���ܷ�ʽ���н���
*   
*   [head]    [message]
*   AABBCDDC  .......
*
*   [   m   e  s   s   a   g   e   ]
*   [title][��������]
*   �Լ������ݽ��ܺ���Ҫ����title�Խ��ܺ������yyyyyyyyyyyyyyyyyyyyyyyy
*   ����ժ�������õ�ʵ�ʵ���Կ
*/

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

//��FLASH��ַ0X0800F000����ȡ����id��server id���ŵ�
//FROG+[32λ serverid]+[32λ ����id]+[32λ�ŵ�]
bool FLASH_Init(uint16_t *serverid, uint16_t *localid, uint8_t *sendch, uint8_t *recvch)
{
  uint32_t buffer[3];  //������Կ
  uint32_t dat;
  uint16_t channel;
  
  dat = READ_FLASH_WORD(FLASH_DATABASE_START);
  if( FLASH_MSG_HEAD == dat) {
    //���flash���Ѿ�������Ч������
    buffer[0] = READ_FLASH_WORD(FLASH_DATABASE_START + 4);
    buffer[1] = READ_FLASH_WORD(FLASH_DATABASE_START + 8);
    buffer[2] = READ_FLASH_WORD(FLASH_DATABASE_START + 12);
    
    strtohex(serverid,  &buffer[0], 1);
    strtohex(localid,   &buffer[1], 1);
    strtohex(&channel,  &buffer[2], 1);
    //��ȡ�����һ��λ�õ����ݾ���ͨ������16b�ǽ��գ���16b�Ƿ���
    
    *sendch = (uint8_t)channel;
    *recvch = (uint8_t)(channel >> 8);
    
    return true;    
  }

  return false;
}
