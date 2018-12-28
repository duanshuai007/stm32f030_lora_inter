#include "flash.h"
#include "stm32f1xx.h"
#include "stdlib.h"
#include "maincontrol.h"
#include "rtc.h"
#include "list.h"

static FLASHDeviceList *gFDList = NULL;     //������flash�е��豸����
static uint8_t su8TotalNum = 0;

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

/*
*   FLASH��ʼ��������ÿ���豸��Ϣ�ṹ��ռ��4���ֽڣ����200���豸����800�ֽ�
*   ��Ϊstm32f103c8ÿ��������1024�ֽڣ�����һ����Ҫʹ��1��������
*
*   �����flash�з�����Ч������ͷ�����豸��Ϣ�������뵽������
*   ���û������Ч������ͷ�������������������д������ͷ���豸����0
*/
bool FLASH_Init(uint16_t *sid, uint8_t *sch, uint8_t *rch)
{
  uint32_t dat;
  uint32_t num;
  uint32_t i;
  uint32_t data;
  uint16_t channel;
  
  //��ȡflash�ڵķ�������ַ��������
  dat = READ_FLASH_WORD(FLASH_SERVERADDR_START);
  if (SAVE_MESSAGE_HEAD == dat) {
    dat = READ_FLASH_WORD(FLASH_SERVERADDR_START + 4);
    strtohex(sid, &dat, 1);
    dat = READ_FLASH_WORD(FLASH_SERVERADDR_START + 12);
    strtohex(&channel, &dat, 1);
    
    *sch = (uint8_t)(channel >> 8);
    *rch = (uint8_t)channel;
    
  } else {
    return false;
  }
  
  gFDList = (FLASHDeviceList *)malloc(sizeof(FLASHDeviceList));
  if ( NULL == gFDList ) {
    DEBUG_ERROR("FLASH_Init:gFDList == null\r\n");
    return false;
  }
  
  gFDList->Head = (SaveMSG *)malloc(sizeof(SaveMSG));
  if( NULL == gFDList->Head ) {
    DEBUG_ERROR("FLASH_Init:Head == NULL\r\n");
    free(gFDList);
    return false;
  }
  
  gFDList->Head->next = NULL;
  gFDList->Head->u16DeviceID = 0xffff;
  gFDList->Head->u16LastTime = 0;
  gFDList->u8Total = 0;
  
  dat = READ_FLASH_WORD(FLASH_DATABASE_START);
  if( SAVE_MESSAGE_HEAD == dat) {
    //���flash���Ѿ�������Ч������
    num = READ_FLASH_WORD(FLASH_DATABASE_START + 4);
    
    if ( num > 0 ) {
      //���豸��Ϣ
      for( i = 0; i < num; i++) {
        data = READ_FLASH_WORD(FLASH_DATABASE_START + 8 + (i * MSG_SIZE));
        FlashAddDeviceToList((uint16_t)((data & 0xffff0000) >> 16), (uint16_t)(data & 0x0000ffff));
        AddDeviceToList((uint16_t)((data & 0xffff0000) >> 16));
      }
    }
  } else {
    //���flash�ڻ��ǿտյ�
    uint32_t buffer[4];
    
    buffer[0] = READ_FLASH_WORD(FLASH_SERVERADDR_START);
    buffer[1] = READ_FLASH_WORD(FLASH_SERVERADDR_START + 4);
    buffer[2] = READ_FLASH_WORD(FLASH_SERVERADDR_START + 8);
    buffer[3] = READ_FLASH_WORD(FLASH_SERVERADDR_START + 12);
    
    HAL_FLASH_Unlock();
    
    uint32_t pageerror = 0;
    FLASH_EraseInitTypeDef f;
    
    f.TypeErase = FLASH_TYPEERASE_PAGES;
    f.PageAddress = FLASH_DATABASE_START;
    f.NbPages = 1;
    
    if ( HAL_OK != HAL_FLASHEx_Erase(&f, &pageerror)) {
      DEBUG_ERROR("FLASH:erase %08x failed\r\n", pageerror);
      free(gFDList->Head);
      free(gFDList);
      return false;
    }
    
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_SERVERADDR_START, buffer[0]);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_SERVERADDR_START + 4, buffer[1]);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_SERVERADDR_START + 8, buffer[2]);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_SERVERADDR_START + 12, buffer[3]);
    
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START, SAVE_MESSAGE_HEAD);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START + 4, 0);
    HAL_FLASH_Lock();
  }
  
  su8TotalNum = gFDList->u8Total;
  
  return true;
}

/*
*   ���豸��ӵ������б���
*/
bool FlashAddDeviceToList(uint16_t id, uint16_t time) 
{
  FLASHDeviceList *fdl = gFDList;
  SaveMSG *sm = fdl->Head->next;
  SaveMSG *pre = fdl->Head;
  //Ѱ�����һ���ڵ�
  while( sm ) {
    if ( sm->u16DeviceID == id) {
      sm->u16LastTime = time;
      return true;
    }
    pre = sm;
    sm = sm->next;
  }
  
  pre->next = (SaveMSG *)malloc(sizeof(SaveMSG));
  if ( NULL == pre->next ) {
    DEBUG_ERROR("FLASHLIST:add malloc failed\r\n");
    return false;
  }
  
  fdl->u8Total++;
  pre->next->u16DeviceID = id;
  pre->next->u16LastTime = time;
  pre->next->next = NULL;
  
  return true;
}

/*
*   ��Flash������ɾ���ڵ�
*/
void FlashDelDeviceFromList(uint16_t id)
{
  SaveMSG *del = gFDList->Head->next;
  SaveMSG *pre = gFDList->Head;
  
  while(del) {
    if(del->u16DeviceID == id) {
    
      if(del->next) {
        SaveMSG *next = del->next;
        pre->next = next;
      } else {
        pre->next = NULL;
      }
      
      free(del);
      gFDList->u8Total -= 1;
      return;
    }
    
    pre = del;
    del = del->next;
  }
}

/*
*   ��������ȡ������д��flash
*/
static void FlashWriteDevie(FLASHDeviceList *list)
{
  uint32_t pageerror = 0;
  uint32_t dat;
  uint8_t i = 0;
  FLASH_EraseInitTypeDef f;
  uint32_t buffer[4];
  
  dat = READ_FLASH_WORD(FLASH_DATABASE_START);
  
  if ( SAVE_MESSAGE_HEAD != dat ) {
    DEBUG_ERROR("FLASH:cant't read flash head\r\n");
    return;
  }
  
  HAL_FLASH_Unlock();
  
  f.TypeErase = FLASH_TYPEERASE_PAGES;
  f.PageAddress = FLASH_DATABASE_START;
  f.NbPages = 1;
  
  //��Ϊ�ڳ�ʼ����ʱ���Ѿ����豸��Ϣ��ȡ�������浽�������ˣ������������ֱ��
  //���и���ʽд�롣
    
  //�����flash����֮ǰ��Ҫ�ȱ���flash�ڵ�lora��Ϣ(server id, local id, send channel, recv channel)
  buffer[0] = READ_FLASH_WORD(FLASH_SERVERADDR_START);
  buffer[1] = READ_FLASH_WORD(FLASH_SERVERADDR_START + 4);
  buffer[2] = READ_FLASH_WORD(FLASH_SERVERADDR_START + 8);
  buffer[3] = READ_FLASH_WORD(FLASH_SERVERADDR_START + 12);
  
  if ( HAL_OK != HAL_FLASHEx_Erase(&f, &pageerror)) {
    DEBUG_ERROR("FLASH:erase %08x failed\r\n", pageerror);
    return;
  }
  
  //�ָ�lora��Ϣ
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_SERVERADDR_START, buffer[0]);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_SERVERADDR_START + 4, buffer[1]);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_SERVERADDR_START + 8, buffer[2]);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_SERVERADDR_START + 12, buffer[3]);
  
  //д����֤ͷ
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START, SAVE_MESSAGE_HEAD);
  //д�����豸��
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START + 4, list->u8Total);
  //д�������豸��Ϣ
  SaveMSG *sm = list->Head->next;
  
  while ( sm ) {
    dat = (((uint32_t)sm->u16DeviceID) << 16) | sm->u16LastTime;
    if ( sm->u16DeviceID != 0xFFFF ) {
      DEBUG_INFO("FLASH:write D:%04x T:%04x\r\n", sm->u16DeviceID, sm->u16LastTime);
      HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, (FLASH_DATABASE_START + 8 + (i * MSG_SIZE)), dat);
    }
    i++;
    //д�����Ϣ��ɾ������Ϊ��һ��д�뻹��Ҫ���������豸ִ�еĹ�����Ҫά��һ���豸����
    //ÿ�λ��޸������ڶ�Ӧ���豸��Ϣ״̬
    //�豸״̬�ĸı������ִ����ָ�������
    sm = sm->next;
  }
  
  DEBUG_INFO("FLASH:write done\r\n");
  HAL_FLASH_Lock();
}

/*
*   ��F405���������豸������Ϣ
*/
void SendALLDeviceNodeToF405(void)
{
  SaveMSG *sm = gFDList->Head->next;
  
  while ( sm ) {
    UartSendMSGToF405(NODE_ONLINE, sm->u16DeviceID, 0);
    sm = sm->next;
  }
}

/*
*   Flash�����豸��Ϣ��ÿ�� FLASH_SAVE_TIMEINTERVAL ��
*   ��flash����gFDList������д�뵽flash�ڴ��С�
*/
void FlashTask(void)
{
  static uint32_t lasttime = 0;
  FLASHDeviceList *fdl = gFDList;
  uint32_t time = 0;

  if ( su8TotalNum < fdl->u8Total ) {
    time = GetRTCTime();
    //����������µ��豸�Ļ����ǾͶ�ʱ���ڸ���flash��Ϣ
    if ( time - lasttime > 5 ) {
      su8TotalNum = fdl->u8Total;
      lasttime = time;
      FlashWriteDevie(fdl);     
    }
  } else {
    //ÿ��FLASH_SAVE_TIMEINTERVAL�뱣��һ��״̬��Ϣ��д�뵽flash��
    time = GetRTCTime();
    if ( time - lasttime > FLASH_SAVE_TIMEINTERVAL ) {
      DEBUG_INFO("FLASH:write device info to flash\r\n");
      lasttime = time;
      FlashWriteDevie(fdl);
    }
  }
}

#if 0
void LookList(void)
{
  SaveMSG *sm = gFDList->Head->next;
  
  while ( sm ) {
    
    DEBUG_INFO("LOOK:D:%04x, T:%04x\r\n", sm->u16DeviceID, sm->u16LastTime);
    
    sm = sm->next;
  }
}

void FLASH_CLEAN(void)
{
  FLASH_EraseInitTypeDef f;
  uint32_t pageerror;
  
  HAL_FLASH_Unlock();
  
  f.TypeErase = FLASH_TYPEERASE_PAGES;
  f.PageAddress = FLASH_DATABASE_START;
  f.NbPages = 1;
  
  if ( HAL_OK != HAL_FLASHEx_Erase(&f, &pageerror)) {
    DEBUG_ERROR("FLASH:erase %08x failed\r\n", pageerror);
    return;
  }
 
  HAL_FLASH_Lock();
}
#endif
