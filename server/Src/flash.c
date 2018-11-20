#include "flash.h"
#include "stm32f1xx.h"
#include "stdlib.h"
#include "lora.h"
#include "maincontrol.h"
#include "rtc.h"
#include "list.h"

static FLASHDeviceList *gFDList = NULL;     //������flash�е��豸����

/*
*   FLASH��ʼ��������ÿ���豸��Ϣ�ṹ��ռ��4���ֽڣ����200���豸����800�ֽ�
*   ��Ϊstm32f103c8ÿ��������1024�ֽڣ�����һ����Ҫʹ��1��������
*
*   �����flash�з�����Ч������ͷ�����豸��Ϣ�������뵽������
*   ���û������Ч������ͷ�������������������д������ͷ���豸����0
*/
void FLASH_Init(void)
{
  uint32_t dat;
  uint32_t num;
  uint32_t i;
  uint32_t data;

  gFDList = (FLASHDeviceList *)malloc(sizeof(FLASHDeviceList));
  if ( NULL == gFDList ) {
    PRINT("FLASH_Init:gFDList == null\r\n");
    return;
  }
  
  gFDList->Head = (SaveMSG *)malloc(sizeof(SaveMSG));
  if( NULL == gFDList->Head ) {
    PRINT("FLASH_Init:Head == NULL\r\n");
    free(gFDList);
    return;
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
    HAL_FLASH_Unlock();
    
    uint32_t pageerror = 0;
    FLASH_EraseInitTypeDef f;
    
    f.TypeErase = FLASH_TYPEERASE_PAGES;
    f.PageAddress = FLASH_DATABASE_START;
    f.NbPages = 1;
    
    if ( HAL_OK != HAL_FLASHEx_Erase(&f, &pageerror)) {
      PRINT("FLASH:erase %08x failed\r\n", pageerror);
      free(gFDList->Head);
      free(gFDList);
      return;
    }
    
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START, SAVE_MESSAGE_HEAD);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START + 4, 0);
    HAL_FLASH_Lock();
  }
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
    PRINT("FLASHLIST:add malloc failed\r\n");
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
  
  dat = READ_FLASH_WORD(FLASH_DATABASE_START);
  
  if ( SAVE_MESSAGE_HEAD != dat ) {
    PRINT("FLASH:cant't read flash head\r\n");
    return;
  }
  
  HAL_FLASH_Unlock();
  
  f.TypeErase = FLASH_TYPEERASE_PAGES;
  f.PageAddress = FLASH_DATABASE_START;
  f.NbPages = 1;
  
  //��Ϊ�ڳ�ʼ����ʱ���Ѿ����豸��Ϣ��ȡ�������浽�������ˣ������������ֱ��
  //���и���ʽд�롣
  
  if ( HAL_OK != HAL_FLASHEx_Erase(&f, &pageerror)) {
    PRINT("FLASH:erase %08x failed\r\n", pageerror);
    return;
  }
  //д����֤ͷ
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START, SAVE_MESSAGE_HEAD);
  //д�����豸��
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START + 4, list->u8Total);
  //д�������豸��Ϣ
  SaveMSG *sm = list->Head->next;
  
  while ( sm ) {
    dat = (((uint32_t)sm->u16DeviceID) << 16) | sm->u16LastTime;
    PRINT("FLASH:write D:%04x T:%04x\r\n", sm->u16DeviceID, sm->u16LastTime);
    HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, (FLASH_DATABASE_START + 8 + (i * MSG_SIZE)), dat);
    i++;
    //д�����Ϣ��ɾ������Ϊ��һ��д�뻹��Ҫ���������豸ִ�еĹ�����Ҫά��һ���豸����
    //ÿ�λ��޸������ڶ�Ӧ���豸��Ϣ״̬
    //�豸״̬�ĸı������ִ����ָ�������
    sm = sm->next;
  }
  
  PRINT("FLASH:write done\r\n");
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
  uint32_t time = 0;
  //ÿ��FLASH_SAVE_TIMEINTERVAL�뱣��һ��״̬��Ϣ��д�뵽flash��
  time = GetRTCTime();
  if ( time - lasttime > FLASH_SAVE_TIMEINTERVAL ) {
    PRINT("FLASH:write device info to flash\r\n");
    lasttime = time;
    FlashWriteDevie(gFDList);
  }
}

#if 0
void LookList(void)
{
  SaveMSG *sm = gFDList->Head->next;
  
  while ( sm ) {
    
    PRINT("LOOK:D:%04x, T:%04x\r\n", sm->u16DeviceID, sm->u16LastTime);
    
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
    PRINT("FLASH:erase %08x failed\r\n", pageerror);
    return;
  }
 
  HAL_FLASH_Lock();
}
#endif
