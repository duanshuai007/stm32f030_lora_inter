#include "flash.h"
#include "stm32f1xx.h"
#include "stdlib.h"
#include "lora.h"
#include "maincontrol.h"
#include "rtc.h"
#include "list.h"

extern UartModule gUartMx;

/*
*   FLASH��ʼ��������ÿ���豸��Ϣ�ṹ��ռ��4���ֽڣ����200���豸����800�ֽ�
*   ��Ϊstm32f103c8ÿ��������1024�ֽڣ�����һ����Ҫʹ��1��������
*
*   �����flash�з�����Ч������ͷ�����豸��Ϣ�������뵽������
*   ���û������Ч������ͷ�������������������д������ͷ���豸����0
*/
FLASHDeviceList *FLASH_Init(List *list)
{
  FLASHDeviceList *fdl = (FLASHDeviceList *)malloc(sizeof(FLASHDeviceList));
  if ( NULL == fdl ) {
    PRINT("FLASH_Init:fdl == null\r\n");
    return NULL;
  }
  
  fdl->Head = (SaveMSG *)malloc(sizeof(SaveMSG));
  if( NULL == fdl->Head ) {
    PRINT("FLASH_Init:Head == NULL\r\n");
    free(fdl);
    return NULL;
  }
  
  fdl->Head->next = NULL;
  fdl->Head->u16DeviceID = 0xffff;
  fdl->Head->u16LastTime = 0;
  fdl->Head->u32RegStatus = 0;
  fdl->u32Total = 0;
  
  uint32_t dat;
  
  dat = READ_FLASH_WORD(FLASH_DATABASE_START);
  if( SAVE_MESSAGE_HEAD == dat) {
    //���flash���Ѿ�������Ч������
    uint32_t num = READ_FLASH_WORD(FLASH_DATABASE_START + 4);
    
    if ( num > 0 ) {
      //���豸��Ϣ
      uint32_t i;
      uint32_t data;
      
      for( i = 0; i < num; i++) {
        data = READ_FLASH_WORD(FLASH_DATABASE_START + 8 + (i * MSG_SIZE));
        FlashAddDeviceToList(fdl, (uint16_t)((data & 0xffff0000) >> 16), (uint16_t)(data & 0x0000ffff));
        add_device_to_list(list, (uint16_t)((data & 0xffff0000) >> 16));
      }
    }
    
    if ( fdl->u32Total != num ) 
      PRINT("FLASH:init error,%d %d\r\n", num, fdl->u32Total);
    
    return fdl;
    
  } else {
    //���flash�ڻ��ǿտյ�
    HAL_FLASH_Unlock();
    
    FLASH_EraseInitTypeDef f;
    f.TypeErase = FLASH_TYPEERASE_PAGES;
    f.PageAddress = FLASH_DATABASE_START;
    f.NbPages = 1;
    
    uint32_t pageerror = 0;
    
    if ( HAL_OK != HAL_FLASHEx_Erase(&f, &pageerror)) {
      PRINT("FLASH:erase %08x failed\r\n", pageerror);
      free(fdl->Head);
      free(fdl);
      return NULL;
    }
    
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START, SAVE_MESSAGE_HEAD);
    
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START + 4, 0);
    
    HAL_FLASH_Lock();
    
    return fdl; 
  }
}

/*
*   ���豸��ӵ������б���
*/
uint8_t FlashAddDeviceToList(FLASHDeviceList *list, uint16_t id, uint16_t time) 
{
  FLASHDeviceList *fdl = list;
  SaveMSG *sm = fdl->Head;
  //Ѱ�����һ���ڵ�
  while ( sm->next ) {
    
    if(sm->u16DeviceID == id) {
      sm->u16LastTime = time;
      return 1;
    }
    sm = sm->next;
  }
  //����ѭ����ȱ�ݣ����һ��sm�ṹ�岻���ж�id��������Ҫ�����ﴦ��һ��
  if(sm->u16DeviceID == id) {
    sm->u16LastTime = time;
    return 1;
  }
  
  sm->next = (SaveMSG *)malloc(sizeof(SaveMSG));
  if ( NULL == sm->next ) {
    PRINT("FLASHLIST:add malloc failed\r\n");
    return 0;
  }
  
  fdl->u32Total++;
  sm->next->u16DeviceID = id;
  sm->next->u16LastTime = time;
  sm->next->u32RegStatus = 0;
  sm->next->next = NULL;
  
  return 1;
}

void FlashSetDeviceOnline(FLASHDeviceList *list, uint16_t id)
{
  FLASHDeviceList *fdl = list;
  SaveMSG *sm = fdl->Head->next;
  
  while ( sm ) {
    if ( sm->u16DeviceID == id ) {
      sm->u32RegStatus = 1;
    }
    sm = sm->next;
  }
}

/*
*   �������ж�ȡ��һ��Ԫ��
*/
SaveMSG *FlashGetDeviceFromList(FLASHDeviceList *list)
{
  FLASHDeviceList *fdl = list;
  SaveMSG *sm = fdl->Head->next;  //Head�ǿսڵ㣬û����Ч���ݣ�������һ���ڵ���ʵ�ʵĿ�ʼ�ڵ㣬����Ҫ�ж��Ƿ�Ϊ�� 
  
  //��ȡһ���ڵ����������ȡ�Ľڵ��������ɾ��
  if ( sm ) {
    SaveMSG *nextsm = sm->next;
    fdl->Head->next = nextsm;   //��������ɾ���ýڵ�
    
    return sm;
  }
  
  return NULL;
}

/*
*   ��������ȡ������д��flash
*/
uint8_t FlashWriteDevie(FLASHDeviceList *list)
{
  uint32_t pageerror = 0;
  uint32_t dat;
  FLASH_EraseInitTypeDef f;
  
  dat = READ_FLASH_WORD(FLASH_DATABASE_START);
  
  if ( SAVE_MESSAGE_HEAD != dat ) {
    PRINT("FLASH:cant't read flash head\r\n");
    return 0;
  }
  
  HAL_FLASH_Unlock();
  
  f.TypeErase = FLASH_TYPEERASE_PAGES;
  f.PageAddress = FLASH_DATABASE_START;
  f.NbPages = 1;
  
  //��Ϊ�ڳ�ʼ����ʱ���Ѿ����豸��Ϣ��ȡ�������浽�������ˣ������������ֱ��
  //���и���ʽд�롣
  
  if ( HAL_OK != HAL_FLASHEx_Erase(&f, &pageerror)) {
    PRINT("FLASH:erase %08x failed\r\n", pageerror);
    return 0;
  }
  //д����֤ͷ
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START, SAVE_MESSAGE_HEAD);
  //д�����豸��
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START + 4, list->u32Total);
  //д�������豸��Ϣ
  SaveMSG *sm = list->Head->next;
  uint8_t i = 0;
  
  while ( sm ) {
    uint32_t data = (((uint32_t)sm->u16DeviceID) << 16) | sm->u16LastTime;
    PRINT("FLASH:write D:%04x T:%04x\r\n", sm->u16DeviceID, sm->u16LastTime);
    HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, (FLASH_DATABASE_START + 8 + (i * MSG_SIZE)), data);
    i++;
    //д�����Ϣ��ɾ������Ϊ��һ��д�뻹��Ҫ���������豸ִ�еĹ�����Ҫά��һ���豸����
    //ÿ�λ��޸������ڶ�Ӧ���豸��Ϣ״̬
    //�豸״̬�ĸı������ִ����ָ�������
    //      free(sm);
    sm = sm->next;
  }
  
  PRINT("FLASH:write done\r\n");
  HAL_FLASH_Lock();
  
  return 1;
}

void DeviceOnlineStatusProcess(FLASHDeviceList *list)
{
  FLASHDeviceList *fdl = list;
  SaveMSG *sm = fdl->Head->next;
  static uint32_t time = 0;
  static uint32_t lasttime = 0;
  
  while ( sm ) {
  
    if ( sm->u32RegStatus == 0 ) {
      //UartSendOnLineToF405(&gUartMx, sm->u16DeviceID);
    }
    
    sm = sm->next;
  }
  //ÿ��60�뱣��һ��״̬��Ϣ��д�뵽flash��
  time = GetRTCTime();
  if ( time - lasttime > 60 ) {
    PRINT("FLASH:write device info to flash\r\n");
    lasttime = time;
    FlashWriteDevie(list);
  }
}

void LookList(FLASHDeviceList *list)
{
  FLASHDeviceList *fdl = list;
  SaveMSG *sm = fdl->Head->next;
  
  while ( sm ) {
    
    PRINT("LOOK:D:%04x, T:%04x, R:%d\r\n", sm->u16DeviceID, sm->u16LastTime, sm->u32RegStatus);
    
    sm = sm->next;
  }
}

void printFlashTest(uint32_t addr)
{
  uint32_t temp = *(__IO uint32_t*)(addr);
  
  PRINT("addr:0x%x, data:0x%x\r\n", addr, temp);
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

