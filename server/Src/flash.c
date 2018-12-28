#include "flash.h"
#include "stm32f1xx.h"
#include "stdlib.h"
#include "maincontrol.h"
#include "rtc.h"
#include "list.h"

static FLASHDeviceList *gFDList = NULL;     //保存在flash中的设备链表
static uint8_t su8TotalNum = 0;

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

/*
*   FLASH初始化函数：每个设备信息结构体占用4个字节，最大200个设备，共800字节
*   因为stm32f103c8每个扇区有1024字节，所以一共需要使用1个扇区。
*
*   如果在flash中发现有效的数据头，则将设备信息读出加入到链表中
*   如果没发现有效的数据头，则清空整个扇区，并写入数据头和设备个数0
*/
bool FLASH_Init(uint16_t *sid, uint8_t *sch, uint8_t *rch)
{
  uint32_t dat;
  uint32_t num;
  uint32_t i;
  uint32_t data;
  uint16_t channel;
  
  //读取flash内的服务器地址参数配置
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
    //如果flash内已经有了有效的数据
    num = READ_FLASH_WORD(FLASH_DATABASE_START + 4);
    
    if ( num > 0 ) {
      //有设备信息
      for( i = 0; i < num; i++) {
        data = READ_FLASH_WORD(FLASH_DATABASE_START + 8 + (i * MSG_SIZE));
        FlashAddDeviceToList((uint16_t)((data & 0xffff0000) >> 16), (uint16_t)(data & 0x0000ffff));
        AddDeviceToList((uint16_t)((data & 0xffff0000) >> 16));
      }
    }
  } else {
    //如果flash内还是空空的
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
*   将设备添加到保存列表中
*/
bool FlashAddDeviceToList(uint16_t id, uint16_t time) 
{
  FLASHDeviceList *fdl = gFDList;
  SaveMSG *sm = fdl->Head->next;
  SaveMSG *pre = fdl->Head;
  //寻找最后一个节点
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
*   从Flash链表中删除节点
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
*   从链表中取出数据写入flash
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
  
  //因为在初始化的时候已经把设备信息读取出来保存到链表中了，所以这里可以直接
  //进行覆盖式写入。
    
  //在清空flash内容之前需要先保存flash内的lora信息(server id, local id, send channel, recv channel)
  buffer[0] = READ_FLASH_WORD(FLASH_SERVERADDR_START);
  buffer[1] = READ_FLASH_WORD(FLASH_SERVERADDR_START + 4);
  buffer[2] = READ_FLASH_WORD(FLASH_SERVERADDR_START + 8);
  buffer[3] = READ_FLASH_WORD(FLASH_SERVERADDR_START + 12);
  
  if ( HAL_OK != HAL_FLASHEx_Erase(&f, &pageerror)) {
    DEBUG_ERROR("FLASH:erase %08x failed\r\n", pageerror);
    return;
  }
  
  //恢复lora信息
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_SERVERADDR_START, buffer[0]);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_SERVERADDR_START + 4, buffer[1]);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_SERVERADDR_START + 8, buffer[2]);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_SERVERADDR_START + 12, buffer[3]);
  
  //写入验证头
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START, SAVE_MESSAGE_HEAD);
  //写入总设备数
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START + 4, list->u8Total);
  //写入所有设备信息
  SaveMSG *sm = list->Head->next;
  
  while ( sm ) {
    dat = (((uint32_t)sm->u16DeviceID) << 16) | sm->u16LastTime;
    if ( sm->u16DeviceID != 0xFFFF ) {
      DEBUG_INFO("FLASH:write D:%04x T:%04x\r\n", sm->u16DeviceID, sm->u16LastTime);
      HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, (FLASH_DATABASE_START + 8 + (i * MSG_SIZE)), dat);
    }
    i++;
    //写入的信息不删除，因为下一次写入还需要，在整个设备执行的过程中要维护一个设备链表
    //每次会修改链表内对应的设备信息状态
    //设备状态的改变包括：执行了指令，心跳包
    sm = sm->next;
  }
  
  DEBUG_INFO("FLASH:write done\r\n");
  HAL_FLASH_Lock();
}

/*
*   向F405发送所有设备上线信息
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
*   Flash保存设备信息，每隔 FLASH_SAVE_TIMEINTERVAL 秒
*   将flash链表gFDList的内容写入到flash内存中。
*/
void FlashTask(void)
{
  static uint32_t lasttime = 0;
  FLASHDeviceList *fdl = gFDList;
  uint32_t time = 0;

  if ( su8TotalNum < fdl->u8Total ) {
    time = GetRTCTime();
    //如果发现有新的设备的话，那就短时间内更新flash信息
    if ( time - lasttime > 5 ) {
      su8TotalNum = fdl->u8Total;
      lasttime = time;
      FlashWriteDevie(fdl);     
    }
  } else {
    //每隔FLASH_SAVE_TIMEINTERVAL秒保存一次状态信息，写入到flash中
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
