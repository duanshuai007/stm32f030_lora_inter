#include "list.h"
#include "stdint.h"
#include "stm32f1xx.h"
#include "stdlib.h"
#include "string.h"
#include "user_config.h"
#include "maincontrol.h"
#include "reboot.h"
#include "flash_if.h"


//设备链表
//每个endpoint对应一个链表的节点
//在节点内保存设备信息，设备指令等
//通过链表的一个循环判断线程来执行
//设备的动作。
uint8_t g_sound_check_distance = 0;
static List *gList = NULL;       

void writeAllDeviceToFlash(void)
{
  List *ln = gList;

  while(ln)
  {
    if(NULL != ln->Device)
    {
      AddDeviceToFlash(ln->Device->u16ID);
      HAL_Delay(10);
    }

    ln = ln->next;
  }
  
}

void resetSoundCheckDist(void)
{
  uint16_t distance = READ_FLASH_HALFWORD(FLASH_SAVE_SECTION+4);
  if(0xffff == distance)
  {
    g_sound_check_distance = 40;
  }
  else
  {
    g_sound_check_distance = (uint8_t)distance;
  }  
}

void AddDeviceToFlash(uint16_t id)
{
  uint32_t i = 0;
  uint16_t dev_id = 0;

  
  while(1)
  {
    dev_id = *((uint16_t *)(FLASH_SAVE_DEVICEID_ADDR + i*sizeof(uint16_t)));    
    if(dev_id == id)
    {
      return;
    }
    if(0xFFFF != dev_id)
    {
      i++;
    }
    else
    {
      break;
    }
  };

  FLASH_If_Write_Halfword(FLASH_SAVE_DEVICEID_ADDR + i*sizeof(uint16_t), id);            

  return;
}

/*
*   设备链表初始化
*   申请链表地址。初始化链表的内容
*/
void ListInit(void)
{
  gList = (List *)malloc(sizeof(List));
  if ( NULL == gList ) {
    DEBUG_ERROR("gList == null\r\n");
    return;
  }
  
  gList->Device = NULL;
  gList->next = NULL;

  uint32_t i = 0;
  uint16_t dev_id = 0;  

  
  while(1)  
  {
    dev_id = *((uint16_t *)(FLASH_SAVE_DEVICEID_ADDR + i*sizeof(uint16_t)));

    if(0xFFFF != dev_id)
    {
      AddDeviceToList(dev_id, false);
      i++;
    }
    else
    {
      break;
    }
    
  }

  uint16_t distance = READ_FLASH_HALFWORD(FLASH_SAVE_SECTION+4);
  if(0xffff == distance)
  {
    g_sound_check_distance = 40;
  }
  else
  {
    g_sound_check_distance = (uint8_t)distance;
  }
    
}

/*
*   将设备加入到链表中
*   如果设备已经存在于链表内，则直接退出
*/
bool AddDeviceToList(uint16_t id, bool onlineFlg)
{  
  List *last = gList;
  List *pre = gList;
  
  DEBUG_INFO("List:add new device[%d]\r\n", id);
  
  //首先判断要加入到设备id是否已经在链表内
  while(last)
  {
    if(last->Device)
    {
      if(last->Device->u16ID == id) 
      {
        return true;
      }
    }

    pre = last;
    last = last->next;
  }
  
  //要加入的设备id不在链表内
  //创建新的链表节点保存设备信息
  List *lastlist = (List *)malloc(sizeof(List));
  if ( NULL == lastlist ) {
    DEBUG_ERROR("add_device_to_list: malloc(lastlist) == NULL\r\n");
    return false;
  }
  
  DeviceNode *devnode = (DeviceNode *)malloc(sizeof(DeviceNode));
  if ( NULL == devnode ) {
    DEBUG_ERROR("add_device_to_list:malloc(devnode) == NULL\r\n");
    free(lastlist);
    return false;
  }
  memset(devnode, 0, sizeof(DeviceNode));
  
  //将新节点加入链表中
  pre->next = lastlist;
  //设置节点内容
  lastlist->Device = devnode;
  lastlist->next = NULL;

  
  //设置设备内容
  devnode->u16ID = id;
  devnode->u32Identify = 0;            
  devnode->u32LastTime = HAL_GetTick();
  devnode->u32ActionStartTime = 0;
  devnode->u32CmdStartTime = 0;
  devnode->u8CMD = CMD_NONE;
  devnode->u8CMDSTATUS = CMD_STATUS_IDLE;
  devnode->u8RESP = 0;
  devnode->u8CMDRetry = 0;
  devnode->u8EpStatus = EP_STATUS_NONE;
  devnode->u8HaveCarFlg = STATUS_VALID;          
  devnode->DeviceOnLine = onlineFlg;  
  devnode->u8AbnormalRESP = 0x0;
  devnode->u8CarCheckRESP = 0xFF;
  devnode->u8HeartRESP = 0xFF;
    
  return true;
}

#if 1
/*
*   从链表内寻找指定ID的设备节点并删除
*/
bool delete_device_from_list(uint16_t id)
{
  DEBUG_INFO("List:delete device[%04x]\r\n", id);
  //需要删除的节点
  List *del = gList->next;
  //需要删除的节点的上一个节点
  List *pre = gList;
  
  while ( del ) {
    if ( del->Device ) {
      if ( del->Device->u16ID == id ) {
        //找到了设备，删除
        if ( del->next ) {
          //如果需要删除的节点不是最后一个节点
          List *next = del->next;
          pre->next = next;
        } else {
          //如果是最后一个节点
          pre->next = NULL;
        }
        
        free(del->Device);
        del->Device = NULL;
        del->next = NULL;
        free(del);
        del = NULL;
        
        return true;
      }
    }
    //指向下一个节点
    pre = del;
    del = del->next;
  }
  
  return false;
}
#endif

/*
*   设置指令ID设备节点的CMD和IDENTIFY
*   设备不存在返回false
*   设备忙或设备正常返回true
*/
bool SendCMDToList(uint16_t id, uint8_t cmd, uint32_t identify, uint32_t data)
{
  List *ln = gList;
  DEBUG_INFO("List:set device[%04x],cmd:%d\r\n", id, cmd);
  while ( ln ) 
  {
    if ( ln->Device ) 
    {
      if ( ln->Device->u16ID == id ) 
      {
        if(!insertCmdToList(id, identify, cmd, data))
        {
          UartSendAutoMSGToF405(NORMAL_BUSY, id, identify);
        }
        
        return true;
      }
    }
    ln = ln->next;
  }
  
  return false;
}

/*
*   发送resp信息到指定ID的节点
*/
void SendCMDRespToList(uint16_t id, uint8_t cmd, uint32_t identify, uint8_t resp)
{
  List *ln = gList;
  DEBUG_INFO("list: get resp, id[%d]!\r\n",id);
  while ( ln ) {
    if( ln->Device ) {
      if (ln->Device->u16ID == id) {
        //正常的指令响应
        if (( ln->Device->u8CMD == cmd) && (ln->Device->u32Identify == identify)) {
          ln->Device->u8RESP = resp;
          ln->Device->u8CMDSTATUS = CMD_STATUS_DONE;
        }
        //异常动作响应
        else if ((DEVICE_ABNORMAL == cmd) && (0 == identify)) {
          ln->Device->u8AbnormalRESP = resp;
        }
        else if ((HW_ULTRA_GET == cmd) && (0 == identify)) {
          ln->Device->u8CarCheckRESP = resp;
        }
        else if ((DEVICE_HEART == cmd) && (0 == identify)) {
          ln->Device->u8HeartRESP = resp;
        }
        return;
      }
    }
    ln = ln->next;
  }
}


/*
*   设备链表进程，在主循环内调用，循环判断链表内每个设备的状态
*   当对应的u8CMDSTATUS状态为:
*     CMD_STATUS_IDLE       设备空闲
*     CMD_STATUS_STANDBY    设备有指令等待执行
*     CMD_STATUS_RUN        设备指令正在运行
*     CMD_STATUS_DONE       设备指令运行完成
*/
void setEndpointStatus(DeviceNode *device)
{
  if(EP_STATUS_DOWN>device->u8RESP || device->u8RESP>EP_STATUS_BACK)
  {
    return;
  }
  
  if(CMD_MOTOR_UP == device->u8CMD)
  {
    if(0 == device->u8RESP)
    {
      device->u8EpStatus = EP_STATUS_UP;
    }
    else
    {
      device->u8EpStatus = device->u8RESP;
    }
  }
  else if(CMD_MOTOR_DOWN == device->u8CMD)
  {
    if(0 == device->u8RESP)
    {
      device->u8EpStatus = EP_STATUS_DOWN;
    }
    else
    {
      device->u8EpStatus = device->u8RESP;     
    }
  }
  else if(CMD_MOTOR_STATUS_GET == device->u8CMD)
  {
    device->u8EpStatus = device->u8RESP;
  }

}


void doAbnormalResp(DeviceNode *device)
{
  uint8_t resp_code = 0;
  switch(device->u8AbnormalRESP)
  {
  case EP_STATUS_DOWN:
    resp_code = INTERUPT_DOWN;
    device->u8EpStatus = EP_STATUS_DOWN;
    break;
  case EP_STATUS_FORWARD:
    resp_code = INTERUPT_FORWARD;
    device->u8EpStatus = EP_STATUS_FORWARD;
    break;
  case EP_STATUS_UP:
    resp_code = INTERUPT_UP;
    device->u8EpStatus = EP_STATUS_UP;
    break;
  case EP_STATUS_BACK:
    resp_code = INTERUPT_BACK;
    device->u8EpStatus = EP_STATUS_BACK;
    break;
  default:
    //error;
    resp_code = 0;
    break;
  }

  if(0 != resp_code)
  {
    UartSendAutoMSGToF405(resp_code, device->u16ID, 0);
  }

}

#define NEW_PROTOCOL_BETWEEN_SERVER_AND_EP //使用新定义的协议(resp字段只表示地锁状态或者超声波的距离)
#ifdef NEW_PROTOCOL_BETWEEN_SERVER_AND_EP
static void doHeartResp(DeviceNode *device)
{
  uint8_t status = device->u8HeartRESP;

  device->u8EpStatus = status;

  //由103触发的心跳数据，需要将device内部恢复初始设置。
  if(DEVICE_HEART == device->u8CMD)
  {
    device->u8RESP = 0;
    device->u8CMD = CMD_NONE;
    device->u8CMDSTATUS = CMD_STATUS_IDLE;          
    device->u32Identify = 0;            

    device->u32CmdStartTime = 0;
    device->u8CMDRetry = 0;     
  }
}

static void doSoundCheckResp(DeviceNode *device)
{
  device->u8EpStatus = EP_STATUS_DOWN;
  uint8_t height = device->u8CarCheckRESP;
  
  uint8_t haveCarStatus = STATUS_VALID;

  if(0xff == height)
  {
    haveCarStatus = STATUS_TROUBLE;
  }
  else
  {
    if(height > g_sound_check_distance)
    {
      haveCarStatus = STATUS_NO_HAVE_CAR;
    }
    else
    {
      haveCarStatus = STATUS_HAVE_CAR;      
    }
  }
  
  if(haveCarStatus != device->u8HaveCarFlg)
  {      
    device->u8HaveCarFlg = haveCarStatus;

    if(device->DeviceOnLine)
    {        
      if(STATUS_HAVE_CAR == haveCarStatus)
      {
        UartSendAutoMSGToF405(UPLOAD_HAVE_CAR, device->u16ID, 0);  
      }
      else if(STATUS_NO_HAVE_CAR == haveCarStatus)
      {
        UartSendAutoMSGToF405(UPLOAD_NO_HAVE_CAR, device->u16ID, 0);
      }
      else if(STATUS_TROUBLE == haveCarStatus)
      {
        UartSendAutoMSGToF405(UPLOAD_SOUNDWAVE_TROUBLE, device->u16ID, 0);
      }
      else
      {
        //error
      }
    }
  }

}

#else //使用旧定义的协议(resp字段高4位表示地锁状态，低4位表示有车还是没车)
static void doHeartResp(DeviceNode *device)
{
  uint8_t resp = device->u8HeartRESP;
  
  uint8_t respStatus = (resp >> 4);

  device->u8EpStatus = respStatus;


  //由103触发的心跳数据，需要将device内部恢复初始设置。
  if(DEVICE_HEART == device->u8CMD)
  {
    device->u8RESP = 0;
    device->u8CMD = CMD_NONE;
    device->u8CMDSTATUS = CMD_STATUS_IDLE;          
    device->u32Identify = 0;            

    device->u32CmdStartTime = 0;
    device->u8CMDRetry = 0;     
  }
}

static void doSoundCheckResp(DeviceNode *device)
{
  uint8_t resp = device->u8CarCheckRESP;
  uint8_t haveCarStatus = (resp & 0xf);

  device->u8EpStatus = EP_STATUS_DOWN;

  if(haveCarStatus != device->u8HaveCarFlg)
  {      
    device->u8HaveCarFlg = haveCarStatus;
  
    if(device->DeviceOnLine)
    {        
      if(STATUS_HAVE_CAR == haveCarStatus)
      {
        UartSendAutoMSGToF405(UPLOAD_HAVE_CAR, device->u16ID, 0);  
      }
      else if(STATUS_NO_HAVE_CAR == haveCarStatus)
      {
        UartSendAutoMSGToF405(UPLOAD_NO_HAVE_CAR, device->u16ID, 0);
      }
      else if(STATUS_TROUBLE == haveCarStatus)
      {
        UartSendAutoMSGToF405(UPLOAD_SOUNDWAVE_TROUBLE, device->u16ID, 0);
      }
      else
      {
        //error
      }
    }
  }
}
#endif


void ListTask(void)
{
  List *ln = gList->next;
  uint32_t nowTime = 0;
  
  while ( ln ) {    
    if ( ln->Device ) {

      //地锁异常信息
      if( 0 != ln->Device->u8AbnormalRESP)
      {        
        doAbnormalResp(ln->Device);
        
        ln->Device->u8AbnormalRESP = 0;  
      }
      
      //处理超声波检测消息
      if(0xFF != ln->Device->u8CarCheckRESP)
      {
        doSoundCheckResp(ln->Device);
        
        if (!ln->Device->DeviceOnLine) 
        {
          UartSendOnlineMSGToF405(ln->Device);
          ln->Device->DeviceOnLine = true;
        } 
        
        ln->Device->u8CarCheckRESP = 0xFF;
      }

      //处理心跳检测消息
      if(0xFF != ln->Device->u8HeartRESP)
      {
        doHeartResp(ln->Device);
        
        if (!ln->Device->DeviceOnLine) 
        {
          UartSendOnlineMSGToF405(ln->Device);
          ln->Device->DeviceOnLine = true;
        } 
        
        ln->Device->u8HeartRESP = 0xFF;
      }
      
      nowTime = HAL_GetTick();
      //CMD_HEART_INTERVAL_TIME秒没有收到该device的消息，则给该device发心跳包
      if(nowTime - ln->Device->u32HeartTime > CMD_HEART_INTERVAL_TIME)
      {
        insertCmdToList(ln->Device->u16ID, 0, DEVICE_HEART, 0);
        ln->Device->u32HeartTime = nowTime;
      }
      
      //CMD_ONLINE_TIMEOUT秒没有收到该device的消息，认为该device掉线了
      if(nowTime - ln->Device->u32LastTime > CMD_ONLINE_TIMEOUT)
      {
        if ( ln->Device->DeviceOnLine == true )
        {
          UartSendAutoMSGToF405(NODE_OFFLINE, ln->Device->u16ID, 0);
          ln->Device->DeviceOnLine = false;
          //掉线后发送reset命令，使其重启
          insertCmdToList(ln->Device->u16ID, 0, CMD_SYSTEM_RESET, 0);          
        }        
      }
        
      //地锁正常命令与相应
      switch(ln->Device->u8CMDSTATUS) {              
      case CMD_STATUS_RUN: 
      {
        //等待判断指令超时
        nowTime = HAL_GetTick();

        //当u8RESP字段为MOTOR_RUNING时，表示抬起或者落下地锁时的中间状态
        if(ln->Device->u8RESP == MOTOR_RUNING)
        {
          if(0 != ln->Device->u32ActionStartTime && nowTime - ln->Device->u32ActionStartTime > CMD_WAIT_FIN_ACTION_TIMEOUT)
          {
            ln->Device->u32ActionStartTime = 0;
              
            UartSendAutoMSGToF405(NORMAL_UNKNOWN_ERROR, ln->Device->u16ID, ln->Device->u32Identify);

            ln->Device->u32Identify = 0;            
            ln->Device->u32CmdStartTime = 0;
            ln->Device->u8CMD = CMD_NONE;
            ln->Device->u8CMDSTATUS = CMD_STATUS_IDLE;
            ln->Device->u8RESP = 0;
            ln->Device->u8CMDRetry = 0;  
          }
        }
        else
        {          
          if (0 != ln->Device->u32CmdStartTime && nowTime - ln->Device->u32CmdStartTime >= CMD_TIMEOUT ) 
          {
            ln->Device->u32CmdStartTime = 0;         
            ln->Device->u8CMDSTATUS = CMD_STATUS_IDLE;
              
            if ( ln->Device->u8CMDRetry >= CMD_MAX_RETRY_TIMES ) {
              //达到重试次数的上限都没有响应，则认为设备掉线了。
              //从链表中删除设备，并通知f405设备掉线。

              if(DEVICE_HEART != ln->Device->u8CMD)
              {
                UartSendAutoMSGToF405(NORMAL_UNKNOWN_ERROR, ln->Device->u16ID, ln->Device->u32Identify);
              }

              //if ( ln->Device->DeviceOnLine == true ) {
              //  UartSendAutoMSGToF405(NODE_OFFLINE, ln->Device->u16ID, 0);
              //  ln->Device->DeviceOnLine = false;
              //}
              
              ln->Device->u8CMD = CMD_NONE;
              ln->Device->u8RESP = 0;
              ln->Device->u8CMDRetry = 0;  
            }
            else
            {
              insertCmdToList(ln->Device->u16ID, ln->Device->u32Identify, ln->Device->u8CMD, ln->Device->u32Data);
            }
            
          }          
        }

      }
      break;
      
      case CMD_STATUS_DONE:

        //更新EP的状态        
        setEndpointStatus(ln->Device);

        if(MOTOR_BUSY != ln->Device->u8RESP)
        {
          UartSendRespToF405(ln->Device);
        }
        
        //对电机控制命令要特殊处理，因为发送了电机指令会立刻返回一条信息。
        //在电机执行结束之后还会返回一条信息。
        if (MOTOR_RUNING == ln->Device->u8RESP || MOTOR_BUSY == ln->Device->u8RESP) 
        {
          ln->Device->u8RESP = MOTOR_RUNING;
          ln->Device->u8CMDSTATUS = CMD_STATUS_RUN;
          ln->Device->u32ActionStartTime = HAL_GetTick();
        } 
        else
        {
          ln->Device->u8RESP = 0;
          ln->Device->u8CMD = CMD_NONE;
          ln->Device->u8CMDSTATUS = CMD_STATUS_IDLE;          
          ln->Device->u32Identify = 0;            
        }

        //初始化device的某些字段
        ln->Device->u32CmdStartTime = 0;

        ln->Device->u8CMDRetry = 0;  
        
        break;
        
      default:
        break;
      }
    }

    if ( ln )
      ln = ln->next;  
  }/*end while(1)*/
}

/*
*   查看整个链表的所有节点
*/
void LookAllList(void)
{
  List *ln = gList;
  
  while( ln != NULL ) {
    
    DEBUG_INFO("Node %08x \r\n", ln);
    
    if ( ln->Device != NULL ) {
      DEBUG_INFO("\tID[%04x] CMD[%d] CMDSTATUS[%d]\r\n", ln->Device->u16ID, ln->Device->u8CMD, ln->Device->u8CMDSTATUS);
    }
    
    ln = ln->next;
  }
}

DeviceNode *getDevice(uint16_t id)
{
  if(NULL == gList)
  {
    return NULL;
  }
  
  List *ln = gList->next;
  while(NULL != ln)
  {
    if(id == ln->Device->u16ID)
    {
      return ln->Device;
    }

    ln = ln->next;
  }
  
  return NULL;
  
}

/*
*   向F405发送所有设备上线信息
*/
void SendALLDeviceNodeToF405(void)
{
  List *ln = gList;

  while(ln)
  {
    if(NULL != ln->Device)
    {
      if(ln->Device->DeviceOnLine)
      {
        UartSendOnlineMSGToF405(ln->Device);
      }      
    }
    
    ln = ln->next;
  }  
}



