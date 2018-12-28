#include "list.h"
#include "stdint.h"
#include "stm32f1xx.h"
#include "stdlib.h"
#include "string.h"
#include "user_config.h"
#include "maincontrol.h"
#include "rtc.h"
#include "flash.h"

extern RTC_HandleTypeDef hrtc;

//设备链表
//每个endpoint对应一个链表的节点
//在节点内保存设备信息，设备指令等
//通过链表的一个循环判断线程来执行
//设备的动作。
static List *gList = NULL;       

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
}

/*
*   将设备加入到链表中
*   如果设备已经存在于链表内，则直接退出
*/
bool AddDeviceToList(uint16_t id)
{  
  List *last = gList;
  List *pre = gList;
  
  DEBUG_INFO("List:add new device[%d]\r\n", id);
  
  //首先判断要加入到设备id是否已经在链表内
  while(last)
  {
    if(last) {
      if(last->Device){
        if(last->Device->u16ID == id) {
          last->Device->u8CMDSTATUS = CMD_STATUS_IDLE;
          last->Device->u8CMDRetry = 0;
          last->Device->DeviceOnLine = true;
          last->Device->u32Time = GetRTCTime();
          return true;
        }
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
  devnode->u8CMDSTATUS = CMD_STATUS_IDLE;
  devnode->u8CMDRetry = 0;
  devnode->DeviceOnLine = true;
  devnode->u32Time = GetRTCTime();
  
  return true;
}

#if 0
/*
*   从链表内寻找指定ID的设备节点并删除
*/
static bool delete_device_from_list(uint16_t id)
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
bool SendCMDToList(uint16_t id, uint8_t cmd, uint32_t identify)
{
  List *ln = gList;
  DEBUG_INFO("List:set device[%04x],cmd:%d\r\n", id, cmd);
  while ( ln ) {
    if ( ln->Device ) {
      if ( ln->Device->u16ID == id ) {
        if( ln->Device->u8CMDSTATUS == CMD_STATUS_IDLE ) {
          //设备空闲，可以写入新指令
          ln->Device->u8CMD = cmd;
          ln->Device->u32Identify = identify;
          ln->Device->u8RESP = 0;
          ln->Device->u8CMDSTATUS = CMD_STATUS_STANDBY;
        } else {
          //设备忙，不允许写入新指令，返回忙信息给F405
          UartSendMSGToF405(NORMAL_BUSY, 0, identify);
          DEBUG_INFO("set_device_of_list_cmd: device busy\r\n");
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
          ln->Device->u8CMD = cmd;
          ln->Device->u8RESP = resp;
          ln->Device->u8CMDSTATUS = CMD_STATUS_DONE;
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
void ListTask(void)
{
  List *ln = gList;
  uint32_t nowTime;
  
  while ( ln ) {
    
    if ( ln->Device ) {
      switch(ln->Device->u8CMDSTATUS) {
      case CMD_STATUS_IDLE:{
        nowTime = GetRTCTime();
        if ((( ln->Device->DeviceOnLine == true )   //设备在线则发送心跳包
             && ( nowTime - ln->Device->u32Time > HEART_TIMEINTERVAL )) ||
            ((ln->Device->DeviceOnLine == false)    //设备掉线则需要等待更久时间进行心跳检测
             && ( nowTime - ln->Device->u32Time > CHECK_OFFLINE_DEVICE_MAX_TIMEINTERVAL )))
        {
            ln->Device->u8CMDSTATUS = CMD_STATUS_STANDBY;
            ln->Device->u8CMD = DEVICE_HEART;
            ln->Device->u8RESP = 0;
            ln->Device->u32Identify = 0;
        }
      }
      break;
      
      case CMD_STATUS_STANDBY:
        //执行准备就绪的指令
        DEBUG_INFO("list process:id:%04x\r\n",ln->Device->u16ID);
        if (true == LoraCtrlEndPoint(ln->Device->u16ID, /*ln->Device->u8CHAN, */ln->Device->u8CMD, ln->Device->u32Identify )) {
          ln->Device->u8CMDSTATUS = CMD_STATUS_RUN;
          ln->Device->u32Time = GetRTCTime();
        }
        break;
        
      case CMD_STATUS_RUN: {
        //等待判断指令超时
        nowTime = GetRTCTime();
        uint32_t timeout;
        //设置不同的超时时间
        if ( ln->Device->u8CMD != DEVICE_HEART ) {
          timeout = CMD_TIMEOUT;
        } else {
          timeout = CMD_RETRY_TIMEINTERVAL;
        }
        //如果是心跳指令，则超时CMD_RETRY_TIMEINTERVAL后重试CMD_MAX_RETRY_TIMES次
        //后，若没有接收到响应包，则认为掉线
//        if ( ln->Device->DeviceOnLine == true ) {
          //只对在线的设备进行心跳检测
          if ( nowTime - ln->Device->u32Time >= timeout ) {
            //设置命令重新执行
            ln->Device->u8CMDSTATUS = CMD_STATUS_STANDBY;
            //重试次数加一
            ln->Device->u8CMDRetry++;
            if ( ln->Device->u8CMDRetry >= CMD_MAX_RETRY_TIMES ) {
              //达到重试次数的上限都没有响应，则认为设备掉线了。
              //从链表中删除设备，并通知f405设备掉线。
              if ( ln->Device->DeviceOnLine == true ) {
                UartSendMSGToF405(NODE_OFFLINE, ln->Device->u16ID, 0);
              }
                //通知成功则删除从设备链表和flash链表内删除对应的设备
                
                //2018-12-11修改，修改设备掉线后的动作，不再从列表内删除，改为设置在线状态标志
#if 0
                FlashDelDeviceFromList(ln->Device->u16ID);
                delete_device_from_list(ln->Device->u16ID);
                //删除节点以后必须将设备节点指针置空，否则会出现指针错误
                ln = NULL;
#else
                ln->Device->u8CMDSTATUS = CMD_STATUS_IDLE;
                ln->Device->u8CMDRetry = 0;
                ln->Device->DeviceOnLine = false;
              }
#endif
            }
//          }
//        }
      }break;
      
      case CMD_STATUS_DONE: 
        //接收到resp
        ln->Device->u32Time = GetRTCTime();
        
        if (ln->Device->u8CMD != DEVICE_HEART) {
          if (UartSendRespToF405(ln->Device)) {
            //对电机控制命令要特殊处理，因为发送了电机指令会立刻返回一条信息。
            //在电机执行结束之后还会返回一条信息。
            if ( ln->Device->u8RESP == MOTOR_RUNING ) {
              ln->Device->u8CMDSTATUS = CMD_STATUS_RUN;
            } else {
              ln->Device->u8CMDSTATUS = CMD_STATUS_IDLE;
              ln->Device->u32Identify = 0;
              ln->Device->u8CMD = CMD_NONE;
              ln->Device->u8RESP = 0;
            }
            ln->Device->u8CMDRetry = 0;
          }
        } else {
          ln->Device->u8CMDSTATUS = CMD_STATUS_IDLE;
          ln->Device->u32Identify = 0;
          ln->Device->u8CMD = CMD_NONE;
          ln->Device->u8RESP = 0;
          ln->Device->u8CMDRetry = 0;
          //如果是曾经掉线的设备接收到心跳包响应
          //则重新上线该设备
          if ( ln->Device->DeviceOnLine == false ) {
            UartSendMSGToF405(NODE_ONLINE, ln->Device->u16ID, 0);
            ln->Device->DeviceOnLine = true;
          }
        }
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




