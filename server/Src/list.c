#include "list.h"
#include "stdint.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal_def.h"
#include "stdlib.h"
#include "string.h"
#include "user_config.h"
#include "lora.h"
#include "maincontrol.h"
#include "rtc.h"
#include "flash.h"

extern LoraModule gLoraMS;
extern UartModule gUartMx;
extern RTC_HandleTypeDef hrtc;
extern FLASHDeviceList *gFDList;

volatile uint8_t rw_bit = 0;

List *list_init(void)
{
  List *list = (List *)malloc(sizeof(List));
  if ( NULL == list ) {
    PRINT("list == null\r\n");
    return NULL;
  }
  
  PRINT("sizeof(List)=%d\r\n", sizeof(List));
  
  list->Device = NULL;
  list->next = NULL;
  
  return list;
}

//将设备加入到链表中
void add_device_to_list(List *list, uint16_t id)
{  
  List *last = list;
  PRINT("List:add new device[%04x]\r\n", id);
  //取出最后一个node
  while( last->next != NULL ) {
    //如果设备已经在链表中，则返回
    if ( last->Device ) {
      if ( last->Device->u16ID == id ) {
        //PRINT("List:already have same id device,[%04x]\r\n", id);
        return;
      }
    }
    
    last = last->next;
  }
  //对最后一个节点做处理
  if ( last->Device ) {
    if ( last->Device->u16ID == id ) {
      //PRINT("List:already have same id device,[%04x]\r\n", id);
      return;
    }
  }
  
  List *lastN = (List *)malloc(sizeof(List));
  if ( NULL == lastN ) {
    PRINT("add_device_to_list: malloc(last) == NULL\r\n");
    return;
  }
  
  DeviceNode *devN = (DeviceNode *)malloc(sizeof(DeviceNode));
  if ( NULL == devN ) {
    PRINT("add_device_to_list:malloc(device) == NULL\r\n");
    free(lastN);
    return;
  }
  memset(devN, 0, sizeof(DeviceNode));
  
  //将lcn加入链表中
  last->next = lastN;
  lastN->Device = devN;
  lastN->next = NULL;
  
  devN->u16ID = id;
  devN->u8CHAN = DEFAULT_CHANNEL;
  devN->u8CMDSTATUS = CMD_STATUS_IDLE;
  devN->u8CMDRetry = 0;
}

//删除设备
static uint8_t delete_device_from_list(List *list, uint16_t id)
{
  PRINT("List:delete device[%04x]\r\n", id);
  List *del = list->next;
  List *pre = list;
  
  while ( del ) {
    if ( del->Device ) {
      if ( del->Device->u16ID == id ) {
        //找到了设备，删除
        if ( del->next ) {
          List *next = del->next;
          pre->next = next;
        } else {
          pre->next = NULL;
        }
        
        free(del->Device);
        del->Device = NULL;
        del->next = NULL;
        free(del);
        del = NULL;
        
        return 1;
      }
    }
    
    pre = del;
    del = del->next;
  }
  
  return 0;
}

void set_device_of_list_cmd(List *list, uint16_t id, uint8_t cmd, uint32_t identify)
{
  List *ln = list;
  PRINT("List:set device[%04x],cmd:%d\r\n", id, cmd);
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
          UartSendDeviceBusyToF405(&gUartMx, identify);
          PRINT("set_device_of_list_cmd: device busy\r\n");
        }
      }
    }
    ln = ln->next;
  }
}

//将接收到的resp信息写入到链表中，修改对应的设备命令节点
void SendCMDRespToList(List *list, uint16_t id, uint8_t cmd, uint32_t identify, uint8_t resp)
{
  List *ln = list;
  PRINT("list: get resp!\r\n");
  while ( ln ) {
    if( ln->Device ) {
      if (ln->Device->u16ID == id) {
        if (( ln->Device->u8CMD == cmd) && (ln->Device->u32Identify == identify)) {
          ln->Device->u8RESP = resp;
          ln->Device->u8CMDSTATUS = CMD_STATUS_DONE;
        }
        //异常动作resp
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

void ListProcess(List *list)
{
  List *ln = list;
  
  while ( ln ) {
    
    if ( ln->Device ) {
      switch(ln->Device->u8CMDSTATUS) {
      case CMD_STATUS_IDLE:{
        uint32_t nowTime = GetRTCTime();
        //设备在空闲状态时如果设备累计一分钟在空闲状态，则发送一个心跳包
        if ( nowTime - ln->Device->u32Time > HEART_TIMESTAMP ) {
          ln->Device->u8CMDSTATUS = CMD_STATUS_STANDBY;
          ln->Device->u8CMD = DEVICE_HEART;
          ln->Device->u8RESP = 0;
          ln->Device->u32Identify = 0;
        }
      }
      break;
      case CMD_STATUS_STANDBY:
        //执行准备就绪的指令
        PRINT("list process:id:%04x\r\n",ln->Device->u16ID);
        
        if (true == LoraCtrlEndPoint(&gLoraMS, ln->Device->u16ID, ln->Device->u8CHAN, ln->Device->u8CMD, ln->Device->u32Identify )) {
          ln->Device->u8CMDSTATUS = CMD_STATUS_RUN;
          ln->Device->u32Time = GetRTCTime();
        }
        break;
      case CMD_STATUS_RUN: {
        //等待判断指令超时
        uint32_t nowTime = GetRTCTime();
        uint32_t timeout;
        if ( ln->Device->u8CMD != DEVICE_HEART ) {
          timeout = CMD_TIMEOUT;
        } else {
          timeout = CMD_RETRY_TIMEINTERVAL;
        }
        //如果是心跳指令，则超时CMD_RETRY_TIMEINTERVAL后重试CMD_MAX_RETRY_TIMES次
        //若没有接收到响应包，则认为掉线
        if ( nowTime - ln->Device->u32Time >= timeout ) {
          //设置命令重新执行
          ln->Device->u8CMDSTATUS = CMD_STATUS_STANDBY;
          //10秒内没有响应，设置为掉线
          ln->Device->u8CMDRetry++;
          if ( ln->Device->u8CMDRetry >= CMD_MAX_RETRY_TIMES ) {
            //五次都没有响应，则认为设备掉线了。从链表中删除设备，并通知f405设备掉线。
            UartSendOffLineToF405(&gUartMx, ln->Device->u16ID);
            FlashDelDeviceFromList(gFDList, ln->Device->u16ID);
            delete_device_from_list(list, ln->Device->u16ID);
            //删除节点以后必须将设备节点指针置空，否则会出现指针错误
            ln = NULL;
          }
        }
        break;
      }
      case CMD_STATUS_DONE: {
        //接收到resp
        ln->Device->u32Time = GetRTCTime();
        
        if (ln->Device->u8CMD != DEVICE_HEART) {
          UartSendRespToF405(&gUartMx, ln->Device);
        }
        
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
        break;
      }
      
      default:
        break;
      }
    }
    
    //    printf("ln=%08x\r\n", ln);
    if ( ln )
      ln = ln->next;
    
  }/*end while(1)*/
}

/*
*   调试用，查看整个链表的所有节点
*/
void LookAllList(List *list)
{
  List *ln = list;
  
  while( ln != NULL ) {
    
    PRINT("Node %08x \r\n", ln);
    
    if ( ln->Device != NULL ) {
      PRINT("\tID[%04x] CHAN[%02x]\r\n", ln->Device->u16ID, ln->Device->u8CHAN);
      PRINT("\tCMD[%d] CMDSTATUS[%d]\r\n", ln->Device->u8CMD, ln->Device->u8CMDSTATUS);       
    }
    
    ln = ln->next;
  }
}

