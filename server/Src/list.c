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

extern LoraModule gLoraMS;
extern UartModule gUartMx;
extern RTC_HandleTypeDef hrtc;
//在服务端每发送一条控制命令，都创建一个node，并将node加入链表
//在终端执行完指令之后，会返回resp，服务端检测到这个resp后，
//将node从链表移除

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
  
  PRINT("List:add new device[%04x]\r\n", id);
  //将lcn加入链表中
  last->next = lastN;
  lastN->Device = devN;
  lastN->next = NULL;
  
  devN->u16ID = id;
  devN->u8CHAN = DEFAULT_CHANNEL;
  devN->u8CMDSTATUS = CMD_STATUS_IDLE;
}

//删除设备
uint8_t delete_device_from_list(List *list, uint16_t id)
{
  List *del = list;
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
        free(del);
        
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
//          T_Resp_Info *respInfo = (T_Resp_Info *)gUartMx.dma_sbuff;
//          
//          printf("list process:send resp to f405\r\n");
//          if(generalRespInfo(ln->Device, respInfo))
//          {
//            UartSendRespToF405(&gUartMx);
//          }
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
        
        return;
      }
    }
    
    ln = ln->next;
  }
}

void ListProcess(List *list)
{
  List *ln = list;
  
  uint32_t u32NowTime = GetRTCTime();
  
  while(ln) {
    
    if ( ln->Device ) {
      switch(ln->Device->u8CMDSTATUS) {
      case CMD_STATUS_IDLE:
        break;
      case CMD_STATUS_STANDBY:
        //执行准备就绪的指令
        PRINT("list process:send cmd %d\r\n", ln->Device->u8CMD);
        if (true == LoraCtrlEndPoint(&gLoraMS, ln->Device->u16ID, ln->Device->u8CHAN, ln->Device->u8CMD, ln->Device->u32Identify )) {
          ln->Device->u8CMDSTATUS = CMD_STATUS_RUN;
          ln->Device->u32Time = GetRTCTime();
        }
        break;
      case CMD_STATUS_RUN: {
        //等待判断指令超时
        uint32_t nowTime = GetRTCTime();
        if ( nowTime - ln->Device->u32Time > 2 ) {
          //10秒内没有响应，设置为超时。
          //do timeout 
          UartSendCMDTimeOutToF405(&gUartMx, ln->Device->u32Identify);
          ln->Device->u8CMDSTATUS = CMD_STATUS_IDLE;
        }
        break;
      }
      case CMD_STATUS_DONE: {
        //接收到resp
        printf("LIST:send resp to f405\r\n");
        UartSendRespToF405(&gUartMx, ln->Device);
        //test start
        //        LookAllList(ln);
        //test end
        //对电机控制命令要特殊处理，因为发送了电机指令会立刻返回一条信息。
        //在电机执行结束之后还会返回一条信息。
        if ((ln->Device->u8CMD == CMD_MOTOR_UP) || (ln->Device->u8CMD == CMD_MOTOR_DOWN))  {
          if ( ln->Device->u8RESP == MOTOR_RUNING ) {
            ln->Device->u8CMDSTATUS = CMD_STATUS_RUN;
          } else {
            ln->Device->u8CMDSTATUS = CMD_STATUS_IDLE;
          }
        } else {
          ln->Device->u8CMDSTATUS = CMD_STATUS_IDLE;
        }
        printf("--------------------\r\n");
        break;
      }
      case CMD_STATUS_HEART:
        break;
        
      default:
        break;
      }
    }
    
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
    
    printf("Node %08x \r\n", ln);
    
    if ( ln->Device != NULL ) {
      printf("\tID[%04x] CHAN[%02x]\r\n", ln->Device->u16ID, ln->Device->u8CHAN);
      printf("\tCMD[%d] CMDSTATUS[%d]\r\n", ln->Device->u8CMD, ln->Device->u8CMDSTATUS);       
    }
    
    ln = ln->next;
  }
}

