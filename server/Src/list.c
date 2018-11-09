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
//�ڷ����ÿ����һ���������������һ��node������node��������
//���ն�ִ����ָ��֮�󣬻᷵��resp������˼�⵽���resp��
//��node�������Ƴ�

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

//���豸���뵽������
void add_device_to_list(List *list, uint16_t id)
{  
  List *last = list;

  //ȡ�����һ��node
  while( last->next != NULL ) {
    //����豸�Ѿ��������У��򷵻�
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
  //��lcn����������
  last->next = lastN;
  lastN->Device = devN;
  lastN->next = NULL;
  
  devN->u16ID = id;
  devN->u8CHAN = DEFAULT_CHANNEL;
  devN->u8CMDSTATUS = CMD_STATUS_IDLE;
}

//ɾ���豸
uint8_t delete_device_from_list(List *list, uint16_t id)
{
  List *del = list;
  List *pre = list;
  
  while ( del ) {
    
    if ( del->Device ) {
      if ( del->Device->u16ID == id ) {
        //�ҵ����豸��ɾ��
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
          //�豸���У�����д����ָ��
          ln->Device->u8CMD = cmd;
          ln->Device->u32Identify = identify;
          ln->Device->u8RESP = 0;
          ln->Device->u8CMDSTATUS = CMD_STATUS_STANDBY;
        } else {
          //�豸æ��������д����ָ�����æ��Ϣ��F405
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

//�����յ���resp��Ϣд�뵽�����У��޸Ķ�Ӧ���豸����ڵ�
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
        //ִ��׼��������ָ��
        PRINT("list process:send cmd %d\r\n", ln->Device->u8CMD);
        if (true == LoraCtrlEndPoint(&gLoraMS, ln->Device->u16ID, ln->Device->u8CHAN, ln->Device->u8CMD, ln->Device->u32Identify )) {
          ln->Device->u8CMDSTATUS = CMD_STATUS_RUN;
          ln->Device->u32Time = GetRTCTime();
        }
        break;
      case CMD_STATUS_RUN: {
        //�ȴ��ж�ָ�ʱ
        uint32_t nowTime = GetRTCTime();
        if ( nowTime - ln->Device->u32Time > 2 ) {
          //10����û����Ӧ������Ϊ��ʱ��
          //do timeout 
          UartSendCMDTimeOutToF405(&gUartMx, ln->Device->u32Identify);
          ln->Device->u8CMDSTATUS = CMD_STATUS_IDLE;
        }
        break;
      }
      case CMD_STATUS_DONE: {
        //���յ�resp
        printf("LIST:send resp to f405\r\n");
        UartSendRespToF405(&gUartMx, ln->Device);
        //test start
        //        LookAllList(ln);
        //test end
        //�Ե����������Ҫ���⴦����Ϊ�����˵��ָ������̷���һ����Ϣ��
        //�ڵ��ִ�н���֮�󻹻᷵��һ����Ϣ��
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
*   �����ã��鿴������������нڵ�
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

