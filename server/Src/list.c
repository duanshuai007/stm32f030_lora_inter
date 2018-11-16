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

//���豸���뵽������
void add_device_to_list(List *list, uint16_t id)
{  
  List *last = list;
  PRINT("List:add new device[%04x]\r\n", id);
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
  //�����һ���ڵ�������
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
  
  //��lcn����������
  last->next = lastN;
  lastN->Device = devN;
  lastN->next = NULL;
  
  devN->u16ID = id;
  devN->u8CHAN = DEFAULT_CHANNEL;
  devN->u8CMDSTATUS = CMD_STATUS_IDLE;
  devN->u8CMDRetry = 0;
}

//ɾ���豸
static uint8_t delete_device_from_list(List *list, uint16_t id)
{
  PRINT("List:delete device[%04x]\r\n", id);
  List *del = list->next;
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
          //�豸���У�����д����ָ��
          ln->Device->u8CMD = cmd;
          ln->Device->u32Identify = identify;
          ln->Device->u8RESP = 0;
          ln->Device->u8CMDSTATUS = CMD_STATUS_STANDBY;
        } else {
          //�豸æ��������д����ָ�����æ��Ϣ��F405
          UartSendDeviceBusyToF405(&gUartMx, identify);
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
        //�쳣����resp
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
        //�豸�ڿ���״̬ʱ����豸�ۼ�һ�����ڿ���״̬������һ��������
        if ( nowTime - ln->Device->u32Time > HEART_TIMESTAMP ) {
          ln->Device->u8CMDSTATUS = CMD_STATUS_STANDBY;
          ln->Device->u8CMD = DEVICE_HEART;
          ln->Device->u8RESP = 0;
          ln->Device->u32Identify = 0;
        }
      }
      break;
      case CMD_STATUS_STANDBY:
        //ִ��׼��������ָ��
        PRINT("list process:id:%04x\r\n",ln->Device->u16ID);
        
        if (true == LoraCtrlEndPoint(&gLoraMS, ln->Device->u16ID, ln->Device->u8CHAN, ln->Device->u8CMD, ln->Device->u32Identify )) {
          ln->Device->u8CMDSTATUS = CMD_STATUS_RUN;
          ln->Device->u32Time = GetRTCTime();
        }
        break;
      case CMD_STATUS_RUN: {
        //�ȴ��ж�ָ�ʱ
        uint32_t nowTime = GetRTCTime();
        uint32_t timeout;
        if ( ln->Device->u8CMD != DEVICE_HEART ) {
          timeout = CMD_TIMEOUT;
        } else {
          timeout = CMD_RETRY_TIMEINTERVAL;
        }
        //���������ָ���ʱCMD_RETRY_TIMEINTERVAL������CMD_MAX_RETRY_TIMES��
        //��û�н��յ���Ӧ��������Ϊ����
        if ( nowTime - ln->Device->u32Time >= timeout ) {
          //������������ִ��
          ln->Device->u8CMDSTATUS = CMD_STATUS_STANDBY;
          //10����û����Ӧ������Ϊ����
          ln->Device->u8CMDRetry++;
          if ( ln->Device->u8CMDRetry >= CMD_MAX_RETRY_TIMES ) {
            //��ζ�û����Ӧ������Ϊ�豸�����ˡ���������ɾ���豸����֪ͨf405�豸���ߡ�
            UartSendOffLineToF405(&gUartMx, ln->Device->u16ID);
            FlashDelDeviceFromList(gFDList, ln->Device->u16ID);
            delete_device_from_list(list, ln->Device->u16ID);
            //ɾ���ڵ��Ժ���뽫�豸�ڵ�ָ���ÿգ���������ָ�����
            ln = NULL;
          }
        }
        break;
      }
      case CMD_STATUS_DONE: {
        //���յ�resp
        ln->Device->u32Time = GetRTCTime();
        
        if (ln->Device->u8CMD != DEVICE_HEART) {
          UartSendRespToF405(&gUartMx, ln->Device);
        }
        
        //�Ե����������Ҫ���⴦����Ϊ�����˵��ָ������̷���һ����Ϣ��
        //�ڵ��ִ�н���֮�󻹻᷵��һ����Ϣ��
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
*   �����ã��鿴������������нڵ�
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

