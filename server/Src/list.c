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

//�豸����
//ÿ��endpoint��Ӧһ������Ľڵ�
//�ڽڵ��ڱ����豸��Ϣ���豸ָ���
//ͨ�������һ��ѭ���ж��߳���ִ��
//�豸�Ķ�����
static List *gList = NULL;       

/*
*   �豸�����ʼ��
*   ���������ַ����ʼ�����������
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
*   ���豸���뵽������
*   ����豸�Ѿ������������ڣ���ֱ���˳�
*/
bool AddDeviceToList(uint16_t id)
{  
  List *last = gList;
  List *pre = gList;
  
  DEBUG_INFO("List:add new device[%d]\r\n", id);
  
  //�����ж�Ҫ���뵽�豸id�Ƿ��Ѿ���������
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
  
  //Ҫ������豸id����������
  //�����µ�����ڵ㱣���豸��Ϣ
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
  
  //���½ڵ����������
  pre->next = lastlist;
  //���ýڵ�����
  lastlist->Device = devnode;
  lastlist->next = NULL;
  //�����豸����
  devnode->u16ID = id;
  devnode->u8CMDSTATUS = CMD_STATUS_IDLE;
  devnode->u8CMDRetry = 0;
  devnode->DeviceOnLine = true;
  devnode->u32Time = GetRTCTime();
  
  return true;
}

#if 0
/*
*   ��������Ѱ��ָ��ID���豸�ڵ㲢ɾ��
*/
static bool delete_device_from_list(uint16_t id)
{
  DEBUG_INFO("List:delete device[%04x]\r\n", id);
  //��Ҫɾ���Ľڵ�
  List *del = gList->next;
  //��Ҫɾ���Ľڵ����һ���ڵ�
  List *pre = gList;
  
  while ( del ) {
    if ( del->Device ) {
      if ( del->Device->u16ID == id ) {
        //�ҵ����豸��ɾ��
        if ( del->next ) {
          //�����Ҫɾ���Ľڵ㲻�����һ���ڵ�
          List *next = del->next;
          pre->next = next;
        } else {
          //��������һ���ڵ�
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
    //ָ����һ���ڵ�
    pre = del;
    del = del->next;
  }
  
  return false;
}
#endif

/*
*   ����ָ��ID�豸�ڵ��CMD��IDENTIFY
*   �豸�����ڷ���false
*   �豸æ���豸��������true
*/
bool SendCMDToList(uint16_t id, uint8_t cmd, uint32_t identify)
{
  List *ln = gList;
  DEBUG_INFO("List:set device[%04x],cmd:%d\r\n", id, cmd);
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
*   ����resp��Ϣ��ָ��ID�Ľڵ�
*/
void SendCMDRespToList(uint16_t id, uint8_t cmd, uint32_t identify, uint8_t resp)
{
  List *ln = gList;
  DEBUG_INFO("list: get resp, id[%d]!\r\n",id);
  while ( ln ) {
    if( ln->Device ) {
      if (ln->Device->u16ID == id) {
        //������ָ����Ӧ
        if (( ln->Device->u8CMD == cmd) && (ln->Device->u32Identify == identify)) {
          ln->Device->u8RESP = resp;
          ln->Device->u8CMDSTATUS = CMD_STATUS_DONE;
        }
        //�쳣������Ӧ
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
*   �豸������̣�����ѭ���ڵ��ã�ѭ���ж�������ÿ���豸��״̬
*   ����Ӧ��u8CMDSTATUS״̬Ϊ:
*     CMD_STATUS_IDLE       �豸����
*     CMD_STATUS_STANDBY    �豸��ָ��ȴ�ִ��
*     CMD_STATUS_RUN        �豸ָ����������
*     CMD_STATUS_DONE       �豸ָ���������
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
        if ((( ln->Device->DeviceOnLine == true )   //�豸��������������
             && ( nowTime - ln->Device->u32Time > HEART_TIMEINTERVAL )) ||
            ((ln->Device->DeviceOnLine == false)    //�豸��������Ҫ�ȴ�����ʱ������������
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
        //ִ��׼��������ָ��
        DEBUG_INFO("list process:id:%04x\r\n",ln->Device->u16ID);
        if (true == LoraCtrlEndPoint(ln->Device->u16ID, /*ln->Device->u8CHAN, */ln->Device->u8CMD, ln->Device->u32Identify )) {
          ln->Device->u8CMDSTATUS = CMD_STATUS_RUN;
          ln->Device->u32Time = GetRTCTime();
        }
        break;
        
      case CMD_STATUS_RUN: {
        //�ȴ��ж�ָ�ʱ
        nowTime = GetRTCTime();
        uint32_t timeout;
        //���ò�ͬ�ĳ�ʱʱ��
        if ( ln->Device->u8CMD != DEVICE_HEART ) {
          timeout = CMD_TIMEOUT;
        } else {
          timeout = CMD_RETRY_TIMEINTERVAL;
        }
        //���������ָ���ʱCMD_RETRY_TIMEINTERVAL������CMD_MAX_RETRY_TIMES��
        //����û�н��յ���Ӧ��������Ϊ����
//        if ( ln->Device->DeviceOnLine == true ) {
          //ֻ�����ߵ��豸�����������
          if ( nowTime - ln->Device->u32Time >= timeout ) {
            //������������ִ��
            ln->Device->u8CMDSTATUS = CMD_STATUS_STANDBY;
            //���Դ�����һ
            ln->Device->u8CMDRetry++;
            if ( ln->Device->u8CMDRetry >= CMD_MAX_RETRY_TIMES ) {
              //�ﵽ���Դ��������޶�û����Ӧ������Ϊ�豸�����ˡ�
              //��������ɾ���豸����֪ͨf405�豸���ߡ�
              if ( ln->Device->DeviceOnLine == true ) {
                UartSendMSGToF405(NODE_OFFLINE, ln->Device->u16ID, 0);
              }
                //֪ͨ�ɹ���ɾ�����豸�����flash������ɾ����Ӧ���豸
                
                //2018-12-11�޸ģ��޸��豸���ߺ�Ķ��������ٴ��б���ɾ������Ϊ��������״̬��־
#if 0
                FlashDelDeviceFromList(ln->Device->u16ID);
                delete_device_from_list(ln->Device->u16ID);
                //ɾ���ڵ��Ժ���뽫�豸�ڵ�ָ���ÿգ���������ָ�����
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
        //���յ�resp
        ln->Device->u32Time = GetRTCTime();
        
        if (ln->Device->u8CMD != DEVICE_HEART) {
          if (UartSendRespToF405(ln->Device)) {
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
          }
        } else {
          ln->Device->u8CMDSTATUS = CMD_STATUS_IDLE;
          ln->Device->u32Identify = 0;
          ln->Device->u8CMD = CMD_NONE;
          ln->Device->u8RESP = 0;
          ln->Device->u8CMDRetry = 0;
          //������������ߵ��豸���յ���������Ӧ
          //���������߸��豸
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
*   �鿴������������нڵ�
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




