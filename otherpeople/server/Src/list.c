#include "list.h"
#include "stdint.h"
#include "stm32f1xx.h"
#include "stdlib.h"
#include "string.h"
#include "user_config.h"
#include "maincontrol.h"
#include "reboot.h"
#include "flash_if.h"


//�豸����
//ÿ��endpoint��Ӧһ������Ľڵ�
//�ڽڵ��ڱ����豸��Ϣ���豸ָ���
//ͨ�������һ��ѭ���ж��߳���ִ��
//�豸�Ķ�����
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
*   ���豸���뵽������
*   ����豸�Ѿ������������ڣ���ֱ���˳�
*/
bool AddDeviceToList(uint16_t id, bool onlineFlg)
{  
  List *last = gList;
  List *pre = gList;
  
  DEBUG_INFO("List:add new device[%d]\r\n", id);
  
  //�����ж�Ҫ���뵽�豸id�Ƿ��Ѿ���������
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
*   ��������Ѱ��ָ��ID���豸�ڵ㲢ɾ��
*/
bool delete_device_from_list(uint16_t id)
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
*   �豸������̣�����ѭ���ڵ��ã�ѭ���ж�������ÿ���豸��״̬
*   ����Ӧ��u8CMDSTATUS״̬Ϊ:
*     CMD_STATUS_IDLE       �豸����
*     CMD_STATUS_STANDBY    �豸��ָ��ȴ�ִ��
*     CMD_STATUS_RUN        �豸ָ����������
*     CMD_STATUS_DONE       �豸ָ���������
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

#define NEW_PROTOCOL_BETWEEN_SERVER_AND_EP //ʹ���¶����Э��(resp�ֶ�ֻ��ʾ����״̬���߳������ľ���)
#ifdef NEW_PROTOCOL_BETWEEN_SERVER_AND_EP
static void doHeartResp(DeviceNode *device)
{
  uint8_t status = device->u8HeartRESP;

  device->u8EpStatus = status;

  //��103�������������ݣ���Ҫ��device�ڲ��ָ���ʼ���á�
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

#else //ʹ�þɶ����Э��(resp�ֶθ�4λ��ʾ����״̬����4λ��ʾ�г�����û��)
static void doHeartResp(DeviceNode *device)
{
  uint8_t resp = device->u8HeartRESP;
  
  uint8_t respStatus = (resp >> 4);

  device->u8EpStatus = respStatus;


  //��103�������������ݣ���Ҫ��device�ڲ��ָ���ʼ���á�
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

      //�����쳣��Ϣ
      if( 0 != ln->Device->u8AbnormalRESP)
      {        
        doAbnormalResp(ln->Device);
        
        ln->Device->u8AbnormalRESP = 0;  
      }
      
      //�������������Ϣ
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

      //�������������Ϣ
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
      //CMD_HEART_INTERVAL_TIME��û���յ���device����Ϣ�������device��������
      if(nowTime - ln->Device->u32HeartTime > CMD_HEART_INTERVAL_TIME)
      {
        insertCmdToList(ln->Device->u16ID, 0, DEVICE_HEART, 0);
        ln->Device->u32HeartTime = nowTime;
      }
      
      //CMD_ONLINE_TIMEOUT��û���յ���device����Ϣ����Ϊ��device������
      if(nowTime - ln->Device->u32LastTime > CMD_ONLINE_TIMEOUT)
      {
        if ( ln->Device->DeviceOnLine == true )
        {
          UartSendAutoMSGToF405(NODE_OFFLINE, ln->Device->u16ID, 0);
          ln->Device->DeviceOnLine = false;
          //���ߺ���reset���ʹ������
          insertCmdToList(ln->Device->u16ID, 0, CMD_SYSTEM_RESET, 0);          
        }        
      }
        
      //����������������Ӧ
      switch(ln->Device->u8CMDSTATUS) {              
      case CMD_STATUS_RUN: 
      {
        //�ȴ��ж�ָ�ʱ
        nowTime = HAL_GetTick();

        //��u8RESP�ֶ�ΪMOTOR_RUNINGʱ����ʾ̧��������µ���ʱ���м�״̬
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
              //�ﵽ���Դ��������޶�û����Ӧ������Ϊ�豸�����ˡ�
              //��������ɾ���豸����֪ͨf405�豸���ߡ�

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

        //����EP��״̬        
        setEndpointStatus(ln->Device);

        if(MOTOR_BUSY != ln->Device->u8RESP)
        {
          UartSendRespToF405(ln->Device);
        }
        
        //�Ե����������Ҫ���⴦����Ϊ�����˵��ָ������̷���һ����Ϣ��
        //�ڵ��ִ�н���֮�󻹻᷵��һ����Ϣ��
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

        //��ʼ��device��ĳЩ�ֶ�
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
*   ��F405���������豸������Ϣ
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



