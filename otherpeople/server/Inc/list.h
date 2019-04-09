#ifndef __MY_LIST_H__
#define __MY_LIST_H__

#include "stdint.h"
#include "user_config.h"

//�������״̬��Ϣ��������CMD�ǵ������ʱ��resp���ڸ�ֵ
#define MOTOR_RUNING          99   //���������ʼ������
#define MOTOR_BUSY          9   //��������æµ״̬

//�豸��ָ��ִ��״̬������server����cmd״̬�ĸ���
#define CMD_STATUS_IDLE       100   //����״̬
#define CMD_STATUS_RUN        101   //��������
#define CMD_STATUS_DONE       102   //�������
#define CMD_STATUS_STOP       103   //��ʱֹͣ����
#define CMD_STATUS_STANDBY    104   //����ָ��ȴ�����

#pragma pack(1)
typedef struct DeviceNode {
  uint32_t u32Identify;     //ָ��identify

  uint32_t u32LastTime;     //�����յ��ն���Ϣ��ʱ��
  
  uint32_t u32HeartTime;     //�����յ��ն���Ϣ��ʱ��,������������������ʼʱ��

  uint32_t u32ActionStartTime; //̧��������յ��м�״̬��ʱ��

  uint32_t u32CmdStartTime;    //��������Ŀ�ʼʱ��
  
  uint16_t u16ID;           //�豸ID
  uint8_t u8CMD;            //�豸��ָ��  
  uint8_t u8CMDSTATUS;      //ָ��ִ��״̬
  
  uint8_t u8RESP;           //��������Ӧ
  uint8_t u8CMDRetry;       //cmd���ԵĴ���
  uint8_t u8EpStatus;       //����״̬��ǰ�㣬���㣬ֱ�����Ե�
  uint8_t u8HaveCarFlg;       //�������Ƿ��г�

  bool DeviceOnLine;        //�豸����״̬

  uint32_t u32Data;
  
  uint8_t u8AbnormalRESP;           //�����쳣֪ͨ
  uint8_t u8HeartRESP;           //����֪ͨ
  uint8_t u8CarCheckRESP;           //���������֪ͨ
  
} DeviceNode;

typedef struct List {
  struct List *next;
  struct DeviceNode *Device;
} List;
#pragma pack()

bool delete_device_from_list(uint16_t id);

//��list�е�����deviceд��FLASH������дFLASHʱ��Ҫ
void writeAllDeviceToFlash(void);

void resetSoundCheckDist(void);


/*
*  IDд��flash
*/
void AddDeviceToFlash(uint16_t id);

/*
*   �����ʼ��
*/
void ListInit(void);

/*
*   ����豸��������
*   �ɹ�:true
*   ʧ��:false
*/
bool  AddDeviceToList(uint16_t id, bool onlineFlg);

/*
*   ����ָ��ID�豸�ڵ��CMD��IDENTIFY
*   �豸�����ڷ���false
*   �豸æ���豸��������true
*/
bool SendCMDToList(uint16_t id, uint8_t cmd, uint32_t identify, uint32_t data);

/*
*   ���������豸����Ӧ��Ϣ
*   �ɹ�:true
*   ʧ��:false
*/
void SendCMDRespToList(uint16_t id, uint8_t cmd, uint32_t identify, uint8_t resp);

/*
*   �豸������̣�����ѭ���ڵ��ã�ѭ���ж�������ÿ���豸��״̬
*   ����Ӧ��u8CMDSTATUS״̬Ϊ:
*     CMD_STATUS_IDLE       �豸����
*     CMD_STATUS_STANDBY    �豸��ָ��ȴ�ִ��
*     CMD_STATUS_RUN        �豸ָ����������
*     CMD_STATUS_DONE       �豸ָ���������
*/
void ListTask(void);

/*
*   ��ӡ������
*/
void LookAllList(void);

DeviceNode *getDevice(uint16_t id);

/*
*   ��F405���������豸������Ϣ
*/
void SendALLDeviceNodeToF405(void);

#endif



