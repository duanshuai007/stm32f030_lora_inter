#ifndef __MY_LIST_H__
#define __MY_LIST_H__

#include "stdint.h"
#include "user_config.h"

typedef struct DeviceNode {
  uint32_t u32Identify;     //ָ��identify
  uint32_t u32Time;         //��ʱ��ʱ��1:����ʱ�䣬2:ָ��ִ�г�ʱ
  
  uint16_t u16ID;           //�豸ID
  uint8_t u8CHAN;           //�ŵ�
  uint8_t u8CMD;
  
  uint8_t u8CMDSTATUS;
  uint8_t u8RESP;
  uint8_t u8CMDRetry;       //cmdû��Ӱ�����ԵĴ���
} DeviceNode;

typedef struct List {
  struct List *next;
  struct DeviceNode *Device;
} List;

/*
*   �����ʼ��
*/
List *list_init(void);

/*
*   ����豸��������
*/
void add_device_to_list(List *list, uint16_t id);

/*
*   ��������ɾ��һ���豸
*/
//uint8_t delete_device_from_list(List *list, uint16_t id);

/*
*   ���������ж�Ӧid���豸����
*/
void set_device_of_list_cmd(List *list, uint16_t id, uint8_t cmd, uint32_t identify);

/*
*   ��ӡ������
*/
void LookAllList(List *lcn);

/*
*   ����������
*/
void ListProcess(List *LCmdNode);

/*
*   ���������豸����Ӧ��Ϣ
*/
void SendCMDRespToList(List *lcn, uint16_t id, uint8_t cmd, uint32_t time, uint8_t resp);


#endif



