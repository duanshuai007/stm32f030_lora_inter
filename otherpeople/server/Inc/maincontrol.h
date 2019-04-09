#ifndef _MAINCONTROL_H_
#define _MAINCONTROL_H_

#include "stdint.h"
#include "user_config.h"
#include "lora.h"
#include "list.h"

//�豸����ʶ���cmd
typedef enum 
{
  CMD_NONE              = 0,
  CMD_MOTOR_UP          = 1,
  CMD_MOTOR_DOWN        = 2,
  CMD_MOTOR_STATUS_GET  = 3,
  CMD_BEEP_ON           = 4,
  CMD_BEEP_OFF          = 5,
  CMD_BEEP_STATUS_GET   = 6,
  CMD_ADC_GET           = 7, 
  
  HW_ULTRA_GET          = 8,
  CMD_MODIFY_DISTANCE   = 9,
  CMD_GET_DISTANCE      = 10,
  CMD_MODIFY_SCANCYCLE  = 11,
  CMD_GET_SCANCYCLE     = 12,
  CMD_SYSTEM_RESET      = 13, 
  
  CMD_GET_ALL_NODE      = 31,
  CMD_UPDATE            = 32,     
  CMD_ADD_ENDPOINT      = 33,
  CMD_DEL_ENDPOINT      = 34,
  CMD_MODIFY_CHANNEL    = 35,
  CMD_MODIFY_SERVER_DISTANCE    = 36,
  CMD_GET_SERVER_DISTANCE    = 37,
  

}CMD_TYPE;

//������״̬
typedef enum 
{
  EP_STATUS_NONE             = 0,
  EP_STATUS_DOWN,
  EP_STATUS_FORWARD,
  EP_STATUS_UP,  
  EP_STATUS_BACK

}EP_STATUS;

//������״̬
typedef enum 
{
  STATUS_NO_HAVE_CAR          = 0,
  STATUS_HAVE_CAR             = 1,
  STATUS_TROUBLE              = 0xe,
  STATUS_VALID                = 0xf
}STATUS_ULTRA;

//server�ڲ���endpoint֮���ͨ�������405 cmd��ռ����Դ
#define DEVICE_REGISTER     100
#define DEVICE_HEART        101
#define DEVICE_ABNORMAL     102

typedef enum
{
  //Normal
  NORMAL_SUCCESS            = 101,
  NORMAL_DOWN               = 102,
  NORMAL_FORWARD            = 103,
  NORMAL_UP                 = 104,
  NORMAL_BACK               = 105,
  NORMAL_BUSY               = 106,
  NORMAL_BEEP_OPEN_FAILED   = 107,
  NORMAL_BEEP_CLOSE_FAILED  = 108,
  NORMAL_BEEP_STATUS_OPEN   = 109,
  NORMAL_BEEP_STATUS_CLOSED = 110,
  NORMAL_MOTOR_RUNNING      = 111,
  NORMAL_HAVE_CAR           = 112,
  NORMAL_UNKNOWN_ERROR      = 113,
  NORMAL_CAR_CHECK_ERROR    = 114,

  NODE_ONLINE               = 120,
  NODE_OFFLINE              = 121,
  NODE_NOTEXIST             = 122,
  NODE_NO_SUPPORT_CMD       = 123,

  //get firmware
  GET_RUN_BIN = 131,  
  GET_RECOVERY_BIN = 132,  
  GET_RUN_BIN_FIN = 133,  
  GET_RECOVERY_BIN_FIN = 134,  
  ENDPOINT_ADD_SUCCESS = 135,  
  ENDPOINT_DEL_SUCCESS = 136,  
  CHANNEL_MODIFY_SUCCESS = 137,  
  SOUND_DIST_SET_SUCCESS = 138, 
  SOUND_DIST_SET_FAILED = 139, 
  SOUND_DIST_GET_SUCCESS = 140, 
  SOUND_SCANCYCLE_SET_SUCCESS = 141, 
  SOUND_SCANCYCLE_SET_FAILED = 142, 
  SOUND_SCANCYCLE_GET_SUCCESS = 143,
    
    
  //֪ͨ�������Ƿ��г�
  UPLOAD_HAVE_CAR = 150,
  UPLOAD_NO_HAVE_CAR = 151,
  UPLOAD_SOUNDWAVE_TROUBLE = 152,  
  
  //interupt
  INTERUPT_DOWN             = 201,
  INTERUPT_FORWARD          = 202,    
  INTERUPT_UP               = 203,
  INTERUPT_BACK             = 204,
}RESP_CODE;

/*
*   ��ʼ����F405ͨ�ŵĴ�����ؽṹ��
*/
void UARTF405_Init(UART_HandleTypeDef *uart);

void UartSendOnlineMSGToF405(DeviceNode *device);
void UartSendAutoMSGToF405(uint8_t status, uint16_t id, uint32_t identify);
void UartSendVerMSGToF405(uint8_t resp_code, uint8_t *version);


/*
*   ��F405����ָ����Ӧ��Ϣ���ɹ�����true��ʧ�ܷ���false
*/
bool UartSendRespToF405(DeviceNode *devn);

/*
*   ��ȡuart module�ṹ��
*/
UartModule *GetF405UartModule(UART_HandleTypeDef *huart);

/*
*   �жϺ���:F405���պ���
*   �ڴ��ڽ�������ж��б�����
*/
void F405ReveiveHandler(UART_HandleTypeDef *huart);

/*
*   F405������߳�
*   F405ֻ�ܿ����ɱ������ͳ�ȥ���豸ID����������ID�����в���
*/
void F405Task(void);

/*
*   ֹͣ��F405�����µ�����
*   ��Ӱ��ȴ������е�����
*/
void closeF405Communication(void);

#endif



