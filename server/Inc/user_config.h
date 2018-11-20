#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

#include "stdint.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal_def.h"
#include "lora_paramter.h"

typedef enum {
  false,
  true,
}bool;

#if 0
#define PRINT(...) printf(__VA_ARGS__)
#else
#define PRINT(...)
#endif

//Ĭ�ϵķ������ŵ�
#define DEFAULT_CHANNEL 0x1e

//Loraģ����endpointͨ�ŵ���Сʱ��������λ10ms
//���ڻ���ģʽ��Loraģ��ÿ����Ϣ֮����뱣��һ��
//��ʱ���������������ӻ����룬����������ͨ��
#define LORA_SEND_MIN_TIMEINTERVAL  60

//����ϵͳ������������ʱ�䣬����Ӧ�豸����HEART_TIMESTAMP ��󣬷���һ��������
#define HEART_TIMESTAMP         30
//���û����Ӧ�����Դ���
#define CMD_MAX_RETRY_TIMES     3
//����ÿ��֮��ļ��
#define CMD_RETRY_TIMEINTERVAL  4 
//CMDû����ӦCMD_TIMEOUT������Ϊ��ʱ
#define CMD_TIMEOUT             5

//ÿ��60�뱣��һ���豸��Ϣ��flash��
#define FLASH_SAVE_TIMEINTERVAL   60

//����DMA���ݿռ�
//���������͵��ڵ�������
#define SERVER_CMD_LEN          12
#define SERVER_CMD_HEAD_SIZE    3
//�������ӽڵ���յ���resp����
#define SERVER_RESP_LEN         13

//lora receive module
#define UART1_RX_DMA_LEN        64
#define UART1_RX_DATAPOOL_SIZE  128
//lora send module
#define UART3_TX_DMA_LEN        (SERVER_CMD_LEN + SERVER_CMD_HEAD_SIZE) //15
#define UART3_TX_DATAPOOL_SIZE  128

//f405
#define F405_SEND_CMD_LEN       6
//uart2 send buffer size
#define UART2_TX_DMA_LEN        (F405_SEND_CMD_LEN)
#define UART2_TX_DATAPOOL_SIZE  128
//uart2 receive buffer size
#define UART2_RX_DMA_LEN        64
#define UART2_RX_DATAPOOL_SIZE  128

//Ŀ���豸����Ϣ
#pragma pack(1)
typedef struct {
  uint8_t   u8Cmd;        //���͵�����
  uint16_t  u16TargetID;  //Ŀ��ID
  uint32_t  u32Identify;
} Device;
#pragma pack()

#endif
