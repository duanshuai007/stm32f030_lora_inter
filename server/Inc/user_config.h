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

//#define PRINT(...) printf(__VA_ARGS__)
#define PRINT(...)

#define DEFAULT_CHANNEL 0x1e

#define LORA_MSG_HEAD 0xA5
#define LORA_MSG_TAIL 0x5A
#define POOL_CLEAN    0x00

  //�������״̬��Ϣ
#define MOTOR_RUNING          99   //���������ʼ������
#define CMD_STATUS_IDLE       100   //����״̬
#define CMD_STATUS_RUN        101   //��������
#define CMD_STATUS_DONE       102   //�������
#define CMD_STATUS_STOP       103   //��ʱֹͣ����
#define CMD_STATUS_TIMEOUT    104   //������Ӧʱ��û��Ӧ��
#define CMD_STATUS_HEART      105   //���յ��豸��������Ӧ
#define CMD_STATUS_STANDBY    106   //����ָ��ȴ�����
//modify by liyongsheng begin

typedef enum 
{
  CMD_MOTOR_UP = 1,
  CMD_MOTOR_DOWN = 2,
  CMD_MOTOR_STATUS_GET = 3,
  CMD_BEEP_ON = 4,
  CMD_BEEP_OFF = 5,
  CMD_BEEP_STATUS_GET = 6,
  CMD_ADC_GET = 7,  
  CMD_MOTOR_ABNORMAL = 8,
  CMD_DEVICE_REGISTER = 9,
  CMD_DEVICE_HEART = 10,

}CMD_TYPE;

typedef enum
{
  //Normal
  NORMAL_SUCCESS = 101,
  NORMAL_DOWN = 102,
  NORMAL_FORWARD = 103,    
  NORMAL_UP = 104,
  NORMAL_BACK = 105,
  NORMAL_BUSY = 106,
  NORMAL_BEEP_OPEN_FAILED = 107,
  NORMAL_BEEP_CLOSE_FAILED = 108,
  NORMAL_BEEP_STATUS_OPEN = 109,
  NORMAL_BEEP_STATUS_CLOSED = 110,

  NODE_ONLINE = 120,
  NODE_OFFLINE = 121,
  
  //interupt
  INTERUPT_DOWN = 201,
  INTERUPT_FORWARD = 202,    
  INTERUPT_UP = 203,
  INTERUPT_BACK = 204,  
  
  ERROR_CMD_TIMEOUT = 210,
  
}RESP_CODE;

//modify by liyongsheng end

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

//f405
#define F405_SEND_CMD_LEN       10
#define F405_RECV_CMD_LEN       10
//uart2 send buffer size
#define UART2_TX_DMA_LEN        (F405_SEND_CMD_LEN)
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

//LORA Receive��ռ�õ�����
//PA8 M0
#define GPIO_Lora_M0_R 	        GPIOA
#define GPIO_Lora_M0_R_Pin	GPIO_PIN_8
//PA12 M1
#define GPIO_Lora_M1_R	        GPIOA
#define GPIO_Lora_M1_R_Pin	GPIO_PIN_12
//PA11 AUX
#define GPIO_Lora_AUX_R	        GPIOA
#define GPIO_Lora_AUX_R_Pin	GPIO_PIN_11

//LORA Send��ռ�õ�����
//PB13 M0
#define GPIO_Lora_M0_S          GPIOB
#define GPIO_Lora_M0_S_Pin      GPIO_PIN_13
//PB12 M1
#define GPIO_Lora_M1_S          GPIOB
#define GPIO_Lora_M1_S_Pin      GPIO_PIN_12
//PA4 AUX
#define GPIO_Lora_AUX_S         GPIOA
#define GPIO_Lora_AUX_S_Pin     GPIO_PIN_4

//PA7 led
#define GPIO_LED                GPIOA
#define GPIO_LED_Pin            GPIO_PIN_7

#endif
