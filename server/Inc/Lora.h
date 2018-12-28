#ifndef _LORA_H_
#define _LORA_H_

#include "stdint.h"
#include "stm32f1xx.h"
#include "lora_datapool.h"
#include "user_config.h"

//Loraͨ����غ�
#define LORA_MSG_HEAD 0xA5
#define LORA_MSG_TAIL 0x5A
#define POOL_CLEAN    0x00

//LORA Receive��ռ�õ�����
//PA8 M0
#define GPIO_Lora_M0_R      GPIOA
#define GPIO_Lora_M0_R_Pin	GPIO_PIN_8
//PA12 M1
#define GPIO_Lora_M1_R      GPIOA
#define GPIO_Lora_M1_R_Pin	GPIO_PIN_12
//PA11 AUX
#define GPIO_Lora_AUX_R     GPIOA
#define GPIO_Lora_AUX_R_Pin	GPIO_PIN_11

//LORA Send��ռ�õ�����
//PB13 M0
#define GPIO_Lora_M0_S      GPIOB
#define GPIO_Lora_M0_S_Pin  GPIO_PIN_13
//PB12 M1
#define GPIO_Lora_M1_S      GPIOB
#define GPIO_Lora_M1_S_Pin  GPIO_PIN_12
//PA4 AUX
#define GPIO_Lora_AUX_S     GPIOA
#define GPIO_Lora_AUX_S_Pin GPIO_PIN_4

//PA7 led
#define GPIO_LED            GPIOA
#define GPIO_LED_Pin        GPIO_PIN_7

typedef enum { 
  LoraMode_Normal,
  LoraMode_WakeUp,
  LoraMode_LowPower,
  LoraMode_Sleep,
}Lora_Mode;

#pragma pack(1)

/*
*     loraģ������ṹ��
*     ����loraģ�����������һ����6�ֽ�
*     byte0...byte1...byte2...byte3...byte4...byte5
*     [0xC0]  [    addr   ]  [    ]  [chan]  [   ]   
*/
typedef struct {
  uint16_t u16Addr;   //Byte1-Byte2 ��Ӧģ���ַ��Ϣ     
  
  //Byte3 �����ô��ںͿ�������
  //��Byte3�����ݲ�ֳ�����������������
  uint8_t u8Parity;     //λ��Byte3��6-7bit
  uint8_t u8Baud;       //λ��Byte3��3-5bit        
  uint8_t u8Speedinair; //λ��Byte3��0-2bit
  
  //Byte4 ����ģ����ŵ�
  uint8_t u8Channel;
  
  //Byte5 ����ģ�黽��ʱ�乤��ģʽ��
  uint8_t u8TranferMode;      //λ��Byte5��7bit 
  uint8_t u8IOMode;           //λ��Byte5��6bit
  uint8_t u8WakeUpTime;       //λ��Byte5��3-5bit
  uint8_t u8FEC;              //λ��Byte5��2bit
  uint8_t u8SendDB;           //λ��Byte5��0-1bit
} LoraPar;

//endpoint���͸�server����Ӧ��Ϣ�ṹ��
typedef struct {
  uint8_t   u8Head;
  uint8_t   u8Len;
  uint16_t  u16Id;
  uint8_t   u8Cmd;
  uint8_t   u8Resp;
  uint32_t  u32Identify;
  uint16_t  u16Crc;
  uint8_t   u8Tail;
} RespDataPacket;

//��F405�������ķ�����cmd�ṹ��
typedef struct {
  uint8_t   u8Head;
  uint8_t   u8Len;
  uint16_t  u16Id;
  uint8_t   u8Cmd;
  uint32_t  u32Identify;
  uint16_t  u16Crc;
  uint8_t   u8Tail;
} CmdDataPacket;

//gpio �ṹ��
typedef struct {
  GPIO_TypeDef  *GPIOX;   //�˿�
  uint16_t      Pin;      //����  
} LoraPin;

//lora�������ýṹ��
typedef struct {
  LoraPin AUX;            //aux����
  LoraPin M0;             //m0��������
  LoraPin M1;             //m1��������
} LoraGPIO;

//���ڽṹ��
typedef struct {
  volatile uint16_t   pos;    //buffer current postion
  UART_HandleTypeDef  *uart;    //����
  DMA_HandleTypeDef   *uart_tx_hdma;  //���ڷ���dma���
  DMA_HandleTypeDef   *uart_rx_hdma;  //���ڽ���dma���

  uint16_t dma_rbuff_size;  //dma���ջ������Ĵ�С
  uint16_t dma_sbuff_size;  //dma���ͻ������Ĵ�С
  
  uint8_t *dma_rbuff;         //ָ��dma���ջ�����
  uint8_t *dma_sbuff;         //ָ��dma���ͻ�����
  
  DataPool *txDataPool;   //�������ݳ�
  DataPool *rxDataPool;   //�������ݳ�
} UartModule;

//���汾��lora�豸�����Ϣ
typedef struct {
//  volatile bool  isIdle; //IDLE FLAG
  Lora_Mode       mode;   //work mode
  LoraPar         paramter;//lora�����ṹ��
  LoraGPIO        gpio;     //lora gpio�ṹ��
  UartModule      muart;    //lora ���ڽṹ��
} LoraModule;

//modify by liyongsheng begin
typedef union
{
  uint32_t identity;
  uint16_t endpointId;  
}U_Resp_Data;

typedef struct 
{
  uint8_t resp_code;
  U_Resp_Data resp_data;
  uint8_t crc;
}T_Resp_Info;

typedef struct 
{
  uint32_t identity;
  uint16_t endpointId;
  uint8_t action;
  uint8_t crc;
}T_Control_Cmd;
//modify by liyongsheng end

#pragma pack()

/*
*   �жϺ������ڶ�ʱ��4�ж��е���
*   Loraģ�鷢�ͼ����ʱ��
*   ��Ϊloraģ���ǹ����ڻ���ģʽ����ͬ���豸id֮����Ҫ��������ʱ����ܹ�
*   ��֤ÿһ֡���ݶ���ӻ�����
*/
void LoraSendTimerHandler(void);

/*
*   �жϺ�������GPIO�ж��е��á�
*   �ж�loraģ��(����ģ��PA11������ģ��PA4)�����ŵ�ƽ������lora�Ŀ���״̬
*/
//void SetLoraModuleIdleFlagHandler(uint16_t pin);

/*
*   ϵͳ��ʼ�������
*   ����loraģ�����õ�������
*/
void LoraModuleGPIOInit(UART_HandleTypeDef *huart);

/*
*   �����ú�ģ��Ĳ����󱻵���
*   ����loraģ���dma��dma���շ��ͻ�������datapool�Ȳ���
*   ��ʹ��DMA���գ����ÿ����ж�
*/
void LoraModuleDMAInit(UART_HandleTypeDef *huart);

/*
*   ����Loraģ�����
*   ��������ʹ��������ʽ���ͽ��գ�������96008n1
*/
void SetLoraParamter(UART_HandleTypeDef *huart, LoraPar *lp);

/*
*   ��ȡLoraģ�����
*   ��������ʹ��������ʽ���ͽ��գ�������96008n1
*/
void ReadLoraParamter(UART_HandleTypeDef *huart);

/*
*   ��Ŀ���豸���Ϳ�������       
*   id:Ŀ���豸�ĵ�ַ
*   cmd:  ���͸�Ŀ���豸������
*   identify: ���͸�Ŀ���豸�������Ψһʶ����
*/
bool LoraCtrlEndPoint(uint16_t id, uint8_t cmd, uint32_t identify);

/*
*   �жϺ�������������ʱ�Ὣ��Ӧ�Ĵ���DMA�������е�
*             ���ݸ��Ƶ���Ӧ�����Լ������ݳ��С�
*   ֻ��uart1(lora recv)��uart2(f405 recv)�Ŀ����ж��л���á�
*/
bool CopyDataFromDMAHandler(UART_HandleTypeDef *huart);

/*
*   �жϺ���:����DMA�����ƶ�������ʹ��dma���չ���
*   �ڴ��ڽ�������ж��е���
*/
void LoraModuleReceiveHandler(UART_HandleTypeDef *huart); 

/*
*   Loraģ������ݴ���
*   ��Main��whileѭ���е���
*/
void LoraModuleTask(void);

#endif



