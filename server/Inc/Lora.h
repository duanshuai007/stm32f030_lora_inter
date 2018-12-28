#ifndef _LORA_H_
#define _LORA_H_

#include "stdint.h"
#include "stm32f1xx.h"
#include "lora_datapool.h"
#include "user_config.h"

//Lora通信相关宏
#define LORA_MSG_HEAD 0xA5
#define LORA_MSG_TAIL 0x5A
#define POOL_CLEAN    0x00

//LORA Receive所占用的引脚
//PA8 M0
#define GPIO_Lora_M0_R      GPIOA
#define GPIO_Lora_M0_R_Pin	GPIO_PIN_8
//PA12 M1
#define GPIO_Lora_M1_R      GPIOA
#define GPIO_Lora_M1_R_Pin	GPIO_PIN_12
//PA11 AUX
#define GPIO_Lora_AUX_R     GPIOA
#define GPIO_Lora_AUX_R_Pin	GPIO_PIN_11

//LORA Send所占用的引脚
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
*     lora模块参数结构体
*     设置lora模块参数的命令一共有6字节
*     byte0...byte1...byte2...byte3...byte4...byte5
*     [0xC0]  [    addr   ]  [    ]  [chan]  [   ]   
*/
typedef struct {
  uint16_t u16Addr;   //Byte1-Byte2 对应模块地址信息     
  
  //Byte3 是设置串口和空中速率
  //将Byte3的内容拆分出来是以下三个内容
  uint8_t u8Parity;     //位于Byte3的6-7bit
  uint8_t u8Baud;       //位于Byte3的3-5bit        
  uint8_t u8Speedinair; //位于Byte3的0-2bit
  
  //Byte4 设置模块的信道
  uint8_t u8Channel;
  
  //Byte5 设置模块唤醒时间工作模式等
  uint8_t u8TranferMode;      //位于Byte5的7bit 
  uint8_t u8IOMode;           //位于Byte5的6bit
  uint8_t u8WakeUpTime;       //位于Byte5的3-5bit
  uint8_t u8FEC;              //位于Byte5的2bit
  uint8_t u8SendDB;           //位于Byte5的0-1bit
} LoraPar;

//endpoint发送给server的响应信息结构体
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

//从F405接收来的服务器cmd结构体
typedef struct {
  uint8_t   u8Head;
  uint8_t   u8Len;
  uint16_t  u16Id;
  uint8_t   u8Cmd;
  uint32_t  u32Identify;
  uint16_t  u16Crc;
  uint8_t   u8Tail;
} CmdDataPacket;

//gpio 结构体
typedef struct {
  GPIO_TypeDef  *GPIOX;   //端口
  uint16_t      Pin;      //引脚  
} LoraPin;

//lora引脚配置结构体
typedef struct {
  LoraPin AUX;            //aux引脚
  LoraPin M0;             //m0功能引脚
  LoraPin M1;             //m1功能引脚
} LoraGPIO;

//串口结构体
typedef struct {
  volatile uint16_t   pos;    //buffer current postion
  UART_HandleTypeDef  *uart;    //串口
  DMA_HandleTypeDef   *uart_tx_hdma;  //串口发送dma句柄
  DMA_HandleTypeDef   *uart_rx_hdma;  //串口接收dma句柄

  uint16_t dma_rbuff_size;  //dma接收缓冲区的大小
  uint16_t dma_sbuff_size;  //dma发送缓冲区的大小
  
  uint8_t *dma_rbuff;         //指向dma接收缓冲区
  uint8_t *dma_sbuff;         //指向dma发送缓冲区
  
  DataPool *txDataPool;   //发送数据池
  DataPool *rxDataPool;   //接收数据池
} UartModule;

//保存本机lora设备相关信息
typedef struct {
//  volatile bool  isIdle; //IDLE FLAG
  Lora_Mode       mode;   //work mode
  LoraPar         paramter;//lora参数结构体
  LoraGPIO        gpio;     //lora gpio结构体
  UartModule      muart;    //lora 串口结构体
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
*   中断函数，在定时器4中断中调用
*   Lora模块发送间隔定时器
*   因为lora模块是工作在唤醒模式，不同的设备id之间需要保留空闲时间才能够
*   保证每一帧数据都添加唤醒码
*/
void LoraSendTimerHandler(void);

/*
*   中断函数，在GPIO中断中调用。
*   判断lora模块(发送模块PA11，接收模块PA4)的引脚电平来设置lora的空闲状态
*/
//void SetLoraModuleIdleFlagHandler(uint16_t pin);

/*
*   系统初始化后调用
*   设置lora模块所用到的引脚
*/
void LoraModuleGPIOInit(UART_HandleTypeDef *huart);

/*
*   在配置好模块的参数后被调用
*   设置lora模块的dma，dma接收发送缓冲区，datapool等参数
*   并使能DMA接收，设置空闲中断
*/
void LoraModuleDMAInit(UART_HandleTypeDef *huart);

/*
*   设置Lora模块参数
*   串口数据使用阻塞方式发送接收，波特率96008n1
*/
void SetLoraParamter(UART_HandleTypeDef *huart, LoraPar *lp);

/*
*   读取Lora模块参数
*   串口数据使用阻塞方式发送接收，波特率96008n1
*/
void ReadLoraParamter(UART_HandleTypeDef *huart);

/*
*   向目标设备发送控制命令       
*   id:目标设备的地址
*   cmd:  发送给目标设备的命令
*   identify: 发送给目标设备的命令的唯一识别码
*/
bool LoraCtrlEndPoint(uint16_t id, uint8_t cmd, uint32_t identify);

/*
*   中断函数：当被调用时会将对应的串口DMA缓冲区中的
*             数据复制到对应串口自己的数据池中。
*   只有uart1(lora recv)和uart2(f405 recv)的空闲中断中会调用。
*/
bool CopyDataFromDMAHandler(UART_HandleTypeDef *huart);

/*
*   中断函数:用于DMA数据移动和重新使能dma接收功能
*   在串口接收完成中断中调用
*/
void LoraModuleReceiveHandler(UART_HandleTypeDef *huart); 

/*
*   Lora模块的数据处理
*   在Main的while循环中调用
*/
void LoraModuleTask(void);

#endif



