#include "Lora.h"
#include "stdint.h"
#include "stm32f0xx.h"
#include "crc16.h"
#include "hardware.h"
#include "user_config.h"
#include "lora_paramter.h"
#include "string.h"

extern UART_HandleTypeDef huart1;

static LoraPacket gLoraPacket;
static uint8_t uart1_dma_rbuff[SERVER_SEND_CMD_LEN] = {0};
static uint8_t uart1_dma_sbuff[SERVER_REC_RESP_LEN] = {0};

extern Device gDevice;
extern DMA_HandleTypeDef hdma_usart1_rx;

void LoraModuleInit(void)
{
  memset(&gLoraPacket, 0, sizeof(LoraPacket));
  
  gLoraPacket.dma_rbuff = &uart1_dma_rbuff[0];
  gLoraPacket.dma_rbuff_size = SERVER_SEND_CMD_LEN;
  
  gLoraPacket.dma_sbuff = &uart1_dma_sbuff[0];
  gLoraPacket.dma_sbuff_size = SERVER_REC_RESP_LEN;
}

void SetServer(uint16_t serverid, uint8_t ch)
{
  gLoraPacket.u16ServerID = serverid;
  gLoraPacket.u8ServerCH = ch;
}

static bool set_loramodule_workmode(Lora_Mode lm)
{
#define LORA_SET_MAX_TIME 	500
  
  uint16_t delay = 0;
  
  gLoraPacket.mode = lm;
  
  HAL_Delay(5);       //必须的延时，至少5ms
  
  switch(gLoraPacket.mode)
  {
  case LoraMode_Normal:
    HAL_GPIO_WritePin(GPIO_Lora_M0, GPIO_Lora_M0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_Lora_M1, GPIO_Lora_M1_Pin, GPIO_PIN_RESET);
    break;
  case LoraMode_WakeUp:
    HAL_GPIO_WritePin(GPIO_Lora_M0, GPIO_Lora_M0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_Lora_M1, GPIO_Lora_M1_Pin, GPIO_PIN_RESET);
    break;
  case LoraMode_LowPower:
    HAL_GPIO_WritePin(GPIO_Lora_M0, GPIO_Lora_M0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_Lora_M1, GPIO_Lora_M1_Pin, GPIO_PIN_SET);
    break;
  case LoraMode_Sleep:
    HAL_GPIO_WritePin(GPIO_Lora_M0, GPIO_Lora_M0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_Lora_M1, GPIO_Lora_M1_Pin, GPIO_PIN_SET);
    break;
  default:
    return false;
  }
  
  do {
    HAL_Delay(2);   //必须的do{}while(...)循环，先执行一遍延时
    delay++;
  } while((!LoraModuleIsIdle()) && (delay < LORA_SET_MAX_TIME));
  
  if(delay >= LORA_SET_MAX_TIME)
    return false;

  
  return true;
}

/*
*       Generate Lora Paramter
*	根据参数生成数据：成功返回0，失败返回1
*/
static bool loramodule_generate_paramter(uint8_t *buff, SaveType st)
{
  if (SAVE_IN_FLASH == st) {
    buff[0] = 0xC0;
  } else {
    buff[0] = 0xC2;
  }
  
  buff[1] = (uint8_t)((gLoraPacket.paramter.u16Addr & 0xff00) >> 8);
  buff[2] = (uint8_t)(gLoraPacket.paramter.u16Addr);
  
  switch(gLoraPacket.paramter.u8Parity)
  {
  case PARITY_8N1:
  case PARITY_8N1_S:
    buff[3] |= 0 << 6;
    break;
  case PARITY_8O1:
    buff[3] |= 1 << 6;
    break;
  case PARITY_8E1:
    buff[3] |= 2 << 6;
    break;
  default:
    return false;
  }
  
  switch(gLoraPacket.paramter.u8Baud)
  {
  case BAUD_1200:
    buff[3] |= 0 << 3;
    break;
  case BAUD_2400:
    buff[3] |= 1 << 3;
    break;
  case BAUD_4800:
    buff[3] |= 2 << 3;
    break;
  case BAUD_9600:
    buff[3] |= 3 << 3;
    break;
  case BAUD_19200:
    buff[3] |= 4 << 3;
    break;
  case BAUD_38400:
    buff[3] |= 5 << 3;
    break;
  case BAUD_57600:
    buff[3] |= 6 << 3;
    break;
  case BAUD_115200:
    buff[3] |= 7 << 3;
    break;
  default:
    return false;
  }
  
  switch(gLoraPacket.paramter.u8Speedinair)
  {
  case SPEED_IN_AIR_0_3K:
    buff[3] |= 0;
    break;
  case SPEED_IN_AIR_1_2K:
    buff[3] |= 1;
    break;
  case SPEED_IN_AIR_2_4K:
    buff[3] |= 2;
    break;
  case SPEED_IN_AIR_4_8K:
    buff[3] |= 3;
    break;
  case SPEED_IN_AIR_9_6K:
    buff[3] |= 4;
    break;
  case SPEED_IN_AIR_19_2K:
  case SPEED_IN_AIR_19_2K_1:
  case SPEED_IN_AIR_19_2K_2:
    buff[3] |= 5;
    break;
  default:
    return false;
  }
  
  if(gLoraPacket.paramter.u8Channel > CHAN_441MHZ)
    return false;
  
  buff[4] = gLoraPacket.paramter.u8Channel;
  
  switch(gLoraPacket.paramter.u8TranferMode)
  {
  case TRANSFER_MODE_TOUMING:
    buff[5] |= 0 << 7;
    break;
  case TRANSFER_MODE_DINGDIAN:
    buff[5] |= 1 << 7;
    break;
  default:
    return false;
  }
  
  switch(gLoraPacket.paramter.u8IOMode)
  {
  case IOMODE_PP:
    buff[5] |= 1 << 6;
    break;
  case IOMODE_OD:
    buff[5] |= 0 << 6;
    break;
  default:
    return false;
  }
  
  switch(gLoraPacket.paramter.u8WakeUpTime)
  {
  case WAKEUP_TIME_250MS:
    buff[5] |= 0 << 3;
    break;
  case WAKEUP_TIME_500MS:
    buff[5] |= 1 << 3;
    break;
  case WAKEUP_TIME_750MS:
    buff[5] |= 2 << 3;
    break;
  case WAKEUP_TIME_1000MS:
    buff[5] |= 3 << 3;
    break;
  case WAKEUP_TIME_1250MS:
    buff[5] |= 4 << 3;
    break;
  case WAKEUP_TIME_1500MS:
    buff[5] |= 5 << 3;
    break;
  case WAKEUP_TIME_1750MS:
    buff[5] |= 6 << 3;
    break;
  case WAKEUP_TIME_2000MS:
    buff[5] |= 7 << 3;
    break;
  default:
    return false;
  }
  
  if(gLoraPacket.paramter.u8FEC)
  {
    buff[5] |= 1 << 2;
  }
  else
  {
    buff[5] |= 0 << 2;
  }
  
  switch(gLoraPacket.paramter.u8SendDB)
  {
  case SEND_20DB:
    buff[5] |= 0;
    break;
  case SEND_17DB:
    buff[5] |= 1;
    break;
  case SEND_14DB:
    buff[5] |= 2;
    break;
  case SEND_10DB:
    buff[5] |= 3;
    break;
  default:
    return false;
  }
  
  return true;
}

/*
*       Set Lora Paramter
*/
static bool set_loramodule_paramter(void)
{  
#define LORA_SET_PAR_MAX_TIME 	100
  uint8_t config[6] = {0};
  
  if (!loramodule_generate_paramter(config, SAVE_IN_FLASH))
    return false;
  
  //必要的延时函数
  HAL_Delay(10);
  
  if (HAL_UART_Transmit(&huart1, config, 6, 100) == HAL_OK) {
    memset( config, 0, 6);
    if (HAL_UART_Receive(&huart1, config, 6, 1000) == HAL_OK) {
      return true;
    }
    return false;
  }
  
  return false;
}

//从接收到的数据包中分析配置参数
static void get_loramodule_paramter(uint8_t *buff)
{
  gLoraPacket.paramter.u16Addr         = (((uint16_t)buff[1] << 8) | buff[2]);
  /*
  *   7       6       5       4       3       2       1       0
  *   [ parity ]      [   ttl   baud  ]       [ speed in air  ]
  */
  gLoraPacket.paramter.u8Parity        = (buff[3] & 0xC0) >> 6;
  gLoraPacket.paramter.u8Baud          = (buff[3] & 0x38) >> 3;
  gLoraPacket.paramter.u8Speedinair    = (buff[3] & 0x07);
  
  gLoraPacket.paramter.u8Channel       = buff[4];
  /*
  *       7         6       5  4  3       2        1    0
  * [TransMode] [IOMode] [ WakeUp time]  [fec]   [ send db]
  */
  gLoraPacket.paramter.u8TranferMode   = (buff[5] & 0x80) >> 7;
  gLoraPacket.paramter.u8IOMode        = (buff[5] & 0x40) >> 6;
  gLoraPacket.paramter.u8WakeUpTime    = (buff[5] & 0x38) >> 3;
  gLoraPacket.paramter.u8FEC           = (buff[5] & 0x04) >> 2;
  gLoraPacket.paramter.u8SendDB        = (buff[5] & 0x03);
}

/*
*       Read Lora Version or Paramter
*       PS:在发送了读取版本号命令之后不能再发送命令
*/
static bool read_loramodule(void)
{
#define LORA_READ_PAR_MAX_TIME 100
  
  uint8_t cmd[3] = {0xC1, 0xC1, 0xC1};
  uint8_t buffer[6] = {0};

  if (HAL_UART_Transmit(&huart1, cmd, 3, 100) == HAL_OK) {
    if ( HAL_UART_Receive(&huart1, buffer, 6, 1000) == HAL_OK ) {
      //解析参数，保存到gLoraPacket中
      get_loramodule_paramter(buffer);
      return true;
    }
  }
  
  return false;
}

static void uart_set_9600_8n1(UART_HandleTypeDef *huart)
{
  __HAL_UART_DISABLE(huart);
  huart->Init.BaudRate = 9600;
  huart->Init.WordLength = UART_WORDLENGTH_8B;
  huart->Init.Parity = UART_PARITY_NONE;
  if (HAL_UART_Init(huart) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  HAL_Delay(10);
}

static void uart_set_baud_parity(UART_HandleTypeDef *huart, BaudType baud, ParityType parity)
{
  uint32_t baud_array[8] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
  
  __HAL_UART_DISABLE(huart);
  huart->Init.BaudRate = baud_array[baud];
  if (( PARITY_8N1 == parity) || ( PARITY_8N1_S == parity)) {
    //无校验
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.Parity = UART_PARITY_NONE;
  } else if ( PARITY_8O1 == parity ) {
    huart->Init.WordLength = UART_WORDLENGTH_9B;
    huart->Init.Parity = UART_PARITY_ODD;
  } else if ( PARITY_8E1 == parity ) {
    huart->Init.WordLength = UART_WORDLENGTH_9B;
    huart->Init.Parity = UART_PARITY_EVEN;
  }
  
  if (HAL_UART_Init(huart) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  HAL_Delay(10);
}

/*
*       AUX高电平表示空闲状态，低电平表示忙状态
*/
bool LoraModuleIsIdle(void)
{
//  return gLoraPacket.isIdle;
  if ( HAL_GPIO_ReadPin(GPIO_Lora_AUX, GPIO_Lora_AUX_Pin) == GPIO_PIN_SET )
    return true;
  else
    return false;
}

void LoraModuleDMAInit(void)
{
  HAL_UART_Receive_DMA(&huart1, gLoraPacket.dma_rbuff, gLoraPacket.dma_rbuff_size);
  //使能空闲中断，仅对接收起作用
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

/*
*   向Lora模块写入配置参数
*/
void LoraWriteParamter(LoraPar *lp)
{
  //进入配置模式,在配置模式下9600，8N1
  uart_set_9600_8n1(&huart1);
  set_loramodule_workmode(LoraMode_Sleep);
  memcpy(&gLoraPacket.paramter, lp, sizeof(LoraPar));
  while(!set_loramodule_paramter());
  
  uart_set_baud_parity(&huart1, (BaudType)gLoraPacket.paramter.u8Baud, (ParityType)gLoraPacket.paramter.u8Parity);
  set_loramodule_workmode(LoraMode_Normal);
  memset(&gLoraPacket.paramter, 0, sizeof(LoraPar));
}

/*
*   从Lora模块读取配置参数
*/
void LoraReadParamter(void)
{
  uart_set_9600_8n1(&huart1);
  set_loramodule_workmode(LoraMode_Sleep);
  HAL_Delay(100);
  while(!read_loramodule());
  uart_set_baud_parity(&huart1, (BaudType)gLoraPacket.paramter.u8Baud, (ParityType)gLoraPacket.paramter.u8Parity);
  set_loramodule_workmode(LoraMode_Normal);
}


/*
*   通过lora发送一串数据
*   Device 结构体指针，指向全局变量gDevice      
*
*/
bool LoraTransfer(uint8_t flag)
{
#define LORA_MAX_IDLE_WAIT      200
  
  uint8_t delay = 0;
  
  while((!LoraModuleIsIdle()) && (delay < LORA_MAX_IDLE_WAIT))
  {
    delay++;
    HAL_Delay(1);
  }
  
  if (delay >= LORA_MAX_IDLE_WAIT) {
    return false;
  }
  
  if ( LoraMode_Normal != gLoraPacket.mode) {
    set_loramodule_workmode(LoraMode_Normal);
  }
  
//  gLoraPacket.dma_sbuff[0] = (uint8_t)((TARGET_ID & 0xff00) >> 8);
//  gLoraPacket.dma_sbuff[1] = (uint8_t)(TARGET_ID & 0x00ff);
  gLoraPacket.dma_sbuff[0] = (uint8_t)((gLoraPacket.u16ServerID & 0xff00) >> 8);
  gLoraPacket.dma_sbuff[1] = (uint8_t)(gLoraPacket.u16ServerID & 0x00ff);
  gLoraPacket.dma_sbuff[2] = gLoraPacket.u8ServerCH;
  
  RespDataPacket *ptr = (RespDataPacket *)(gLoraPacket.dma_sbuff + 3);
  ptr->u8Head = LORA_MSG_HEAD;
  ptr->u8Len  = SERVER_REC_RESP_LEN;
  ptr->u16Id  = gLoraPacket.paramter.u16Addr;
  
  ptr->u8Resp = gDevice.u8Resp;
  if ( flag == RECMD ) {
    ptr->u8Cmd = gDevice.u8ReCmd;
    ptr->u32Identify = gDevice.u32ReIdentify;
  } else {
    ptr->u8Cmd  = gDevice.u8Cmd;
    ptr->u32Identify = gDevice.u32Identify;
  }
  ptr->u16Crc = CRC16_IBM((gLoraPacket.dma_sbuff + 3), SERVER_REC_RESP_LEN - 3);
  ptr->u8Tail = LORA_MSG_TAIL;
  
  HAL_UART_Transmit_DMA(&huart1, gLoraPacket.dma_sbuff, ptr->u8Len + 3);
  
  HAL_Delay(3);
  
  while(!LoraModuleIsIdle());
  
  set_loramodule_workmode(LoraMode_LowPower);
  
  return true;
}

bool LoraRegister(void)
{
#define LORA_MAX_IDLE_WAIT      200
#define UART_MAX_SEND_WAIT      200
  
  bool ret = false;
  uint8_t delay = 0;
  uint8_t buffer[12] = {0};
  
  while((!LoraModuleIsIdle()) && (delay < LORA_MAX_IDLE_WAIT))
  {
    delay++;
    HAL_Delay(1);
  }
  
  if (delay >= LORA_MAX_IDLE_WAIT) {
    return false;
  }
  
  if ( LoraMode_Normal != gLoraPacket.mode) {
    set_loramodule_workmode(LoraMode_Normal);
  }
  
//  gLoraPacket.dma_sbuff[0] = (uint8_t)((TARGET_ID & 0xff00) >> 8);
//  gLoraPacket.dma_sbuff[1] = (uint8_t)(TARGET_ID & 0x00ff);
  gLoraPacket.dma_sbuff[0] = (uint8_t)((gLoraPacket.u16ServerID & 0xff00) >> 8);
  gLoraPacket.dma_sbuff[1] = (uint8_t)(gLoraPacket.u16ServerID & 0x00ff);
  gLoraPacket.dma_sbuff[2] = gLoraPacket.u8ServerCH;

  RespDataPacket *ptr = (RespDataPacket *)(gLoraPacket.dma_sbuff + 3);
  ptr->u8Head = LORA_MSG_HEAD;
  ptr->u8Len  = SERVER_REC_RESP_LEN;
  ptr->u16Id  = gLoraPacket.paramter.u16Addr;
  ptr->u8Resp = 0;
  ptr->u8Cmd  = HW_DEVICE_ONLINE;
  ptr->u32Identify = 0;
  ptr->u16Crc = CRC16_IBM((uint8_t *)ptr, SERVER_REC_RESP_LEN - 3);
  ptr->u8Tail = LORA_MSG_TAIL;
  
  if ( HAL_OK == HAL_UART_Transmit(&huart1, gLoraPacket.dma_sbuff, ptr->u8Len + 3, 100)) {
    if ( HAL_OK == HAL_UART_Receive(&huart1, buffer, SERVER_SEND_CMD_LEN, 2000) ) {
    
      CmdDataPacket *cmd = (CmdDataPacket *)buffer;
      if(cmd->u8Cmd == HW_DEVICE_ONLINE)
//        return true;
        ret = true;
      else
        ret = false;
//        return false;
    }
  }
  
  HAL_Delay(3);
  
  while(!LoraModuleIsIdle());
  
  set_loramodule_workmode(LoraMode_LowPower);
  
  return ret;
}

/**
*   AUX引脚中断处理函数，判断lora是否是空闲状态
*/
//void LoraModuleIsIdleHandler(void)
//{
////  if ( HAL_GPIO_ReadPin(GPIO_Lora_AUX, GPIO_Lora_AUX_Pin) == GPIO_PIN_SET )
////    gLoraPacket.isIdle = true;
////  else
////    gLoraPacket.isIdle = false;
//}

/**
*   Lora模块接收数据处理
*/
void LoraModuleReceiveHandler(void)
{
  uint16_t crc;
  LoraPacket *lp = &gLoraPacket;
  
  CmdDataPacket *cdp = (CmdDataPacket *)lp->dma_rbuff;
  if (( cdp->u8Head == LORA_MSG_HEAD ) && (cdp->u8Tail == LORA_MSG_TAIL)) {
    crc = CRC16_IBM((uint8_t *)cdp, SERVER_SEND_CMD_LEN -3);
    if ( crc == cdp->u16Crc ) {
      if ((gDevice.u8Cmd != HW_CMD_NONE) && (gDevice.u8CmdRunning == CMD_RUN)) {
        gDevice.u8ReCmdFlag = 1;
        gDevice.u8ReCmd = cdp->u8Cmd;
        gDevice.u32ReIdentify = cdp->u32Identify;
      } else if (cdp->u8Cmd != HW_DEVICE_ONLINE ) {
        gDevice.u8Cmd = cdp->u8Cmd;
        gDevice.u32Identify = cdp->u32Identify;
      }
    }
  }
  
  HAL_UART_Receive_DMA(&huart1, lp->dma_rbuff, lp->dma_rbuff_size);
}
