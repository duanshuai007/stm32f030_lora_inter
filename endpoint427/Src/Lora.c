#include "Lora.h"
#include "stdint.h"
#include "stm32f0xx.h"
#include "crc16.h"
#include "hardware.h"
#include "user_config.h"
#include "lora_paramter.h"
#include "string.h"

extern UART_HandleTypeDef huart1;

extern Device gDevice;
extern LoraPacket gLoraPacket;
static uint8_t uart1_dma_rbuff[SERVER_SEND_CMD_LEN] = {0};

void SetServer(uint16_t serverid, uint8_t ch)
{
  gLoraPacket.u16ServerID = serverid;
  gLoraPacket.u8ServerCH = ch;
}

void LoraUartEnable(void) 
{
  UART_HandleTypeDef *huart = &huart1;

  huart->Instance = USART1;
  huart->Init.BaudRate = 57600;
  huart->Init.WordLength = UART_WORDLENGTH_9B;
  huart->Init.StopBits = UART_STOPBITS_1;
  huart->Init.Parity = UART_PARITY_ODD;
  huart->Init.Mode = UART_MODE_TX_RX;
  huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart->Init.OverSampling = UART_OVERSAMPLING_16;
  huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(huart);
  
  HAL_UART_Receive_IT(&huart1, gLoraPacket.rbuff, gLoraPacket.rbuff_size);
}

void LoraUartDisable(void)
{
  HAL_UART_Abort_IT(&huart1);
  HAL_UART_DeInit(&huart1);
//  __HAL_RCC_USART1_CLK_DISABLE();
}

/*
*       AUX高电平表示空闲状态，低电平表示忙状态
*/
bool LoraModuleIsIdle(void)
{
  if ( HAL_GPIO_ReadPin(GPIO_Lora_AUX, GPIO_Lora_AUX_Pin) == GPIO_PIN_SET )
    return true;
  else
    return false;
//  return HAL_GPIO_ReadPin(GPIO_Lora_AUX, GPIO_Lora_AUX_Pin);
}

static bool set_loramodule_workmode(Lora_Mode lm)
{
  gLoraPacket.mode = lm;
  
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

  return true;
}

void LoraModuleInit(void)
{
  memset(&gLoraPacket, 0, sizeof(LoraPacket));
  
  gLoraPacket.rbuff = &uart1_dma_rbuff[0];
  gLoraPacket.rbuff_size = SERVER_SEND_CMD_LEN;
}

static void uart_set_baud_parity(UART_HandleTypeDef *huart, BaudType baud, ParityType parity)
{
  uint32_t baud_array[8] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
  
  __HAL_UART_DISABLE(huart);
  huart->Init.BaudRate = baud_array[baud];
  
  switch (parity) {
  case PARITY_8N1:
  case PARITY_8N1_S:
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.Parity = UART_PARITY_NONE;
    break;
  case PARITY_8O1:
    huart->Init.WordLength = UART_WORDLENGTH_9B;
    huart->Init.Parity = UART_PARITY_ODD;
    break;
  case PARITY_8E1:
    huart->Init.WordLength = UART_WORDLENGTH_9B;
    huart->Init.Parity = UART_PARITY_EVEN;
    break;
  default:
    break;
  }
  
  HAL_UART_Init(huart);
  HAL_Delay(10);
}


/*
*   向Lora模块写入配置参数
*/
void LoraWriteParamter(void)
{
  //进入配置模式,在配置模式下9600，8N1
  uart_set_baud_parity(&huart1, BAUD_9600, PARITY_8N1);
  set_loramodule_workmode(LoraMode_Sleep);

  do {
    //必要的延时函数
    HAL_Delay(10);
    if (HAL_UART_Transmit(&huart1, (uint8_t *)&gLoraPacket.paramter, 6, 100) == HAL_OK) {
      memset( &gLoraPacket.paramter, 0, 6);
      if (HAL_UART_Receive(&huart1, (uint8_t *)&gLoraPacket.paramter, 6, 1000) == HAL_OK) {
        break;
      }
    }
  }while(1);
  
  uart_set_baud_parity(&huart1, \
    (BaudType)gLoraPacket.paramter.sParaOne.u8Baud, \
      (ParityType)gLoraPacket.paramter.sParaOne.u8Parity);
  
  set_loramodule_workmode(LoraMode_Normal);
  memset(&gLoraPacket.paramter, 0, sizeof(LoraPar));
}

/*
*   从Lora模块读取配置参数
*/
void LoraReadParamter(void)
{
  uint8_t cmd[3] = {0xC1, 0xC1, 0xC1};
  
  uart_set_baud_parity(&huart1, BAUD_9600, PARITY_8N1);
  set_loramodule_workmode(LoraMode_Sleep);
  do {
    HAL_Delay(100);
    if (HAL_UART_Transmit(&huart1, cmd, 3, 100) == HAL_OK) {
      if ( HAL_UART_Receive(&huart1, (uint8_t *)&gLoraPacket.paramter, 6, 1000) == HAL_OK ) {
        //解析参数，保存到gLoraPacket中
        break;
      }
    }
  }while(1);
  
  uart_set_baud_parity(&huart1, \
    (BaudType)gLoraPacket.paramter.sParaOne.u8Baud, \
      (ParityType)gLoraPacket.paramter.sParaOne.u8Parity);

  set_loramodule_workmode(LoraMode_LowPower);
}

#pragma optimize=none
static void LoraSend(uint8_t *sendbuff, uint8_t len)
{
  #define LORA_MAX_IDLE_WAIT      200
  uint8_t delay = 0;
  
//  LoraUartEnable();
  
  while((!LoraModuleIsIdle()) && (delay < LORA_MAX_IDLE_WAIT))
  {
    delay++;
    HAL_Delay(2);
  }
  if (delay >= LORA_MAX_IDLE_WAIT) {
    return;
  }
  if ( LoraMode_Normal != gLoraPacket.mode) {
    HAL_Delay(10);       //必须的延时，至少5ms
    set_loramodule_workmode(LoraMode_Normal);
  }
  HAL_Delay(10); //必要的延时，不加就会导致发送不成功
  HAL_UART_Transmit(&huart1, sendbuff, len, 200);
  HAL_Delay(10); //必要的延时，不加就会导致发送不成功
  while(!LoraModuleIsIdle());
  HAL_Delay(10);
  set_loramodule_workmode(LoraMode_LowPower);
//  HAL_Delay(10);
}

void LoraSendReCmd(void)
{
  uint8_t len = 0;
  uint8_t i = 0;
  uint8_t pos = gDevice.u8ReCmdPosInter;
  uint8_t number = gDevice.u8ReCmdNumberInter;
  uint8_t sendbuff[SERVER_REC_RESP_HEAD_LEN + SERVER_REC_RESP_LEN * RECMD_MAX_NUMBER] = {0};
  
  if (number == 0)
    return;
  
  //获取起始地址
  if (pos >= number) {
    pos = pos - number;
  } else {
    pos = RECMD_MAX_NUMBER - (number - pos);
  }
  
  sendbuff[0] = (uint8_t)(gLoraPacket.u16ServerID >> 8);
  sendbuff[1] = (uint8_t)(gLoraPacket.u16ServerID & 0x00ff);
  sendbuff[2] = gLoraPacket.u8ServerCH;
  
  for(i=0; i < number; i++)
  {
    RespDataPacket *pRDP = (RespDataPacket *)(sendbuff + SERVER_REC_RESP_HEAD_LEN + (SERVER_REC_RESP_LEN * i));
    pRDP->u8Head = LORA_MSG_HEAD;
    pRDP->u8Len  = SERVER_REC_RESP_LEN;
    pRDP->u16Id  = (((uint16_t)gLoraPacket.paramter.u8AddrH << 8) | (uint16_t)gLoraPacket.paramter.u8AddrL);
    
    pRDP->u8Resp = HW_DEVICE_BUSY;
    pRDP->u8Cmd  = gDevice.sReCmd[pos].u8Cmd;
    pRDP->u32Identify = gDevice.sReCmd[pos].u32Identify;
    pRDP->u16Crc = CRC16_IBM((sendbuff + SERVER_REC_RESP_HEAD_LEN), SERVER_REC_RESP_LEN - SERVER_REC_RESP_HEAD_LEN);
    pRDP->u8Tail = LORA_MSG_TAIL;
    pos++;
    if (pos == RECMD_MAX_NUMBER) {
      pos = 0;
    }
  }
  
  len = SERVER_REC_RESP_HEAD_LEN + (i * SERVER_REC_RESP_LEN);
  
  gDevice.u8ReCmdNumberInter -= number;
  
  LoraSend(sendbuff, len);
}

void LoraSendResp(uint8_t cmd, uint8_t resp, uint32_t identify)
{
  uint8_t sbuff[SERVER_REC_RESP_LEN + SERVER_REC_RESP_HEAD_LEN] = {0};

  sbuff[0] = (uint8_t)((gLoraPacket.u16ServerID & 0xff00) >> 8);
  sbuff[1] = (uint8_t)(gLoraPacket.u16ServerID & 0x00ff);
  sbuff[2] = gLoraPacket.u8ServerCH;
  
  RespDataPacket *pRDP = (RespDataPacket *)(sbuff + SERVER_REC_RESP_HEAD_LEN);
  pRDP->u8Head = LORA_MSG_HEAD;
  pRDP->u8Len  = SERVER_REC_RESP_LEN;
  pRDP->u16Id  = (((uint16_t)gLoraPacket.paramter.u8AddrH << 8) | (uint16_t)gLoraPacket.paramter.u8AddrL);  
  pRDP->u8Resp = resp;
  pRDP->u8Cmd  = cmd;
  pRDP->u32Identify = identify;
  pRDP->u16Crc = CRC16_IBM((sbuff + SERVER_REC_RESP_HEAD_LEN), SERVER_REC_RESP_LEN - SERVER_REC_RESP_HEAD_LEN);
  pRDP->u8Tail = LORA_MSG_TAIL;
  
  LoraSend(sbuff, SERVER_REC_RESP_LEN + SERVER_REC_RESP_HEAD_LEN);
}

/**
*   Lora模块接收数据处理
*/
void LoraModuleReceiveHandler(void)
{
  uint16_t crc;
  LoraPacket *lp = &gLoraPacket;
  
  CmdDataPacket *cdp = (CmdDataPacket *)lp->rbuff;
  
  if ((cdp->u8Head != LORA_MSG_HEAD) || (cdp->u8Tail != LORA_MSG_TAIL))
    return;
  
  if (cdp->u16Id != (((uint16_t)gLoraPacket.paramter.u8AddrH << 8) | (uint16_t)gLoraPacket.paramter.u8AddrL))
    return;
  
  crc = CRC16_IBM((uint8_t *)cdp, SERVER_SEND_CMD_LEN - SERVER_REC_RESP_HEAD_LEN);
  if ( crc != cdp->u16Crc )
    return;

  if (gDevice.u8Cmd != HW_CMD_NONE) {
    //处理重复命令的逻辑
    gDevice.sReCmd[gDevice.u8ReCmdPosInter].u8Cmd = cdp->u8Cmd;
    gDevice.sReCmd[gDevice.u8ReCmdPosInter].u32Identify = cdp->u32Identify;
    gDevice.u8ReCmdNumberInter++;
    gDevice.u8ReCmdPosInter++;
    if (gDevice.u8ReCmdPosInter >= RECMD_MAX_NUMBER)
      gDevice.u8ReCmdPosInter = 0;
  } else {
    //处理非重复命令的逻辑，包括控制命令和注册响应
    gDevice.u8Cmd = cdp->u8Cmd;
    gDevice.u32Identify = cdp->u32Identify;
    //如果是设置超声波安全距离的指令，
    //设置的超声波距离通过len发送过来
    if (cdp->u8Cmd == HW_ULTRA_SAFE_SET) {
      gDevice.u8UltraSafeDistance = cdp->u8Len;
    }
    if (cdp->u8Cmd == HW_ULTRA_CHECKTIME_SET) {
      gDevice.u8UltraCheckTimeinterval = cdp->u8Len;
    }
  }
  gDevice.bHasLoraInter = true;
}
