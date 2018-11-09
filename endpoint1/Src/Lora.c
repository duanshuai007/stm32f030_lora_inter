#include "Lora.h"
#include "stdint.h"
#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_def.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_hal_uart.h"
#include "crc16.h"
#include "crc_stm32f0.h"
#include "hardware.h"
#include "user_config.h"
#include "lora_paramter.h"
#include "lora_datapool.h"
#include "string.h"

extern UART_HandleTypeDef huart1;
extern LoraPacket       gLoraPacket;
extern Device           gDevice;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DataPool *gUartDataPool;

/*
*       AUX高电平表示空闲状态，低电平表示忙状态
*/
static uint8_t Lora_is_idle(LoraPacket *lp)
{
//  return lp->isIdle;
  return gLoraPacket.isIdle;
}

bool Lora_Module_Set_Mode(LoraPacket *lp, Lora_Mode lm)
{
#define LORA_SET_MAX_TIME 	500
  
  uint16_t delay = 0;
  
  lp->mode = lm;
  
  HAL_Delay(5);       //必须的延时，至少5ms
  
  switch(lp->mode)
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
  } while((!Lora_is_idle(lp)) && (delay < LORA_SET_MAX_TIME));
  
  if(delay >= LORA_SET_MAX_TIME)
    return false;
  else
    return true;
}

void LoraModuleDMAInit(LoraPacket *lp)
{
  HAL_UART_Receive_DMA(&huart1, lp->dma_rbuff, MAX_DMA_LEN);
  //使能空闲中断，仅对接收起作用
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

/*
*       Generate Lora Paramter
*	根据参数生成数据：成功返回0，失败返回1
*/
static uint8_t Lora_GeneratePar(LoraPar *lp, uint8_t *buff, SaveType st)
{
  
  if (SAVE_IN_FLASH == st) {
    buff[0] = 0xC0;
  } else {
    buff[0] = 0xC2;
  }
  
  buff[1] = (uint8_t)((lp->u16Addr & 0xff00) >> 8);
  buff[2] = (uint8_t)(lp->u16Addr);
  
  switch(lp->u8Parity)
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
    return 1;
  }
  
  switch(lp->u8Baud)
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
    return 1;
  }
  
  switch(lp->u8Speedinair)
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
    return 1;
  }
  
  if(lp->u8Channel > CHAN_441MHZ)
    return 1;
  
  buff[4] = lp->u8Channel;
  
  switch(lp->u8TranferMode)
  {
  case TRANSFER_MODE_TOUMING:
    buff[5] |= 0 << 7;
    break;
  case TRANSFER_MODE_DINGDIAN:
    buff[5] |= 1 << 7;
    break;
  default:
    return 1;
  }
  
  switch(lp->u8IOMode)
  {
  case IOMODE_PP:
    buff[5] |= 1 << 6;
    break;
  case IOMODE_OD:
    buff[5] |= 0 << 6;
    break;
  default:
    return 1;
  }
  
  switch(lp->u8WakeUpTime)
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
    return 1;
  }
  
  if(lp->u8FEC)
  {
    buff[5] |= 1 << 2;
  }
  else
  {
    buff[5] |= 0 << 2;
  }
  
  switch(lp->u8SendDB)
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
    return 1;
  }
  
  return 0;
}

/*
*       Set Lora Paramter
*/
static uint8_t Lora_SetPar(LoraPacket *lp)
{  
#define LORA_SET_PAR_MAX_TIME 	100
  uint8_t config[6] = {0};
  
  if(lp == NULL || &lp->paramter == NULL)
    return 0;
  
  if (Lora_GeneratePar(&lp->paramter, config, SAVE_IN_FLASH))
    return 0;
  
  //必要的延时函数
  HAL_Delay(10);
  
  if (HAL_UART_Transmit(&huart1, config, 6, 100) == HAL_OK) {
    memset( config, 0, 6);
    if (HAL_UART_Receive(&huart1, config, 6, 1000) == HAL_OK) { 
      //printf("rec:%02x-%02x-%02x-%02x-%02x-%02x\r\n",
       //      config[0], config[1],config[2],
         //    config[3],config[4],config[5]);
      return 1;
    }
    return 0;
  }
  
  return 0;
}

//从接收到的数据包中分析配置参数
static void Lora_GetPar(LoraPar *lp, uint8_t *buff)
{
  lp->u16Addr         = (((uint16_t)buff[1] << 8) | buff[2]);
  /*
  *   7       6       5       4       3       2       1       0
  *   [ parity ]      [   ttl   baud  ]       [ speed in air  ]
  */
  lp->u8Parity        = (buff[3] & 0xC0) >> 6;
  lp->u8Baud          = (buff[3] & 0x38) >> 3;
  lp->u8Speedinair    = (buff[3] & 0x07);
  
  lp->u8Channel       = buff[4];
  /*
  *       7         6       5  4  3       2        1    0
  * [TransMode] [IOMode] [ WakeUp time]  [fec]   [ send db]
  */
  lp->u8TranferMode   = (buff[5] & 0x80) >> 7;
  lp->u8IOMode        = (buff[5] & 0x40) >> 6;
  lp->u8WakeUpTime    = (buff[5] & 0x38) >> 3;
  lp->u8FEC           = (buff[5] & 0x04) >> 2;
  lp->u8SendDB        = (buff[5] & 0x03);
}

/*
*       Read Lora Version or Paramter
*       PS:在发送了读取版本号命令之后不能再发送命令
*/
uint8_t Lora_Read(uint8_t no, LoraPacket *lp)
{
#define LORA_READ_PAR_MAX_TIME 100
  
  uint8_t cmd[3] = {0};
  uint8_t buffer[6] = {0};
  
  if (LoraReadPar == no) {
    cmd[0] = 0xC1;
    cmd[1] = 0xC1;
    cmd[2] = 0xC1;
  } else if (LoraReadVer == no) {
    cmd[0] = 0xC3;
    cmd[1] = 0xC3;
    cmd[2] = 0xC3;
  } else {
    return 0;
  }
  
  if (HAL_UART_Transmit(&huart1, cmd, 3, 100) == HAL_OK) {
    if ( HAL_UART_Receive(&huart1, buffer, 6, 1000) == HAL_OK ) {
      //printf("read:%02x-%02x-%02x-%02x-%02x-%02x\r\n",
      //       buffer[0],buffer[1],buffer[2],
      //       buffer[3],buffer[4],buffer[5]);
      Lora_GetPar(&lp->paramter, buffer);
      return 1;
    }
    return 0;
  }
  
  return 0;
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

void LoraSetParamter(LoraPacket *lm, 
                     uint16_t addr, 
                     BaudType baud, 
                     CHANType channel, 
                     ParityType parity,
                     SpeedInAirType speedinair, 
                     TransferModeType transmode, 
                     WakeUpTimeType wutime)
{
  
  uart_set_9600_8n1(&huart1);
  
  //进入配置模式,在配置模式下，不能带校验
  Lora_Module_Set_Mode(lm, LoraMode_Sleep);
  
  //写入配置参数
  lm->paramter.u16Addr      = addr;
  lm->paramter.u8Baud       = baud;
  lm->paramter.u8Channel    = channel;
  lm->paramter.u8FEC        = FEC_ENABLE;
  lm->paramter.u8Parity     = parity;
  lm->paramter.u8Speedinair = speedinair;
  lm->paramter.u8TranferMode = transmode; //TRANSFER_MODE_TOUMING;
  lm->paramter.u8IOMode     = IOMODE_PP;
  lm->paramter.u8WakeUpTime = wutime;
  lm->paramter.u8SendDB     = SEND_20DB;
  while(!Lora_SetPar( lm ));
  
  uart_set_baud_parity(&huart1, baud, parity);
  Lora_Module_Set_Mode(lm, LoraMode_Normal);
  memset(&lm->paramter, 0, sizeof(LoraPar));
}

void LoraReadParamter(LoraPacket *lp)
{
  uart_set_9600_8n1(&huart1);
  Lora_Module_Set_Mode(lp, LoraMode_Sleep);
  HAL_Delay(100);
  while(!Lora_Read(LoraReadPar, lp));
  uart_set_baud_parity(&huart1, (BaudType)lp->paramter.u8Baud, (ParityType)lp->paramter.u8Parity);
  Lora_Module_Set_Mode(lp, LoraMode_Normal);
}

//reset lora module
uint8_t Lora_Reset(LoraPacket *lp)
{
#define LORA_RESET_MAX_TIME 	200
  uint8_t delay = 0;
  
  *(lp->dma_sbuff + 0) = 0xC4;
  *(lp->dma_sbuff + 1) = 0xC4;
  *(lp->dma_sbuff + 2) = 0xC4;
  
  if (HAL_UART_Transmit_DMA(&huart1, lp->dma_sbuff, 3) == HAL_OK) {
    while ((!Lora_is_idle(lp)) && (delay < LORA_RESET_MAX_TIME))
    {
      delay++;
      HAL_Delay(5);
    }
    
    if (delay >= LORA_RESET_MAX_TIME) {
      return 1;
    } else {
      return 0;
    }
  } else {
    return 1;
  }
}

//通过lora发送一串数据
/*
*       id:目标设备的地址
*       len：buff的信息长度，如果长度超过58则需要分段发送
*/
bool LoraTransfer(Device *d, uint8_t flag)
{
  bool ret = false;
#define LORA_MAX_IDLE_WAIT      200
#define UART_MAX_SEND_WAIT      200
  
  uint8_t delay = 0;
  uint8_t pos;        //如果是定点模式，则在信息头部需要附加目标地址和信道共三个字节数据
  
  while((!Lora_is_idle(&gLoraPacket)) && (delay < LORA_MAX_IDLE_WAIT))
  {
    delay++;
    HAL_Delay(1);
  }
  
  if (delay >= LORA_MAX_IDLE_WAIT) {
    return false;
  }
  
  if ( LoraMode_Normal != gLoraPacket.mode) {
    Lora_Module_Set_Mode(&gLoraPacket, LoraMode_Normal);
  }
  
  if ( TRANSFER_MODE_DINGDIAN == gLoraPacket.paramter.u8TranferMode) {
    gLoraPacket.dma_sbuff[0] = (uint8_t)((TARGET_ID & 0xff00) >> 8);
    gLoraPacket.dma_sbuff[1] = (uint8_t)(TARGET_ID & 0x00ff);
    gLoraPacket.dma_sbuff[2] = gLoraPacket.paramter.u8Channel;
    pos = 3;
  } else {
    pos = 0;
  }
  
  RespDataPacket *ptr = (RespDataPacket *)&gLoraPacket.dma_sbuff[pos];
  ptr->u8Head = LORA_MSG_HEAD;
  ptr->u8Len  = SERVER_REC_RESP_LEN;
  ptr->u16Id  = gLoraPacket.paramter.u16Addr;
  
  ptr->u8Resp = d->u8Resp;
  if ( flag == RECMD ) {
    ptr->u8Cmd = d->u8ReCmd;
    ptr->u32Identify = d->u32ReIdentify;
  } else {
    ptr->u8Cmd  = d->u8Cmd;
    ptr->u32Identify = d->u32Identify;
  }
  ptr->u16Crc = CRC16_IBM(&gLoraPacket.dma_sbuff[pos], SERVER_REC_RESP_LEN - 3);
  ptr->u8Tail = LORA_MSG_TAIL;
  
  if ( HAL_OK == HAL_UART_Transmit_DMA(&huart1, gLoraPacket.dma_sbuff, ptr->u8Len + pos)) {

  }
  
  HAL_Delay(3);
  
  while(!Lora_is_idle(&gLoraPacket));
  
  Lora_Module_Set_Mode(&gLoraPacket, LoraMode_LowPower);
  
  return ret;
}

uint8_t LoraRegister(void)
{
#define LORA_MAX_IDLE_WAIT      200
#define UART_MAX_SEND_WAIT      200
  
  uint8_t delay = 0;
  uint8_t buffer[12] = {0};
  
  while((!Lora_is_idle(&gLoraPacket)) && (delay < LORA_MAX_IDLE_WAIT))
  {
    delay++;
    HAL_Delay(1);
  }
  
  if (delay >= LORA_MAX_IDLE_WAIT) {
    return 0;
  }
  
  if ( LoraMode_Normal != gLoraPacket.mode) {
    Lora_Module_Set_Mode(&gLoraPacket, LoraMode_Normal);
  }
  
  gLoraPacket.dma_sbuff[0] = (uint8_t)((TARGET_ID & 0xff00) >> 8);
  gLoraPacket.dma_sbuff[1] = (uint8_t)(TARGET_ID & 0x00ff);
  gLoraPacket.dma_sbuff[2] = gLoraPacket.paramter.u8Channel;

  RespDataPacket *ptr = (RespDataPacket *)&gLoraPacket.dma_sbuff[3];
  ptr->u8Head = LORA_MSG_HEAD;
  ptr->u8Len  = SERVER_REC_RESP_LEN;
  ptr->u16Id  = gLoraPacket.paramter.u16Addr;
  ptr->u8Resp = 0;
  ptr->u8Cmd  = HW_DEVICE_ONLINE;
  ptr->u32Identify = 0;
  ptr->u16Crc = CRC16_IBM((uint8_t *)ptr, SERVER_REC_RESP_LEN - 3);
  ptr->u8Tail = LORA_MSG_TAIL;
  
  if ( HAL_OK == HAL_UART_Transmit(&huart1, gLoraPacket.dma_sbuff, ptr->u8Len + 3, 100)) {
    if ( HAL_OK == HAL_UART_Receive(&huart1, buffer, 12, 2000) ) {
    
      CmdDataPacket *cmd = (CmdDataPacket *)buffer;
      if(cmd->u8Cmd == HW_DEVICE_ONLINE)
        return 1;
      else
        return 0;
    }
  }
  
  HAL_Delay(3);
  
  while(!Lora_is_idle(&gLoraPacket));
  
  Lora_Module_Set_Mode(&gLoraPacket, LoraMode_LowPower);
  
  return 0;
}

void CopyDataFromDMA(void)
{
  uint16_t len;
  LoraPacket *lp = &gLoraPacket;
  
  //len 等于已经用于存放数据的长度(空间)
  len = (MAX_DMA_LEN - (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_usart1_rx));
  
  //将dma缓冲区数据取出放入到数据处理的内存内
  //用len和pos来取出数据
  //分为几种情况
  //1：len=pos=0 没有数据
  //2：len > pos 可以出去从pos到len的数据
  //3：len < pos 说明产生了串口接收完成中断，在该缓冲区内数据存满了，需要取下一个缓冲区内数据
  //4: len = 0, pos > 0 产生数据接收完成中断，需要取出当前dma内的从pos到最后的所有数据
  
  if ((0 == len) && (0 == lp->pos))
    return;
  
  if (( 0 == len ) && ( lp->pos > 0)) {
    //        printf("pool rec int 1\r\n");
    //数据接收完成中断，将数据写入pool中后，将pos重置。
    uint16_t pos = lp->pos;

    len = DataPoolWrite(gUartDataPool, 
                        lp->dma_rbuff + pos, 
                        MAX_DMA_LEN - pos );
    
    lp->pos = 0;
    
  } else  if (len > lp->pos) {
    //空闲中断，取出当前接收的数据，然后重新设置pos值
    uint16_t pos = lp->pos;
    len = DataPoolWrite(gUartDataPool, 
                        lp->dma_rbuff + pos, 
                        len - pos);
    lp->pos += len;
  }
}

void UartDataProcess(void)
{
  uint8_t retry = 0;
  uint8_t dat;
  uint8_t ret;
  
  DataPool * dp = gUartDataPool;
  
  while(1) {
    
    dat = 0;
    
    ret = DataPoolLookByte(dp, &dat);
    
    if ( LORA_MSG_HEAD == dat ) {
      //接收到了response信息
      uint8_t buff[SERVER_SEND_CMD_LEN];
      uint8_t len = DataPoolGetNumByte(dp, &buff[0], SERVER_SEND_CMD_LEN);
      
      if ( len == SERVER_SEND_CMD_LEN ) {
        
        CmdDataPacket *cmd = (CmdDataPacket *)buff;
        
        if ( cmd->u8Tail == LORA_MSG_TAIL ) {
          
          uint16_t crc = CRC16_IBM((uint8_t *)cmd, SERVER_SEND_CMD_LEN -3);
          
          if ( crc == cmd->u16Crc ) {
            retry = 0;
            //接收到正确的消息
            if ((gDevice.u8Cmd != HW_CMD_NONE) && (gDevice.u8CmdRunning == CMD_RUN)) {
              gDevice.u8ReCmdFlag = 1;
              gDevice.u8ReCmd = cmd->u8Cmd;
              gDevice.u32ReIdentify = cmd->u32Identify;
            } else if (cmd->u8Cmd == HW_DEVICE_ONLINE ) {
              //do nothing
            } else {
              gDevice.u8Cmd = cmd->u8Cmd;
              gDevice.u32Identify = cmd->u32Identify;
            }
          }
        } /*if ( resp->u8Tail == LORA_MSG_TAIL )*/
        else {
          //读取到错误的数据包，恢复pool，
          DataPoolResume(dp, SERVER_SEND_CMD_LEN);
          //跳过当前字节寻找下一个head
          ret = DataPoolGetByte(dp, &dat);
        }
      } /*end if ( len == SERVER_REC_RESP_LEN )*/
      else {
        DataPoolResume(dp, len);
        //跳过当前字节寻找下一个head
        retry++;
        HAL_Delay(1);
        if (retry > 200) {
          retry = 0;
          ret = DataPoolGetByte(dp, &dat);
        }
      }
    } /*end if ( LORA_MSG_HEAD == dat ) */
    else {
      if ( ret != POOL_CLEAN ) {
        ret = DataPoolGetByte(dp, &dat);
      } else {
        break;
      }
    }
  }
}

