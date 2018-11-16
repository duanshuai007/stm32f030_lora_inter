#include "Lora.h"
#include "stdint.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_uart.h"
#include "crc16.h"
#include "user_config.h"
#include "string.h"
#include "stdlib.h"
#include "lora_datapool.h"
#include "flash.h"
#include "rtc.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

extern LoraModule gLoraMS;
extern LoraModule gLoraMR;
extern Device     gDevice;
extern UartModule gUartMx;
extern List       *gList;
extern FLASHDeviceList *gFDList;

extern uint8_t dma_uart3_sbuff[UART3_TX_DMA_LEN];
extern uint8_t dma_uart1_rbuff[UART1_RX_DMA_LEN];
extern volatile uint8_t lora_send_timer;

static uint8_t Lora_is_idle(LoraModule *lm)
{
  return lm->isIdle;
}

static bool Lora_Module_Set_Mode(LoraModule *lp, Lora_Mode mode)
{
#define LORA_SET_MAX_TIME 	500
  
  uint16_t delay = 0;
  
  lp->mode = mode;
  
  HAL_Delay(5);       //�������ʱ������5ms
  
  switch(lp->mode)
  {
  case LoraMode_Normal:
    HAL_GPIO_WritePin(lp->gpio.M0.GPIOX, lp->gpio.M0.Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lp->gpio.M1.GPIOX, lp->gpio.M1.Pin, GPIO_PIN_RESET);
    break;
  case LoraMode_WakeUp:
    HAL_GPIO_WritePin(lp->gpio.M0.GPIOX, lp->gpio.M0.Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(lp->gpio.M1.GPIOX, lp->gpio.M1.Pin, GPIO_PIN_RESET);
    break;
  case LoraMode_LowPower:
    HAL_GPIO_WritePin(lp->gpio.M0.GPIOX, lp->gpio.M0.Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lp->gpio.M1.GPIOX, lp->gpio.M1.Pin, GPIO_PIN_SET);
    break;
  case LoraMode_Sleep:
    HAL_GPIO_WritePin(lp->gpio.M0.GPIOX, lp->gpio.M0.Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(lp->gpio.M1.GPIOX, lp->gpio.M1.Pin, GPIO_PIN_SET);
    break;
  default:
    return false;
  }
  
  do {
    HAL_Delay(2);   //�����do{}while(...)ѭ������ִ��һ����ʱ
    delay++;
  } while((!Lora_is_idle(lp)) && (delay < LORA_SET_MAX_TIME));
  
  if(delay >= LORA_SET_MAX_TIME)
    return false;
  else
    return true;
}

void LoraModuleGPIOInit(LoraModule *lm, UART_HandleTypeDef *huart)
{
  memset(lm, 0, sizeof(LoraModule));
  if (USART1 == huart->Instance) {
    lm->gpio.AUX.GPIOX      = GPIO_Lora_AUX_R;
    lm->gpio.AUX.Pin        = GPIO_Lora_AUX_R_Pin;
    lm->gpio.M0.GPIOX       = GPIO_Lora_M0_R;
    lm->gpio.M0.Pin         = GPIO_Lora_M0_R_Pin;
    lm->gpio.M1.GPIOX       = GPIO_Lora_M1_R;
    lm->gpio.M1.Pin         = GPIO_Lora_M1_R_Pin;
  } else if (USART3 == huart->Instance) {
    lm->gpio.AUX.GPIOX      = GPIO_Lora_AUX_S;
    lm->gpio.AUX.Pin        = GPIO_Lora_AUX_S_Pin;
    lm->gpio.M0.GPIOX       = GPIO_Lora_M0_S;
    lm->gpio.M0.Pin         = GPIO_Lora_M0_S_Pin;
    lm->gpio.M1.GPIOX       = GPIO_Lora_M1_S;
    lm->gpio.M1.Pin         = GPIO_Lora_M1_S_Pin;
  }
  
  lm->muart.uart = huart;
}

void LoraModuleDMAInit( LoraModule *lm)
{
  /*
  *   UART1 ��ΪLora���մ���
  */
  if (USART1 == lm->muart.uart->Instance) {
    lm->muart.dma_rbuff_size      = UART1_RX_DMA_LEN;
    lm->muart.dma_sbuff_size      = 0;
    
    lm->muart.dma_rbuff           = &dma_uart1_rbuff[0];
    lm->muart.dma_sbuff           = NULL;
    
    lm->muart.uart_rx_hdma        = &hdma_usart1_rx;
    lm->muart.uart_tx_hdma        = NULL;
    /*      
    *       ʹ��DMA���գ����ý��ջ�����
    *       { [buffer:1] [buffer:2] [buffer:3] }
    *       ��ʱ��������buffer��DMAÿ��ʹ������һ�����������������
    *       ����DMA���ջ�����Ϊ��һ��buffer��ѭ��ʹ�á�
    *       ����������������ݻ�����ΪUART1_RX_DMA_LEN
    *       ʹ�ܴ��ڿ����ж����������ж�
    *       �����յ�x��resp��Ϣ����һ�ν�������жϡ�
    *       ���������յ�����resp��Ϣʱ�ᴥ�������жϽ��д���
    *       ���ǽ��յ����respʱ�ᴥ����������жϽ��д���
    *       ��������������жϺ󣬽����ж���ȡ��ǰbuffer�е�ָ��λ�ã�
    *       Ȼ��ָ��ָ����һ�����е��ڴ�Ƭ������ʹ�ܽ���(HAL_UART_RECEIVE_DMA)
    */
    HAL_UART_Receive_DMA(lm->muart.uart, lm->muart.dma_rbuff, lm->muart.dma_rbuff_size);
    //ʹ�ܿ����жϣ����Խ���������
    __HAL_UART_ENABLE_IT(lm->muart.uart, UART_IT_IDLE);
    
    Lora_Module_Set_Mode( lm, LoraMode_Normal);
    
    lm->muart.rxDataPool = DataPoolInit(UART1_RX_DATAPOOL_SIZE);
    if ( lm->muart.rxDataPool == NULL ) {
      PRINT("uart1 rxdatapool NULL\r\n");
    }
    
    lm->muart.txDataPool = NULL;
  }
  /*
  *   UART3 ��ΪLora���ʹ���
  */
  else if (USART3 == lm->muart.uart->Instance) {
    lm->muart.dma_rbuff_size  = 0;
    lm->muart.dma_sbuff_size  = UART3_TX_DMA_LEN;
    
    lm->muart.dma_rbuff       = NULL;
    lm->muart.dma_sbuff       = &dma_uart3_sbuff[0];
    
    lm->muart.uart_rx_hdma    = NULL;
    lm->muart.uart_tx_hdma    = &hdma_usart3_tx;
    /* UART2��Ϊ�����ʹ��ڣ�����������ģʽ������ʱ������������� */
    
    Lora_Module_Set_Mode( lm, LoraMode_WakeUp);
    
    lm->muart.rxDataPool = NULL;
    lm->muart.txDataPool = DataPoolInit(UART1_RX_DATAPOOL_SIZE);
    if ( lm->muart.txDataPool == NULL ) {
      PRINT("uart3 txdatapool NULL\r\n");
    }
  }
}

/*
*       Generate Lora Paramter
*	���ݲ����������ݣ��ɹ�����0��ʧ�ܷ���1
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
    return 0;
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
    return 0;
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
    return 0;
  }
  
  if(lp->u8Channel > CHAN_441MHZ)
    return 0;
  
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
    return 0;
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
    return 0;
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
    return 0;
  }
  
  if (lp->u8FEC) {
    buff[5] |= 1 << 2;
  } else {
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
    return 0;
  }
  
  return 1;
}

/*
*       Set Lora Paramter
*/
static uint8_t Lora_SetPar(LoraModule *lp)
{
#define LORA_SET_PAR_MAX_TIME 	100
  uint8_t config[6] = {0};
  
  if(lp == NULL || &lp->paramter == NULL)
    return 0;
  
  if (!Lora_GeneratePar(&lp->paramter, config, SAVE_IN_FLASH))
    return 0;
  
  //��Ҫ����ʱ����
  HAL_Delay(10);
  
  if (HAL_UART_Transmit(lp->muart.uart, config, 6, 100) == HAL_OK) {
    memset( config, 0, 6);
    if (HAL_UART_Receive(lp->muart.uart, config, 6, 1000) == HAL_OK) { 
      PRINT("rec:%02x-%02x-%02x-%02x-%02x-%02x\r\n",
             config[0], config[1],config[2],
             config[3],config[4],config[5]);
      return 1;
    }
    return 0;
  }
  
  return 0;
}
//�ӽ��յ������ݰ��з������ò���
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
*       PS:�ڷ����˶�ȡ�汾������֮�����ٷ�������
*/
static uint8_t Lora_Read(uint8_t no, LoraModule *lp)
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
  
  if (HAL_UART_Transmit(lp->muart.uart, cmd, 3, 100) == HAL_OK) {
    if ( HAL_UART_Receive(lp->muart.uart, buffer, 6, 1000) == HAL_OK ) {
      PRINT("read:%02x-%02x-%02x-%02x-%02x-%02x\r\n",
             buffer[0],buffer[1],buffer[2],
             buffer[3],buffer[4],buffer[5]);
      Lora_GetPar(&lp->paramter, buffer);
      return 1;
    }
    return 0;
  }
  
  return 0;
}

static void uart_set_9600_8n1(LoraModule *lm)
{
  __HAL_UART_DISABLE(lm->muart.uart);
  lm->muart.uart->Init.BaudRate = 9600;
  lm->muart.uart->Init.WordLength = UART_WORDLENGTH_8B;
  lm->muart.uart->Init.Parity = UART_PARITY_NONE;
  if (HAL_UART_Init(lm->muart.uart) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  HAL_Delay(10);
}

static void uart_set_baud_parity(LoraModule *lm, BaudType baud, ParityType parity)
{
  uint32_t baud_array[8] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
  
  __HAL_UART_DISABLE(lm->muart.uart);
  lm->muart.uart->Init.BaudRate = baud_array[baud];
  if (( PARITY_8N1 == parity) || ( PARITY_8N1_S == parity)) {
    //��У��
    lm->muart.uart->Init.WordLength = UART_WORDLENGTH_8B;
    lm->muart.uart->Init.Parity = UART_PARITY_NONE;
  } else if ( PARITY_8O1 == parity ) {
    lm->muart.uart->Init.WordLength = UART_WORDLENGTH_9B;
    lm->muart.uart->Init.Parity = UART_PARITY_ODD;
  } else if ( PARITY_8E1 == parity ) {
    lm->muart.uart->Init.WordLength = UART_WORDLENGTH_9B;
    lm->muart.uart->Init.Parity = UART_PARITY_EVEN;
  }
  if (HAL_UART_Init(lm->muart.uart) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  HAL_Delay(10);
}

void LoraSetParamter(LoraModule *lm, 
                     uint16_t addr, 
                     BaudType baud, 
                     CHANType channel, 
                     ParityType parity, 
                     SpeedInAirType speedinair, 
                     TransferModeType transmode, 
                     WakeUpTimeType wutime)
{
  
  uart_set_9600_8n1(lm);
  
  //��������ģʽ,������ģʽ�£����ܴ�У��
  Lora_Module_Set_Mode(lm, LoraMode_Sleep);
  
  //д�����ò���
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
  while(!Lora_SetPar(lm));
  
  uart_set_baud_parity(lm, baud, parity);
  Lora_Module_Set_Mode(lm, LoraMode_Normal);
  memset(&lm->paramter, 0, sizeof(LoraPar));
}

void LoraReadParamter(LoraModule *lm)
{
  uart_set_9600_8n1(lm);
  Lora_Module_Set_Mode(lm, LoraMode_Sleep);
  while(!Lora_Read(LoraReadPar, lm));
  uart_set_baud_parity(lm, (BaudType)lm->paramter.u8Baud, (ParityType)lm->paramter.u8Parity);
  Lora_Module_Set_Mode(lm, LoraMode_Normal);
}

/*
*   ��Ŀ���豸���Ϳ�������       
*   id:Ŀ���豸�ĵ�ַ
*/
bool LoraCtrlEndPoint(LoraModule *lm, uint16_t id, uint8_t channel, uint8_t cmd, uint32_t identify)
{
//#define LORA_MAX_IDLE_WAIT      200
//  
//  uint8_t delay = 0;
//  
//  while((!Lora_is_idle(lm)) && (delay < LORA_MAX_IDLE_WAIT))
//  {
//    delay++;
//    HAL_Delay(1);
//  }
//  
//  if (delay >= LORA_MAX_IDLE_WAIT) {
//    return false;
//  }
//  
//  lm->muart.dma_sbuff[0] = (uint8_t)((id & 0xff00) >> 8);        //Ŀ���ַ
//  lm->muart.dma_sbuff[1] = (uint8_t)(id & 0x00ff);
//  lm->muart.dma_sbuff[2] = channel;                      //Ŀ���ŵ�
//  
//  CmdDataPacket *ptr = (CmdDataPacket *)(lm->muart.dma_sbuff + 3);
//  ptr->u8Head = LORA_MSG_HEAD;
//  ptr->u8Len  = SERVER_CMD_LEN;
//  ptr->u16Id  = id; //Ŀ���ַ
//  ptr->u8Cmd  = cmd;
//  ptr->u32Identify = identify;
//  ptr->u16Crc = CRC16_IBM((uint8_t *)ptr, SERVER_CMD_LEN - 3);
//  ptr->u8Tail = LORA_MSG_TAIL;
//  
//  if ( HAL_OK == HAL_UART_Transmit_DMA(lm->muart.uart, lm->muart.dma_sbuff, ptr->u8Len + 3)) {
//    //���ͳɹ��������豸״̬��־Ϊrun
//    HAL_Delay(20);
//    while(!Lora_is_idle(lm));
//    return true;
//  }
  
  uint8_t buffer[16];
  
  buffer[0] = (uint8_t)((id & 0xff00) >> 8);
  buffer[1] = (uint8_t)(id & 0x00ff);
  buffer[2] = channel;
  
  CmdDataPacket *ptr = (CmdDataPacket *)&buffer[3];
  ptr->u8Head = LORA_MSG_HEAD;
  ptr->u8Len  = SERVER_CMD_LEN;
  ptr->u16Id  = id; //Ŀ���ַ
  ptr->u8Cmd  = cmd;
  ptr->u32Identify = identify;
  ptr->u16Crc = CRC16_IBM((uint8_t *)ptr, SERVER_CMD_LEN - 3);
  ptr->u8Tail = LORA_MSG_TAIL;
  
  DataPoolWrite(lm->muart.txDataPool, buffer, sizeof(CmdDataPacket) + 3);
  

  return true;
}

static UartModule *GetUartModule(UART_HandleTypeDef *huart)
{
  if ( huart->Instance == gLoraMR.muart.uart->Instance ) {
    return &gLoraMR.muart;
  }
  if ( huart->Instance == gUartMx.uart->Instance ) {
    return &gUartMx;
  }
  
  return NULL;
}

//��dma��ȡ������������
//uart2 ��uart1 ������øú������������ƶ�
void CopyDataFromDMA(UART_HandleTypeDef *huart)
{
  uint16_t len;
  UartModule *um = GetUartModule(huart);
  
  //len �����Ѿ����ڴ�����ݵĳ���(�ռ�)
  len = (um->dma_rbuff_size - (uint16_t)__HAL_DMA_GET_COUNTER(um->uart_rx_hdma));
  
  //��dma����������ȡ�����뵽���ݴ�����ڴ���
  //��len��pos��ȡ������
  //��Ϊ�������
  //1��len=pos=0 û������
  //2��len > pos ���Գ�ȥ��pos��len������
  //3��len < pos ˵�������˴��ڽ�������жϣ��ڸû����������ݴ����ˣ���Ҫȡ��һ��������������
  //4: len = 0, pos > 0 �������ݽ�������жϣ���Ҫȡ����ǰdma�ڵĴ�pos��������������
  
  if ((0 == len) && (0 == um->pos))
    return;
  
  if (( 0 == len ) && ( um->pos > 0)) {
    //���ݽ�������жϣ�������д��pool�к󣬽�pos���á�
    uint16_t pos = um->pos;

    len = DataPoolWrite(um->rxDataPool, 
                        um->dma_rbuff + pos, 
                        um->dma_rbuff_size - pos );
    um->pos = 0;
    
  } else  if (len > um->pos) {
    //�����жϣ�ȡ����ǰ���յ����ݣ�Ȼ����������posֵ
    uint16_t pos = um->pos;
    len = DataPoolWrite(um->rxDataPool, 
                        um->dma_rbuff + pos, 
                        len - pos);
    um->pos += len;
    
    if( um->pos == um->dma_rbuff_size ) 
      um->pos = 0;
  }
}

//uart1 �Ľ������ݴ�����
void LoraDataPoolProcess(LoraModule *lp)
{
  static uint8_t retry = 0;
  uint8_t time = 0;
  uint8_t dat;
  uint8_t ret;
  
  DataPool * dp = lp->muart.rxDataPool;
  
  while(1) {
    dat = 0;
    ret = DataPoolLookByte(dp, &dat);
    if ( LORA_MSG_HEAD == dat ) {
      //���յ���response��Ϣ
      uint8_t buff[SERVER_RESP_LEN];
      uint8_t len = DataPoolGetNumByte(dp, &buff[0], SERVER_RESP_LEN);
      if ( len == SERVER_RESP_LEN ) {
        RespDataPacket *resp = (RespDataPacket *)buff;
        if ( resp->u8Tail == LORA_MSG_TAIL ) {
          uint16_t crc = CRC16_IBM((uint8_t *)resp, SERVER_RESP_LEN -3);
          if ( crc == resp->u16Crc ) {
            retry = 0;
            //PRINT("Process: cmd = %d, id=%04x, resp=%d\r\n", resp->u8Cmd, resp->u16Id, resp->u8Resp);
            //�����յ����豸��Ϣд��������
            switch ( resp->u8Cmd ) {
            case DEVICE_REGISTER:
              //register device
              add_device_to_list(gList, resp->u16Id);
              //give endpoint resp msg
              LoraCtrlEndPoint(&gLoraMS, resp->u16Id, DEFAULT_CHANNEL, DEVICE_REGISTER, 0);
              //save endpoint into flash
              break;
            case DEVICE_ABNORMAL:
              //���õ�ǰ�豸cmdΪ�쳣����cmd��Ȼ��������������
              SendCMDRespToList(gList, resp->u16Id, resp->u8Cmd, 0, resp->u8Resp);
              break;
            default:
              //printf("EndP:cmd:%d,resp:%d\r\n", resp->u8Cmd, resp->u8Resp);
              SendCMDRespToList(gList, resp->u16Id, resp->u8Cmd, resp->u32Identify, resp->u8Resp);
              //save endpoint into flash
              break;
            }
            //���浽flash�����ڵĽڵ����f405����������Ϣ���յ�f405����Ӧ�����������״̬Ϊ�ɹ�
            FlashAddDeviceToList(gFDList, resp->u16Id, GetRTCTimeMinAndSec());
          }
        } /*if ( resp->u8Tail == LORA_MSG_TAIL )*/
        else {
          //��ȡ����������ݰ����ָ�pool��
          DataPoolResume(dp, SERVER_RESP_LEN);
          //������ǰ�ֽ�Ѱ����һ��head
          DataPoolGetByte(dp, &dat);
        }
      } /*end if ( len == SERVER_REC_RESP_LEN )*/
      else {
        DataPoolResume(dp, len);
        //������ǰ�ֽ�Ѱ����һ��head
        retry++;
        //HAL_Delay(1);
        if (retry > 100) {
          retry = 0;
          DataPoolGetByte(dp, &dat);
        }
        //ʹ��break����ѭ���������ʱ��
        //Ҫ�����������б����ܹ�ռ��һ����ʱ�䣬
        //�������ٶȹ��쵼������û��������жϣ������ڳ���
        break;
      }
    } /*end if ( LORA_MSG_HEAD == dat ) */
    else {
      if ( ret != POOL_CLEAN ) {
        DataPoolGetByte(dp, &dat);
      } else {
        break;
      }
    }

    //�öδ���������֤��ѭ������һֱռ��cpu
    time++;
    if ( 50 == time ) {
      time = 0;
      break;
    }
  }
  
  UartModule *um = &gLoraMS.muart;
  
  if ( lora_send_timer == 0 ) {
    if (Lora_is_idle(lp)) {
      if ( DataPoolGetNumByte(um->txDataPool, um->dma_sbuff, um->dma_sbuff_size)) {
        //���������6���ֽڣ����Ƶ�
//        printf("lora send message\r\n");
        HAL_UART_Transmit_DMA(um->uart, um->dma_sbuff, um->dma_sbuff_size);
        lora_send_timer = 1;
      } else {
        lora_send_timer = 0;
      }
    }
  }
}
