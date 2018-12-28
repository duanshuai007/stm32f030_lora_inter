#include "Lora.h"
#include "stdint.h"
#include "stm32f1xx.h"
#include "user_config.h"
#include "string.h"
#include "crc16.h"
#include "flash.h"
#include "rtc.h"
#include "maincontrol.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

static LoraModule gLoraMS;
static LoraModule gLoraMR;

static uint8_t dma_uart3_sbuff[UART3_TX_DMA_LEN] = {0};
static uint8_t dma_uart1_rbuff[UART1_RX_DMA_LEN] = {0};
static volatile uint8_t lora_send_timer = 0;

/*
*   ͨ�����ڻ�ȡloramodule�ṹ��
*   usart1��Ӧ����lora��usart3��Ӧ����lora
*/
static LoraModule *get_loramodule_by_uart(UART_HandleTypeDef *uart)
{
  if ( uart->Instance == USART1 ) 
    return &gLoraMR;
  else if ( uart->Instance == USART3 )
    return &gLoraMS;
  else 
    return NULL;
}

/*
*   ͨ�����ڻ�ȡ��Ӧloraģ�鹤��״̬
*   ���ص���AUX��ƽ״̬���͵�ƽ��ʾ��æ���ߵ�ƽ��ʾ����
*/
static bool get_loramodule_idle_flag(UART_HandleTypeDef *uart)
{
  LoraModule *lm = get_loramodule_by_uart(uart);
  
  if (HAL_GPIO_ReadPin(lm->gpio.AUX.GPIOX, lm->gpio.AUX.Pin) == GPIO_PIN_SET)
    return true;
  else 
    return false;
}

/*
*   Loraģ�鷢�ͼ����ʱ��
*   ��Ϊloraģ���ǹ����ڻ���ģʽ����ͬ���豸id֮����Ҫ���������г����ܹ�
*   ��֤ÿһ֡���ݶ���ӻ�����
*/
void LoraSendTimerHandler(void)
{
  if (lora_send_timer > 0) {
    lora_send_timer++;
    if ( lora_send_timer >= LORA_SEND_MIN_TIMEINTERVAL ) {
      lora_send_timer = 0;
    }
  }
}

/*
*   �ж�loraģ������ŵ�ƽ������lora�Ŀ���״̬
*/
//void SetLoraModuleIdleFlagHandler(uint16_t pin)
//{
//  if ( gLoraMS.gpio.AUX.Pin == pin ) {
//    if ( HAL_GPIO_ReadPin(gLoraMS.gpio.AUX.GPIOX, gLoraMS.gpio.AUX.Pin) == GPIO_PIN_SET ) {
//      gLoraMS.isIdle = true;
//    } else {
//      gLoraMS.isIdle = false;
//    }
//  } else if ( gLoraMR.gpio.AUX.Pin == pin) {
//    if ( HAL_GPIO_ReadPin(gLoraMR.gpio.AUX.GPIOX, gLoraMR.gpio.AUX.Pin) == GPIO_PIN_SET ) {
//      gLoraMR.isIdle = true;
//    } else {
//      gLoraMR.isIdle = false;
//    }
//  }
//}

/*
*   ����Loraģ��Ĺ���״̬
*/
static bool set_loramodule_workmode(LoraModule *lp, Lora_Mode mode)
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
  } while((!get_loramodule_idle_flag(lp->muart.uart)) && (delay < LORA_SET_MAX_TIME));
  
  if(delay >= LORA_SET_MAX_TIME)
    return false;
  else
    return true;
}

/*
*   ����loraģ�����õ�������
*/
void LoraModuleGPIOInit(UART_HandleTypeDef *huart)
{
  LoraModule *lm = get_loramodule_by_uart(huart);
  
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

/*
*   ����loraģ���dma��dma���շ��ͻ�������datapool�Ȳ���
*/
void LoraModuleDMAInit(UART_HandleTypeDef *huart)
{
  LoraModule *lm = get_loramodule_by_uart(huart);
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
    
    set_loramodule_workmode( lm, LoraMode_Normal);
    
    lm->muart.rxDataPool = DataPoolInit(UART1_RX_DATAPOOL_SIZE);
    if ( lm->muart.rxDataPool == NULL ) {
      DEBUG_ERROR("uart1 rxdatapool NULL\r\n");
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
    
    set_loramodule_workmode( lm, LoraMode_WakeUp);
    
    lm->muart.rxDataPool = NULL;
    lm->muart.txDataPool = DataPoolInit(UART1_RX_DATAPOOL_SIZE);
    if ( lm->muart.txDataPool == NULL ) {
      DEBUG_ERROR("uart3 txdatapool NULL\r\n");
    }
  }
}

/*
*   Generate Lora Paramter
*	  ���ݲ����������ݣ��ɹ�����0��ʧ�ܷ���1
*/
static bool generate_loramodule_paramter(LoraPar *lp, uint8_t *buff, SaveType st)
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
    return false;
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
    return false;
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
    return false;
  }
  
  if(lp->u8Channel > CHAN_441MHZ)
    return false;
  
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
    return false;
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
    return false;
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
    return false;
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
    return false;
  }
  
  return true;
}

/*
*   ����loraģ��Ĳ���
*/
static bool set_loramodule_paramter(LoraModule *lp)
{
#define LORA_SET_PAR_MAX_TIME 	100
  uint8_t config[6] = {0};
  
  if(lp == NULL || &lp->paramter == NULL)
    return false;
  
  if (!generate_loramodule_paramter(&lp->paramter, config, SAVE_IN_FLASH))
    return false;
  
  //��Ҫ����ʱ����
  HAL_Delay(10);
  
  if (HAL_UART_Transmit(lp->muart.uart, config, 6, 100) == HAL_OK) {
    memset( config, 0, 6);
    if (HAL_UART_Receive(lp->muart.uart, config, 6, 1000) == HAL_OK) { 
      DEBUG_INFO("rec:%02x-%02x-%02x-%02x-%02x-%02x\r\n",
             config[0], config[1],config[2],
             config[3],config[4],config[5]);
      return true;
    }
    return false;
  }
  
  return false;
}

/*
*     �������ܵ���loraģ��Ĳ���
*/
static void get_loramodule_paramter(LoraPar *lp, uint8_t *buff)
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
*       
*       no = LoraReadPar:��ȡģ�����
*       no = LoraReadVer:��ȡģ��汾��Ϣ
*       PS:�ڷ����˶�ȡ�汾������֮�����ٷ�������
*/
static bool read_loramodule(uint8_t no, LoraModule *lp)
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
    return false;
  }
  
  if (HAL_UART_Transmit(lp->muart.uart, cmd, 3, 100) == HAL_OK) {
    if ( HAL_UART_Receive(lp->muart.uart, buffer, 6, 1000) == HAL_OK ) {
      DEBUG_INFO("read:%02x-%02x-%02x-%02x-%02x-%02x\r\n",
             buffer[0],buffer[1],buffer[2],
             buffer[3],buffer[4],buffer[5]);
      get_loramodule_paramter(&lp->paramter, buffer);
      return true;
    }
    return false;
  }
  
  return false;
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

static void uart_set_baud_parity(LoraModule *lm)
{
  uint32_t baud_array[8] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
  
  __HAL_UART_DISABLE(lm->muart.uart);
  lm->muart.uart->Init.BaudRate = baud_array[lm->paramter.u8Baud];
  if (( PARITY_8N1 == lm->paramter.u8Parity) || ( PARITY_8N1_S == lm->paramter.u8Parity)) {
    //��У��
    lm->muart.uart->Init.WordLength = UART_WORDLENGTH_8B;
    lm->muart.uart->Init.Parity = UART_PARITY_NONE;
  } else if ( PARITY_8O1 == lm->paramter.u8Parity ) {
    lm->muart.uart->Init.WordLength = UART_WORDLENGTH_9B;
    lm->muart.uart->Init.Parity = UART_PARITY_ODD;
  } else if ( PARITY_8E1 == lm->paramter.u8Parity ) {
    lm->muart.uart->Init.WordLength = UART_WORDLENGTH_9B;
    lm->muart.uart->Init.Parity = UART_PARITY_EVEN;
  }
  if (HAL_UART_Init(lm->muart.uart) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  HAL_Delay(10);
}

/*
*   ����Loraģ�����
*/
void SetLoraParamter(UART_HandleTypeDef *huart, LoraPar *lp)
{
  
  LoraModule *lm = get_loramodule_by_uart(huart);
  
  uart_set_9600_8n1(lm);
  //��������ģʽ,������ģʽ�£����ܴ�У��
  set_loramodule_workmode(lm, LoraMode_Sleep);
  //д�����ò���
  memcpy(&lm->paramter, (void *)lp, sizeof(LoraPar));
  
  while(!set_loramodule_paramter(lm));
  
  uart_set_baud_parity(lm);
  set_loramodule_workmode(lm, LoraMode_Normal);
  memset(&lm->paramter, 0, sizeof(LoraPar));
}

/*
*   ��ȡLoraģ�����
*/
void ReadLoraParamter(UART_HandleTypeDef *huart)
{
  LoraModule *lm = get_loramodule_by_uart(huart);
  
  uart_set_9600_8n1(lm);
  set_loramodule_workmode(lm, LoraMode_Sleep);
  while(!read_loramodule(LoraReadPar, lm));
  uart_set_baud_parity(lm);
  set_loramodule_workmode(lm, LoraMode_Normal);
}

/*
*   ��Ŀ���豸���Ϳ�������       
*   id:Ŀ���豸�ĵ�ַ
*   channel:Ŀ���豸�����ŵ�
*   cmd:  ���͸�Ŀ���豸������
*   identify: ���͸�Ŀ���豸�������Ψһʶ����
*/
bool LoraCtrlEndPoint(uint16_t id, uint8_t cmd, uint32_t identify)
{ 
  LoraModule *lm = &gLoraMS;
  uint8_t buffer[16];
  
  buffer[0] = (uint8_t)((id & 0xff00) >> 8);
  buffer[1] = (uint8_t)(id & 0x00ff);
  buffer[2] = gLoraMS.paramter.u8Channel;
  
  CmdDataPacket *ptr = (CmdDataPacket *)&buffer[3];
  ptr->u8Head = LORA_MSG_HEAD;
  ptr->u8Len  = SERVER_CMD_LEN;
  ptr->u16Id  = id;
  ptr->u8Cmd  = cmd;
  ptr->u32Identify = identify;
  ptr->u16Crc = CRC16_IBM((uint8_t *)ptr, SERVER_CMD_LEN - 3);
  ptr->u8Tail = LORA_MSG_TAIL;
  
  //����װ��ɵ�������Ϣ���͵�lora����ģ������ݳ��еȴ�����
  if ( DataPoolWrite(lm->muart.txDataPool, buffer, sizeof(CmdDataPacket) + 3))
    return true;
  else
    return false;
}

/*
*   ���ݴ��ڻ�ȡ��Ӧ��ģ��ָ��
*/
static UartModule *get_uartmodule(UART_HandleTypeDef *huart)
{
  if ( huart->Instance == gLoraMR.muart.uart->Instance ) {
    return &gLoraMR.muart;
  } else {
    return GetF405UartModule(huart);
  }
}

/*
*   �жϺ�������������ʱ�Ὣ��Ӧ�Ĵ���DMA�������е�
*             ���ݸ��Ƶ���Ӧ�����Լ������ݳ��С�
*   ֻ��uart1��uart2�Ŀ����ж��л���á�
*/
bool CopyDataFromDMAHandler(UART_HandleTypeDef *huart)
{
  uint16_t len;
  uint16_t pos;
  UartModule *um = get_uartmodule(huart);
  
  //len �����Ѿ����ڴ�����ݵĳ���(�ռ�)
  len = (um->dma_rbuff_size - (uint16_t)__HAL_DMA_GET_COUNTER(um->uart_rx_hdma));
  
  //��dma����������ȡ�����뵽���ݴ�����ڴ���
  //��len��pos��ȡ������,��Ϊ�������
  //1��len=pos=0 û������
  //2��len > pos ���Գ�ȥ��pos��len������
  //3��len < pos ˵�������˴��ڽ�������жϣ��ڸû����������ݴ����ˣ���Ҫȡ��һ��������������
  //4: len = 0, pos > 0 �������ݽ�������жϣ���Ҫȡ����ǰdma�ڵĴ�pos��������������
  
  if (((0 == len) && (0 == um->pos)) || (len == um->pos))
    return true;
  
  if (( 0 == len ) && ( um->pos > 0)) {
    //���ݽ�������жϣ�������д��pool�к󣬽�pos���á�
    pos = um->pos;

    if (DataPoolWrite(um->rxDataPool, um->dma_rbuff + pos, um->dma_rbuff_size - pos )) {
      um->pos = 0;
      return true;
    }
  } else  if (len > um->pos) {
    //�����жϣ�ȡ����ǰ���յ����ݣ�Ȼ����������posֵ
    pos = um->pos;
    //len���������ݿռ��������ݵ��ܳ���
    //pos����һ��ȡ�����ݺ��ָ��λ��
    //���б������ݳ��Ⱦ���len-pos
    if (DataPoolWrite(um->rxDataPool, um->dma_rbuff + pos, len - pos)) { 
      um->pos += (len - pos);
      if( um->pos == um->dma_rbuff_size ) 
        um->pos = 0;
      return true;
    }
  }
  
  return false;
}

/*
*   �жϺ���:����DMA�����ƶ�������ʹ��dma���չ���
*   �ڴ��ڽ�������ж��е���
*/
void LoraModuleReceiveHandler(UART_HandleTypeDef *huart) 
{
  if ( huart->Instance != gLoraMR.muart.uart->Instance )
    return;
  
  CopyDataFromDMAHandler(gLoraMR.muart.uart);
  HAL_UART_Receive_DMA(gLoraMR.muart.uart,
                       gLoraMR.muart.dma_rbuff,
                       gLoraMR.muart.dma_rbuff_size);
}
 
/*
*   Loraģ������ݴ���
*/
void LoraModuleTask(void)
{
  uint8_t dat;
  uint8_t buff[SERVER_RESP_LEN];
  //�������ݵĴ���
  LoraModule *lm = &gLoraMR;
  DataPool *dp = lm->muart.rxDataPool;
  
  if (DataPoolGetNumByte(dp, &buff[0], SERVER_RESP_LEN)) {
    RespDataPacket *resp = (RespDataPacket *)buff;
    if (( LORA_MSG_HEAD == resp->u8Head ) && ( LORA_MSG_TAIL == resp->u8Tail )) {
      uint16_t crc = CRC16_IBM((uint8_t *)resp, SERVER_RESP_LEN -3);
      if ( crc == resp->u16Crc ) {
        switch ( resp->u8Cmd ) {
        case DEVICE_REGISTER:
          //ע����Ϣ
          if (AddDeviceToList(resp->u16Id) == true) {
            UartSendMSGToF405(NODE_ONLINE, resp->u16Id, 0);
            //give endpoint resp msg
            //�������ʧ��endpoint���ղ�����Ӧ��Ϣ���������ע����Ϣ�����Բ���Ҫ�жϷ���ֵ
            LoraCtrlEndPoint(resp->u16Id, /*lmsend->paramter.u8Channel,*/ DEVICE_REGISTER, 0);
          }
          break;
        case DEVICE_ABNORMAL:
          //�쳣������Ϣ
          //������쳣������Ϣ����Ҫ����identifyΪ0
          SendCMDRespToList(resp->u16Id, resp->u8Cmd, 0, resp->u8Resp);
          break;
        default:
          //��������ָ�����Ӧ��Ϣ
          SendCMDRespToList(resp->u16Id, resp->u8Cmd, resp->u32Identify, resp->u8Resp);
          break;
        }
        //���浽flash����,�����豸ʱ����Ϣ
        FlashAddDeviceToList(resp->u16Id, GetRTCTimeMinAndSec());
      }
    }/*end of if (( LORA_MSG_HEAD == resp->u8Head ) && ( LORA_MSG_TAIL == resp->u8Tail ))*/
    else {
      //�ָ�ȡ����������
      DataPoolResume(dp, SERVER_RESP_LEN);
      //����ͷָ����һ���ֽ�
      DataPoolGetByte(dp, &dat);
    }
  }
  
  //�������ݴ���
  //ÿһ��ָ���Ҫ����һ��ָ����һ����ʱ����ܷ��ͳɹ�
  //lora_send_timer ���ÿ���ÿһ��ָ���ʱ����
  UartModule *um = &gLoraMS.muart;
  
  if ( lora_send_timer == 0 ) {
    //��ʱ�������˻���Ҫ�ж��豸�Ƿ����
    if (get_loramodule_idle_flag(lm->muart.uart)) {
      //�豸�����ˣ�������ݳ�����ȡ��һ��ָ��ȵ���Ϣ
      if ( DataPoolGetNumByte(um->txDataPool, um->dma_sbuff, um->dma_sbuff_size)) {
        //ͨ��DMA���ͣ���������ʱ����
#if 0
        uint8_t i;
        for(i=0;i < um->dma_sbuff_size; i++)
          printf("%02x ", um->dma_sbuff[i]);
        printf("\r\n");
#endif   
        HAL_UART_Transmit_DMA(um->uart, um->dma_sbuff, um->dma_sbuff_size);
        lora_send_timer = 1;
      } else {
        lora_send_timer = 0;
      }
    }
  }
}
