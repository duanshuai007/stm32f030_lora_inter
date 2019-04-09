#include "Lora.h"
#include "stdint.h"
#include "stm32f1xx.h"
#include "user_config.h"
#include "string.h"
#include "crc16.h"
#include "maincontrol.h"
#include "cmdlist.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

static LoraModule gLoraMS;
static LoraModule gLoraMR;

static uint8_t dma_uart3_sbuff[UART3_TX_DMA_LEN] = {0};
static uint8_t dma_uart1_rbuff[UART1_RX_DMA_LEN] = {0};
static volatile uint8_t lora_send_timer = 0;

T_CmdList *g_ControlCmdListHead = NULL;
T_CmdList *g_NormalCmdListHead = NULL;

/*
*   通过串口获取loramodule结构体
*   usart1对应接收lora，usart3对应发送lora
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
*   通过串口获取对应lora模块工作状态
*   返回的是AUX电平状态，低电平表示繁忙，高电平表示空闲
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
*   Lora模块发送间隔定时器
*   因为lora模块是工作在唤醒模式，不同的设备id之间需要保留空闲市场才能够
*   保证每一帧数据都添加唤醒码
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
*   设置Lora模块的工作状态
*/
static bool set_loramodule_workmode(LoraModule *lp, Lora_Mode mode)
{
#define LORA_SET_MAX_TIME 500
  
  uint16_t delay = 0;
  
  lp->mode = mode;
  
  HAL_Delay(5);       //必须的延时，至少5ms
  
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
    HAL_Delay(2);   //必须的do{}while(...)循环，先执行一遍延时
    delay++;
  } while((!get_loramodule_idle_flag(lp->muart.uart)) && (delay < LORA_SET_MAX_TIME));
  
  if(delay >= LORA_SET_MAX_TIME)
    return false;
  else
    return true;
}

/*
*   设置lora模块所用到的引脚
*/
void LoraModuleGPIOInit(UART_HandleTypeDef *huart)
{
  LoraModule *lm = get_loramodule_by_uart(huart);
  if (NULL == lm) {
    return;
  }
  
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
  } else {
    return;
  }
  
  lm->muart.uart = huart;
}

/*
*   设置lora模块的dma，dma接收发送缓冲区，datapool等参数
*/
void LoraModuleDMAInit(UART_HandleTypeDef *huart)
{
  LoraModule *lm = get_loramodule_by_uart(huart);
  if (NULL == lm) {
    return ;
  }
  /*
     *   UART1 作为Lora接收串口
     */
  if (USART1 == lm->muart.uart->Instance) {
    lm->muart.dma_rbuff_size      = UART1_RX_DMA_LEN;
    lm->muart.dma_sbuff_size      = 0;
    
    lm->muart.dma_rbuff           = &dma_uart1_rbuff[0];
    lm->muart.dma_sbuff           = NULL;
    
    lm->muart.uart_rx_hdma        = &hdma_usart1_rx;
    lm->muart.uart_tx_hdma        = NULL;
    /*      
    *       使能DMA接收，设置接收缓冲区
    *       { [buffer:1] [buffer:2] [buffer:3] }
    *       暂时设置三个buffer，DMA每次使用其中一个，当这个缓冲满后
    *       设置DMA接收缓冲区为下一个buffer，循环使用。
    *       单个缓冲区最大数据缓冲区为UART1_RX_DMA_LEN
    *       使能串口空闲中断与接收完成中断
    *       即接收到x个resp信息触发一次接收完成中断。
    *       这样当接收到单个resp信息时会触发空闲中断进行处理
    *       或是接收到多个resp时会触发接收完成中断进行处理
    *       当触发接收完成中断后，进入中断中取当前buffer中的指针位置，
    *       然后将指针指向下一个空闲的内存片，重新使能接收(HAL_UART_RECEIVE_DMA)
    */
    HAL_UART_Receive_DMA(lm->muart.uart, lm->muart.dma_rbuff, lm->muart.dma_rbuff_size);
    //使能空闲中断，仅对接收起作用
    __HAL_UART_ENABLE_IT(lm->muart.uart, UART_IT_IDLE);
    
    set_loramodule_workmode( lm, LoraMode_Normal);
    
    lm->muart.rxDataPool = DataPoolInit(UART1_RX_DATAPOOL_SIZE);
    if ( lm->muart.rxDataPool == NULL ) {
      DEBUG_ERROR("uart1 rxdatapool NULL\r\n");
    }
    
    lm->muart.txDataPool = NULL;
  }
  /*
  *   UART3 作为Lora发送串口
  */
  else if (USART3 == lm->muart.uart->Instance) {
    lm->muart.dma_rbuff_size  = 0;
    lm->muart.dma_sbuff_size  = UART3_TX_DMA_LEN;
    
    lm->muart.dma_rbuff       = NULL;
    lm->muart.dma_sbuff       = &dma_uart3_sbuff[0];
    
    lm->muart.uart_rx_hdma    = NULL;
    lm->muart.uart_tx_hdma    = &hdma_usart3_tx;
    /* UART2作为主发送串口，除了在配置模式，其余时间均不接收数据 */
    
    set_loramodule_workmode( lm, LoraMode_WakeUp);
    
    lm->muart.rxDataPool = NULL;
    lm->muart.txDataPool = NULL;

    
    g_ControlCmdListHead = initCmdList(g_ControlCmdListHead);
    if(NULL == g_ControlCmdListHead)
    {
      //error
    }
    g_NormalCmdListHead = initCmdList(g_NormalCmdListHead);
    if(NULL == g_NormalCmdListHead)
    {
      //error
    }    
  }
  else {
  }
}

/*
  *   Generate Lora Paramter
  *   根据参数生成数据：成功返回0，失败返回1
  */
static bool generate_loramodule_paramter(LoraPar *lp, uint8_t *buff, SaveType st)
{
  if (SAVE_IN_FLASH == st) {
    buff[0] = 0xC0;
  } else {
    buff[0] = 0xC2;
  }
  
  buff[1] = (uint8_t)(lp->u16Addr >> 8);
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
*   设置lora模块的参数
*/
static bool set_loramodule_paramter(LoraModule *lp)
{
#define LORA_SET_PAR_MAX_TIME 100
  uint8_t config[6] = {0};
  
  if(lp == NULL || &lp->paramter == NULL)
    return false;
  
  if (!generate_loramodule_paramter(&lp->paramter, config, SAVE_IN_FLASH))
    return false;
  
  //必要的延时函数
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
*     解析接受到的lora模块的参数
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
*       no = LoraReadPar:读取模块参数
*       no = LoraReadVer:读取模块版本信息
*       PS:在发送了读取版本号命令之后不能再发送命令
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
    //无校验
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
*   设置Lora模块参数
*/
void SetLoraParamter(UART_HandleTypeDef *huart, LoraPar *lp)
{
  
  LoraModule *lm = get_loramodule_by_uart(huart);
  
  uart_set_9600_8n1(lm);
  //进入配置模式,在配置模式下，不能带校验
  set_loramodule_workmode(lm, LoraMode_Sleep);
  //写入配置参数
  memcpy(&lm->paramter, (void *)lp, sizeof(LoraPar));
  
  while(!set_loramodule_paramter(lm));
  
  uart_set_baud_parity(lm);
  set_loramodule_workmode(lm, LoraMode_Normal);
  memset(&lm->paramter, 0, sizeof(LoraPar));
}

/*
*   读取Lora模块参数
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
*   向目标设备发送控制命令       
*   id:目标设备的地址
*   channel:目标设备所在信道
*   cmd:  发送给目标设备的命令
*   identify: 发送给目标设备的命令的唯一识别码
*/

static E_CmdType cmdType(uint8_t u8Cmd)
{
  if(CMD_NONE == u8Cmd)
  {
    return CMD_TYPE_NONE;
  }
  else if(DEVICE_HEART == u8Cmd || HW_ULTRA_GET == u8Cmd)
  {
    return CMD_TYPE_NORMAL;
  }
  else
  {
    return CMD_TYPE_CONTROL;
  }
}

bool insertCmdToList(uint16_t id, uint32_t u32Identify, uint8_t u8Cmd, uint32_t data)
{
  DeviceNode *device = getDevice(id);
  if(NULL == device)
  {
    return false;
  }
  
  if(CMD_TYPE_CONTROL == cmdType(u8Cmd))
  {
    if(!device->DeviceOnLine)
    {
      return false;
    }
    
    //1、先检查控制命令list中是否已经存在属于该id控制命令
    //存在:则返回false
    if(isExist(g_ControlCmdListHead, id))
    {
      return false;
    }

    //2、检查device list中该device是否有控制命令在执行
    //有:返回false
    if(CMD_TYPE_CONTROL == cmdType(device->u8CMD) && CMD_STATUS_IDLE != device->u8CMDSTATUS)
    {
      return false;
    }

    //3、1,2情况都不存在的情况下，将该命令插入控制命令的list中
    insertNode(g_ControlCmdListHead, id, u32Identify, u8Cmd, data);    

    //4、插入完该device的控制命令后，如果该设备online，则需要将该节点的普通命令删除
    delNode(g_NormalCmdListHead, id);

  }
  else
  {
    if(device->DeviceOnLine)
    {
      //1、先检查控制命令list中是否已经存在属于该id控制命令
      //存在:则返回false
      if(isExist(g_ControlCmdListHead, id))
      {
        return false;
      }
      
      //2、检查device list中该device是否有命令在执行
      //有:返回false
      
      if(CMD_STATUS_IDLE != device->u8CMDSTATUS)
      {
        return false;
      }
      //3、检查normal cmd list中是否已经添加了该device相关的cmd
      if(isExist(g_NormalCmdListHead, id))
      {
        return false;
      }
      
      //4、1,2,3情况都不存在的情况下，将该插入普通命令的list中
      insertNode(g_NormalCmdListHead, id, u32Identify, u8Cmd, data);
      
    }
    else
    {
      //offline情况下，只添加心跳命令
      if(DEVICE_HEART != u8Cmd)
      {
        return false;
      }

      if(isExist(g_NormalCmdListHead, id))
      {
        return false;
      }
      
      insertNode(g_NormalCmdListHead, id, u32Identify, u8Cmd, data);
    }
    
  }

  return true;
}

static bool getNextCmd(T_CmdList *cmd)
{
  if(!isNull(g_ControlCmdListHead))
  {
    //控制命令list不为空时，循环遍历控制命令list
    //当device offline时，需要将该device对应的控制命令删除
    //当device online时，取出满足以下条件的控制命令：
    //1、对应的设备目前处于idle状态
    //如果没有满足以上条件的控制命令，则取普通命令的第一条
    
    T_CmdList *pControlCmd = popFrontNode(g_ControlCmdListHead);
    T_CmdList *pNormalCmd = popFrontNode(g_NormalCmdListHead);
    

    while(NULL != pControlCmd)
    {
      DeviceNode *device = getDevice(pControlCmd->u16Id);
      if(NULL == device)
      {
        return false;

      }

      if(device->DeviceOnLine)
      {
        if(CMD_STATUS_IDLE != device->u8CMDSTATUS)
        {
          //1、对应的设备不处于idle状态,继续遍历
          pControlCmd = pControlCmd->next;
        }
        else
        {
          //1、对应的设备目前处于idle状态,找到cmd
          memcpy(cmd, pControlCmd, sizeof(T_CmdList));
          delNode(g_ControlCmdListHead, pControlCmd->u16Id);   

          return true;
        }
      }
      else
      {
        //当device offline时，需要将该device对应的控制命令删除
        T_CmdList *pNeedDel = pControlCmd;
        pControlCmd = pControlCmd->next;

        delNode(g_ControlCmdListHead, pNeedDel->u16Id);
        
      }

    }
  }

  //找不到合适的控制命令，发送普通命令
  T_CmdList *pNextCmd = popFrontNode(g_NormalCmdListHead);
  if (NULL == pNextCmd)
  {
    return false;
  }
  memcpy(cmd, pNextCmd, sizeof(T_CmdList));
  
  delNode(g_NormalCmdListHead, pNextCmd->u16Id);
  
  return true;
}

/*
*   根据串口获取对应的模块指针
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
*   中断函数：当被调用时会将对应的串口DMA缓冲区中的
*             数据复制到对应串口自己的数据池中。
*   只有uart1和uart2的空闲中断中会调用。
*/
bool CopyDataFromDMAHandler(UART_HandleTypeDef *huart)
{
  uint16_t len;
  uint16_t pos;
  UartModule *um = get_uartmodule(huart);
  
  //len 等于已经用于存放数据的长度(空间)
  len = (um->dma_rbuff_size - (uint16_t)__HAL_DMA_GET_COUNTER(um->uart_rx_hdma));
  
  //将dma缓冲区数据取出放入到数据处理的内存内
  //用len和pos来取出数据,分为几种情况
  //1：len=pos=0 没有数据
  //2：len > pos 可以出去从pos到len的数据
  //3：len < pos 说明产生了串口接收完成中断，在该缓冲区内数据存满了，需要取下一个缓冲区内数据
  //4: len = 0, pos > 0 产生数据接收完成中断，需要取出当前dma内的从pos到最后的所有数据
  
  if (len == um->pos)
    return true;
  
  if (( 0 == len ) && ( um->pos > 0)) {
    //数据接收完成中断，将数据写入pool中后，将pos重置。
    pos = um->pos;

    if (DataPoolWrite(um->rxDataPool, um->dma_rbuff + pos, um->dma_rbuff_size - pos )) {
      um->pos = 0;
      return true;
    }
  } else  if (len > um->pos) {
    //空闲中断，取出当前接收的数据，然后重新设置pos值
    pos = um->pos;
    //len是整个数据空间已有数据的总长度
    //pos是上一次取完数据后的指针位置
    //所有本次数据长度就是len-pos
    if (DataPoolWrite(um->rxDataPool, um->dma_rbuff + pos, len - pos)) { 
      um->pos += (len - pos);
      if( um->pos == um->dma_rbuff_size ) 
        um->pos = 0;
      return true;
    }
  } else { //len < um->pos && len != 0
    // impossible
  }
  
  return false;
}

/*
*   中断函数:用于DMA数据移动和重新使能dma接收功能
*   在串口接收完成中断中调用
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
*   Lora模块的数据处理
*/
void LoraModuleTask(void)
{
  uint8_t dat;
  uint8_t buff[SERVER_RESP_LEN];
  //接收数据的处理
  LoraModule *lm = &gLoraMR;
  DataPool *dp = lm->muart.rxDataPool;
  
  if (DataPoolGetNumByte(dp, &buff[0], SERVER_RESP_LEN)) {
    RespDataPacket *resp = (RespDataPacket *)buff;
    if (( LORA_MSG_HEAD == resp->u8Head ) && ( LORA_MSG_TAIL == resp->u8Tail )) {
      uint16_t crc = CRC16_IBM((uint8_t *)resp, SERVER_RESP_LEN -3);
      if ( crc == resp->u16Crc ) {
        DeviceNode *pDevice = getDevice(resp->u16Id);
        if(NULL != pDevice)
        {
          pDevice->u32LastTime = HAL_GetTick();
          pDevice->u32HeartTime = HAL_GetTick();
        }
        
        switch ( resp->u8Cmd ) {
          case DEVICE_ABNORMAL:
          case DEVICE_HEART:
          case HW_ULTRA_GET:
            //异常动作信息
            //如果是异常动作信息，需要设置identify为0
            SendCMDRespToList(resp->u16Id, resp->u8Cmd, 0, resp->u8Resp);
            break;
          default:
            //正常控制指令的响应信息
            SendCMDRespToList(resp->u16Id, resp->u8Cmd, resp->u32Identify, resp->u8Resp);
            break;
        }
      }
    }/*end of if (( LORA_MSG_HEAD == resp->u8Head ) && ( LORA_MSG_TAIL == resp->u8Tail ))*/
    else {
      //恢复取出来的数据
      DataPoolResume(dp, SERVER_RESP_LEN);
      //数据头指向下一个字节
      DataPoolGetByte(dp, &dat);
    }
  }
  
  //发送数据处理
  //每一条指令都需要与上一条指令间隔一定的时间才能发送成功
  //lora_send_timer 是用控制每一条指令的时间间隔
  UartModule *um = &gLoraMS.muart;
  
  if ( lora_send_timer == 0 ) {
    //set_loramodule_workmode(&gLoraMS, LoraMode_Normal);
    //set_loramodule_workmode(&gLoraMS, LoraMode_WakeUp);

    //当时间间隔够了还需要判断设备是否空闲
    if (get_loramodule_idle_flag(lm->muart.uart)) {
      //设备空闲了，则从数据池中提取出一条指令长度的信息

      T_CmdList cmd = { 0 };      
      if (getNextCmd(&cmd)) {

        DeviceNode *pDeviceNode = getDevice(cmd.u16Id);
        if(NULL == pDeviceNode)
        {
          return;
        }

        //构造发送的数据
        CmdDataPacket stPacket = {0};

        stPacket.u8IdHigh = (uint8_t)(cmd.u16Id >> 8);
        stPacket.u8IdLow = (uint8_t)(cmd.u16Id & 0x00ff);
        stPacket.u8Channel = cmd.u16Id % 30;

        stPacket.stData.u8Head = LORA_MSG_HEAD;
        if((CMD_MODIFY_DISTANCE == cmd.u8Cmd) || (CMD_MODIFY_SCANCYCLE == cmd.u8Cmd))
        {
          stPacket.stData.u8Len  = (uint8_t)cmd.u32Data;          
        }
        else
        {
          stPacket.stData.u8Len  = sizeof(CmdData);
        }
          
        stPacket.stData.u16Id  = cmd.u16Id;
        stPacket.stData.u8Cmd  = cmd.u8Cmd;
        stPacket.stData.u32Identify = cmd.u32Identify;
        stPacket.stData.u16Crc = CRC16_IBM((uint8_t *)(&(stPacket.stData)), sizeof(CmdData) - 3);
        stPacket.stData.u8Tail = LORA_MSG_TAIL;


        //通过DMA发送，重新设置时间间隔
        memcpy(um->dma_sbuff, &stPacket, sizeof(CmdDataPacket));
        HAL_UART_Transmit_DMA(um->uart, um->dma_sbuff, um->dma_sbuff_size);

        
        pDeviceNode->u32CmdStartTime = HAL_GetTick();
        
        pDeviceNode->u8CMDSTATUS = CMD_STATUS_RUN;
        pDeviceNode->u8CMD = cmd.u8Cmd;
        pDeviceNode->u8RESP = 0;
        pDeviceNode->u32Data= cmd.u32Data;
        pDeviceNode->u32Identify = cmd.u32Identify;
        pDeviceNode->u8CMDRetry++;

        lora_send_timer = 1;
      }      
    }
  }
}
