#include "maincontrol.h"
#include "stdint.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_uart.h"
#include "crc16.h"
#include "user_config.h"
#include "list.h"
#include "string.h"
#include "stdlib.h"
#include "lora_datapool.h"
#include "flash.h"

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

extern Device   gDevice;
extern List     *gList;
extern FLASHDeviceList *gFDList;

extern uint8_t uart2_dma_rbuff[UART2_RX_DMA_LEN];
extern uint8_t uart2_dma_sbuff[UART2_TX_DMA_LEN];

void UARTF405_Init(UartModule *um, UART_HandleTypeDef *uart)
{
  um->pos = 0;
  um->uart = uart;
  
  um->uart_rx_hdma = &hdma_usart2_rx;
  um->uart_tx_hdma = &hdma_usart2_tx;
  
  um->dma_rbuff_size = UART2_RX_DMA_LEN;
  um->dma_sbuff_size = UART2_TX_DMA_LEN;
  
  um->dma_rbuff = &uart2_dma_rbuff[0];
  um->dma_sbuff = &uart2_dma_sbuff[0];
  
  um->rxDataPool = DataPoolInit(UART2_RX_DATAPOOL_SIZE);
  if ( um->rxDataPool == NULL ) {
    PRINT("uart2 rxdatapool null\r\n");
  }
  
  um->txDataPool = DataPoolInit(UART2_TX_DATAPOOL_SIZE);
  if ( um->txDataPool == NULL ) {
    PRINT("uart2 txdatapool null\r\n");
  }
  
  HAL_UART_Receive_DMA(um->uart, um->dma_rbuff, um->dma_rbuff_size);
  //使能空闲中断，仅对接收起作用
  __HAL_UART_ENABLE_IT(um->uart, UART_IT_IDLE);
}

//generate resp info
bool generalRespInfo(DeviceNode *pdev, T_Resp_Info *pRespInfo)
{
  if(NULL == pdev || NULL == pRespInfo)
  {
    PRINT("generalRespInfo failed : pdev = %p, pRespInfo = %p\n", pdev, pRespInfo);
    return false;
  }
  
  switch(pdev->u8CMD)
  {
  case CMD_MOTOR_UP:
  case CMD_MOTOR_DOWN:
    switch(pdev->u8RESP)
    {
    case 0:
      pRespInfo->resp_code = NORMAL_SUCCESS;
      break;
    case 1:
      pRespInfo->resp_code = NORMAL_DOWN;
      break;
    case 2:
      pRespInfo->resp_code = NORMAL_FORWARD;
      break;
    case 3:
      pRespInfo->resp_code = NORMAL_UP;
      break;
    case 4:
      pRespInfo->resp_code = NORMAL_BACK;
      break;
    case 9:
      pRespInfo->resp_code = NORMAL_BUSY;
      break;
    default:
      PRINT("generalRespInfo error : pdev->u8CMD = %d, pdev->u8Resp = %d\n", pdev->u8CMD, pdev->u8RESP);
      return false;
    }
    
    pRespInfo->resp_data.identity = pdev->u32Identify;
    break;
  case CMD_MOTOR_STATUS_GET:
    switch(pdev->u8RESP)
    {
    case 1:
      pRespInfo->resp_code = NORMAL_DOWN;
      break;
    case 2:
      pRespInfo->resp_code = NORMAL_FORWARD;
      break;
    case 3:
      pRespInfo->resp_code = NORMAL_UP;
      break;
    case 4:
      pRespInfo->resp_code = NORMAL_BACK;
      break;
    case 9:
      pRespInfo->resp_code = NORMAL_BUSY;
      break;
    default:
      PRINT("generalRespInfo error : pdev->u8Cmd = %d, pdev->u8Resp = %d\n", pdev->u8CMD, pdev->u8RESP);          
      return false;
    }
    
    pRespInfo->resp_data.identity = pdev->u32Identify;
    break;
  case CMD_BEEP_ON:
    switch(pdev->u8RESP)
    {
    case 0:
      pRespInfo->resp_code = NORMAL_SUCCESS;
      break;
    case 1:
      pRespInfo->resp_code = NORMAL_BEEP_OPEN_FAILED;
      break;
    default:
      PRINT("generalRespInfo error : pdev->u8Cmd = %d, pdev->u8Resp = %d\n", pdev->u8CMD, pdev->u8RESP);          
      return false;
    }
    pRespInfo->resp_data.identity = pdev->u32Identify;
    
    break;
  case CMD_BEEP_OFF:
    switch(pdev->u8RESP)
    {
    case 0:
      pRespInfo->resp_code = NORMAL_SUCCESS;
      break;
    case 1:
      pRespInfo->resp_code = NORMAL_BEEP_CLOSE_FAILED;
      break;
    default:
      PRINT("generalRespInfo error : pdev->u8Cmd = %d, pdev->u8Resp = %d\n", pdev->u8CMD, pdev->u8RESP);          
      return false;
    }
    pRespInfo->resp_data.identity = pdev->u32Identify;
    
    break;
  case CMD_BEEP_STATUS_GET:
    switch(pdev->u8RESP)
    {
    case 0:
      pRespInfo->resp_code = NORMAL_BEEP_STATUS_OPEN;
      break;
    case 1:
      pRespInfo->resp_code = NORMAL_BEEP_STATUS_CLOSED;
      break;
    default:
      PRINT("generalRespInfo error : pdev->u8Cmd = %d, pdev->u8Resp = %d\n", pdev->u8CMD, pdev->u8RESP);          
      return false;
    }
    
    pRespInfo->resp_data.identity = pdev->u32Identify;
    
    break;
  case CMD_ADC_GET:
    if(pdev->u8RESP > 100)
    {
      PRINT("generalRespInfo error : pdev->u8Cmd = %d, pdev->u8Resp = %d\n", pdev->u8CMD, pdev->u8RESP);          
      return false;        
    }
    
    pRespInfo->resp_code = pdev->u8RESP;
    pRespInfo->resp_data.identity = pdev->u32Identify;
    
    break;
  case DEVICE_ABNORMAL:
    switch(pdev->u8RESP)
    {
    case 1:
      pRespInfo->resp_code = INTERUPT_DOWN;
      break;
    case 2:
      pRespInfo->resp_code = INTERUPT_FORWARD;
      break;
    case 3:
      pRespInfo->resp_code = INTERUPT_UP;
      break;
    case 4:
      pRespInfo->resp_code = INTERUPT_BACK;
      break;
    default:
      PRINT("generalRespInfo error : pdev->u8Cmd = %d, pdev->u8Resp = %d\n", pdev->u8CMD, pdev->u8RESP);          
      return false;
    }
    
    pRespInfo->resp_data.endpointId = pdev->u16ID;
    break;
  default:
    PRINT("generalRespInfo error : pdev->u8Cmd = %d\n", pdev->u8CMD);          
    return false;
  } /* end switch(pdev->u8CMD)*/
  
  pRespInfo->crc = crc8_chk_value((uint8_t *)pRespInfo, 5);
  
  return true;
}

void UartSendRespToF405(UartModule *um, DeviceNode *devn)
{
  T_Resp_Info resp;
  if ( generalRespInfo(devn, &resp) ) {
    DataPoolWrite(um->txDataPool, (uint8_t *)&resp, sizeof(T_Resp_Info));
  }
}

void UartSendOnLineToF405(UartModule *um, uint16_t id)
{ 
  PRINT("dev[%04x] online\r\n", id);
  T_Resp_Info resp;
  resp.resp_code = NODE_ONLINE;
  resp.resp_data.endpointId = id;
  resp.crc = crc8_chk_value((uint8_t *)&resp, 5);
  DataPoolWrite(um->txDataPool, (uint8_t *)&resp, sizeof(T_Resp_Info));
}

void UartSendOffLineToF405(UartModule *um, uint16_t id)
{
  PRINT("dev[%04x] offline\r\n", id);
  T_Resp_Info resp;
  resp.resp_code = NODE_OFFLINE;
  resp.resp_data.endpointId = id;
  resp.crc = crc8_chk_value((uint8_t *)&resp, 5);
  DataPoolWrite(um->txDataPool, (uint8_t *)&resp, sizeof(T_Resp_Info));
}

void UartSendDeviceBusyToF405(UartModule *um, uint32_t identify)
{
  T_Resp_Info resp;
  resp.resp_code = NORMAL_BUSY;
  resp.resp_data.identity = identify;
  resp.crc = crc8_chk_value((uint8_t *)&resp, 5);
  DataPoolWrite(um->txDataPool, (uint8_t *)&resp, sizeof(T_Resp_Info));
}

static void send_all_node_to_f405(UartModule *um, FLASHDeviceList *list)
{
  SaveMSG *sm = list->Head->next;
  
  while ( sm ) {
    UartSendOnLineToF405(um, sm->u16DeviceID);
    sm = sm->next;
  }
}

/*
*   F405命令处理线程
*   F405只能控制由本机发送出去的设备ID，对于其他ID不进行操作
*/
void F405CmdProcess(UartModule *um)
{
  DataPool *dp = um->rxDataPool;
  DataPool *tdp = um->txDataPool;
  
  T_Control_Cmd cmd;
  
  if ( DataPoolGetNumByte(dp, (uint8_t *)(&cmd), sizeof(T_Control_Cmd)) ) {
    
    uint8_t crc = crc8_chk_value((uint8_t *)(&cmd), 7);
    
    if(crc != cmd.crc)
    {
      PRINT("receive cmd for F405 error: crc check error, crc = %02x, cmd.crc = %02x\r\n", crc, cmd.crc);
      return;
    }
    
    PRINT("rec from F405:id:%04x, cmd:%d, identify:%08x\r\n",
           cmd.endpointId,cmd.action,cmd.identity);
    
    if ( CMD_GET_ALL_NODE == cmd.action ) {
      //接收到f405的上线响应信息，发送所有在线节点给405
      send_all_node_to_f405(um, gFDList);
    } else {
      set_device_of_list_cmd(gList, cmd.endpointId, cmd.action, cmd.identity);
    }
  }
  
  if ( um->uart->gState == HAL_UART_STATE_READY ) {
    if ( DataPoolGetNumByte(tdp, um->dma_sbuff, sizeof(T_Resp_Info))) {
      //如果池中有6个字节，则复制到
      PRINT("send resp to f405, resp=%d\r\n",um->dma_sbuff[0]);
      
      HAL_UART_Transmit_DMA(um->uart, um->dma_sbuff, sizeof(T_Resp_Info));
    }
  }
}
