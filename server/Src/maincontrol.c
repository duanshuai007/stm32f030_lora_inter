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
    
    um->dma_rbuff_size    = UART2_RX_DMA_LEN;
    um->dma_sbuff_size    = UART2_TX_DMA_LEN;
    
    um->dma_rbuff = &uart2_dma_rbuff[0];
    um->dma_sbuff = &uart2_dma_sbuff[0];

    um->dataPool = DataPoolInit(UART2_RX_DATAPOOL_SIZE);
    if ( um->dataPool == NULL ) {
        PRINT("uart2 datapool null\r\n");
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
    case CMD_MOTOR_ABNORMAL:
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
//modify by liyongsheng begin
void UartSendRespToF405(UartModule *um, DeviceNode *devn)
{
  while(um->uart->gState != HAL_UART_STATE_READY);
  
  T_Resp_Info *resp = (T_Resp_Info *)um->dma_sbuff;
 
  if ( generalRespInfo(devn, resp) ) {
    if ( resp->crc == 0xe1 ) {
    printf("find 0xe1\r\n");
  } 
    HAL_UART_Transmit_DMA(um->uart, um->dma_sbuff, sizeof(T_Resp_Info));
  }
}

void UartSendOnLineToF405(UartModule *um, uint16_t id)
{
  while(um->uart->gState != HAL_UART_STATE_READY);
  
  T_Resp_Info *resp = (T_Resp_Info *)um->dma_sbuff;
  
  resp->resp_code = NODE_ONLINE;
  resp->resp_data.endpointId = id;
  resp->crc = crc8_chk_value((uint8_t *)resp, 5);
  if ( resp->crc == 0xe1 ) {
    printf("find 0xe1\r\n");
  } 
  HAL_UART_Transmit_DMA(um->uart, um->dma_sbuff, sizeof(T_Resp_Info));
}

void UartSendDeviceBusyToF405(UartModule *um, uint32_t identify)
{
//  while(HAL_UART_GetState(um->uart) & HAL_UART_STATE_READY != HAL_UART_STATE_READY);
  while(um->uart->gState != HAL_UART_STATE_READY);
  
  T_Resp_Info *resp = (T_Resp_Info *)um->dma_sbuff;

  resp->resp_code = NORMAL_BUSY;
  resp->resp_data.identity = identify;
  resp->crc = crc8_chk_value((uint8_t *)resp, 5);
  printf("BUSY\r\n");
  if ( resp->crc == 0xe1 ) {
    printf("find 0xe1\r\n");
  } 
  HAL_UART_Transmit_DMA(um->uart, um->dma_sbuff, sizeof(T_Resp_Info));
}

void UartSendCMDTimeOutToF405(UartModule *um, uint32_t identify)
{
  while(um->uart->gState != HAL_UART_STATE_READY);
  
  T_Resp_Info *resp = (T_Resp_Info *)um->dma_sbuff;
  
  resp->resp_code = ERROR_CMD_TIMEOUT;
  resp->resp_data.identity = identify;
  resp->crc = crc8_chk_value((uint8_t *)resp, 5);
  printf("CMD TimeOut\r\n");

  for(uint8_t i=0; i<6;i++)
  {
    printf("%02x ", um->dma_sbuff[i]);
  }
  printf("\r\n");
  
  HAL_UART_Transmit_DMA(um->uart, um->dma_sbuff, sizeof(T_Resp_Info));
}


/*
*   F405命令处理线程
*   F405只能控制由本机发送出去的设备ID，对于其他ID不进行操作
*/
void F405CmdProcess(UartModule *um)
{
  static uint8_t time = 0;
  
  DataPool * dp = um->dataPool;

  T_Control_Cmd cmd;
  
  while(DataPoolGetNumByte(dp, (uint8_t *)(&cmd), sizeof(T_Control_Cmd)))
  {
    uint8_t crc = crc8_chk_value((uint8_t *)(&cmd), 7);
    
    if(crc != cmd.crc)
    {
      PRINT("receive cmd for F405 error: crc check error, crc = %02x, cmd.crc = %02x\r\n", crc, cmd.crc);
      
      continue;
    }

    printf("F405:id:%04x, cmd:%d, identify:%08x\r\n",
           cmd.endpointId,cmd.action,cmd.identity);

    if ( CMD_DEVICE_REGISTER == cmd.action ) {
      //接收到f405的上线响应信息，设置对应的flash内的设备节点为上线成功状态
      FlashSetDeviceOnline(gFDList, cmd.endpointId);
    } else {
      set_device_of_list_cmd(gList, cmd.endpointId, cmd.action, cmd.identity);
    }
    //
    time++;
    if ( 2 == time ) {
      time = 0;
      break;
    }
  }
}
//modify by liyongsheng end

