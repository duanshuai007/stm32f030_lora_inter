#include "maincontrol.h"
#include "stdint.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "user_config.h"
#include "stdlib.h"
#include "crc16.h"
#include "flash_if.h"
#include "string.h"
#include "reboot.h"
#include "list.h"
#include "Lora.h"

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

static UartModule *gUartMx;      //f405�����豸�ṹ��
static uint8_t uart2_dma_rbuff[UART2_RX_DMA_LEN] = {0};
static uint8_t uart2_dma_sbuff[UART2_TX_DMA_LEN] = {0};
static bool enable_communication = true;
/*
*   ��ʼ����F405ͨ�ŵĴ�����ؽṹ��
*/
void UARTF405_Init(UART_HandleTypeDef *uart)
{
  gUartMx = (UartModule *)malloc(sizeof(UartModule));
  if(gUartMx == NULL) {
    DEBUG_ERROR("gUartMx == NULL\r\n");
    return;
  }
  
  gUartMx->pos = 0;
  gUartMx->uart = uart;
  
  gUartMx->uart_rx_hdma = &hdma_usart2_rx;
  gUartMx->uart_tx_hdma = &hdma_usart2_tx;
  
  gUartMx->dma_rbuff_size = UART2_RX_DMA_LEN;
  gUartMx->dma_sbuff_size = UART2_TX_DMA_LEN;
  
  gUartMx->dma_rbuff = &uart2_dma_rbuff[0];
  gUartMx->dma_sbuff = &uart2_dma_sbuff[0];
  
  gUartMx->rxDataPool = DataPoolInit(UART2_RX_DATAPOOL_SIZE);
  if ( gUartMx->rxDataPool == NULL ) {
    DEBUG_ERROR("uart2 rxdatapool null\r\n");
    free(gUartMx);
    return;
  }
  
  gUartMx->txDataPool = DataPoolInit(UART2_TX_DATAPOOL_SIZE);
  if ( gUartMx->txDataPool == NULL ) {
    DEBUG_ERROR("uart2 txdatapool null\r\n");
    free(gUartMx->rxDataPool);
    free(gUartMx);
    return;
  }
  
  HAL_UART_Receive_DMA(gUartMx->uart, gUartMx->dma_rbuff, gUartMx->dma_rbuff_size);
  //ʹ�ܿ����жϣ����Խ���������
//  __HAL_UART_ENABLE_IT(gUartMx->uart, UART_IT_IDLE);
}

void HAL_UART_Receive_DMA_Reset(void)
{  
  HAL_UART_Receive_DMA(gUartMx->uart, gUartMx->dma_rbuff, gUartMx->dma_rbuff_size);
}

/*
*   ����resp��Ϣ
*/
static bool generalRespInfo(DeviceNode *pdev, T_Resp_Info *pRespInfo)
{
  if(NULL == pdev || NULL == pRespInfo)
  {
    DEBUG_ERROR("generalRespInfo failed : pdev = %p, pRespInfo = %p\n", pdev, pRespInfo);
    return false;
  }
  
  pRespInfo->endpointId = pdev->u16ID;
  pRespInfo->identity= pdev->u32Identify;
  pRespInfo->data.value = 0;
  
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
    case 99:
      pRespInfo->resp_code = NORMAL_MOTOR_RUNNING;
      break;
    case 5:   //��������⵽�ϰ����ִ��̧�����
      pRespInfo->resp_code = NORMAL_HAVE_CAR;
      break;
    case 6:   //�������Ĵ��ڲ��ܽ��յ���ȷ��ʽ���ݣ���ִ��̧�����
      pRespInfo->resp_code = NORMAL_CAR_CHECK_ERROR;
      break;
    default:
      DEBUG_ERROR("generalRespInfo error : pdev->u8CMD = %d, pdev->u8Resp = %d\n", pdev->u8CMD, pdev->u8RESP);
      return false;
    }
    
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
      DEBUG_ERROR("generalRespInfo error : pdev->u8Cmd = %d, pdev->u8Resp = %d\n", pdev->u8CMD, pdev->u8RESP);          
      return false;
    }
    
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
      DEBUG_ERROR("generalRespInfo error : pdev->u8Cmd = %d, pdev->u8Resp = %d\n", pdev->u8CMD, pdev->u8RESP);          
      return false;
    }
    
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
      DEBUG_ERROR("generalRespInfo error : pdev->u8Cmd = %d, pdev->u8Resp = %d\n", pdev->u8CMD, pdev->u8RESP);          
      return false;
    }
    
    break;


  case CMD_MODIFY_DISTANCE:
    switch(pdev->u8RESP)
    {
    case 0:
      pRespInfo->resp_code = SOUND_DIST_SET_SUCCESS;
      break;
    case 1:
      pRespInfo->resp_code = SOUND_DIST_SET_FAILED;
      break;
    default:
      DEBUG_ERROR("generalRespInfo error : pdev->u8Cmd = %d, pdev->u8Resp = %d\n", pdev->u8CMD, pdev->u8RESP);          
      return false;
    }
    
    break;

  case CMD_GET_DISTANCE:
    pRespInfo->resp_code = SOUND_DIST_GET_SUCCESS;
    pRespInfo->data.distance = pdev->u8RESP;
    
    break;

  case CMD_MODIFY_SCANCYCLE:
    switch(pdev->u8RESP)
    {
    case 0:
      pRespInfo->resp_code = SOUND_SCANCYCLE_SET_SUCCESS;
      break;
    case 1:
      pRespInfo->resp_code = SOUND_SCANCYCLE_SET_FAILED;
      break;
    default:
      DEBUG_ERROR("generalRespInfo error : pdev->u8Cmd = %d, pdev->u8Resp = %d\n", pdev->u8CMD, pdev->u8RESP);          
      return false;
    }
    
    break;

  case CMD_GET_SCANCYCLE:
    
    pRespInfo->resp_code = SOUND_SCANCYCLE_GET_SUCCESS;
    pRespInfo->data.scanCycle = pdev->u8RESP;
    
    break;
    
  case CMD_BEEP_STATUS_GET:
    switch(pdev->u8RESP)
    {
    case 0:
      pRespInfo->resp_code = NORMAL_BEEP_STATUS_CLOSED;
      break;
    case 1:
      pRespInfo->resp_code = NORMAL_BEEP_STATUS_OPEN;
      break;
    default:
      DEBUG_ERROR("generalRespInfo error : pdev->u8Cmd = %d, pdev->u8Resp = %d\n", pdev->u8CMD, pdev->u8RESP);          
      return false;
    }
        
    break;
  case CMD_ADC_GET:
//    if(pdev->u8RESP > 100)
//    {
//      DEBUG_ERROR("generalRespInfo error : pdev->u8Cmd = %d, pdev->u8Resp = %d\n", pdev->u8CMD, pdev->u8RESP);          
//      return false;        
//    }
//  
    if(pdev->u8RESP > 100)
    {
      pRespInfo->resp_code = 100;
    }
    else
    {
      pRespInfo->resp_code = pdev->u8RESP;
    }
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
      DEBUG_ERROR("generalRespInfo error : pdev->u8Cmd = %d, pdev->u8Resp = %d\n", pdev->u8CMD, pdev->u8RESP);          
      return false;
    }
    break;
    
  default:
    DEBUG_ERROR("generalRespInfo error : pdev->u8Cmd = %d\n", pdev->u8CMD);          
    return false;
  } /* end switch(pdev->u8CMD)*/
  
  pRespInfo->crc = crc8_chk_value((uint8_t *)pRespInfo, 11);
  
  return true;
}

/*
*   ��F405����ָ����Ӧ��Ϣ���ɹ�����true��ʧ�ܷ���false
*/
bool UartSendRespToF405(DeviceNode *devn)
{
  UartModule *um = gUartMx;
  T_Resp_Info resp;
  
  if (generalRespInfo(devn, &resp)) {
    if (DataPoolWrite(um->txDataPool, (uint8_t *)&resp, sizeof(T_Resp_Info))) {
      return true;
    }
  }
  
  return false;
}

/*
*   ��F405����֪ͨ��Ϣ
*/
void UartSendOnlineMSGToF405(DeviceNode *device)
{
  UartModule *um = gUartMx;
  T_Resp_Info resp;
  
  resp.resp_code = NODE_ONLINE;
  resp.endpointId = device->u16ID;
  resp.identity = 0;
  resp.data.online_data.endpointId = device->u16ID;  
  resp.data.online_data.EPStatus = device->u8EpStatus;

  if(EP_STATUS_DOWN != device->u8EpStatus)
  {
    resp.data.online_data.haveCarFlg = 255;
  }
  else
  {
    if(STATUS_HAVE_CAR == device->u8HaveCarFlg)
    {
      resp.data.online_data.haveCarFlg = 1;
    }
    else if(STATUS_NO_HAVE_CAR == device->u8HaveCarFlg)
    {
      resp.data.online_data.haveCarFlg = 0;
    }
    else if(STATUS_TROUBLE == device->u8HaveCarFlg)
    {
      resp.data.online_data.haveCarFlg = 2;
    }
    else
    {
      resp.data.online_data.haveCarFlg = 255;
    }    
  }

  resp.crc = crc8_chk_value((uint8_t *)&resp, 11);
  DataPoolWrite(um->txDataPool, (uint8_t *)&resp, sizeof(T_Resp_Info));

}

/*
*   ��F405��������֪ͨ��Ϣ
*/
void UartSendAutoMSGToF405(uint8_t status, uint16_t id, uint32_t identify)
{
  UartModule *um = gUartMx;
  T_Resp_Info resp;
  
  resp.resp_code = status;
  resp.endpointId = id;
  resp.identity = identify;
  resp.data.value = 0;
  
  resp.crc = crc8_chk_value((uint8_t *)&resp, 11);
  DataPoolWrite(um->txDataPool, (uint8_t *)&resp, sizeof(T_Resp_Info));
}

void UartSendGetDistMSGToF405(uint8_t status, uint32_t identify, uint16_t sound_dist)
{
  UartModule *um = gUartMx;
  T_Resp_Info resp;
  
  resp.resp_code = status;
  resp.endpointId = 0;
  resp.identity = identify;
  resp.data.distance = sound_dist;
  
  resp.crc = crc8_chk_value((uint8_t *)&resp, 11);
  DataPoolWrite(um->txDataPool, (uint8_t *)&resp, sizeof(T_Resp_Info));
}


/*
*   ��F405���͹̼��汾��Ϣ
*/
void UartSendVerMSGToF405(uint8_t resp_code, uint8_t *version)
{
  UartModule *um = gUartMx;
  T_Resp_Info resp;
  
  resp.resp_code = resp_code;
  resp.endpointId = 0;
  resp.identity = 0;
  memcpy(resp.data.version, version, VERSION_LEN);
  
  resp.crc = crc8_chk_value((uint8_t *)&resp, 11);
  DataPoolWrite(um->txDataPool, (uint8_t *)&resp, sizeof(T_Resp_Info));
}


/*
*   ��ȡuart module�ṹ��
*/
UartModule *GetF405UartModule(UART_HandleTypeDef *huart)
{
  if ( huart->Instance == gUartMx->uart->Instance ) {
    return gUartMx;
  } else 
    return NULL;
}

/*
*   �жϺ���:F405���պ���
*   �ڴ��ڽ�������ж��б�����
*/
void F405ReveiveHandler(UART_HandleTypeDef *huart)
{
//  if ( huart->Instance != gUartMx->uart->Instance ) 
//    return;
  
//  CopyDataFromDMAHandler(gUartMx->uart);
//  HAL_UART_Receive_DMA(gUartMx->uart, 
//                       gUartMx->dma_rbuff,
//                       gUartMx->dma_rbuff_size);

UartModule *um = GetF405UartModule(huart);

DataPoolWrite(um->rxDataPool, um->dma_rbuff, um->dma_rbuff_size);

}

void closeF405Communication(void)
{
  enable_communication = false;
}
/*
*   F405������߳�
*   F405ֻ�ܿ����ɱ������ͳ�ȥ���豸ID����������ID�����в���
*/
static bool isCmdSupport(uint32_t cmd)
{
  if(cmd > CMD_NONE && cmd <= CMD_ADC_GET)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void F405Task(void)
{
  DataPool *dp = gUartMx->rxDataPool;
  DataPool *tdp = gUartMx->txDataPool;
  
  T_Control_Cmd cmd;
  
  //F405������մ���
  if ( DataPoolGetNumByte(dp, (uint8_t *)(&cmd), sizeof(T_Control_Cmd)) ) {
    
    if (enable_communication == true) {
      uint8_t crc = crc8_chk_value((uint8_t *)(&cmd), 11);
      
      if(crc != cmd.crc) {
        DEBUG_ERROR("receive cmd for F405 error: crc check error, crc = %02x, cmd.crc = %02x\r\n", crc, cmd.crc);
        return;
      }
      
      DEBUG_INFO("rec from F405:id:%04x, cmd:%d, identify:%08x\r\n",
           cmd.endpointId,cmd.action,cmd.identify);
      
      if ( CMD_GET_ALL_NODE == cmd.action ) {
        //���յ�f405��������Ӧ��Ϣ�������������߽ڵ��405
        SendALLDeviceNodeToF405();
      } 
      else if(CMD_UPDATE == cmd.action)
      {
        T_BootConfig stBootConfig;
        memcpy(&stBootConfig, (char *)BOOT_CONFIG_ADDR, sizeof(T_BootConfig));
        if(0 != strcmp((char *)stBootConfig.ucFWVer, (char *)cmd.data.verName))
        {
          stBootConfig.ucBootType = TYPE_UART2;
          stBootConfig.ucTryTimes = 0;
          stBootConfig.ucBootSeccessFlg = 0;
          strcpy((char *)stBootConfig.ucNewFWVer, (char *)cmd.data.verName);

          FLASH_If_Erase(BOOT_CONFIG_ADDR);
          FLASH_If_Write(BOOT_CONFIG_ADDR, (uint32_t*)(&stBootConfig), sizeof(T_BootConfig));

          NVIC_SystemReset();
        }
        else
        {
          UartSendVerMSGToF405(GET_RUN_BIN_FIN, CURRENT_FIRMWARE_VERSION);               
        }
      }
      else if(CMD_ADD_ENDPOINT == cmd.action)
      {        
        AddDeviceToFlash(cmd.data.endpointId);
        AddDeviceToList(cmd.data.endpointId, false);

        UartSendAutoMSGToF405(ENDPOINT_ADD_SUCCESS, cmd.endpointId, cmd.identify);        
      }
      else if(CMD_DEL_ENDPOINT == cmd.action)
      {
        uint16_t serverID = READ_FLASH_HALFWORD(FLASH_SAVE_SECTION);
        uint16_t recv_channel = READ_FLASH_HALFWORD(FLASH_SAVE_SECTION+2);
        uint16_t sound_distance = READ_FLASH_HALFWORD(FLASH_SAVE_SECTION+4);
        
        FLASH_If_Erase(FLASH_SAVE_SECTION);
        HAL_Delay(100);
        
        delete_device_from_list(cmd.data.endpointId);

        FLASH_If_Write_Halfword(FLASH_SAVE_SECTION, serverID);
        HAL_Delay(100);
        
        FLASH_If_Write_Halfword(FLASH_SAVE_SECTION + 2, recv_channel);
        HAL_Delay(100);
        
        FLASH_If_Write_Halfword(FLASH_SAVE_SECTION + 4, sound_distance);
        HAL_Delay(100);
        
        writeAllDeviceToFlash();

        UartSendAutoMSGToF405(ENDPOINT_DEL_SUCCESS, cmd.endpointId, cmd.identify); 
        
      }
      else if(CMD_MODIFY_CHANNEL == cmd.action)
      {
        uint16_t serverID = READ_FLASH_HALFWORD(FLASH_SAVE_SECTION);
        uint16_t recv_channel = cmd.data.channel;
        uint16_t sound_distance = READ_FLASH_HALFWORD(FLASH_SAVE_SECTION+4);
        

        FLASH_If_Erase(FLASH_SAVE_SECTION);
        HAL_Delay(100);
        
        FLASH_If_Write_Halfword(FLASH_SAVE_SECTION, serverID);
        HAL_Delay(100);
        
        FLASH_If_Write_Halfword(FLASH_SAVE_SECTION + 2, recv_channel);
        HAL_Delay(100);
        
        FLASH_If_Write_Halfword(FLASH_SAVE_SECTION + 4, sound_distance);
        HAL_Delay(100);

        writeAllDeviceToFlash();
        
        UartSendAutoMSGToF405(CHANNEL_MODIFY_SUCCESS, cmd.endpointId, cmd.identify); 
        
      }
      else if(CMD_MODIFY_SERVER_DISTANCE == cmd.action)
      {
        uint16_t serverID = READ_FLASH_HALFWORD(FLASH_SAVE_SECTION);
        uint16_t recv_channel = READ_FLASH_HALFWORD(FLASH_SAVE_SECTION+2);
        uint16_t sound_distance = cmd.data.disance;
        
        FLASH_If_Erase(FLASH_SAVE_SECTION);
        HAL_Delay(100);
        
        FLASH_If_Write_Halfword(FLASH_SAVE_SECTION, serverID);
        HAL_Delay(100);
        
        FLASH_If_Write_Halfword(FLASH_SAVE_SECTION + 2, recv_channel);
        HAL_Delay(100);
        
        FLASH_If_Write_Halfword(FLASH_SAVE_SECTION + 4, sound_distance);
        HAL_Delay(100);

        writeAllDeviceToFlash();

        resetSoundCheckDist();
        
        UartSendAutoMSGToF405(SOUND_DIST_SET_SUCCESS, cmd.endpointId, cmd.identify);         
      }
      else if(CMD_GET_SERVER_DISTANCE == cmd.action)
      {
        UartSendGetDistMSGToF405(SOUND_DIST_GET_SUCCESS, cmd.identify, READ_FLASH_HALFWORD(FLASH_SAVE_SECTION+4));
      }      
      else {

        if(isCmdSupport(cmd.action))
        {
          if (SendCMDToList(cmd.endpointId, cmd.action, cmd.identify, cmd.data.value)  != true) {
            //send device not exist msg to f405
            UartSendAutoMSGToF405(NODE_NOTEXIST, cmd.endpointId, cmd.identify);
          }
        }
        else
        {
          UartSendAutoMSGToF405(NODE_NO_SUPPORT_CMD, cmd.endpointId, cmd.identify);
          
        }
      }
  
    }
  }
  //F405�������ݴ���
  if ( gUartMx->uart->gState == HAL_UART_STATE_READY ) {
    if ( DataPoolGetNumByte(tdp, gUartMx->dma_sbuff, sizeof(T_Resp_Info))) {
      DEBUG_INFO("send resp to f405, resp=%d\r\n",gUartMx->dma_sbuff[0]);
      HAL_UART_Transmit_DMA(gUartMx->uart, gUartMx->dma_sbuff, sizeof(T_Resp_Info));
    }
  }
}
