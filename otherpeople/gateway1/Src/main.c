
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "debuglog.h"
#include "at_command.h"
#include "pub_enum.h"
#include "flash_if.h"
#include "md5sum.h"
#include "stdarg.h"
#include "cjson.h"
#include "response.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


#define VERSION_LEN 4
#pragma pack (1)
typedef struct
{
  uint16_t endpointId;
  uint8_t EPStatus;
  uint8_t haveCarFlg;
}Child_Online_Data;


typedef union
{
  uint32_t value;
  uint8_t version[VERSION_LEN];      
  Child_Online_Data online_data;
  uint16_t endpointId;
  uint16_t channel; 
  uint8_t distance; 
  uint8_t scanCycle;   
}U_Resp_Data;

typedef struct 
{
  uint8_t resp_code;
  uint32_t identity;
  uint16_t endpointId;
  U_Resp_Data data;
  uint8_t crc;
}T_Resp_Info;

typedef union
{
  uint32_t value;
  uint8_t verName[VERSION_LEN];    
  uint16_t endpointId;
  uint16_t channel;
  uint8_t disance;      
  uint8_t scanCycle;  
}U_Cmd_Data;

typedef struct 
{
  uint32_t identify;
  uint16_t endpointId;
  uint8_t action;
  U_Cmd_Data data;
  uint8_t crc;
}T_Control_Cmd;

#pragma pack ()

#define MQTT_SEND_SUCCESS 0
#define MQTT_SEND_BUSY 1
#define MQTT_SEND_INVALID_TOPIC_TYPE 2


typedef enum
{
  MQTT_TOPIC_TYPE_INVALID,
  MQTT_TOPIC_TYPE_CHILD_ONLINE,
  MQTT_TOPIC_TYPE_CHILD_OFFLINE,
  MQTT_TOPIC_TYPE_ACTION_RSP,
  MQTT_TOPIC_TYPE_INTERRUPT_RSP,
  MQTT_TOPIC_TYPE_FIRMWARE_UPDATE,
  MQTT_TOPIC_TYPE_HAVE_CAR
}MQTT_TOPIC_TYPE;

#define MD5_LEN 32
#define PATH_NAME_LEN 128
#define BOARD_NAME_LEN 16

#define UPDATE_TYPE_STM32F405 0
#define UPDATE_TYPE_STM32F103 1
#define UPDATE_TYPE_ENDPOINT_ADD 2
#define UPDATE_TYPE_ENDPOINT_DEL 3
#define UPDATE_TYPE_MODIFY_GWID 4
#define UPDATE_TYPE_MODIFY_SERVER_CH 5
#define UPDATE_TYPE_MODIFY_SOUND_DIST 6
#define UPDATE_TYPE_GET_SOUND_DIST    7
#define UPDATE_TYPE_MODIFY_SOUND_CYCLE 8
#define UPDATE_TYPE_GET_SOUND_CYCLE    9


#define BOARD_NAME_STM32F405 "STM32F405"
#define BOARD_NAME_STM32F103 "STM32F103"


typedef struct
{
  uint32_t uiUpdateStep;
  uint32_t uiFWType;  
  uint8_t  ucPath[PATH_NAME_LEN];
  uint8_t  ucBoardName[BOARD_NAME_LEN];
  uint32_t uiMd5Len;
  uint32_t uiListDataSize;
  uint32_t uiPerDataSize;
  uint8_t  ucBinMd5[MD5_LEN];    
  uint32_t uiDLFinLen;
  uint32_t uiTotalLen;
  uint8_t  ucFWVer[VERSION_LEN];
  
}T_FWUpdateInfo;

T_FWUpdateInfo g_stFWUpdateInfo = {0};

typedef struct 
{
  uint32_t uiUpdateVerAddr;
  uint32_t uiRecoveryVerAddr;
  
  uint32_t uiUpdateVerDataTotalLen;  
  uint32_t uiRecoveryVerDataLen;
  
  uint8_t ucTryTimes;
  uint8_t ucNeedUpdateFlg;
  uint8_t ucBootSeccessFlg;  
  uint8_t ucRev[1];
  
  uint32_t uiF103RunRomAddr;
  uint32_t uiF103UpdateRomAddr;

  uint8_t ucFWVer[VERSION_LEN];
  uint8_t ucNewFWVer[VERSION_LEN];

}T_BootConfig;

T_BootConfig g_stBootConfig;

typedef struct
{
  uint8_t  ucBinMd5[MD5_LEN];    
  uint8_t  ucFWVer[VERSION_LEN];
  uint32_t uiTotalLen;  
}T_FWF103Head;


uint8_t g_topicType = MQTT_TOPIC_TYPE_INVALID;
#define MQTT_MAX_SEND_PAYLOAD_SIZE 256
uint8_t g_mqttSndPayload[MQTT_MAX_SEND_PAYLOAD_SIZE];
uint32_t g_sndStartTick = 0;

#define DMA_RX_BUFFER_SIZE          64
uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];
 
#define UART_BUFFER_SIZE            1280
uint8_t UART_Buffer[UART_BUFFER_SIZE];

#define DMA_TX_BUFFER_SIZE          256
uint8_t DMA_TX_Buffer[DMA_TX_BUFFER_SIZE];

#define DMA2_RX_BUFFER_SIZE          (sizeof(T_Resp_Info))
uint8_t DMA2_RX_Buffer[DMA2_RX_BUFFER_SIZE];
 
#define UART2_BUFFER_SIZE            1500
uint8_t *UART2_Buffer;

#define DMA2_TX_BUFFER_SIZE          (sizeof(T_Control_Cmd))
uint8_t DMA2_TX_Buffer[DMA2_TX_BUFFER_SIZE];

#define UART2_TX_BUFFER_SIZE          1200
uint8_t *UART2_TX_Buffer;

#define RET_POS_INVALID 0xFFFFFFFF

uint32_t Uart1_Read = 0;
volatile uint32_t Uart1_Write = 0;

uint32_t Uart2_Read = 0;
volatile uint32_t Uart2_Write = 0;

uint32_t Uart2_Send_Read = 0;
volatile uint32_t Uart2_Send_Write = 0;


uint32_t g_retBegin = RET_POS_INVALID;
uint32_t g_retEnd = RET_POS_INVALID;
volatile uint8_t g_uart1Phase = MQTT_INIT; // init and send
volatile uint32_t g_startTick = 0;

uint8_t g_simCardId[24] = {"WM_"}; // prefix + 20 sim card id

#define  MQTT_CHANNEL_LENGTH 32
uint8_t mqtt_channel[32] = {0}; 

uint32_t g_rcvTopicLen = 0;
uint32_t g_rcvPayloadLen = 0;
uint8_t g_uart1RcvPhase = MQTT_RCV_INIT;
uint8_t g_uart1RcvTopicType = MQTT_RCV_TOPIC_CONTROL;

uint8_t g_ftpInitPhase = FTP_INIT;

#define UART1_SENDING_NONE 0
#define UART1_SENDING_MQTT 1
#define UART1_SENDING_FTP  2
uint32_t g_isUart1Sending = UART1_SENDING_NONE;
volatile uint32_t g_Uart1SendTimeOut = 0;

bool g_isF103Sending = false;
volatile uint32_t g_F103SendTimeOut = 0;

bool g_isNeedSendUpdateSuccMsg = false;

DMA_Event_t dma_uart_rx = {DMA_RX_BUFFER_SIZE, 0, 0};


#define SYSTEM_PREFIX_NAME      "WM_"

#define UART2_RESP_ERROR 0
#define UART2_RESP_CONTROL 1
#define UART2_RESP_GET_RUN_BIN 2
#define UART2_RESP_GET_RECOVERY_BIN 3
#define UART2_RESP_GET_RUN_BIN_FIN 4
#define UART2_RESP_GET_RECOVERY_BIN_FIN 5
#define UART2_RESP_EDIT_FIN 6
#define UART2_RESP_HAVE_CAR 7


#define CURRENT_FIRMWARE      "1.0"

#define UPDATE_PROCESS_NONE 0
#define UPDATE_F405_DOWNLOAD_FIN 1
#define UPDATE_F405_UPDATE_FIN 2
#define UPDATE_F103_DOWNLOAD_FIN 3
#define UPDATE_GWID_MODIFY_FIN 4
#define UPDATE_ERROR 5

typedef struct {

  uint32_t msgType;
  uint32_t msgIndentify;
}T_UpdateProcess;
T_UpdateProcess g_updateProcess = {0};

uint32_t g_needUploadLog = 0;
uint8_t g_Log[256] = {0};

bool g_F103IsUpdating = false;

uint32_t g_update_restart_tick = 0;

uint8_t g_mqttOnlineTopic[256] = {0};
#define DEVICE_MAX_NUM 100

#define DEVICE_STATUS_ONLINE 1
#define DEVICE_STATUS_OFFLINE 0

typedef struct 
{
  uint16_t id;
  uint16_t status;
}T_DeviceStatus;

typedef struct 
{
  T_DeviceStatus device[DEVICE_MAX_NUM];
  uint32_t cur_device_num;
  
}T_DeviceStatusManager;

T_DeviceStatusManager g_device_manager = {0};


void uart3_msg_output(const char *format, ...)
{

  char buffer[256];
  
  va_list arg_list;
  va_start(arg_list, format);
  vsprintf(buffer, format, arg_list);
  va_end(arg_list);  
  HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), 0xffff);  
}

bool change_device_status(uint16_t deviceId, uint32_t status)
{
  uint32_t index = 0;
  Msg_Debug("change_device_status deviceId = %d, status = %d\r\n", deviceId, status);
  Msg_Debug("change_device_status start g_device_manager.cur_device_num = %d\r\n", g_device_manager.cur_device_num);
  
  for(index=0; index<g_device_manager.cur_device_num; index++)
  {
    
    if(deviceId == g_device_manager.device[index].id)
    {
      Msg_Debug("change_device_status start %d has is list\r\n", deviceId);
      
      break;
    }
  }

  Msg_Debug("change_device_status index = %d\r\n", index);
  Msg_Debug("change_device_status status = %d\r\n", status);

  if(index == g_device_manager.cur_device_num)//没找到
  {
    
    if(DEVICE_STATUS_ONLINE == status)
    {
      g_device_manager.device[g_device_manager.cur_device_num].id = deviceId;
      g_device_manager.device[g_device_manager.cur_device_num].status = DEVICE_STATUS_ONLINE;
      
      g_device_manager.cur_device_num++;
      
      return true;      
    }
    else
    {
      Msg_Debug("change_device_status should not run here (%d)\r\n", status);
      
      //error,不应该走到该流程
      return false;
    }
  }
  else//找到
  {
    Msg_Debug("change_device_status g_device_manager.device[index].status =  (%d)\r\n", g_device_manager.device[index].status);
    if(DEVICE_STATUS_ONLINE == status)
    {
      
      if(DEVICE_STATUS_ONLINE == g_device_manager.device[index].status)
      {
        return false;
      }
      else
      {
        g_device_manager.device[index].status = DEVICE_STATUS_ONLINE;
        return true;
      }
    }
    else
    {
      if(DEVICE_STATUS_ONLINE == g_device_manager.device[index].status)
      {
        g_device_manager.device[index].status = DEVICE_STATUS_OFFLINE;
        return true;
        
      }
      else
      {
        return false;
      }
    }
  }
}



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void getMqttChannel(void)
{
  memcpy(mqtt_channel, (char *)(MQTT_CHANNEL_ADDRESS), MQTT_CHANNEL_LENGTH);
}

static uint8_t crc8_chk_value(uint8_t *message, uint8_t len)
{
  uint8_t crc;
  uint8_t i;
  crc = 0;
  while(len--)
  {
    crc ^= *message++;
    for(i = 0;i < 8;i++)
    {
      if(crc & 0x01)
      {
        crc = (crc >> 1) ^ 0x8c;
      }
      else crc >>= 1;
    }
  }
  
  return crc; 
}



void boot_config(void)
{
  Msg_Debug("boot_config start\r\n");
  memcpy(&g_stBootConfig, (char *)CONFIG_ADDRESS, sizeof(T_BootConfig));
  
  memset(&g_stFWUpdateInfo, 0, sizeof(T_FWUpdateInfo));
  
  if(0 == g_stBootConfig.ucBootSeccessFlg)
  {    
    g_stBootConfig.uiRecoveryVerAddr = g_stBootConfig.uiUpdateVerAddr;
    g_stBootConfig.uiUpdateVerAddr = g_stBootConfig.uiRecoveryVerAddr==F405_UPDATA_ADDRESS1?F405_UPDATA_ADDRESS2:F405_UPDATA_ADDRESS1;

    g_stBootConfig.uiRecoveryVerDataLen = g_stBootConfig.uiUpdateVerDataTotalLen;
    
    memcpy(g_stBootConfig.ucFWVer, g_stBootConfig.ucNewFWVer, VERSION_LEN);

    g_isNeedSendUpdateSuccMsg = true;
  } 

  g_stBootConfig.uiUpdateVerDataTotalLen = 0;
  
  g_stBootConfig.ucTryTimes = 0;
  g_stBootConfig.ucNeedUpdateFlg = 0;
  g_stBootConfig.ucBootSeccessFlg = 1;
  
  memset(g_stBootConfig.ucNewFWVer, 0, VERSION_LEN);

  HAL_IWDG_Refresh(&hiwdg);
  FLASH_If_Erase(g_stBootConfig.uiUpdateVerAddr);
  HAL_Delay(100);  

  HAL_IWDG_Refresh(&hiwdg);
  FLASH_If_Erase(g_stBootConfig.uiF103UpdateRomAddr);
  HAL_Delay(100);

  HAL_IWDG_Refresh(&hiwdg);
  FLASH_If_Erase(CONFIG_ADDRESS);
  HAL_Delay(100);

  HAL_IWDG_Refresh(&hiwdg);
  FLASH_If_Write(CONFIG_ADDRESS, (uint32_t*)(&g_stBootConfig), sizeof(T_BootConfig));   
  HAL_Delay(100);

  if(0xff != *(uint8_t *)LOG_ADDRESS)
  {
    strcpy((char *)g_Log, (char *)LOG_ADDRESS);
    g_needUploadLog = 1;
    
    FLASH_If_Erase(LOG_ADDRESS);

  }
  

  Msg_Debug("boot_config end\r\n");
  
}


uint8_t Send_Mqtt_Msg(uint8_t topicType, uint8_t * payload, uint8_t length)
{
  if (MQTT_SND_INIT != g_uart1Phase)
  {
    Msg_Warn("g_uart1Phase = %d\r\n", g_uart1Phase);
    return MQTT_SEND_BUSY;
  }

  g_sndStartTick = HAL_GetTick();
  uint8_t * topic = NULL;
  switch(topicType)
  {
    case MQTT_TOPIC_TYPE_CHILD_ONLINE:
      topic = MQTT_TOPIC_CHILD_ONLINE;
      //topicContent = MQTT_TOPIC_CHILD_ONLINE_CONTENT;
      break;
    case MQTT_TOPIC_TYPE_CHILD_OFFLINE:
      topic = MQTT_TOPIC_CHILD_OFFLINE;
      //topicContent = MQTT_TOPIC_CHILD_OFFLINE_CONTENT;
      break;
    case MQTT_TOPIC_TYPE_ACTION_RSP:
      topic = MQTT_TOPIC_ACTION_RSP;
      //topicContent = MQTT_TOPIC_ACTION_RSP_CONTENT;
      break;
    case MQTT_TOPIC_TYPE_INTERRUPT_RSP:
      topic = MQTT_TOPIC_INTERRUPT_RSP;
      //topicContent = MQTT_TOPIC_INTERRUPT_RSP_CONTENT;
      break;
    case MQTT_TOPIC_TYPE_FIRMWARE_UPDATE:
      topic = MQTT_TOPIC_FW_UPDATE_RSP;
      break;
    case MQTT_TOPIC_TYPE_HAVE_CAR:
      topic = MQTT_TOPIC_HAVE_CAR_RSP;
      break;      
    default:
      return MQTT_SEND_INVALID_TOPIC_TYPE;
  }

  g_topicType = topicType;
  strncpy((char *)g_mqttSndPayload, (char *)payload, length);
  g_mqttSndPayload[length] = '\0';

  while((&huart1)->gState != HAL_UART_STATE_READY);
  strncpy((char *)DMA_TX_Buffer, (char *)topic, strlen((char *)topic));
  HAL_UART_Transmit_DMA(&huart1, DMA_TX_Buffer, strlen((char *)topic)); 
  
  return MQTT_SEND_SUCCESS;
}

static void Send_Mqtt_Msg_Continue(void)
{
  if (MQTT_SND_TOPIC_RSP == g_uart1Phase)
  {
    uint8_t * topicContent = NULL;
    switch(g_topicType)
    {
      case MQTT_TOPIC_TYPE_CHILD_ONLINE:
        topicContent = MQTT_TOPIC_CHILD_ONLINE_CONTENT;
        break;
      case MQTT_TOPIC_TYPE_CHILD_OFFLINE:
        topicContent = MQTT_TOPIC_CHILD_OFFLINE_CONTENT;
        break;
      case MQTT_TOPIC_TYPE_ACTION_RSP:
        topicContent = MQTT_TOPIC_ACTION_RSP_CONTENT;
        break;
      case MQTT_TOPIC_TYPE_INTERRUPT_RSP:
        topicContent = MQTT_TOPIC_INTERRUPT_RSP_CONTENT;
        break;
      case MQTT_TOPIC_TYPE_FIRMWARE_UPDATE:
        topicContent = MQTT_TOPIC_FW_UPDATE_RSP_CONTENT;
        break;
      case MQTT_TOPIC_TYPE_HAVE_CAR:
        topicContent = MQTT_TOPIC_HAVE_CAR_RSP_CONTENT;
        break;
      break;
      default:
        return;
    }
    while((&huart1)->gState != HAL_UART_STATE_READY);
    strncpy((char *)DMA_TX_Buffer, (char *)topicContent, strlen((char *)topicContent));
    HAL_UART_Transmit_DMA(&huart1, DMA_TX_Buffer, strlen((char *)topicContent)); 
  }
  else if (MQTT_SND_TOPIC_CONTENT_RSP == g_uart1Phase)
  {
    while((&huart1)->gState != HAL_UART_STATE_READY);
    sprintf((char *)DMA_TX_Buffer, "AT+CMQTTPAYLOAD=0,%d\r\n\0", strlen((char *)g_mqttSndPayload));
    HAL_UART_Transmit_DMA(&huart1, DMA_TX_Buffer, strlen((char *)DMA_TX_Buffer)); 
  }
  else if (MQTT_SND_PAYLOAD_RSP == g_uart1Phase)
  {
    while((&huart1)->gState != HAL_UART_STATE_READY);
    sprintf((char *)DMA_TX_Buffer, "%s\r\n\0", g_mqttSndPayload);
    HAL_UART_Transmit_DMA(&huart1, DMA_TX_Buffer, strlen((char *)DMA_TX_Buffer));
  }
  else if (MQTT_SND_PAYLOAD_CONTENT_RSP == g_uart1Phase)
  {
    while((&huart1)->gState != HAL_UART_STATE_READY);
    strcpy((char *)DMA_TX_Buffer, PUB);
    HAL_UART_Transmit_DMA(&huart1, DMA_TX_Buffer, strlen(PUB));
  }
  else if (MQTT_SND_CMD_RSP1 == g_uart1Phase)
  {
    g_uart1Phase = MQTT_SND_CMD_RSP2;
  }
  else 
  {
    Msg_Error("Send_Mqtt_Msg_Continue g_uart1Phase : %d\r\n", g_uart1Phase);
  }
}

static void Check_mqtt_init_Timeout(void)
{
  uint32_t timeout = 0;
  switch (g_uart1Phase)
  {
    case MQTT_INIT:
      timeout = 15000;
      break;
    case MQTT_RDY:
    case MQTT_CPIN:
      timeout = 10000;
      break;
    case MQTT_SMS:
      timeout = 10000;
      break;
    case MQTT_PB:
    case MQTT_ATE0:      
      timeout = 10000;
      break;
      
    case MQTT_ATE0_RSP:
    case MQTT_CCID:
    case MQTT_CCID_RSP:
    case MQTT_START:
    case MQTT_START_RSP:
    case MQTT_ACCQ:
    case MQTT_ACCQ_RSP:
    case MQTT_WILLTOPIC:
    case MQTT_WILLTOPIC_RSP:
    case MQTT_WILLTOPIC_CONTENT:
    case MQTT_WILLTOPIC_CONTENT_RSP:
    case MQTT_WILLMSG:
    case MQTT_WILLMSG_RSP:
    case MQTT_WILLMSG_CONTENT:
    case MQTT_WILLMSG_CONTENT_RSP:
      timeout = 2000;
      break;
    case MQTT_CONNECT:
    case MQTT_CONNECT_RSP:
      timeout = 10000;
      break;
    case MQTT_SUB:
    case MQTT_SUB_RSP:
    case MQTT_SUB_UPDATE:
    case MQTT_SUB_UPDATE_RSP:
      timeout = 1000;
      break;
    case MQTT_SUB_CONTENT:
    case MQTT_SUB_UPDATE_CONTENT:
    case MQTT_SUB_UPDATE_CONTENT_RSP:
    case MQTT_PUB_TOPIC:
    case MQTT_PUB_TOPIC_RSP:
    case MQTT_PUB_TOPIC_CONTENT:
    case MQTT_PUB_TOPIC_CONTENT_RSP:
    case MQTT_PUB_PAYLOAD:
    case MQTT_PUB_PAYLOAD_RSP:
    case MQTT_PUB_PAYLOAD_CONTENT:
    case MQTT_PUB_PAYLOAD_CONTENT_RSP:
      timeout = 1000;
      break;
    case MQTT_PUB:
    case MQTT_PUB_RSP:
      timeout = 10000;
      break;
    default:
      Msg_Error("Error Check_mqtt_init_Timeout g_uart1Phase : %d\r\n", g_uart1Phase);
      NVIC_SystemReset();
      return ;
  }
  uint32_t now = HAL_GetTick();
  if (now - g_startTick > timeout)
  {
    Msg_Error("Timeout g_uart1Phase : %d\r\n", g_uart1Phase);
    Msg_Error("g_startTick = %d, now = %d\r\n", g_startTick, now);
    NVIC_SystemReset();
    return ;
  }
}

static void Check_ftp_init_Timeout(void)
{
  uint32_t timeout = 0;
  switch (g_uart1Phase)
  {
    case FTP_INIT:
      timeout = 10000;
    case FTP_PORT:
    case FTP_SERV:
    case FTP_TYPE:
    case FTP_MODE:
    case FTP_USER:
    case FTP_PASSWD:
      timeout = 2000;
      break;
      
    default:
      Msg_Error("Error Check_ftp_init_Timeout g_uart1Phase : %d\r\n", g_uart1Phase);
      NVIC_SystemReset();
      return ;
  }
  uint32_t now = HAL_GetTick();
  if (now - g_startTick > timeout)
  {
    Msg_Error("Timeout g_uart1Phase : %d\r\n", g_uart1Phase);
    Msg_Error("g_startTick = %d, now = %d\r\n", g_startTick, now);
    NVIC_SystemReset();
    return ;
  }
}
static void mqttOnlineTopic(void)
{
  T_FWF103Head *stFwF103Head = (T_FWF103Head *)g_stBootConfig.uiF103RunRomAddr;
  root_online_resp(g_mqttOnlineTopic, mqtt_channel, g_simCardId, CURRENT_FIRMWARE, stFwF103Head->ucFWVer);  

}

static void mqtt_init_process(void)
{
  uint8_t * begin = NULL;
  uint32_t length = 0;
  bool flag = false;
  if (g_retBegin < g_retEnd)
  {
    begin = UART_Buffer + g_retBegin;
    length = g_retEnd - g_retBegin;
  }
  else
  {
    if ((g_retBegin > g_retEnd))
    {
      length = UART_BUFFER_SIZE - g_retBegin + g_retEnd;
    }
    else
    {
      length = UART_BUFFER_SIZE;
    }
    flag = true;
  }
  
  if (MQTT_ATE0 == g_uart1Phase && 11 != length
      || MQTT_CCID == g_uart1Phase && 38 != length
      || MQTT_START == g_uart1Phase && 24 != length
      || MQTT_ACCQ == g_uart1Phase && 2 != length
      || MQTT_WILLTOPIC == g_uart1Phase && 3 != length
      || MQTT_WILLTOPIC_CONTENT == g_uart1Phase && 2 != length
      || MQTT_WILLMSG == g_uart1Phase && 3 != length
      || MQTT_WILLMSG_CONTENT == g_uart1Phase && 2 != length
      || MQTT_CONNECT == g_uart1Phase && 28 !=length
      || MQTT_SUB == g_uart1Phase && 3 != length
      || MQTT_SUB_UPDATE== g_uart1Phase && 6 != length
      || MQTT_SUB_UPDATE_CONTENT== g_uart1Phase && 3 != length
      || MQTT_SUB_CONTENT == g_uart1Phase && 24 != length
      || MQTT_PUB_TOPIC == g_uart1Phase && 3 != length
      || MQTT_PUB_TOPIC_CONTENT == g_uart1Phase && 2 != length
      || MQTT_PUB_PAYLOAD == g_uart1Phase && 3 != length
      || MQTT_PUB_PAYLOAD_CONTENT == g_uart1Phase && 2 != length
      || MQTT_PUB == g_uart1Phase && 24 != length)
  {
    return ;
  }
  
  if (true == flag)
  {
    begin = (uint8_t *)malloc(length);
    if (NULL == begin)
    {
      Msg_Error("malloc Error 1\r\n");
      NVIC_SystemReset();
    }
    memcpy(begin, UART_Buffer + g_retBegin, UART_BUFFER_SIZE - g_retBegin);
    memcpy(begin + UART_BUFFER_SIZE - g_retBegin, UART_Buffer, g_retEnd);
  }
  
  char * expRst = NULL;
  uint32_t delayTime = 0;
  memset(DMA_TX_Buffer, 0, sizeof(DMA_TX_Buffer));
  switch(g_uart1Phase)
  {
    case MQTT_INIT:
      expRst = "RDY";
      
      break;
    case MQTT_RDY:
      expRst = "+CPIN: READY";
      break;
    case MQTT_CPIN:
      expRst = "SMS DONE";
      break;
    case MQTT_SMS:
      expRst = "PB DONE";

      boot_config();
      
      delayTime = 100;
      strcpy((char *)DMA_TX_Buffer, ATE0);//MQTT_ATE0
      //init suc, send "ATE0"
      break;
    case MQTT_ATE0:
      expRst = "ATE0\r\r\nOK\r\n";
      delayTime = 100;
      strcpy((char *)DMA_TX_Buffer, CCID);//MQTT_CCID
      //init suc, send "CCID"
      break;
    case MQTT_CCID:
      if (0 != strncmp((char *)begin, "\r\n+ICCID: ", 10)
          || 0 != strncmp((char *)(begin + 30), "\r\n\r\nOK\r\n", 8))
      {
        Msg_Error("%d POK\r\n", MQTT_CCID);
        for (uint32_t i = 0; i < length; i ++)
        {
          Msg_Error("%02X", begin[i]);
        }
        Msg_Error("\r\n");
        NVIC_SystemReset();
        return;
      }

      memcpy(g_simCardId + 3, begin + 10, 20);

      mqttOnlineTopic();
      
      delayTime = 1000;
      strcpy((char *)DMA_TX_Buffer, CMQTTSTART);//MQTT_START
      //init suc, send "CMQTTSTART"
      break;
    case MQTT_START:
      expRst = "\r\nOK\r\n\r\n+CMQTTSTART: 0\r\n";
      delayTime = 100;
      sprintf((char *)DMA_TX_Buffer, "%s\"%s\"\r\n", ACCQ_PREFIX, (char *)mqtt_channel);
      //init suc, send "CMQTTACCQ"
      break;
    case MQTT_ACCQ:
      expRst = "OK";
      delayTime = 100;
      strcpy((char *)DMA_TX_Buffer, WILLTOPIC);//MQTT_WILLTOPIC
      //init suc, send "WILLTOPIC"
      break;
    case MQTT_WILLTOPIC:
      expRst = "\r\n>";
      delayTime = 100;
      strcpy((char *)DMA_TX_Buffer, WILLTOPIC_CONTENT);//MQTT_WILLTOPIC_CONTENT
      //init suc, send "WILLTOPIC_CONTENT"
      break;
    case MQTT_WILLTOPIC_CONTENT:
      expRst = "OK";
      delayTime = 100;
      sprintf((char *)DMA_TX_Buffer, WILLMSG, strlen((char *)mqtt_channel));//MQTT_WILLMSG
      //init suc, send "WILLMSG"
      break;
    case MQTT_WILLMSG:
      expRst = "\r\n>";
      delayTime = 100;
      sprintf((char *)DMA_TX_Buffer, "%s\r\n", (char *)mqtt_channel);//MQTT_WILLMSG_CONTENT
      //init suc, send "WILLMSG_CONTENT"
      break;
    case MQTT_WILLMSG_CONTENT:
      expRst = "OK";
      delayTime = 100;
      strcpy((char *)DMA_TX_Buffer, CONNECT_PRODUCT);//MQTT_CONNECT
      break;
    case MQTT_CONNECT:
      expRst = "\r\nOK\r\n\r\n+CMQTTCONNECT: 0,0\r\n";
      delayTime = 100;

      strcpy((char *)DMA_TX_Buffer, PUBTOPIC);
      break;

    case MQTT_PUB_TOPIC:
      expRst = "\r\n>";
      delayTime = 100;
      strcpy((char *)DMA_TX_Buffer, PUBTOPIC_CONTENT);//MQTT_PUB_TOPIC_CONTENT
      break;
    case MQTT_PUB_TOPIC_CONTENT:
      expRst = "OK";
      delayTime = 100;
      
      sprintf((char *)DMA_TX_Buffer, PUBPAYLOAD, strlen((char *)g_mqttOnlineTopic));
      //strcpy((char *)DMA_TX_Buffer, PUBPAYLOAD);//MQTT_PUB_PAYLOAD
      break;
    case MQTT_PUB_PAYLOAD:
      expRst = "\r\n>";
      delayTime = 100;
      sprintf((char *)DMA_TX_Buffer, "%s\r\n", g_mqttOnlineTopic);//MQTT_PUB_PAYLOAD_CONTENT
      break;
    case MQTT_PUB_PAYLOAD_CONTENT:
      expRst = "OK";
      delayTime = 100;
      strcpy((char *)DMA_TX_Buffer, PUB);//MQTT_PUB
      break;
    case MQTT_PUB:
      expRst = "\r\nOK\r\n\r\n+CMQTTPUB: 0,0\r\n";
      delayTime = 100;     
      sprintf((char *)DMA_TX_Buffer, SUB, strlen((char *)mqtt_channel));//MQTT_SUB
      break;      
    case MQTT_SUB:
      expRst = "\r\n>";
      delayTime = 100;
      sprintf((char *)DMA_TX_Buffer, "%s\r\n", (char *)mqtt_channel);//MQTT_SUB_CONTENT
      break;
    case MQTT_SUB_UPDATE:
      expRst = "\r\nOK\r\n";
      delayTime = 100;
      sprintf((char *)DMA_TX_Buffer, "%s\r\n", SUB_UPDATE);//MQTT_SUB_UPDATE
      break;
    case MQTT_SUB_UPDATE_CONTENT:
      expRst = "\r\n>";
      delayTime = 100;
      sprintf((char *)DMA_TX_Buffer, "%s\r\n", SUB_UPDATE_CONTENT);//MQTT_SUB_UPDATE_CONTENT
      break;
    
    case MQTT_SUB_CONTENT:
      expRst = "\r\nOK\r\n\r\n+CMQTTSUB: 0,0\r\n";
      break;

    default:
      break;
  }

  if (NULL != expRst)
  {
    if (0 != strncmp(expRst, (char *)begin, length))
    {
      Msg_Error("%d POK\r\n", g_uart1Phase);
      for (uint32_t i = 0; i < length; i ++)
      {
        Msg_Error("%02X", begin[i]);
      }
      Msg_Error("\r\n");
      NVIC_SystemReset();
      return;
    }
  }

  if (true == flag && NULL != begin)
  {
    free(begin);
    begin = NULL;
  }
  
  g_uart1Phase ++;
  g_startTick = HAL_GetTick();  
  Msg_Info("%d ok\r\n", g_uart1Phase);

  g_retBegin = RET_POS_INVALID;
  g_retEnd = RET_POS_INVALID;

  if (0 != delayTime)
  {
    HAL_Delay(delayTime);
    HAL_UART_Transmit_DMA(&huart1, DMA_TX_Buffer, strlen((char *)DMA_TX_Buffer)); 
  }
}

static void mqtt_init(void)
{
  while (1)
  {
    Check_mqtt_init_Timeout();
    /**
       * Loop data back to UART data register
       */
    //uint32_t total_count = 0; // total count
    int32_t sub_count = 0; // sub count of one result
    int32_t end_count = 0; // result end 0x0d count
    uint32_t tmpWrite = Uart1_Write;
    while (Uart1_Read != tmpWrite)
    {
      if (MQTT_ATE0 == g_uart1Phase                 /* receive after send ATE0 */
          || MQTT_CCID == g_uart1Phase              /* receive after send CCID */
          || MQTT_START == g_uart1Phase             /* receive after send CMQTTSTART */
          || MQTT_WILLTOPIC == g_uart1Phase         /* receive after send WILLTOPIC */
          || MQTT_WILLMSG == g_uart1Phase           /* receive after send WILLMSG */
          || MQTT_CONNECT == g_uart1Phase           /* receive after send CONNECT */
          || MQTT_SUB == g_uart1Phase               /* receive after send SUB */
          || MQTT_SUB_CONTENT == g_uart1Phase       /* receive after send SUB_CONTENT */
          || MQTT_SUB_UPDATE== g_uart1Phase               /* receive after send SUB */
          || MQTT_SUB_UPDATE_CONTENT == g_uart1Phase       /* receive after send SUB_CONTENT */
          || MQTT_PUB_TOPIC == g_uart1Phase         /* receive after send PUB_TOPIC */
          || MQTT_PUB_PAYLOAD == g_uart1Phase       /* receive after send PUB_PAYLOAD */
          || MQTT_PUB == g_uart1Phase               /* receive after send PUB */) 
      {
          
        if (RET_POS_INVALID == g_retBegin)
        {
          g_retBegin = Uart1_Read;
        }
        g_retEnd = tmpWrite;
        
        mqtt_init_process();
        Uart1_Read = tmpWrite;
        break;
      }

      if (RET_POS_INVALID == g_retBegin)
      {
        if ((0 == sub_count && 0x0d != UART_Buffer[Uart1_Read]) || (1 == sub_count && 0x0a != UART_Buffer[Uart1_Read]))
        {
          //error
          Msg_Error("Error!111 g_uart1Phase = %d\r\n", g_uart1Phase);
          NVIC_SystemReset();
        }
        if (sub_count == 2)
        {
          g_retBegin = Uart1_Read;
        }
      }
      else
      {
        if (0x0d == UART_Buffer[Uart1_Read])
        {
          g_retEnd = Uart1_Read;
          end_count = sub_count;
        }
        if (0 != end_count && sub_count == end_count + 1)
        {
          if (0x0a == UART_Buffer[Uart1_Read])
          {
            // one result analysis suc.

            mqtt_init_process();
            g_retBegin = RET_POS_INVALID;
            g_retEnd = RET_POS_INVALID;
            sub_count = -1;
            end_count = 0;
          }
          else
          {
            //error
            Msg_Error("Error!222\r\n");
            NVIC_SystemReset();
          }
        }
      }
      Uart1_Read++;

      if (Uart1_Read == UART_BUFFER_SIZE)
      {
        /* Check buffer overflow */
        Uart1_Read = 0;
      }

      sub_count ++;
      //total_count ++;
    }
    if (MQTT_OK <= g_uart1Phase)
    {
      break;
    }
  }  
}

static void ftp_init_process(void)
{
  uint8_t * begin = NULL;
  uint16_t length = 0;
  bool flag = false;
  if (g_retBegin < g_retEnd)
  {
    begin = UART_Buffer + g_retBegin;
    length = g_retEnd - g_retBegin;
  }
  else
  {
    if ((g_retBegin > g_retEnd))
    {
      length = UART_BUFFER_SIZE - g_retBegin + g_retEnd;
    }
    else
    {
      length = UART_BUFFER_SIZE;
    }
    flag = true;
  }
  if (true == flag)
  {
    begin = (uint8_t *)malloc(length + 1);
    if (NULL == begin)
    {
      Msg_Error("malloc Error 2\r\n");
      NVIC_SystemReset();
    }
    memcpy(begin, UART_Buffer + g_retBegin, UART_BUFFER_SIZE - g_retBegin);
    memcpy(begin + UART_BUFFER_SIZE - g_retBegin, UART_Buffer, g_retEnd);
    begin[length] = 0;
  }
  begin[length] = 0;

  
  if (2 == length && 0 == strncmp((char *)begin, "OK", 2))
  {
    if (FTP_PORT == g_uart1Phase)
    {
      Msg_Info("g_uart1Phase = %d(FTP_PORT)\r\n", g_uart1Phase);
      while((&huart1)->gState != HAL_UART_STATE_READY);
      strncpy((char *)DMA_TX_Buffer, AT_FTPSERV, strlen(AT_FTPSERV));
      HAL_UART_Transmit_DMA(&huart1, DMA_TX_Buffer, strlen(AT_FTPSERV));
    }
    else if (FTP_SERV == g_uart1Phase)
    {
      Msg_Info("g_uart1Phase = %d(FTP_SERV)\r\n", g_uart1Phase);
      while((&huart1)->gState != HAL_UART_STATE_READY);
      strncpy((char *)DMA_TX_Buffer, AT_FTPTYPE, strlen(AT_FTPTYPE));
      HAL_UART_Transmit_DMA(&huart1, DMA_TX_Buffer, strlen(AT_FTPTYPE));
    }
    else if (FTP_TYPE == g_uart1Phase)
    {
      Msg_Info("g_uart1Phase = %d(FTP_TYPE)\r\n", g_uart1Phase);
      while((&huart1)->gState != HAL_UART_STATE_READY);
      strncpy((char *)DMA_TX_Buffer, AT_FTPMODE, strlen(AT_FTPMODE));
      HAL_UART_Transmit_DMA(&huart1, DMA_TX_Buffer, strlen(AT_FTPMODE));
    }
    else if (FTP_MODE == g_uart1Phase)
    {
      Msg_Info("g_uart1Phase = %d(FTP_MODE)\r\n", g_uart1Phase);
      while((&huart1)->gState != HAL_UART_STATE_READY);
      strncpy((char *)DMA_TX_Buffer, AT_FTPUSER, strlen(AT_FTPUSER));
      HAL_UART_Transmit_DMA(&huart1, DMA_TX_Buffer, strlen(AT_FTPUSER));
    }
    else if (FTP_USER == g_uart1Phase)
    {
      Msg_Info("g_uart1Phase = %d(FTP_USER)\r\n", g_uart1Phase);
      while((&huart1)->gState != HAL_UART_STATE_READY);
      strncpy((char *)DMA_TX_Buffer, AT_FTPPWD, strlen(AT_FTPPWD));
      HAL_UART_Transmit_DMA(&huart1, DMA_TX_Buffer, strlen(AT_FTPPWD));
    }
    else if (FTP_PASSWD == g_uart1Phase)
    {
      Msg_Info("g_uart1Phase = %d(FTP_PASSWD)\r\n", g_uart1Phase);
      g_uart1Phase = FTP_OK;
    }
    else
    {
      Msg_Error("ftp_init_process OK : %s\r\n", begin);
      NVIC_SystemReset();
    }
  }
  else
  {
      Msg_Error("ftp_init_process : %s\r\n", begin);
      NVIC_SystemReset();
  }

  if (true == flag)
  {
    free(begin);
  }
  else
  {
    begin[length] = '\r';
  }
}

void ftp_init(void)
{
  g_uart1Phase = FTP_INIT;
  g_retBegin = RET_POS_INVALID;
  g_retEnd = RET_POS_INVALID;
  
  while((&huart1)->gState != HAL_UART_STATE_READY);
  strncpy((char *)DMA_TX_Buffer, AT_FTPPORT, strlen(AT_FTPPORT));
  HAL_UART_Transmit_DMA(&huart1, DMA_TX_Buffer, strlen(AT_FTPPORT));
  
  while (1)
  {
    Check_ftp_init_Timeout();
    
    uint32_t tmpWrite = Uart1_Write;
    while (Uart1_Read != tmpWrite)
    {
      if (RET_POS_INVALID == g_retBegin)
      {
        if (0x0d == UART_Buffer[Uart1_Read] || 0x0a == UART_Buffer[Uart1_Read])
        {
          goto label;
        }

        g_retBegin = Uart1_Read;
      }
      else
      {
        if (0x0d == UART_Buffer[Uart1_Read])
        {
          g_retEnd = Uart1_Read;
          ftp_init_process();
          //sub_count = 0;
          g_retBegin = RET_POS_INVALID;
          g_retEnd = RET_POS_INVALID;
          //continue;
        }
      }
label:
      Uart1_Read++;

      if (Uart1_Read == UART_BUFFER_SIZE)
      {
        /* Check buffer overflow */
        Uart1_Read = 0;
      }      
    }

    
    if (FTP_OK <= g_uart1Phase)
    {
      break;
    }
    
  }  
}

void setCurrOnlineDeviceCmd(void)
{
  T_Control_Cmd cmd;

  cmd.data.value = 0;
  cmd.identify = 0;
  cmd.endpointId = 0;
  cmd.action = CMD_F405_ONLINE;
  cmd.crc = crc8_chk_value((uint8_t *)(&cmd), 11);

  memcpy(UART2_TX_Buffer+Uart2_Send_Write, &cmd, sizeof(T_Control_Cmd));
  Uart2_Send_Write += sizeof(T_Control_Cmd);
#if 0
  memcpy(DMA2_TX_Buffer, (uint8_t *)(&cmd), sizeof(T_Control_Cmd));
  HAL_UART_Transmit_DMA(&huart2, DMA2_TX_Buffer, sizeof(T_Control_Cmd));
#endif    
}

void checkSndTimeout(void)
{  
  if (0 != g_sndStartTick)
  {
    if (HAL_GetTick() - g_sndStartTick > 10000)
    {
      // send timeout
      Msg_Error("send timeout\r\n");
      NVIC_SystemReset();
    }
  }
}

static void writeDataToFlash(void)
{
  uint32_t dstAddr = 0;
  uint8_t *pBuf = (uint8_t *)malloc(g_stFWUpdateInfo.uiPerDataSize);
  if(NULL == pBuf)
  {
    Msg_Error("writeDataToFlash malloc failed\r\n");
    return;
  }
  
  if(UPDATE_TYPE_STM32F405 == g_stFWUpdateInfo.uiFWType)
  {
    dstAddr = g_stBootConfig.uiUpdateVerAddr + g_stFWUpdateInfo.uiDLFinLen;
  }
  else if(UPDATE_TYPE_STM32F103 == g_stFWUpdateInfo.uiFWType)
  {
    dstAddr = g_stBootConfig.uiF103UpdateRomAddr + sizeof(T_FWF103Head) + g_stFWUpdateInfo.uiDLFinLen;
  }
  
  if(g_retEnd > g_retBegin)
  {
    memcpy((char *)pBuf, &UART_Buffer[g_retBegin], g_stFWUpdateInfo.uiPerDataSize);
  }
  else
  {
    memcpy((char *)pBuf, &UART_Buffer[g_retBegin], UART_BUFFER_SIZE - g_retBegin);
      
    memcpy((char *)(pBuf + UART_BUFFER_SIZE - g_retBegin), &UART_Buffer[0], g_stFWUpdateInfo.uiPerDataSize - (UART_BUFFER_SIZE - g_retBegin));
                               
  }
  

  FLASH_If_Write(dstAddr, (uint32_t *)pBuf, g_stFWUpdateInfo.uiPerDataSize);
  if(NULL != pBuf)
  {
    free(pBuf);
    pBuf = NULL;
  }
  
  g_stFWUpdateInfo.uiDLFinLen += g_stFWUpdateInfo.uiPerDataSize;  
  
  g_stFWUpdateInfo.uiUpdateStep = UPDATE_FW_FTPCACHERD_DATA_OK; 
  
}

static void do_mqtt_ok(void)
{
    if (MQTT_SND_TOPIC_CONTENT == g_uart1Phase)
    {
      g_uart1Phase = MQTT_SND_TOPIC_CONTENT_RSP;
    }
    else if (MQTT_SND_PAYLOAD_CONTENT == g_uart1Phase)
    {
      g_uart1Phase = MQTT_SND_PAYLOAD_CONTENT_RSP;
    }
    else if (MQTT_SND_CMD == g_uart1Phase)
    {
      g_uart1Phase = MQTT_SND_CMD_RSP1;
    }
    else
    {
      Msg_Error("do_mqtt_ok OK : g_uart1Phase = %d\r\n", g_uart1Phase);
      NVIC_SystemReset();
    }
    
    Send_Mqtt_Msg_Continue();    
}

static void do_update_ok(void)
{
  switch (g_stFWUpdateInfo.uiUpdateStep)
  {
    case UPDATE_MD5_RCV_OK:
      g_stFWUpdateInfo.uiUpdateStep = UPDATE_MD5_RCV_DATA_LEN;
      break;
    case UPDATE_SIZE_RCV_OK:
      g_stFWUpdateInfo.uiUpdateStep = UPDATE_SIZE_RCV_DATA_LEN;
      break;
    case UPDATE_FW_RCV_OK:
      g_stFWUpdateInfo.uiUpdateStep = UPDATE_FW_RCV_EVENT;
      break;
    case UPDATE_FW_FTPCACHERD_SIZE_OK:
      g_stFWUpdateInfo.uiUpdateStep = UPDATE_FW_FTPCACHERD_DATA_SEND;
      
      g_isUart1Sending = UART1_SENDING_NONE;
      break;
    case UPDATE_FW_FTPCACHERD_DATA_OK:

      if(g_stFWUpdateInfo.uiDLFinLen == g_stFWUpdateInfo.uiTotalLen)
      {
        uint32_t startAddr = 0;

        Msg_Debug("\r\n uiFWType = %d\r\n", g_stFWUpdateInfo.uiFWType);
        if(UPDATE_TYPE_STM32F405 == g_stFWUpdateInfo.uiFWType)
        {
          startAddr = g_stBootConfig.uiUpdateVerAddr;
        }
        else if(UPDATE_TYPE_STM32F103 == g_stFWUpdateInfo.uiFWType)
        {
          startAddr = g_stBootConfig.uiF103UpdateRomAddr + sizeof(T_FWF103Head);
        }
        
        Msg_Debug("\r\n startAddr = %08x\r\n", startAddr);
        
        uint8_t md5_temp[MD5_LEN];
        
        md5sum(md5_temp, (uint8_t  *)startAddr, g_stFWUpdateInfo.uiTotalLen);
        if(!memcmp(md5_temp, g_stFWUpdateInfo.ucBinMd5, MD5_LEN))
        {
          Msg_Debug("\r\n download success!board:%d, ver:%s, size:%d\r\n", g_stFWUpdateInfo.uiFWType, g_stFWUpdateInfo.ucFWVer, g_stFWUpdateInfo.uiTotalLen);
          
          if(UPDATE_TYPE_STM32F405 == g_stFWUpdateInfo.uiFWType)
          {
            g_stBootConfig.uiUpdateVerDataTotalLen = g_stFWUpdateInfo.uiTotalLen;
            
            memcpy(g_stBootConfig.ucNewFWVer, g_stFWUpdateInfo.ucFWVer, VERSION_LEN);
            
            g_stBootConfig.ucNeedUpdateFlg = 1;
            g_stBootConfig.ucBootSeccessFlg = 0;
            
            HAL_IWDG_Refresh(&hiwdg);
            FLASH_If_Erase(CONFIG_ADDRESS);
            
            HAL_Delay(100);
            HAL_IWDG_Refresh(&hiwdg);
            FLASH_If_Write(CONFIG_ADDRESS, (uint32_t*)(&g_stBootConfig), sizeof(T_BootConfig)); 

            g_update_restart_tick = HAL_GetTick();

//                HAL_Delay(1000);
            
//                NVIC_SystemReset();                  
          }
          else if(UPDATE_TYPE_STM32F103 == g_stFWUpdateInfo.uiFWType)
          {
            T_FWF103Head stFwF103Head;
            stFwF103Head.uiTotalLen = g_stFWUpdateInfo.uiTotalLen;
            memcpy(stFwF103Head.ucFWVer, g_stFWUpdateInfo.ucFWVer, VERSION_LEN);
            memcpy(stFwF103Head.ucBinMd5, g_stFWUpdateInfo.ucBinMd5, MD5_LEN);
            
            HAL_IWDG_Refresh(&hiwdg);
            FLASH_If_Write(g_stBootConfig.uiF103UpdateRomAddr, (uint32_t*)(&stFwF103Head), sizeof(T_FWF103Head));

            T_Control_Cmd cmd;
            
            memcpy((char *)(cmd.data.verName), stFwF103Head.ucFWVer, VERSION_LEN);            
            cmd.identify = 0;
            cmd.endpointId = 0;
            cmd.action = CMD_UPDATE;
            cmd.crc = crc8_chk_value((uint8_t *)(&cmd), 11);
            

            while((&huart2)->gState != HAL_UART_STATE_READY);
            memcpy(DMA2_TX_Buffer, (uint8_t *)(&cmd), sizeof(T_Control_Cmd));
            HAL_UART_Transmit_DMA(&huart2, DMA2_TX_Buffer, sizeof(T_Control_Cmd));                            

            g_F103SendTimeOut = HAL_GetTick();
            g_isF103Sending = true;

            g_F103IsUpdating = true;
          }       

        }
        else
        {
      
          Msg_Debug("\r\n md5 check error\r\n");
      
          HAL_IWDG_Refresh(&hiwdg);
          FLASH_If_Erase(CONFIG_ADDRESS);

          HAL_Delay(100);
          HAL_IWDG_Refresh(&hiwdg);
          FLASH_If_Write(CONFIG_ADDRESS, (uint32_t*)(&g_stBootConfig), sizeof(T_BootConfig)); 

          
          HAL_Delay(100);
          HAL_IWDG_Refresh(&hiwdg);
          FLASH_If_Erase(g_stBootConfig.uiUpdateVerAddr);

          HAL_Delay(100);
          HAL_IWDG_Refresh(&hiwdg);
          FLASH_If_Erase(g_stBootConfig.uiF103UpdateRomAddr);
                
        }

        memset(&g_stFWUpdateInfo, 0, sizeof(T_FWUpdateInfo));
        g_isUart1Sending = UART1_SENDING_NONE;

        
        //下载完成
      }
      else
      {            
        g_stFWUpdateInfo.uiUpdateStep = UPDATE_FW_FTPCACHERD_DATA_SEND;
        g_stFWUpdateInfo.uiPerDataSize = 0;
        
      }
      
      g_isUart1Sending = UART1_SENDING_NONE;
      
      break;
      
    default:
      Msg_Error("update error! step = %d\r\n", g_stFWUpdateInfo.uiUpdateStep);
      break;
      
  }  
}

static bool do_update_other(uint8_t *pBuf)
{
  bool res = false;

  if(NULL == pBuf)
  {
    return res;
  }
  
  if(UPDATE_STEP_NONE == g_stFWUpdateInfo.uiUpdateStep)
  {
     return res;
  }

  Msg_Debug("update: uiUpdateStep = %d\r\n", g_stFWUpdateInfo.uiUpdateStep);
  
  if(UPDATE_MD5_RCV_DATA_LEN == g_stFWUpdateInfo.uiUpdateStep)
  {
    if(!strncmp((char *)pBuf, RES_MD5_LEN, strlen(RES_MD5_LEN)))
    {
      sscanf((char *)(pBuf + strlen(RES_MD5_LEN)), "%u", &(g_stFWUpdateInfo.uiMd5Len));
      g_stFWUpdateInfo.uiUpdateStep = UPDATE_MD5_RCV_DATA;
      res = true;
    }
  }
  else if(UPDATE_MD5_RCV_DATA == g_stFWUpdateInfo.uiUpdateStep)
  {
    if(strlen((char *)pBuf) == g_stFWUpdateInfo.uiMd5Len)
    {
      memcpy((char *)g_stFWUpdateInfo.ucBinMd5, pBuf, MD5_LEN);
      g_stFWUpdateInfo.uiUpdateStep = UPDATE_MD5_FIN;

      res = true;
    }
  }
  else if(UPDATE_MD5_FIN == g_stFWUpdateInfo.uiUpdateStep)
  {
    if(!strncmp((char *)pBuf, RES_MD5_FINISH, strlen(RES_MD5_FINISH)))
    {
      g_stFWUpdateInfo.uiUpdateStep = UPDATE_SIZE_SEND;

      g_isUart1Sending = UART1_SENDING_NONE;
      
      res = true;
    }
  }
  else if(UPDATE_SIZE_RCV_DATA_LEN == g_stFWUpdateInfo.uiUpdateStep)
  {
    if(!strncmp((char *)pBuf, RES_SIZE_LEN, strlen(RES_SIZE_LEN)))
    {
      sscanf((char *)(pBuf + strlen(RES_SIZE_LEN)), "%u", &(g_stFWUpdateInfo.uiListDataSize));
      g_stFWUpdateInfo.uiUpdateStep = UPDATE_SIZE_RCV_DATA;

      res = true;
    }        
  }
  else if(UPDATE_SIZE_RCV_DATA == g_stFWUpdateInfo.uiUpdateStep)
  {
    if(strlen((char *)pBuf) + 2 == g_stFWUpdateInfo.uiListDataSize)//uiListDataSize include "\r\n"
    {
      char buffer[8];
      memset(buffer, 0, 8);
      sscanf((char *)(pBuf + strlen(RES_SIZE_PREFIX)), "%[^ ]", buffer);
      g_stFWUpdateInfo.uiTotalLen = atoi(buffer);
      
      g_stFWUpdateInfo.uiUpdateStep = UPDATE_SIZE_FIN;

      res = true;
    }
  }
  else if(UPDATE_SIZE_FIN == g_stFWUpdateInfo.uiUpdateStep)
  {
    if(!strncmp((char *)pBuf, RES_SIZE_FINISH, strlen(RES_SIZE_FINISH)))
    {
      g_stFWUpdateInfo.uiUpdateStep = UPDATE_FW_SEND;
      
      g_isUart1Sending = UART1_SENDING_NONE;

      res = true;
    }
  }  
  else if(UPDATE_FW_RCV_EVENT == g_stFWUpdateInfo.uiUpdateStep)
  {
    if(!strncmp((char *)pBuf, RES_FTPGET_RECV_EVENT, strlen(RES_FTPGET_RECV_EVENT)))
    {
      g_stFWUpdateInfo.uiUpdateStep = UPDATE_FW_RCV_CACHE_FIN;
      res = true;
    }
  }              
  else if(UPDATE_FW_RCV_CACHE_FIN == g_stFWUpdateInfo.uiUpdateStep)
  {
   if(!strncmp((char *)pBuf, RES_FTPGET_FIN, strlen(RES_FTPGET_FIN)))
   {
     g_stFWUpdateInfo.uiUpdateStep = UPDATE_FW_FTPCACHERD_SIZE_SEND;
     
     g_isUart1Sending = UART1_SENDING_NONE;

     res = true;
   }
  }
  else if(UPDATE_FW_FTPCACHERD_SIZE_RES == g_stFWUpdateInfo.uiUpdateStep)
  {
   if(!strncmp((char *)pBuf, RES_FTPCACHERD_SIZE, strlen(RES_FTPCACHERD_SIZE)))
   {
     int uiBinSize;
     sscanf((char *)(pBuf + strlen(RES_FTPCACHERD_SIZE)), "%u", &uiBinSize);
     if(uiBinSize != g_stFWUpdateInfo.uiTotalLen)
     {
       Msg_Debug("cache get data error !! uiBinSize = %d, g_stFWUpdateInfo.uiTotalLen = %d\r\n", uiBinSize, g_stFWUpdateInfo.uiTotalLen);
       memset(&g_stFWUpdateInfo, 0, sizeof(T_FWUpdateInfo));
     }
     else
     {
       g_stFWUpdateInfo.uiUpdateStep = UPDATE_FW_FTPCACHERD_SIZE_OK;          
     }
     
     res = true;

   }
  }       
  else if(UPDATE_FW_FTPCACHERD_DATA_LEN == g_stFWUpdateInfo.uiUpdateStep)
  {
    if(!strncmp((char *)pBuf, RES_FTPCACHERD_DATA, strlen(RES_FTPCACHERD_DATA)))
    {
      sscanf((char *)(pBuf + strlen(RES_FTPCACHERD_DATA)), "%u", &(g_stFWUpdateInfo.uiPerDataSize));

      g_stFWUpdateInfo.uiUpdateStep = UPDATE_FW_FTPCACHERD_DATA_LEN_END;

      res = true;
    }        
  } 

  return res;
}


static bool generalCmd(uint8_t *jsonStr)
{
  cJSON *root;
  root = cJSON_Parse((char *)jsonStr);

  cJSON *control_json = cJSON_GetObjectItem(root,"control");
  if(NULL == control_json)
  {
    cJSON_Delete(root);
    return false;
  }


  cJSON *json_mac = cJSON_GetObjectItem(control_json, "mac");
  if(NULL == json_mac)
  {
    cJSON_Delete(root);
    return false;    
  }

  cJSON *json_action = cJSON_GetObjectItem(control_json, "action");
  if(NULL == json_action)
  {
    cJSON_Delete(root);
    return false;    
  }

  cJSON *json_identify = cJSON_GetObjectItem(control_json, "identify");
  if(NULL == json_identify)
  {
    cJSON_Delete(root);
    return false;    
  }

  T_Control_Cmd cmd;  
  memset(&cmd, 0, sizeof(T_Control_Cmd));

  cmd.endpointId = json_mac->valueint;
  cmd.action = json_action->valueint;
  cmd.identify = json_identify->valueint;

  cmd.crc = crc8_chk_value((uint8_t *)(&cmd), 11);

  Msg_Debug("receive control cmd(id-%d, action-%d, identify-%d, crc-%02x)\r\n", cmd.endpointId, cmd.action, cmd.identify, cmd.crc);

  memcpy(UART2_TX_Buffer+Uart2_Send_Write, &cmd, sizeof(T_Control_Cmd));

  Uart2_Send_Write += sizeof(T_Control_Cmd);
  if(Uart2_Send_Write == UART2_TX_BUFFER_SIZE)
  {
    Uart2_Send_Write = 0;
  }

  cJSON_Delete(root);
  root = NULL;

  return true;
}

static bool doUpdateStart(cJSON *fireware_update, int iType, char *curr_ver)
{
  char *version = cJSON_GetObjectItem(fireware_update,"version")->valuestring;
  Msg_Debug("doUpdateStart version = %s\r\n", version);

  if(0 == strcmp(curr_ver, (char *)version))
  {
    return false;
  }

  cJSON *json_path = cJSON_GetObjectItem(fireware_update,"path");
  if(NULL == json_path)
  {
    return false;
  }
  
  char *path = json_path->valuestring;
  if(NULL == path)
  {
    return false;
  }

  Msg_Debug("path = %s\r\n", path);
  if(UPDATE_STEP_NONE == g_stFWUpdateInfo.uiUpdateStep)
  {
    memset(&g_stFWUpdateInfo, 0, sizeof(T_FWUpdateInfo));    
    
    g_stFWUpdateInfo.uiFWType = iType;
    
    memcpy(g_stFWUpdateInfo.ucFWVer, version, VERSION_LEN);
    
    if(UPDATE_TYPE_STM32F405 == g_stFWUpdateInfo.uiFWType)
    {
      strcpy((char *)g_stFWUpdateInfo.ucBoardName, BOARD_NAME_STM32F405);
    }
    else if(UPDATE_TYPE_STM32F103 == g_stFWUpdateInfo.uiFWType)
    {
      strcpy((char *)g_stFWUpdateInfo.ucBoardName, BOARD_NAME_STM32F103);
    }

    strcpy((char *)g_stFWUpdateInfo.ucPath, path);
    
    g_stFWUpdateInfo.uiUpdateStep = UPDATE_MD5_SEND;
  }
   
  return true;
}

static bool editGWId(cJSON *fireware_update, uint32_t identify)
{
  cJSON *json_GWID = cJSON_GetObjectItem(fireware_update, "GWID");
  if(NULL == json_GWID)
  {
    return false;
  }

  
  if(NULL == json_GWID->valuestring)
  {
    return false;
  }

  char gwid[MQTT_CHANNEL_LENGTH] = {0};

  strcpy(gwid, json_GWID->valuestring);
  Msg_Debug("editGWId : %s\r\n", gwid);

  FLASH_If_Erase(MQTT_CHANNEL_ADDRESS);

  HAL_Delay(200);
  FLASH_If_Write_Byte(MQTT_CHANNEL_ADDRESS, (uint8_t *)gwid, MQTT_CHANNEL_LENGTH);
  HAL_Delay(200);

  memcpy(mqtt_channel, (char *)(MQTT_CHANNEL_ADDRESS), MQTT_CHANNEL_LENGTH);
  
  g_updateProcess.msgType = UPDATE_GWID_MODIFY_FIN;
  g_updateProcess.msgIndentify = identify;
  
  return true;    
}
static bool isNeedValue(uint32_t iType)
{
  switch(iType)

  {
    case UPDATE_TYPE_GET_SOUND_DIST:
    case UPDATE_TYPE_GET_SOUND_CYCLE:
      return false;
    default:
      return true;
  }
}

static bool editServerConfig(cJSON *fireware_update, uint32_t iType, uint32_t identify)
{
  uint16_t endpointId = 0;
  
#define EDIT_TYPE_SERVER 0
#define EDIT_TYPE_EP 1  
  uint8_t isEditType = EDIT_TYPE_SERVER;
  cJSON *json_epid = cJSON_GetObjectItem(fireware_update, "endpointId");
  if(NULL != json_epid)
  {
    endpointId = json_epid->valueint; 
    isEditType = EDIT_TYPE_EP;
  }
  else
  {
    endpointId = 0; 
    isEditType = EDIT_TYPE_SERVER;
  }


  uint16_t data = 0;
  if(isNeedValue(iType))
  {
    cJSON *json_data = cJSON_GetObjectItem(fireware_update, "data");
    if(NULL == json_data)
    {
      return false;
    }  
    
    data = json_data->valueint;
  }
  
  
  T_Control_Cmd cmd;

  cmd.identify = identify;
  cmd.endpointId = endpointId;
  cmd.data.value = 0;

  switch(iType)
  {
    case UPDATE_TYPE_ENDPOINT_ADD:
      cmd.action = CMD_ADD_ENDPOINT;
      cmd.data.endpointId = data;
      break;
      
    case UPDATE_TYPE_ENDPOINT_DEL:
      cmd.action = CMD_DEL_ENDPOINT;
      cmd.data.endpointId = data;
      break;

    case UPDATE_TYPE_MODIFY_SERVER_CH:
      cmd.action = CMD_MODIFY_CHANNEL;
      cmd.data.channel= data;
      break;

    case UPDATE_TYPE_MODIFY_SOUND_DIST:
      if(EDIT_TYPE_EP == isEditType)
      {
        cmd.action = CMD_MODIFY_DISTANCE;
      }
      else
      {
        cmd.action = CMD_MODIFY_SERVER_DISTANCE;
      }
      cmd.data.disance = data;
      break;
      
    case UPDATE_TYPE_GET_SOUND_DIST:
      if(EDIT_TYPE_EP == isEditType)
      {
        cmd.action = CMD_GET_DISTANCE;
      }
      else
      {
        cmd.action = CMD_GET_SERVER_DISTANCE;
      }      
      break;
    
    case UPDATE_TYPE_MODIFY_SOUND_CYCLE:
      cmd.action = CMD_MODIFY_SCANCYCLE;
      cmd.data.scanCycle = data;
      break;
    
    case UPDATE_TYPE_GET_SOUND_CYCLE:
        cmd.action = CMD_GET_SCANCYCLE;
        break;

    default:
      return false;
      
  }
  
  cmd.crc = crc8_chk_value((uint8_t *)(&cmd), 11);    
  
  while((&huart2)->gState != HAL_UART_STATE_READY);
  memcpy(DMA2_TX_Buffer, (uint8_t *)(&cmd), sizeof(T_Control_Cmd));
  HAL_UART_Transmit_DMA(&huart2, DMA2_TX_Buffer, sizeof(T_Control_Cmd));                            
  
  g_F103SendTimeOut = HAL_GetTick();
  g_isF103Sending = true;
  
  g_F103IsUpdating = true;
  
  return true;
}

static bool generalUpdateCmd(uint8_t *jsonStr)
{
  bool result = false;
  
  cJSON *root;
  root = cJSON_Parse((char *)jsonStr);

  cJSON *fireware_update = cJSON_GetObjectItem(root,"fireware_update");
  if(NULL == fireware_update)
  {
    cJSON_Delete(root);
    return false;
  }


  cJSON *json_identify = cJSON_GetObjectItem(fireware_update, "identify");
  if(NULL == json_identify)
  {
    cJSON_Delete(root);
    return false;    
  }
  
  uint32_t identify = json_identify->valueint;

  cJSON *json_type = cJSON_GetObjectItem(fireware_update, "type");
  if(NULL == json_type)
  {
    cJSON_Delete(root);
    return false;
  }
  
  int iType = json_type->valueint;
  Msg_Debug("generalUpdateCmd iType = %d\r\n", iType);

  cJSON *json_simid = cJSON_GetObjectItem(fireware_update, "simid");
  if(NULL != json_simid)
  {
    if(0 != strcmp((char *)g_simCardId, json_simid->valuestring))
    {
      cJSON_Delete(root);
      //命令格式没有错，只是该更新不属于该网关
      return true;
    }
  }
  else
  {
    if(UPDATE_TYPE_STM32F405 != iType && UPDATE_TYPE_STM32F103 != iType)
    {
      cJSON_Delete(root);
      return false;      
    }
  }

  switch(iType)
  {
    case UPDATE_TYPE_STM32F405:
      result = doUpdateStart(fireware_update, iType, CURRENT_FIRMWARE);
      break;
    case UPDATE_TYPE_STM32F103:
      result = doUpdateStart(fireware_update, iType, (char *)(((T_FWF103Head *)g_stBootConfig.uiF103RunRomAddr)->ucFWVer));      
      break;

    case UPDATE_TYPE_MODIFY_GWID:
      result = editGWId(fireware_update, identify);
      break;

    case UPDATE_TYPE_ENDPOINT_ADD:
    case UPDATE_TYPE_ENDPOINT_DEL:
    case UPDATE_TYPE_MODIFY_SERVER_CH:
    case UPDATE_TYPE_MODIFY_SOUND_DIST:
    case UPDATE_TYPE_GET_SOUND_DIST:
    case UPDATE_TYPE_MODIFY_SOUND_CYCLE:
    case UPDATE_TYPE_GET_SOUND_CYCLE:
      result = editServerConfig(fireware_update, iType, identify);
      break;
      
    default:

      result = false;
      break;
  }
  
  cJSON_Delete(root);
  root = NULL;
  
  return result;
  
}


static void do_mqtt_other(uint8_t *pBuf, uint32_t length)
{
  if(NULL == pBuf)
  {
    return;
  }
  
  uint32_t topicLen = 0;
  uint32_t payloadLen = 0;
  
  switch (g_uart1RcvPhase)
  {
    case MQTT_RCV_INIT:
      if (0 != strncmp((char *)pBuf, RCVMSG1, strlen(RCVMSG1)))
      {
        Msg_Error("ERROR(MQTT_RCV_INIT) : %s\r\n", pBuf);
        NVIC_SystemReset();
        return;
      }
      g_rcvTopicLen = 0;
      g_rcvPayloadLen = 0;
      sscanf((char *)(pBuf + strlen(RCVMSG1)), "%u,%u", &g_rcvTopicLen, &g_rcvPayloadLen);
      if (0 == g_rcvTopicLen || 0 == g_rcvPayloadLen)
      {
        Msg_Error("ERROR(MQTT_RCV_INIT) : %s\r\n", pBuf);
        NVIC_SystemReset();
        return;
      }
      g_uart1RcvPhase = MQTT_RCV_SUM;
      break;
    case MQTT_RCV_SUM:
      #if 1
      if (0 != strncmp((char *)pBuf, RCVMSG2, strlen(RCVMSG2)))
      {
        Msg_Error("ERROR(MQTT_RCV_SUM) : %s\r\n", pBuf);
        NVIC_SystemReset();
        return;
      }

      sscanf((char *)(pBuf + strlen(RCVMSG2)), "%u", &topicLen);
      if (g_rcvTopicLen != topicLen)
      {
        Msg_Error("Error : usTopicLen = %u\r\n", topicLen);
        Msg_Error("ERROR(MQTT_RCV_SUM) : %s\r\n", pBuf);
        NVIC_SystemReset();
        return ;
      }
      #endif
      g_uart1RcvPhase = MQTT_RCV_TOPIC_LEN;
      break;
    case MQTT_RCV_TOPIC_LEN:
      #if 1
      if((15 != length) && strlen((char *)mqtt_channel) != length) 
      {
        Msg_Error("ERROR(MQTT_RCV_TOPIC_LEN) : %s, length = %d\r\n", pBuf, length);
        NVIC_SystemReset();
        return ;
      }

      if(0 == strncmp((char *)pBuf, SUB_UPDATE_CONTENT, strlen(SUB_UPDATE_CONTENT)))
      {
        g_uart1RcvTopicType = MQTT_RCV_TOPIC_UPDATE;
      }
      else if(0 == strncmp((char *)pBuf, (char *)mqtt_channel, strlen((char *)mqtt_channel)))
      {
        g_uart1RcvTopicType = MQTT_RCV_TOPIC_CONTROL;
      }
      else
      {
        Msg_Error("ERROR(MQTT_RCV_TOPIC_LEN) (update topic): %s\r\n", pBuf);
        NVIC_SystemReset();
        return ;
      }
      
      g_uart1RcvPhase = MQTT_RCV_TOPIC_CONTENT;
      #endif
      
      break;
    case MQTT_RCV_TOPIC_CONTENT:
      #if 1
      if (0 != strncmp((char *)pBuf, RCVMSG3, strlen(RCVMSG3)))
      {
        Msg_Error("ERROR(MQTT_RCV_TOPIC_CONTENT) : %s\r\n", pBuf);
        NVIC_SystemReset();
        return;
      }
      sscanf((char *)(pBuf + strlen(RCVMSG3)), "%u", &payloadLen);
      if (payloadLen != g_rcvPayloadLen)
      {
        Msg_Error("ERROR(MQTT_RCV_TOPIC_CONTENT) : %s\r\n", pBuf);
        NVIC_SystemReset();
        return;
      }
      #endif
      g_uart1RcvPhase = MQTT_RCV_PAYLOAD_LEN;
      break;
    case MQTT_RCV_PAYLOAD_LEN:
      #if 1

      
      if (length != g_rcvPayloadLen)
      {
        Msg_Error("ERROR(MQTT_RCV_PAYLOAD_LEN) : %s\r\n", pBuf);
        NVIC_SystemReset();
        return;
      }


      if(MQTT_RCV_TOPIC_CONTROL== g_uart1RcvTopicType)
      {
        generalCmd((uint8_t *)pBuf);
      }
      else if(MQTT_RCV_TOPIC_UPDATE == g_uart1RcvTopicType)
      {
        if(!generalUpdateCmd((uint8_t *)pBuf))
        {
          g_updateProcess.msgType = UPDATE_ERROR;
        }
      }
      //for do ,parser server data,fill cmd

       //analysis and send to uart
      #endif
      g_uart1RcvPhase = MQTT_RCV_PAYLOAD_CONTENT;
      break; 
      
    case MQTT_RCV_PAYLOAD_CONTENT:
      #if 1
      if (0 != strncmp((char *)pBuf, RCVMSG4, strlen(RCVMSG4)))
      {
        Msg_Error("ERROR(MQTT_RCV_PAYLOAD_CONTENT) : %s\r\n", pBuf);
        NVIC_SystemReset();
        return;
      }
      #endif
      g_uart1RcvPhase = MQTT_RCV_INIT;
       
      break;
    default:
      Msg_Error("Error : default\r\n");
      NVIC_SystemReset();
      break;
  }  
}

static void anlyz_uart1_one_data(void)
{
  uint8_t * begin = NULL;
  uint16_t length = 0;
  bool flag = false;
  if (g_retBegin < g_retEnd)
  {
    begin = UART_Buffer + g_retBegin;
    length = g_retEnd - g_retBegin;
  }
  else
  {
    if ((g_retBegin > g_retEnd))
    {
      length = UART_BUFFER_SIZE - g_retBegin + g_retEnd;
    }
    else
    {
      length = UART_BUFFER_SIZE;
    }
    flag = true;
  }
  if (true == flag)
  {
    begin = (uint8_t *)malloc(length + 1);
    if (NULL == begin)
    {
      Msg_Error("malloc Error 2\r\n");
      NVIC_SystemReset();
    }
    memcpy(begin, UART_Buffer + g_retBegin, UART_BUFFER_SIZE - g_retBegin);
    memcpy(begin + UART_BUFFER_SIZE - g_retBegin, UART_Buffer, g_retEnd);
    begin[length] = 0;
  }
  begin[length] = 0;
  
  if (2 == length && 0 == strncmp((char *)begin, "OK", 2))
  {
    if(UART1_SENDING_MQTT == g_isUart1Sending)
    {
      do_mqtt_ok();
    }
    else if(UART1_SENDING_FTP == g_isUart1Sending)
    {      
      do_update_ok();
    }   
  }
  else if (MQTT_SND_CMD_RSP2 == g_uart1Phase && 0 == strncmp((char *)begin, "+CMQTTPUB: 0,", 13))
  {
    g_sndStartTick = 0;
    int32_t retCode = -1;
    sscanf((char *)(begin + 13), "%u", &retCode);
    if (-1 != retCode)
    {
      g_uart1Phase = MQTT_SND_INIT;
      g_isUart1Sending = UART1_SENDING_NONE;
    }
    else
    {
      Msg_Error("MQTT_SND_CMD_RSP2 : %s\r\n", begin);
      NVIC_SystemReset();
    }
  }
  else
  {    
    
    if(!do_update_other(begin))
    {      
      do_mqtt_other(begin, length);
    }
  }

  if (true == flag)
  {
    free(begin);
  }
  else
  {
    begin[length] = '\r';
  }
}

void parser_uart1_msg(void)
{
    uint32_t tmpWrite = Uart1_Write;
    while (Uart1_Read != tmpWrite)
    {
      if (RET_POS_INVALID == g_retBegin)
      {

        if(UPDATE_FW_FTPCACHERD_DATA_LEN_END == g_stFWUpdateInfo.uiUpdateStep)
        {
          if(g_stFWUpdateInfo.uiPerDataSize > 0)
          {
            if (0x0d == UART_Buffer[Uart1_Read])
            {
              goto label;
            }

            if (0x0a == UART_Buffer[Uart1_Read])
            {
              g_stFWUpdateInfo.uiUpdateStep = UPDATE_FW_FTPCACHERD_DATA;
              goto label;
            }
          }
        }
        else if(UPDATE_FW_FTPCACHERD_DATA == g_stFWUpdateInfo.uiUpdateStep)
        {
          g_retBegin = Uart1_Read;
        }
        else 
        {
          if (0x0d == UART_Buffer[Uart1_Read] || 0x0a == UART_Buffer[Uart1_Read])
          {
            goto label;
          }
          if ('>' ==  UART_Buffer[Uart1_Read]) //special handle
          {
            if (MQTT_SND_TOPIC == g_uart1Phase)
            {
              g_uart1Phase = MQTT_SND_TOPIC_RSP;
            }
            else if (MQTT_SND_PAYLOAD == g_uart1Phase)
            {
              g_uart1Phase = MQTT_SND_PAYLOAD_RSP;
            }
            Send_Mqtt_Msg_Continue();
            goto label;
          }
          g_retBegin = Uart1_Read;
        }
        
      }
      else
      {
        if(UPDATE_FW_FTPCACHERD_DATA == g_stFWUpdateInfo.uiUpdateStep)
        {

          if(g_stFWUpdateInfo.uiPerDataSize > 0)
          {          
            uint32_t uiDataLen = (Uart1_Read + UART_BUFFER_SIZE - g_retBegin) % UART_BUFFER_SIZE;

            if(uiDataLen == g_stFWUpdateInfo.uiPerDataSize)
            {
              g_retEnd = Uart1_Read;
              writeDataToFlash();
              g_retBegin = RET_POS_INVALID;
              g_retEnd = RET_POS_INVALID;
            }            
          }              
        }
        else
        {
          if (0x0d == UART_Buffer[Uart1_Read])
          {
            g_retEnd = Uart1_Read;

            anlyz_uart1_one_data();

            g_retBegin = RET_POS_INVALID;
            g_retEnd = RET_POS_INVALID;
          }          
        }
      }
label:
      Uart1_Read++;

      if (Uart1_Read == UART_BUFFER_SIZE)
      {
        /* Check buffer overflow */
        Uart1_Read = 0;
      }
    }
}

static int anlyz_uart2_one_data(T_Resp_Info *resp, uint8_t *topicType, uint8_t *json_resp)
  {
    if(NULL == resp || NULL == json_resp)
    {
      return UART2_RESP_ERROR;
    }
  
#define TYPE_NORMAL 0
#define TYPE_ONLINE 1
#define TYPE_OFFLINE 2
#define TYPE_INTERUPT 3
#define TYPE_GET_RUN_BIN 4
#define TYPE_GET_RECOVERY_BIN 5
#define TYPE_GET_RUN_BIN_FIN 6
#define TYPE_GET_RECOVERY_BIN_FIN 7
#define TYPE_UPLOAD_HAVE_CAR 8
#define TYPE_ENDPOINT_ADD_FIN 9
#define TYPE_ENDPOINT_DEL_FIN 10
#define TYPE_CHANNEL_MODIFY_FIN 11
#define TYPE_SET_SOUND_DIST_SUCCESS 12

#define TYPE_SET_SOUND_DIST_FAILED 13

#define TYPE_GET_SOUND_DIST_FIN 14
#define TYPE_SET_SOUND_SCANCYCLE_SUCCESS 15
#define TYPE_SET_SOUND_SCANCYCLE_FAILED 16

#define TYPE_GET_SOUND_SCANCYCLE_FIN 17

    uint8_t resp_code = 0;
    uint8_t resp_type = UART2_RESP_ERROR;//0->normal,1->online, 2->offline,3->interupt
    switch(resp->resp_code)
    {
      case NORMAL_SUCCESS:
        resp_code = 0;
        resp_type = TYPE_NORMAL;      
        break;
      case NORMAL_DOWN:
        resp_code = 1;
        resp_type = TYPE_NORMAL;  
        break;
      case NORMAL_FORWARD:
        resp_code = 2;
        resp_type = TYPE_NORMAL; 
        break;
      case NORMAL_UP:
        resp_code = 3;
        resp_type = TYPE_NORMAL;
        break;
      case NORMAL_BACK:
        resp_code = 4;
        resp_type = TYPE_NORMAL;
        break;
      case NORMAL_BUSY:
        resp_code = 9;
        resp_type = TYPE_NORMAL;
        break;
      case NORMAL_BEEP_OPEN_FAILED:      
        resp_code = 1;
        resp_type = TYPE_NORMAL;
        break;
      case NORMAL_BEEP_CLOSE_FAILED:      
        resp_code = 1;
        resp_type = TYPE_NORMAL;
        break;
      case NORMAL_BEEP_STATUS_OPEN:      
        resp_code = 1;
        resp_type = TYPE_NORMAL;
  
        break;
      case NORMAL_BEEP_STATUS_CLOSED:      
        resp_code = 0;
        resp_type = TYPE_NORMAL;
        break;

//      case NORMAL_MOTOR_RUNNING:
//        resp_code = 10;
//        resp_type = TYPE_NORMAL;
//        break;
        
      case NORMAL_HAVE_CAR:
        resp_code = 11;
        resp_type = TYPE_NORMAL;
        break;
        
      case NORMAL_UNKNOWN_ERROR:
        resp_code = 255;        
        resp_type = TYPE_NORMAL;
        break;
        
      case NORMAL_CAR_CHECK_ERROR:
        resp_code = 12;        
        resp_type = TYPE_NORMAL;
        break;
        
      case CHILD_ONLINE:
        resp_type = TYPE_ONLINE;
        
        break;
        
      case CHILD_OFFLINE:
        resp_type = TYPE_OFFLINE;
        break;

      
      case NODE_NOTEXIST:
        resp_code = 254;        
        resp_type = TYPE_NORMAL;
        break;
        
      case NODE_NO_SUPPORT_CMD:
        resp_code = 253;
        resp_type = TYPE_NORMAL;
        break;
        
      case GET_RUN_BIN:     
        resp_type = TYPE_GET_RUN_BIN;
        break;
        
      case GET_RECOVERY_BIN:     
        resp_type = TYPE_GET_RECOVERY_BIN;
        break; 
        
      case GET_RUN_BIN_FIN:     
        resp_type = TYPE_GET_RUN_BIN_FIN;
        break;
        
      case GET_RECOVERY_BIN_FIN:     
        resp_type = TYPE_GET_RECOVERY_BIN_FIN;
        break;
        
      case ENDPOINT_ADD_SUCCESS:     
        resp_type = TYPE_ENDPOINT_ADD_FIN;
        break;
        
      case ENDPOINT_DEL_SUCCESS:     
        resp_type = TYPE_ENDPOINT_DEL_FIN;
        break;
        
      case CHANNEL_MODIFY_SUCCESS:     
        resp_type = TYPE_CHANNEL_MODIFY_FIN;
        break;
        
      case SOUND_DIST_SET_SUCCESS:     
        resp_type = TYPE_SET_SOUND_DIST_SUCCESS;
        break;
        
      case SOUND_DIST_SET_FAILED:     
        resp_type = TYPE_SET_SOUND_DIST_FAILED;
        break;
      
      case SOUND_DIST_GET_SUCCESS:     
        resp_type = TYPE_GET_SOUND_DIST_FIN;
        break;
      
      case SOUND_SCANCYCLE_SET_SUCCESS:     
        resp_type = TYPE_SET_SOUND_SCANCYCLE_SUCCESS;
        break;

      case SOUND_SCANCYCLE_SET_FAILED:     
        resp_type = TYPE_SET_SOUND_SCANCYCLE_FAILED;
        break;
      
      case SOUND_SCANCYCLE_GET_SUCCESS:     
        resp_type = TYPE_GET_SOUND_SCANCYCLE_FIN;
        break;

      case UPLOAD_HAVE_CAR:
        resp_code = 1;        
        resp_type = TYPE_UPLOAD_HAVE_CAR;
        break;
        
      case UPLOAD_NO_HAVE_CAR:
        resp_code = 0;        
        resp_type = TYPE_UPLOAD_HAVE_CAR;
        break;
        
      case UPLOAD_SOUNDWAVE_TROUBLE:
        resp_code = 2;        
        resp_type = TYPE_UPLOAD_HAVE_CAR;
        break;
        
      case INTERUPT_DOWN:      
        resp_code = 1;
        resp_type = TYPE_INTERUPT;
        break;
        
      case INTERUPT_FORWARD:      
        resp_code = 2;
        resp_type = TYPE_INTERUPT;
        break;
        
      case INTERUPT_UP:      
        resp_code = 3;
        resp_type = TYPE_INTERUPT;
        break;
        
      case INTERUPT_BACK:     
        resp_code = 4;
        resp_type = TYPE_INTERUPT;
        break;
        
      case INTERUPT_TIMEOUT:     
        resp_code = 9;
        resp_type = TYPE_NORMAL;
        break;
     
      default:
        if(resp->resp_code<=100)
        {
          resp_code = resp->resp_code;
          resp_type = TYPE_NORMAL;          
        }
        else
        {
          Msg_Warn("anlyz_uart2_one_data error! resp->resp_code = %d\n", resp->resp_code);
          return UART2_RESP_ERROR;         
        }
   }
  
    int res = UART2_RESP_CONTROL;
    uint8_t info[8] = {0};
    switch(resp_type)
    {
      case TYPE_NORMAL:
        normal_resp(json_resp, resp_code, resp->identity);
        *topicType = MQTT_TOPIC_TYPE_ACTION_RSP;
        break;
      case TYPE_ONLINE:
        if(change_device_status(resp->endpointId, DEVICE_STATUS_ONLINE))
        {
          child_online_resp(json_resp, mqtt_channel, resp->endpointId, resp->data.online_data.EPStatus, resp->data.online_data.haveCarFlg);
        
	        *topicType = MQTT_TOPIC_TYPE_CHILD_ONLINE;
        }
        else
        {
          return UART2_RESP_ERROR;
        }
        
        break;
      case TYPE_OFFLINE:
        if(change_device_status(resp->endpointId, DEVICE_STATUS_OFFLINE))
        {
          child_offline_resp(json_resp, resp->endpointId, mqtt_channel);
	        *topicType = MQTT_TOPIC_TYPE_CHILD_OFFLINE;
        }
        else
        {
          return UART2_RESP_ERROR;
        }
        break;
      case TYPE_INTERUPT:
        interupt_resp(json_resp, mqtt_channel, resp->endpointId, resp_code);
        *topicType = MQTT_TOPIC_TYPE_INTERRUPT_RSP;
        break;
      case TYPE_GET_RUN_BIN:
        res = UART2_RESP_GET_RUN_BIN;
        break;
      case TYPE_GET_RECOVERY_BIN:
        res = UART2_RESP_GET_RECOVERY_BIN;
        break;
      case TYPE_GET_RUN_BIN_FIN:
        firmware_resp(json_resp, g_simCardId, 1, 0, 0, resp->data.version);
        *topicType = MQTT_TOPIC_TYPE_FIRMWARE_UPDATE;
        
        res = UART2_RESP_GET_RUN_BIN_FIN;
        break;
        
      case TYPE_GET_RECOVERY_BIN_FIN:
        firmware_resp(json_resp, g_simCardId, 1, 1, 0, "F103 update failed");
        *topicType = MQTT_TOPIC_TYPE_FIRMWARE_UPDATE;

        res = UART2_RESP_GET_RECOVERY_BIN_FIN;
        break;
      case TYPE_ENDPOINT_ADD_FIN:
        firmware_resp(json_resp, g_simCardId, 2, 0, resp->identity, "success");
        *topicType = MQTT_TOPIC_TYPE_FIRMWARE_UPDATE;

        res = UART2_RESP_EDIT_FIN;
        break;

      case TYPE_ENDPOINT_DEL_FIN:
        firmware_resp(json_resp, g_simCardId, 3, 0, resp->identity, "success");
        *topicType = MQTT_TOPIC_TYPE_FIRMWARE_UPDATE;
        
        res = UART2_RESP_EDIT_FIN;
        break;

      case TYPE_CHANNEL_MODIFY_FIN:
        firmware_resp(json_resp, g_simCardId, 5, 0, resp->identity, "success");
        *topicType = MQTT_TOPIC_TYPE_FIRMWARE_UPDATE;
        
        res = UART2_RESP_EDIT_FIN;
        break;
        
      case TYPE_SET_SOUND_DIST_SUCCESS:
        firmware_resp(json_resp, g_simCardId, 6, 0, resp->identity, "success");

        *topicType = MQTT_TOPIC_TYPE_FIRMWARE_UPDATE;
        
        res = UART2_RESP_EDIT_FIN;
        break;
        
      case TYPE_SET_SOUND_DIST_FAILED:
        firmware_resp(json_resp, g_simCardId, 6, 1, resp->identity, "failed");
        *topicType = MQTT_TOPIC_TYPE_FIRMWARE_UPDATE;
        
        res = UART2_RESP_EDIT_FIN;
        break;
      
      case TYPE_SET_SOUND_SCANCYCLE_SUCCESS:
        firmware_resp(json_resp, g_simCardId, 8, 0, resp->identity, "success");

        *topicType = MQTT_TOPIC_TYPE_FIRMWARE_UPDATE;
        
        res = UART2_RESP_EDIT_FIN;
        break;

      case TYPE_SET_SOUND_SCANCYCLE_FAILED:
        firmware_resp(json_resp, g_simCardId, 8, 1, resp->identity, "failed");
        *topicType = MQTT_TOPIC_TYPE_FIRMWARE_UPDATE;
        
        res = UART2_RESP_EDIT_FIN;
        break;
        
      case TYPE_GET_SOUND_DIST_FIN:
        sprintf((char *)info, "%d cm", resp->data.distance);
        firmware_resp(json_resp, g_simCardId, 7, 0, resp->identity, info);
        *topicType = MQTT_TOPIC_TYPE_FIRMWARE_UPDATE;
        
        res = UART2_RESP_EDIT_FIN;
        break;
        
      case TYPE_GET_SOUND_SCANCYCLE_FIN:
        sprintf((char *)info, "%d cm", resp->data.scanCycle);
        firmware_resp(json_resp, g_simCardId, 9, 0, resp->identity, info);
        *topicType = MQTT_TOPIC_TYPE_FIRMWARE_UPDATE;

        
        res = UART2_RESP_EDIT_FIN;
        break;
      case TYPE_UPLOAD_HAVE_CAR:
        sound_check_resp(json_resp, mqtt_channel, resp->endpointId, resp_code);
        *topicType = MQTT_TOPIC_TYPE_HAVE_CAR;
      
        res = UART2_RESP_HAVE_CAR;
        
        break;
        
      default:
        return UART2_RESP_ERROR;
  
    }
  
    return res;
  
  }

void parser_uart2_msg(void)
{
  //此处设置一个超时，防止收不到响应，导致g_isMqttSending一直是true，从而无法再正常工作。
  if(HAL_GetTick() - g_Uart1SendTimeOut > 10000)
  {
    g_isUart1Sending = UART1_SENDING_NONE;
  }

  if(UART1_SENDING_NONE != g_isUart1Sending)
  {
    return;
  }

  uint32_t tmpWrite = Uart2_Write;
  if ((Uart2_Read != tmpWrite) && 0 == tmpWrite%sizeof(T_Resp_Info))
  {
    T_Resp_Info resp;
    memcpy((uint8_t *)(&resp), UART2_Buffer+Uart2_Read, sizeof(T_Resp_Info));

    uint8_t crc = crc8_chk_value((uint8_t*)(&resp), 11);
    if(crc != resp.crc)
    {
      Msg_Warn("crc check error!!!! response crc = %02x, resp.crc = %02x\r\n", crc, resp.crc);
      goto next_data;
    }
    
    uint8_t json_resp[256];
    uint8_t topicType;
    uint32_t res;
    
    res = anlyz_uart2_one_data(&resp, &topicType, json_resp);

    
    Msg_Debug("receive res form 103 type(%d)\r\n", res);
    if(UART2_RESP_CONTROL == res)
    {
      g_Uart1SendTimeOut = HAL_GetTick();
      g_isUart1Sending = UART1_SENDING_MQTT;
      Send_Mqtt_Msg(topicType, json_resp, strlen((char *)json_resp));
    }
    else if(UART2_RESP_GET_RUN_BIN == res)
    {
      T_FWF103Head stFwF103Head;
      memcpy(&stFwF103Head, (uint8_t *)g_stBootConfig.uiF103UpdateRomAddr, sizeof(T_FWF103Head));
      
      while((&huart2)->gState != HAL_UART_STATE_READY);
      HAL_UART_Transmit_DMA(&huart2, (uint8_t *)g_stBootConfig.uiF103UpdateRomAddr, stFwF103Head.uiTotalLen + sizeof(T_FWF103Head));       
    }
    else if(UART2_RESP_GET_RECOVERY_BIN == res)
    {
      T_FWF103Head stFwF103Head;
      memcpy(&stFwF103Head, (uint8_t *)g_stBootConfig.uiF103RunRomAddr, sizeof(T_FWF103Head));
      
      while((&huart2)->gState != HAL_UART_STATE_READY);
      HAL_UART_Transmit_DMA(&huart2, (uint8_t *)g_stBootConfig.uiF103RunRomAddr, stFwF103Head.uiTotalLen + sizeof(T_FWF103Head));       
    }
    else if(UART2_RESP_GET_RUN_BIN_FIN == res)
    {
      g_Uart1SendTimeOut = HAL_GetTick();
      g_isUart1Sending = UART1_SENDING_MQTT;
      Send_Mqtt_Msg(topicType, json_resp, strlen((char *)json_resp));
      
      uint32_t temp = g_stBootConfig.uiF103RunRomAddr;
      g_stBootConfig.uiF103RunRomAddr = g_stBootConfig.uiF103UpdateRomAddr;
      g_stBootConfig.uiF103UpdateRomAddr = temp;

      HAL_IWDG_Refresh(&hiwdg);
      FLASH_If_Erase(CONFIG_ADDRESS);
      HAL_IWDG_Refresh(&hiwdg);
      FLASH_If_Write(CONFIG_ADDRESS, (uint32_t*)(&g_stBootConfig), sizeof(T_BootConfig));

      g_F103IsUpdating = false;
      
      HAL_IWDG_Refresh(&hiwdg);
      FLASH_If_Erase(g_stBootConfig.uiF103UpdateRomAddr);

      
      //g_update_restart_tick = HAL_GetTick();

    }
    else if(UART2_RESP_GET_RECOVERY_BIN_FIN == res)
    {
      g_Uart1SendTimeOut = HAL_GetTick();
      g_isUart1Sending = UART1_SENDING_MQTT;
      Send_Mqtt_Msg(topicType, json_resp, strlen((char *)json_resp));      

      g_F103IsUpdating = false;    
      
      HAL_IWDG_Refresh(&hiwdg);
      FLASH_If_Erase(g_stBootConfig.uiF103UpdateRomAddr);


    }    
    else if(UART2_RESP_EDIT_FIN == res)
    {
      g_Uart1SendTimeOut = HAL_GetTick();
      g_isUart1Sending = UART1_SENDING_MQTT;
      Send_Mqtt_Msg(topicType, json_resp, strlen((char *)json_resp));      

      g_F103IsUpdating = false;    
    }    
    else if(UART2_RESP_HAVE_CAR == res)
    {
      g_Uart1SendTimeOut = HAL_GetTick();
      g_isUart1Sending = UART1_SENDING_MQTT;
      Send_Mqtt_Msg(topicType, json_resp, strlen((char *)json_resp));      
    }
    else
    {
      Msg_Warn("res error res = %d\r\n", res);
    }

    next_data:
    Uart2_Read += sizeof(T_Resp_Info);
    
    if (Uart2_Read == UART2_BUFFER_SIZE)
    {
      Uart2_Read = 0;
    }
  } 
}


void send_update_msg(void)
{
  if(UPDATE_STEP_NONE == g_stFWUpdateInfo.uiUpdateStep)
  {
    return;
  }
  
  if(HAL_GetTick() - g_Uart1SendTimeOut > 10000)
  {
    g_isUart1Sending = UART1_SENDING_NONE;
  }

  if(UART1_SENDING_NONE != g_isUart1Sending)
  {
    return;
  }

  g_Uart1SendTimeOut = HAL_GetTick();
  g_isUart1Sending = UART1_SENDING_FTP;

  if(UPDATE_MD5_SEND == g_stFWUpdateInfo.uiUpdateStep)
  {
    
    while((&huart1)->gState != HAL_UART_STATE_READY);
    sprintf((char *)DMA_TX_Buffer, AT_FTPGETMD5, (char *)g_stFWUpdateInfo.ucPath, (char *)g_stFWUpdateInfo.ucBoardName, (char *)g_stFWUpdateInfo.ucFWVer);
    Msg_Debug("UPDATE_MD5_SEND DMA_TX_Buffer = %s\r\n", DMA_TX_Buffer);
    HAL_UART_Transmit_DMA(&huart1, DMA_TX_Buffer, strlen((char *)DMA_TX_Buffer)); 

    g_stFWUpdateInfo.uiUpdateStep = UPDATE_MD5_RCV_OK;
    
  }
  else if(UPDATE_SIZE_SEND == g_stFWUpdateInfo.uiUpdateStep)
  {
    while((&huart1)->gState != HAL_UART_STATE_READY);
    sprintf((char *)DMA_TX_Buffer, AT_FTPGETLIST, (char *)g_stFWUpdateInfo.ucPath, (char *)g_stFWUpdateInfo.ucBoardName, (char *)g_stFWUpdateInfo.ucFWVer, g_stFWUpdateInfo.ucBoardName);
    Msg_Debug("UPDATE_SIZE_SEND DMA_TX_Buffer = %s\r\n", DMA_TX_Buffer);
    HAL_UART_Transmit_DMA(&huart1, DMA_TX_Buffer, strlen((char *)DMA_TX_Buffer)); 

    g_stFWUpdateInfo.uiUpdateStep = UPDATE_SIZE_RCV_OK;    
  }
  else if(UPDATE_FW_SEND == g_stFWUpdateInfo.uiUpdateStep)
  {
    while((&huart1)->gState != HAL_UART_STATE_READY);
    sprintf((char *)DMA_TX_Buffer, AT_FTPGET, (char *)g_stFWUpdateInfo.ucPath, (char *)g_stFWUpdateInfo.ucBoardName, (char *)g_stFWUpdateInfo.ucFWVer, g_stFWUpdateInfo.ucBoardName);
    Msg_Debug("UPDATE_FW_SEND DMA_TX_Buffer = %s\r\n", DMA_TX_Buffer);
    HAL_UART_Transmit_DMA(&huart1, DMA_TX_Buffer, strlen((char *)DMA_TX_Buffer)); 

    g_stFWUpdateInfo.uiUpdateStep = UPDATE_FW_RCV_OK;    
  }
  else if(UPDATE_FW_FTPCACHERD_SIZE_SEND == g_stFWUpdateInfo.uiUpdateStep)
  {
    while((&huart1)->gState != HAL_UART_STATE_READY);
    strcpy((char *)DMA_TX_Buffer, AT_FTPCACHERD_SIZE);
    HAL_UART_Transmit_DMA(&huart1, DMA_TX_Buffer, strlen((char *)DMA_TX_Buffer)); 

    g_stFWUpdateInfo.uiUpdateStep = UPDATE_FW_FTPCACHERD_SIZE_RES;    
  }
  else if(UPDATE_FW_FTPCACHERD_DATA_SEND == g_stFWUpdateInfo.uiUpdateStep)
  {
    while((&huart1)->gState != HAL_UART_STATE_READY);
    strcpy((char *)DMA_TX_Buffer, AT_FTPCACHERD_DATA);
    HAL_UART_Transmit_DMA(&huart1, DMA_TX_Buffer, strlen((char *)DMA_TX_Buffer)); 

    g_stFWUpdateInfo.uiUpdateStep = UPDATE_FW_FTPCACHERD_DATA_LEN;
  }
  else
  {
  }  
}

void uploadUpdateProcess(void)
{
  //此处设置一个超时，防止收不到响应，导致g_isMqttSending一直是true，从而无法再正常工作。
  if(HAL_GetTick() - g_Uart1SendTimeOut > 10000)
  {
    g_isUart1Sending = UART1_SENDING_NONE;
  }

  if(UART1_SENDING_NONE != g_isUart1Sending)
  {
    return;
  }

  uint8_t msgInfo[256] = {0};
  uint8_t topicType = 0;

  if(UPDATE_PROCESS_NONE == g_updateProcess.msgType)
  {
    return;
  }
  else if(UPDATE_F405_DOWNLOAD_FIN == g_updateProcess.msgType)
  {
    topicType = MQTT_TOPIC_TYPE_FIRMWARE_UPDATE;
    //sprintf((char *)msgInfo, "F405 %s download success!", g_stBootConfig.ucNewFWVer); 
    g_update_restart_tick = HAL_GetTick();
  }
  else if(UPDATE_F103_DOWNLOAD_FIN == g_updateProcess.msgType)
  {
    T_FWF103Head stFwF103Head;
    memcpy(&stFwF103Head, (uint8_t *)g_stBootConfig.uiF103UpdateRomAddr, sizeof(T_FWF103Head));

    topicType = MQTT_TOPIC_TYPE_FIRMWARE_UPDATE;
    //sprintf((char *)msgInfo, "F103 %s download success!", stFwF103Head.ucFWVer);
    
  }
  else if(UPDATE_F405_UPDATE_FIN == g_updateProcess.msgType)
  {
    topicType = MQTT_TOPIC_TYPE_FIRMWARE_UPDATE;
    firmware_resp(msgInfo, g_simCardId, 0, 0, 0, CURRENT_FIRMWARE);    
  }
  else if(UPDATE_GWID_MODIFY_FIN == g_updateProcess.msgType)
  {
    topicType = MQTT_TOPIC_TYPE_FIRMWARE_UPDATE;
    firmware_resp(msgInfo, g_simCardId, 4, 0, g_updateProcess.msgIndentify, "success");    

    g_update_restart_tick = HAL_GetTick();
    
  }

  else if(UPDATE_ERROR == g_updateProcess.msgType)
  {
    topicType = MQTT_TOPIC_TYPE_FIRMWARE_UPDATE;
    sprintf((char *)msgInfo, "error !!");      
  }
  
  g_Uart1SendTimeOut = HAL_GetTick();
  g_isUart1Sending = UART1_SENDING_MQTT;
  Send_Mqtt_Msg(topicType, msgInfo, strlen((char *)msgInfo));

  
  Msg_Debug("send update process to server areadly(%s)\r\n", msgInfo);

  memset(&g_updateProcess, 0, sizeof(T_UpdateProcess));
  
}

void uploadFaultLog(void)
{
  if(0 == g_needUploadLog)
  {
    return;
  }
  
  if(HAL_GetTick() - g_Uart1SendTimeOut > 10000)
  {
    g_isUart1Sending = UART1_SENDING_NONE;
  }
  
  if(UART1_SENDING_NONE != g_isUart1Sending)
  {
    return;
  }

  g_Uart1SendTimeOut = HAL_GetTick();
  g_isUart1Sending = UART1_SENDING_MQTT;
  Send_Mqtt_Msg(MQTT_TOPIC_TYPE_FIRMWARE_UPDATE, g_Log, strlen((char *)g_Log));

  memset(g_Log, 0, 256);

  g_needUploadLog = 0;
  
}


void send_msg_to_103(void)
{  
  if(HAL_GetTick() - g_F103SendTimeOut > 1000)
  {
    g_isF103Sending = false;
  }

  if(g_isF103Sending || g_F103IsUpdating)
  {
    return;
  }

  uint32_t tmpWrite = Uart2_Send_Write;
  if ((Uart2_Send_Read != tmpWrite) && 0 == tmpWrite%sizeof(T_Control_Cmd))
  {

    while((&huart2)->gState != HAL_UART_STATE_READY);
    memcpy(DMA2_TX_Buffer, UART2_TX_Buffer+Uart2_Send_Read, sizeof(T_Control_Cmd));
    HAL_UART_Transmit_DMA(&huart2, DMA2_TX_Buffer, sizeof(T_Control_Cmd)); 

    Msg_Debug("send to 103(identify=%d,action = %d)\r\n", *(uint32_t *)(UART2_TX_Buffer+Uart2_Send_Read), *(uint8_t *)(UART2_TX_Buffer+Uart2_Send_Read+6));
    
    g_F103SendTimeOut = HAL_GetTick();
    g_isF103Sending = true;

    Uart2_Send_Read += sizeof(T_Control_Cmd);
    
    if (Uart2_Send_Read == UART2_TX_BUFFER_SIZE)
    {
      Uart2_Send_Read = 0;
    }
  } 

};

void update_restart(void)
{
  if(0 == g_update_restart_tick)
  {
    return;
  }

  //当前还有业务没有处理完毕，先不重启
  if((Uart1_Read != Uart1_Write) 
    || Uart2_Read != Uart2_Write 
    || Uart2_Send_Read != Uart2_Send_Write 
    || g_updateProcess.msgType != UPDATE_PROCESS_NONE)
  {
    Msg_Debug("Uart1_Read = %d, Uart1_Write = %d\r\n", Uart1_Read, Uart1_Write);
    Msg_Debug("Uart2_Read = %d, Uart2_Write = %d\r\n", Uart2_Read, Uart2_Write);
    Msg_Debug("Uart2_Send_Read = %d, Uart2_Send_Write = %d\r\n", Uart2_Send_Read, Uart2_Send_Write);
    Msg_Debug("Uart2_Send_Read = %d\r\n", Uart2_Send_Read);
    g_update_restart_tick = HAL_GetTick();
    return;
  }

  //业务处理完毕后，5秒后重启设备
  if(HAL_GetTick() - g_update_restart_tick > 5000)
  {

    Msg_Debug("update 405 restart\r\n");
    
    NVIC_SystemReset();
  }
}

uint32_t g_LedTick = 0;
GPIO_PinState gpio_pin = GPIO_PIN_RESET;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  __enable_irq();

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  MX_USART3_UART_Init();
    
  /* USER CODE BEGIN 2 */
  HAL_IWDG_Refresh(&hiwdg);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); // reset SIM7600CE
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); // enable idle line interrupt
  __HAL_UART_CLEAR_IDLEFLAG(&huart1);

  HAL_UART_Receive_DMA(&huart1, DMA_RX_Buffer, DMA_RX_BUFFER_SIZE);// 脝么露炉DMA陆脫脢脮
  HAL_UART_Receive_DMA(&huart2, DMA2_RX_Buffer, DMA2_RX_BUFFER_SIZE);// 脝么露炉DMA陆脫脢脮

  UART2_Buffer = (uint8_t *)malloc(UART2_BUFFER_SIZE);
  if(UART2_Buffer == NULL)
  {
    Msg_Error("UART2_Buffer malloc failed!\r\n");
    return 0;
  }

  UART2_TX_Buffer = (uint8_t *)malloc(UART2_TX_BUFFER_SIZE);
  if(UART2_Buffer == NULL)
  {
    Msg_Error("UART2_Buffer malloc failed!\r\n");
    return 0;
  }
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  g_startTick = HAL_GetTick();

  //flash初始化
  
  Msg_Debug("start init flash\r\n");
  FLASH_If_Init();

  //读取mqtt的channel
  getMqttChannel();  

  //填充获取online终端的命令
  Msg_Debug("start get online endpoint\r\n");
  setCurrOnlineDeviceCmd();

  //mqtt初始化
  Msg_Debug("start get init mqtt\r\n");
  mqtt_init();

  //ftp服务初始化
  Msg_Debug("start get init ftp\r\n");
  ftp_init();
   
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */



  Msg_Debug("ready current version:%s\r\n", CURRENT_FIRMWARE);
  
  g_uart1Phase = MQTT_SND_INIT;
  g_retBegin = RET_POS_INVALID;
  g_retEnd = RET_POS_INVALID;

  //远程升级后，系统成功启动需要通知mqtt服务器当前升级状态。
  if(g_isNeedSendUpdateSuccMsg)
  {
    g_updateProcess.msgType = UPDATE_F405_UPDATE_FIN;
    
    g_isNeedSendUpdateSuccMsg = false;
  }

  g_LedTick = 0;
  while (1)
  {
    
    if(HAL_GetTick() - g_LedTick > 500)
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, gpio_pin);
      gpio_pin = (gpio_pin == GPIO_PIN_RESET)?GPIO_PIN_SET:GPIO_PIN_RESET;
      g_LedTick = HAL_GetTick();
    }
    
    //检查当前发送msg是否超时
    checkSndTimeout();

    //解析4G模块发送过来的数据
    parser_uart1_msg();
    
    //解析103模块发送过来的数据
    parser_uart2_msg(); 

    //上报当前版本更新的进度(目前包括下载完成和更新完成)
    uploadUpdateProcess();

    //上报error日志
    uploadFaultLog();

    //发送update相关msg
    send_update_msg(); 

    //向103发送数据
    send_msg_to_103();

    //更新后，重启设备
    update_restart();
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 150;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Reload = 3125;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void HAL_UART1_RxCpltCallback(UART_HandleTypeDef *huart)
{

  uint32_t start, length;
  
  uint32_t currCNDTR = __HAL_DMA_GET_COUNTER(huart->hdmarx);
  
  /* Ignore IDLE Timeout when the received characters exactly filled up the DMA buffer and DMA Rx Complete IT is generated, but there is no new character during timeout */
  if(dma_uart_rx.flag && currCNDTR == DMA_RX_BUFFER_SIZE)
  { 
      dma_uart_rx.flag = 0;
      return;
  }
  
  /* Determine start position in DMA buffer based on previous CNDTR value */
  start = (dma_uart_rx.prevCNDTR < DMA_RX_BUFFER_SIZE) ? (DMA_RX_BUFFER_SIZE - dma_uart_rx.prevCNDTR) : 0;
  //start = DMA_RX_BUFFER_SIZE - dma_uart_rx.prevCNDTR;
  
  if(dma_uart_rx.flag)    /* Timeout event */
  {
      /* Determine new data length based on previous DMA_CNDTR value:
       *  If previous CNDTR is less than DMA buffer size: there is old data in DMA buffer (from previous timeout) that has to be ignored.
       *  If CNDTR == DMA buffer size: entire buffer content is new and has to be processed.
      */
  
      length = (dma_uart_rx.prevCNDTR < DMA_RX_BUFFER_SIZE) ? (dma_uart_rx.prevCNDTR - currCNDTR) : (DMA_RX_BUFFER_SIZE - currCNDTR);
      //length = dma_uart_rx.prevCNDTR - currCNDTR;
      dma_uart_rx.prevCNDTR = currCNDTR;
      dma_uart_rx.flag = 0;
  }
  else                /* DMA Rx Complete event */
  {
      length = DMA_RX_BUFFER_SIZE - start;
      dma_uart_rx.prevCNDTR = DMA_RX_BUFFER_SIZE;
  }

  uint32_t tocopy;
  uint8_t* ptr;

  tocopy = UART_BUFFER_SIZE - Uart1_Write;      /* Get number of bytes we can copy to the end of buffer */


  /* Check how many bytes to copy */
  if (tocopy > length)
  {
    tocopy = length;
  }
    
  /* Uart1_Write received data for UART main buffer for manipulation later */
  ptr = DMA_RX_Buffer+start;
  memcpy(&UART_Buffer[Uart1_Write], ptr, tocopy);   /* Copy first part */
  
  /* Correct values for remaining data */
  Uart1_Write += tocopy;
  length -= tocopy;
  ptr += tocopy;
  
  /* If still data to write for beginning of buffer */
  if (length)
  {
    memcpy(&UART_Buffer[0], ptr, length);      /* Don't care if we override Uart1_Read pointer now */
    Uart1_Write = length;
  }


  if(UART_BUFFER_SIZE == Uart1_Write)
  {
    Uart1_Write = 0;
  } 
}

void HAL_UART2_RxCpltCallback(UART_HandleTypeDef *huart)
{
  memcpy(&UART2_Buffer[Uart2_Write], DMA2_RX_Buffer, DMA2_RX_BUFFER_SIZE);

  Uart2_Write += DMA2_RX_BUFFER_SIZE;

  if(Uart2_Write == UART2_BUFFER_SIZE)
  {
    Uart2_Write = 0;
  }  

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == &huart1)
  {
    HAL_UART1_RxCpltCallback(huart);
  }
  else if(huart == &huart2)
  {
    HAL_UART2_RxCpltCallback(huart);
  }
    
  return;
}

void HAL_UART1_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if(UART1_SENDING_FTP != g_isUart1Sending)
  {
    g_uart1Phase++;
    g_startTick = HAL_GetTick();
  }
}

void HAL_UART2_TxCpltCallback(UART_HandleTypeDef *huart)
{
    g_isF103Sending = false;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == &huart1)
  {
    HAL_UART1_TxCpltCallback(huart);
  }
  else if(huart == &huart2)
  {
    HAL_UART2_TxCpltCallback(huart);
  }
    
  return;
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart == &huart1)
  {
    Msg_Error("HAL_UART_ErrorCallback huart1 error\n");
  }
  else if(huart == &huart2)
  {
    Msg_Error("HAL_UART_ErrorCallback huart2 error\n");
  }

  Msg_Error("HAL_UART_ErrorCallback ErrorCode = %d\r\n", huart->ErrorCode);
  NVIC_SystemReset();
  
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
