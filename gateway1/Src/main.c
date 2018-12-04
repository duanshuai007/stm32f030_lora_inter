
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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef enum
{
  MQTT_INIT = 0,
  MQTT_RDY,
  MQTT_CPIN,
  MQTT_SMS,
  MQTT_PB,
  MQTT_ATE0 = 5,
  MQTT_ATE0_RSP,
  MQTT_CCID,
  MQTT_CCID_RSP,
  MQTT_START,
  MQTT_START_RSP = 10,
  MQTT_ACCQ,
  MQTT_ACCQ_RSP,
  MQTT_WILLTOPIC,
  MQTT_WILLTOPIC_RSP,
  MQTT_WILLTOPIC_CONTENT = 15,
  MQTT_WILLTOPIC_CONTENT_RSP,
  MQTT_WILLMSG,
  MQTT_WILLMSG_RSP,
  MQTT_WILLMSG_CONTENT,
  MQTT_WILLMSG_CONTENT_RSP = 20,
  MQTT_CONNECT,
  MQTT_CONNECT_RSP,
  MQTT_SUB,
  MQTT_SUB_RSP,
  MQTT_SUB_CONTENT = 25,
  MQTT_SUB_CONTENT_RSP,
  MQTT_PUB_TOPIC,
  MQTT_PUB_TOPIC_RSP,
  MQTT_PUB_TOPIC_CONTENT,
  MQTT_PUB_TOPIC_CONTENT_RSP = 30,
  MQTT_PUB_PAYLOAD,
  MQTT_PUB_PAYLOAD_RSP,
  MQTT_PUB_PAYLOAD_CONTENT,
  MQTT_PUB_PAYLOAD_CONTENT_RSP,
  MQTT_PUB = 35,
  MQTT_OK, // pub ok
}MQTT_PHASE;

typedef enum
{
  MQTT_RCV_INIT = 0,
  MQTT_RCV_SUM,
  MQTT_RCV_TOPIC_LEN,
  MQTT_RCV_TOPIC_CONTENT,
  MQTT_RCV_PAYLOAD_LEN,
  MQTT_RCV_PAYLOAD_CONTENT = 5,
}MQTT_RCV_PHASE;

typedef enum
{
  MQTT_SND_INIT = 0,
  MQTT_SND_TOPIC,
  MQTT_SND_TOPIC_RSP,
  MQTT_SND_TOPIC_CONTENT,
  MQTT_SND_TOPIC_CONTENT_RSP,
  MQTT_SND_PAYLOAD = 5,
  MQTT_SND_PAYLOAD_RSP,
  MQTT_SND_PAYLOAD_CONTENT,
  MQTT_SND_PAYLOAD_CONTENT_RSP,
  MQTT_SND_CMD,
  MQTT_SND_CMD_RSP1 = 10,
  MQTT_SND_CMD_RSP2,
}MQTT_SND_PHASE;

typedef enum 
{
  CMD_MOTOR_UP = 1,
  CMD_MOTOR_DOWN = 2,
  CMD_MOTOR_STATUS_GET = 3,
  CMD_BEEP_ON = 4,
  CMD_BEEP_OFF = 5,
  CMD_BEEP_STATUS_GET = 6,
  CMD_ADC_GET = 7,
  CMD_F405_ONLINE = 8,

}CMD_TYPE;

typedef enum
{
  //Normal
  NORMAL_SUCCESS = 101,
  NORMAL_DOWN = 102,
  NORMAL_FORWARD = 103,    
  NORMAL_UP = 104,
  NORMAL_BACK = 105,
  NORMAL_BUSY = 106,
  NORMAL_BEEP_OPEN_FAILED = 107,
  NORMAL_BEEP_CLOSE_FAILED = 108,
  NORMAL_BEEP_STATUS_OPEN = 109,
  NORMAL_BEEP_STATUS_CLOSED = 110,

  //child online or offline
  CHILD_ONLINE = 120,
  CHILD_OFFLINE = 121,

  //interupt
  INTERUPT_DOWN = 201,
  INTERUPT_FORWARD = 202,    
  INTERUPT_UP = 203,
  INTERUPT_BACK = 204,  

  INTERUPT_TIMEOUT = 210,  
  
}RESP_CODE;

typedef union
{
  uint32_t identity;
  uint16_t endpointId;  
}U_Resp_Data;

#pragma pack (1)
typedef struct 
{
  uint8_t resp_code;
  U_Resp_Data resp_data;
  uint8_t crc;
}T_Resp_Info;
#pragma pack ()

typedef struct 
{
  uint32_t identity;
  uint16_t endpointId;
  uint8_t action;
  uint8_t crc;
}T_Control_Cmd;

#define MQTT_SEND_SUCCESS 0
#define MQTT_SEND_BUSY 1
#define MQTT_SEND_INVALID_TOPIC_TYPE 2

#define MQTT_TOPIC_CHILD_ONLINE "AT+CMQTTTOPIC=0,13\r\n"
#define MQTT_TOPIC_CHILD_ONLINE_CONTENT "/child_online\r\n"
#define MQTT_TOPIC_CHILD_OFFLINE "AT+CMQTTTOPIC=0,14\r\n"
#define MQTT_TOPIC_CHILD_OFFLINE_CONTENT "/child_offline\r\n"
#define MQTT_TOPIC_ACTION_RSP "AT+CMQTTTOPIC=0,12\r\n"
#define MQTT_TOPIC_ACTION_RSP_CONTENT "/action_resp\r\n"
#define MQTT_TOPIC_INTERRUPT_RSP "AT+CMQTTTOPIC=0,15\r\n"
#define MQTT_TOPIC_INTERRUPT_RSP_CONTENT "/interrupt_resp\r\n"
typedef enum
{
  MQTT_TOPIC_TYPE_INVALID,
  MQTT_TOPIC_TYPE_CHILD_ONLINE,
  MQTT_TOPIC_TYPE_CHILD_OFFLINE,
  MQTT_TOPIC_TYPE_ACTION_RSP,
  MQTT_TOPIC_TYPE_INTERRUPT_RSP,
}MQTT_TOPIC_TYPE;
uint8_t g_topicType = MQTT_TOPIC_TYPE_INVALID;
#define MQTT_MAX_SEND_PAYLOAD_SIZE 64
uint8_t g_mqttSndPayload[MQTT_MAX_SEND_PAYLOAD_SIZE];
uint32_t g_sndStartTick = 0;

#define DMA_RX_BUFFER_SIZE          64
uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];
 
#define UART_BUFFER_SIZE            256
uint8_t UART_Buffer[UART_BUFFER_SIZE];

#define DMA_TX_BUFFER_SIZE          64
uint8_t DMA_TX_Buffer[DMA_TX_BUFFER_SIZE];

#define DMA2_RX_BUFFER_SIZE          (sizeof(T_Resp_Info))
uint8_t DMA2_RX_Buffer[DMA2_RX_BUFFER_SIZE];
 
#define UART2_BUFFER_SIZE            1500
uint8_t *UART2_Buffer;

#define DMA2_TX_BUFFER_SIZE          8
uint8_t DMA2_TX_Buffer[DMA2_TX_BUFFER_SIZE];

#define UART2_TX_BUFFER_SIZE          256
uint8_t UART2_TX_Buffer[UART2_TX_BUFFER_SIZE];

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

uint32_t g_rcvTopicLen = 0;
uint32_t g_rcvPayloadLen = 0;
uint8_t g_uart1RcvPhase = MQTT_RCV_INIT;

bool g_isMqttSending = false;
volatile uint32_t g_MqttSendTimeOut = 0;

bool g_isF103Sending = false;
volatile uint32_t g_F103SendTimeOut = 0;

DMA_Event_t dma_uart_rx = {DMA_TX_BUFFER_SIZE, 0, 0};


#define ATE0 "ATE0\r\n"
#define CMQTTSTART "AT+CMQTTSTART\r\n"
#define CCID "AT+CICCID\r\n"
#define ACCQ_PREFIX "AT+CMQTTACCQ=0,"
#define WILLTOPIC "AT+CMQTTWILLTOPIC=0,13\r\n"
#define WILLTOPIC_CONTENT "/root_offline\r\n"
#define WILLMSG "AT+CMQTTWILLMSG=0,23,1\r\n"
#define TIMEOUT "AT+CMQTTCNCTTIMEOUT=0,10\r\n"
//#define CONNECT_TEST "AT+CMQTTCONNECT=0,\"tcp://119.119.52.253:1883\",60,1\r\n"
#define CONNECT_PRODUCT "AT+CMQTTCONNECT=0,\"tcp://faye.weixinxk.net:1883\",60,1\r\n"
#define SUB "AT+CMQTTSUB=0,23,1,0\r\n"
#define PUBTOPIC "AT+CMQTTTOPIC=0,12\r\n"
#define PUBTOPIC_CONTENT "/root_online\r\n"
#define PUBPAYLOAD "AT+CMQTTPAYLOAD=0,24\r\n"
#define PUB "AT+CMQTTPUB=0,1,60\r\n"
#define RCVMSG1 "+CMQTTRXSTART: 0,"
#define RCVMSG2 "+CMQTTRXTOPIC: 0,"
#define RCVMSG3 "+CMQTTRXPAYLOAD: 0,"
#define RCVMSG4 "+CMQTTRXEND: 0"

#define SYSTEM_PREFIX_NAME      "WM_"


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void Anlyz_USART1_Rcv(void);
static void Anlyz_USART1_Rcv2(void);
static void Check_Timeout(void);
uint8_t Send_Mqtt_Msg(uint8_t topicType, uint8_t * payload, uint8_t length);
static void Send_Mqtt_Msg_Continue(void);

uint8_t crc8_chk_value(uint8_t *message, uint8_t len);
int generalJsonResp(T_Resp_Info *resp, uint8_t *topicType, uint8_t *json_resp);
void parser_mqtt_msg(void);
void HAL_UART_Transmit_To_103(void);



/* USER CODE END PFP */

/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  /* USER CODE BEGIN 2 */
 //   HAL_Delay(5000);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); // reset SIM7600CE
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); // enable idle line interrupt
//  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE); // enable idle line interrupt
  __HAL_UART_CLEAR_IDLEFLAG(&huart1);
//  __HAL_UART_CLEAR_IDLEFLAG(&huart2);

  //__HAL_DMA_ENABLE_IT (&hdma_usart1_rx, DMA_IT_TC);  // enable DMA Tx cplt interrupt
  //__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT); // disable uart half tx interrupt
  //__HAL_DMA_ENABLE_IT (&hdma_usart1_tx, DMA_IT_TC);  // enable DMA Tx cplt interrupt
  //__HAL_DMA_DISABLE_IT(&hdma_usart1_tx, DMA_IT_HT); // disable uart half tx interrupt
  HAL_UART_Receive_DMA(&huart1, DMA_RX_Buffer, DMA_RX_BUFFER_SIZE);// 脝么露炉DMA陆脫脢脮
  HAL_UART_Receive_DMA(&huart2, DMA2_RX_Buffer, DMA2_RX_BUFFER_SIZE);// 脝么露炉DMA陆脫脢脮

  UART2_Buffer = (uint8_t *)malloc(UART2_BUFFER_SIZE);
  if(UART2_Buffer == NULL)
  {
    Msg_Error("UART2_Buffer malloc failed!\r\n");
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  g_startTick = HAL_GetTick();

#if 1
  while (1)
  {
    Check_Timeout();
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
          || MQTT_PUB_TOPIC == g_uart1Phase         /* receive after send PUB_TOPIC */
          || MQTT_PUB_PAYLOAD == g_uart1Phase       /* receive after send PUB_PAYLOAD */
          || MQTT_PUB == g_uart1Phase               /* receive after send PUB */) 
      {
          
        if (RET_POS_INVALID == g_retBegin)
        {
          g_retBegin = Uart1_Read;
        }
        g_retEnd = tmpWrite;
        
        Anlyz_USART1_Rcv();
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

            Anlyz_USART1_Rcv();
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
  
#endif
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */



  Msg_Info("ready\r\n");
  g_uart1Phase = MQTT_SND_INIT;
  g_retBegin = RET_POS_INVALID;
  g_retEnd = RET_POS_INVALID;


  // T_Control_Cmd cmd;

  // cmd.identity = 0;
  // cmd.endpointId = 0;
  // cmd.action = CMD_F405_ONLINE;
  // cmd.crc = crc8_chk_value((uint8_t *)(&cmd), 7);

  // memcpy(DMA2_TX_Buffer, (uint8_t *)(&cmd), sizeof(T_Control_Cmd));
  // HAL_UART_Transmit_DMA(&huart2, DMA2_TX_Buffer, sizeof(T_Control_Cmd));

  while (1)
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

    uint32_t tmpWrite = Uart1_Write;
    while (Uart1_Read != tmpWrite)
    {
      if (RET_POS_INVALID == g_retBegin)
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
      else
      {
        if (0x0d == UART_Buffer[Uart1_Read])
        {
          g_retEnd = Uart1_Read;
          Anlyz_USART1_Rcv2();
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

      //sub_count ++;
      //total_count ++;
    }

    //test

    //parser data from CPU2
    parser_mqtt_msg(); 

    //send data to F103
    HAL_UART_Transmit_To_103();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
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

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

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
int generalJsonResp(T_Resp_Info *resp, uint8_t *topicType, uint8_t *json_resp)
{
  if(NULL == resp || NULL == json_resp)
  {
    return false;
  }

#define TYPE_NORMAL 0
#define TYPE_ONLINE 1
#define TYPE_OFFLINE 2
#define TYPE_INTERUPT 3  
  uint8_t resp_code = 0;
  uint8_t resp_type = TYPE_NORMAL;//0->normal,1->online, 2->offline,3->interupt
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
    case CHILD_ONLINE:
      resp_type = TYPE_ONLINE;    
      break;
    case CHILD_OFFLINE:
      resp_type = TYPE_OFFLINE;
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
        Msg_Warn("generalJsonResp error! resp->resp_code = %d\n", resp->resp_code);
        return false;         
      }
 }

  switch(resp_type)
  {
    case TYPE_NORMAL:
      sprintf((char *)json_resp, "{\n\"action_resp\":%d,\n\"identify_resp\":%d\n}", resp_code, resp->resp_data.identity);
      *topicType = MQTT_TOPIC_TYPE_ACTION_RSP;
      break;
    case TYPE_ONLINE:
      sprintf((char *)json_resp, "%s%d", SYSTEM_PREFIX_NAME, resp->resp_data.endpointId);
      *topicType = MQTT_TOPIC_TYPE_CHILD_ONLINE;
      break;
    case TYPE_OFFLINE:
      sprintf((char *)json_resp, "%s%d", SYSTEM_PREFIX_NAME, resp->resp_data.endpointId);
      *topicType = MQTT_TOPIC_TYPE_CHILD_OFFLINE;
      break;
    case TYPE_INTERUPT:
      sprintf((char *)json_resp, "{\n\"macaddr\":%d,\n\"status\":%d\n}", resp->resp_data.endpointId, resp_code);
      *topicType = MQTT_TOPIC_TYPE_INTERRUPT_RSP;
      break;
    default:
      return false;

  }

  return true;

}

uint8_t crc8_chk_value(uint8_t *message, uint8_t len)
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
  g_uart1Phase++;
  g_startTick = HAL_GetTick();
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

static void Anlyz_USART1_Rcv(void)
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
        Msg_Info("%d POK\r\n", MQTT_CCID);
        for (uint32_t i = 0; i < length; i ++)
        {
          Msg_Info("%02X", begin[i]);
        }
        Msg_Info("\r\n");
        NVIC_SystemReset();
        return;
      }

      memcpy(g_simCardId + 3, begin + 10, 20);
      
      delayTime = 1000;
      strcpy((char *)DMA_TX_Buffer, CMQTTSTART);//MQTT_START
      //init suc, send "CMQTTSTART"
      break;
    case MQTT_START:
      expRst = "\r\nOK\r\n\r\n+CMQTTSTART: 0\r\n";
      delayTime = 100;
      sprintf((char *)DMA_TX_Buffer, "%s\"%s\"\r\n", ACCQ_PREFIX, g_simCardId);
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
      strcpy((char *)DMA_TX_Buffer, WILLMSG);//MQTT_WILLMSG
      //init suc, send "WILLMSG"
      break;
    case MQTT_WILLMSG:
      expRst = "\r\n>";
      delayTime = 100;
      sprintf((char *)DMA_TX_Buffer, "%s\r\n", g_simCardId);//MQTT_WILLMSG_CONTENT
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
      strcpy((char *)DMA_TX_Buffer, SUB);//MQTT_SUB
      break;
    case MQTT_SUB:
      expRst = "\r\n>";
      delayTime = 100;
      sprintf((char *)DMA_TX_Buffer, "%s\r\n", g_simCardId);//MQTT_SUB_CONTENT
      break;
    case MQTT_SUB_CONTENT:
      expRst = "\r\nOK\r\n\r\n+CMQTTSUB: 0,0\r\n";
      delayTime = 100;
      strcpy((char *)DMA_TX_Buffer, PUBTOPIC);//MQTT_PUB_TOPIC
      break;
    case MQTT_PUB_TOPIC:
      expRst = "\r\n>";
      delayTime = 100;
      strcpy((char *)DMA_TX_Buffer, PUBTOPIC_CONTENT);//MQTT_PUB_TOPIC_CONTENT
      break;
    case MQTT_PUB_TOPIC_CONTENT:
      expRst = "OK";
      delayTime = 100;
      strcpy((char *)DMA_TX_Buffer, PUBPAYLOAD);//MQTT_PUB_PAYLOAD
      break;
    case MQTT_PUB_PAYLOAD:
      expRst = "\r\n>";
      delayTime = 100;
      sprintf((char *)DMA_TX_Buffer, "%s\r\n", g_simCardId);//MQTT_PUB_PAYLOAD_CONTENT
      break;
    case MQTT_PUB_PAYLOAD_CONTENT:
      expRst = "OK";
      delayTime = 100;
      strcpy((char *)DMA_TX_Buffer, PUB);//MQTT_PUB
      break;
    case MQTT_PUB:
      expRst = "\r\nOK\r\n\r\n+CMQTTPUB: 0,0\r\n";
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
  Msg_Info("%d ok\r\n", g_uart1Phase);
  g_startTick = HAL_GetTick();  

  g_retBegin = RET_POS_INVALID;
  g_retEnd = RET_POS_INVALID;

  if (0 != delayTime)
  {
    HAL_Delay(delayTime);
    HAL_UART_Transmit_DMA(&huart1, DMA_TX_Buffer, strlen((char *)DMA_TX_Buffer)); 
  }
}

bool generalCmd(uint8_t *jsonStr)
{
 
  uint8_t id[6];
  uint8_t action[2];
  uint8_t identify[11];
  uint8_t *pTemp = (uint8_t *)strstr((char *)jsonStr, "\"mac\"");
  int i = 0;
  if(NULL == pTemp)
  {
    return false;
  }

  //parser mac
  pTemp = (uint8_t *)strchr((char *)pTemp, ':');
  if(NULL == pTemp)
  {
    return false;
  }

  while(*pTemp>'9' || *pTemp<'0')
  {
    pTemp++;
  }

  while(*pTemp<='9' && *pTemp>='0')
  {
    id[i] = *pTemp;
    pTemp++;
    i++;
  }

  id[i] = '\0';

  //parser action
  i = 0;
  pTemp = (uint8_t *)strchr((char *)pTemp, ':');
  if(NULL == pTemp)
  {
    return false;
  }

  while(*pTemp>'9' || *pTemp<'0')
  {
    pTemp++;
  }

  while(*pTemp<='9' && *pTemp>='0')
  {
    action[i] = *pTemp;
    pTemp++;
    i++;
  }
  action[i] = '\0';

 //parser identify
  i = 0;
  pTemp = (uint8_t *)strchr((char *)pTemp, ':');
  if(NULL == pTemp)
  {
    return false;
  }

  while(*pTemp>'9' || *pTemp<'0')
  {
    pTemp++;
  }

  while(*pTemp<='9' && *pTemp>='0')
  {
    identify[i] = *pTemp;
    pTemp++;
    i++;
  }
  identify[i] = '\0';  

  
  T_Control_Cmd cmd;
  
  memset(&cmd, 0, sizeof(T_Control_Cmd));

  cmd.endpointId = atoi((char *)id);
  cmd.action = atoi((char *)action);
  cmd.identity = atoi((char *)identify);

  cmd.crc = crc8_chk_value((uint8_t *)(&cmd), 7);

  memcpy(UART2_TX_Buffer+Uart2_Send_Write, &cmd, sizeof(T_Control_Cmd));

  Uart2_Send_Write += sizeof(T_Control_Cmd);
  if(Uart2_Send_Write == UART2_TX_BUFFER_SIZE)
  {
    Uart2_Send_Write = 0;
  }

  return true;
}


void HAL_UART_Transmit_To_103(void)
{
  if(HAL_GetTick() - g_F103SendTimeOut > 1000)
  {
    g_isF103Sending = false;
  }

  if(g_isF103Sending)
  {
    return;
  }

  uint32_t tmpWrite = Uart2_Send_Write;
  if ((Uart2_Send_Read != tmpWrite) && 0 == tmpWrite%sizeof(T_Control_Cmd))
  {

    while((&huart2)->gState != HAL_UART_STATE_READY);
    memcpy(DMA2_TX_Buffer, UART2_TX_Buffer+Uart2_Send_Read, sizeof(T_Control_Cmd));
    HAL_UART_Transmit_DMA(&huart2, DMA2_TX_Buffer, sizeof(T_Control_Cmd)); 
    
    g_F103SendTimeOut = HAL_GetTick();
    g_isF103Sending = true;

    Uart2_Send_Read += sizeof(T_Control_Cmd);
    
    if (Uart2_Send_Read == UART2_TX_BUFFER_SIZE)
    {
      Uart2_Send_Read = 0;
    }
  } 

};



static void Anlyz_USART1_Rcv2(void)
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
      Msg_Error("Anlyz_USART1_Rcv2 OK : %s\r\n", begin);
      NVIC_SystemReset();
    }
    Send_Mqtt_Msg_Continue();
  }
  else if (MQTT_SND_CMD_RSP2 == g_uart1Phase && 0 == strncmp((char *)begin, "+CMQTTPUB: 0,", 13))
  {
    g_sndStartTick = 0;
    int32_t retCode = -1;
    sscanf((char *)(begin + 13), "%u", &retCode);
    if (-1 != retCode)
    {
      g_uart1Phase = MQTT_SND_INIT;
      g_isMqttSending = false;
    }
    else
    {
      Msg_Error("MQTT_SND_CMD_RSP2 : %s\r\n", begin);
      NVIC_SystemReset();
    }
  }
  else
  {
    
    uint32_t topicLen = 0;
    uint32_t payloadLen = 0;
    
    switch (g_uart1RcvPhase)
    {
      case MQTT_RCV_INIT:
        if (0 != strncmp((char *)begin, RCVMSG1, strlen(RCVMSG1)))
        {
          Msg_Error("ERROR(MQTT_RCV_INIT) : %s\r\n", begin);
          NVIC_SystemReset();
          return;
        }
        g_rcvTopicLen = 0;
        g_rcvPayloadLen = 0;
        sscanf((char *)(begin + strlen(RCVMSG1)), "%u,%u", &g_rcvTopicLen, &g_rcvPayloadLen);
        if (0 == g_rcvTopicLen || 0 == g_rcvPayloadLen)
        {
          Msg_Error("ERROR(MQTT_RCV_INIT) : %s\r\n", begin);
          NVIC_SystemReset();
          return;
        }
        g_uart1RcvPhase = MQTT_RCV_SUM;
        break;
      case MQTT_RCV_SUM:
        #if 1
        if (0 != strncmp((char *)begin, RCVMSG2, strlen(RCVMSG2)))
        {
          Msg_Error("ERROR(MQTT_RCV_SUM) : %s\r\n", begin);
          NVIC_SystemReset();
          return;
        }

        sscanf((char *)(begin + strlen(RCVMSG2)), "%u", &topicLen);
        if (g_rcvTopicLen != topicLen)
        {
          Msg_Error("Error : usTopicLen = %u\r\n", topicLen);
          Msg_Error("ERROR(MQTT_RCV_SUM) : %s\r\n", begin);
          NVIC_SystemReset();
          return ;
        }
        #endif
        g_uart1RcvPhase = MQTT_RCV_TOPIC_LEN;
        break;
      case MQTT_RCV_TOPIC_LEN:
        #if 1
        if (23 != length || 0 != strncmp((char *)begin, (char *)g_simCardId, 23))
        {
          Msg_Error("ERROR(MQTT_RCV_TOPIC_LEN) : %s\r\n", begin);
          NVIC_SystemReset();
          return ;
        }
        #endif
        g_uart1RcvPhase = MQTT_RCV_TOPIC_CONTENT;
        break;
      case MQTT_RCV_TOPIC_CONTENT:
        #if 1
        if (0 != strncmp((char *)begin, RCVMSG3, strlen(RCVMSG3)))
        {
          Msg_Error("ERROR(MQTT_RCV_TOPIC_CONTENT) : %s\r\n", begin);
          NVIC_SystemReset();
          return;
        }
        sscanf((char *)(begin + strlen(RCVMSG3)), "%u", &payloadLen);
        if (payloadLen != g_rcvPayloadLen)
        {
          Msg_Error("ERROR(MQTT_RCV_TOPIC_CONTENT) : %s\r\n", begin);
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
          Msg_Error("ERROR(MQTT_RCV_PAYLOAD_LEN) : %s\r\n", begin);
          NVIC_SystemReset();
          return;
        }


        generalCmd((uint8_t *)begin);
        //for do ,parser server data,fill cmd

         //analysis and send to uart
        #endif
        g_uart1RcvPhase = MQTT_RCV_PAYLOAD_CONTENT;
        break;
      case MQTT_RCV_PAYLOAD_CONTENT:
        #if 1
        if (0 != strncmp((char *)begin, RCVMSG4, strlen(RCVMSG4)))
        {
          Msg_Error("ERROR(MQTT_RCV_PAYLOAD_CONTENT) : %s\r\n", begin);
          NVIC_SystemReset();
          return;
        }
        #endif
        g_uart1RcvPhase = MQTT_RCV_INIT;
         
        //Send_Mqtt_Msg(MQTT_TOPIC_TYPE_ACTION_RSP, "action_resp", 11);
        break;
      default:
        Msg_Error("Error : default\r\n");
        NVIC_SystemReset();
        break;
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

void parser_mqtt_msg(void)
{
  //此处设置一个超时，防止收不到响应，导致g_isMqttSending一直是true，从而无法再正常工作。
  if(HAL_GetTick() - g_MqttSendTimeOut > 1000)
  {
    g_isMqttSending = false;
  }

  if(g_isMqttSending)
  {
    return;
  }

  uint32_t tmpWrite = Uart2_Write;
  if ((Uart2_Read != tmpWrite) && 0 == tmpWrite%sizeof(T_Resp_Info))
  {
    T_Resp_Info resp;
    memcpy((uint8_t *)(&resp), UART2_Buffer+Uart2_Read, sizeof(T_Resp_Info));

    uint8_t crc = crc8_chk_value((uint8_t*)(&resp), 5);
    if(crc != resp.crc)
    {
      Msg_Warn("crc check error!!!! response crc = %02x, resp.crc = %02x\r\n", crc, resp.crc);
      goto next_data;
    }
    
    uint8_t json_resp[64];
    uint8_t topicType;

    if(generalJsonResp(&resp, &topicType, json_resp))
    {
      Send_Mqtt_Msg(topicType, json_resp, strlen((char *)json_resp));
      g_MqttSendTimeOut = HAL_GetTick();
      g_isMqttSending = true;
    }

    next_data:
    Uart2_Read += sizeof(T_Resp_Info);
    
    if (Uart2_Read == UART2_BUFFER_SIZE)
    {
      Uart2_Read = 0;
    }
  } 
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

static void Check_Timeout(void)
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
      timeout = 1000;
      break;
    case MQTT_SUB_CONTENT:
    case MQTT_SUB_CONTENT_RSP:
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
      timeout = 10000;
      break;
    default:
      Msg_Error("Error Check_Timeout g_uart1Phase : %d\r\n", g_uart1Phase);
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
