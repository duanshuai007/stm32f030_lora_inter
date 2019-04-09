
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "flash_if.h"
#include "md5sum.h"
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
T_BootConfig g_stBootConfig;
pFunction Jump_To_Application;
uint32_t JumpAddress;

#define DMA_RX_BUFFER_SIZE          64
uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];
 
#define UART_BUFFER_SIZE            1280
uint8_t UART_Buffer[UART_BUFFER_SIZE];


uint32_t Uart_Read = 0;
volatile uint32_t Uart_Write = 0;

DMA_Event_t dma_uart_rx = {DMA_RX_BUFFER_SIZE, 0, 0};

#pragma pack(1)
typedef struct 
{
  uint8_t resp_code;
  uint32_t identity;
  uint16_t endpointId;
  uint32_t data;
  uint8_t crc;
}T_Resp_Info;


#pragma pack()

typedef enum
{
  PHASE_CONFIG = 0,
  PHASE_DATA = 1
  
}T_Update_Phase;

uint32_t g_UpdatePhase = PHASE_CONFIG;

typedef struct
{
  uint8_t  ucBinMd5[MD5_LEN];    
  uint8_t  ucFWVer[VERSION_LEN];
  uint32_t uiTotalLen;  
}T_FWF103Head;

T_FWF103Head stFwF103Head;
uint32_t uiDlFinSize = 0;
uint32_t uiPerWriteSize = FLASH_PAGE_SIZE;
uint8_t md5_temp[MD5_LEN];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
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


void initUart2(void)
{
//  MX_GPIO_Init();
//  MX_DMA_Init();
//  MX_USART2_UART_Init();

  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE); // enable idle line interrupt
  __HAL_UART_CLEAR_IDLEFLAG(&huart2); 

  HAL_UART_Receive_DMA(&huart2, DMA_RX_Buffer, DMA_RX_BUFFER_SIZE); 

//  UART_Buffer = (uint8_t *)malloc(UART_BUFFER_SIZE);

  HAL_Delay(1000);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  FLASH_If_Init();

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
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  memcpy(&g_stBootConfig, (char *)BOOT_CONFIG_ADDR, sizeof(T_BootConfig));

  if(TYPE_LOCAL == g_stBootConfig.ucBootType)
  {
    if(g_stBootConfig.ucTryTimes >= 10)
    {
      initUart2();

      HAL_IWDG_Refresh(&hiwdg);
      FLASH_If_Erase_App();

      T_Resp_Info stReqRec;
      
      stReqRec.resp_code = 132;
      stReqRec.endpointId = 0;
      stReqRec.identity = 0;
      stReqRec.data = 0;      
      stReqRec.crc = crc8_chk_value((uint8_t *)&stReqRec, 11);
     
      while((&huart2)->gState != HAL_UART_STATE_READY);
      HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&stReqRec, sizeof(T_Resp_Info));

    }
    else
    {
      if (((*(__IO uint32_t*)APPLICATION_ADDR) & 0x2FFE0000 ) == 0x20000000)
      { 
        if(g_stBootConfig.ucBootSeccessFlg == 0)
        {
          g_stBootConfig.ucTryTimes++;

          HAL_IWDG_Refresh(&hiwdg);
          FLASH_If_Erase_Config();
          
          HAL_IWDG_Refresh(&hiwdg);
          FLASH_If_Write(BOOT_CONFIG_ADDR, (uint32_t*)(&g_stBootConfig), sizeof(T_BootConfig));      
        }
        
        /* Jump to user application */
        JumpAddress = *(__IO uint32_t*) (APPLICATION_ADDR + 4);
        Jump_To_Application = (pFunction) JumpAddress;
        /* Initialize user application's Stack Pointer */
        __set_MSP(*(__IO uint32_t*) APPLICATION_ADDR);
        Jump_To_Application();
      }
      else
      {
        initUart2();

        HAL_IWDG_Refresh(&hiwdg);
        FLASH_If_Erase_App();        
        
        T_Resp_Info stReqRec;
        
        stReqRec.resp_code = 132;
        stReqRec.endpointId = 0;
        stReqRec.identity = 0;
        stReqRec.data = 0;      
        stReqRec.crc = crc8_chk_value((uint8_t *)&stReqRec, 11);
          
        while((&huart2)->gState != HAL_UART_STATE_READY);
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&stReqRec, sizeof(T_Resp_Info));        
      }
    }
  }
  else if(TYPE_UART2== g_stBootConfig.ucBootType)
  {
    initUart2();

    HAL_IWDG_Refresh(&hiwdg);
    FLASH_If_Erase_App();
        
    T_Resp_Info stReqUpdate;      
    stReqUpdate.resp_code = 131;
    stReqUpdate.endpointId = 0;
    stReqUpdate.identity = 0;
    stReqUpdate.data = 0;      
    stReqUpdate.crc = crc8_chk_value((uint8_t *)&stReqUpdate, 11);


    while((&huart2)->gState != HAL_UART_STATE_READY);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&stReqUpdate, sizeof(T_Resp_Info));
    
  }
  else
  {
    //ERROR
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uiDlFinSize = 0;
  uiPerWriteSize = FLASH_PAGE_SIZE;
  
  while (1)
  {
    uint32_t tmpWrite = Uart_Write;
    while (Uart_Read != tmpWrite)
    {
      if(PHASE_CONFIG == g_UpdatePhase)
      {
        if(tmpWrite - Uart_Read >= sizeof(T_FWF103Head))
        {
          memcpy(&stFwF103Head, UART_Buffer+Uart_Read, sizeof(T_FWF103Head));

          Uart_Read += sizeof(T_FWF103Head);

          g_UpdatePhase = PHASE_DATA;
        }
      }
      else if(PHASE_DATA == g_UpdatePhase)
      {
        uint32_t uiDataLen = 0;
        if(tmpWrite > Uart_Read)
        {
          uiDataLen = tmpWrite - Uart_Read;
        }
        else
        {
          uiDataLen = UART_BUFFER_SIZE - Uart_Read + tmpWrite;
        }

        uiPerWriteSize = (0 == (stFwF103Head.uiTotalLen - uiDlFinSize) / FLASH_PAGE_SIZE ? (stFwF103Head.uiTotalLen - uiDlFinSize) % FLASH_PAGE_SIZE : FLASH_PAGE_SIZE);
        if(uiDataLen < uiPerWriteSize)
        {
          break;
        }
        else
        {
          char *pData = (char *)malloc(uiPerWriteSize);
          if(NULL == pData)
          {
            //error
          }
          
          if(tmpWrite > Uart_Read)
          {
            memcpy((char *)pData, &UART_Buffer[Uart_Read], uiPerWriteSize);
            Uart_Read += uiPerWriteSize;
            
          }
          else
          {

            uiDataLen = UART_BUFFER_SIZE - Uart_Read;
            if(uiDataLen >= uiPerWriteSize)
            {
              memcpy((char *)pData, &UART_Buffer[Uart_Read], uiPerWriteSize);

              Uart_Read += uiPerWriteSize;
              if(Uart_Read == UART_BUFFER_SIZE)
              {
                Uart_Read = 0;
              }
            }
            else
            {
              memcpy((char *)pData, &UART_Buffer[Uart_Read], UART_BUFFER_SIZE - Uart_Read);                
              memcpy((char *)(pData + UART_BUFFER_SIZE - Uart_Read), &UART_Buffer[0], uiPerWriteSize - (UART_BUFFER_SIZE - Uart_Read));

              Uart_Read = uiPerWriteSize - (UART_BUFFER_SIZE - Uart_Read);              
            }
          } 
          
          HAL_IWDG_Refresh(&hiwdg);
          FLASH_If_Write(APPLICATION_ADDR + uiDlFinSize, (uint32_t *)pData, uiPerWriteSize);
          if(NULL != pData)
          {
            free(pData);
            pData = NULL;
          }
          
          uiDlFinSize += uiPerWriteSize;

          if(uiDlFinSize == stFwF103Head.uiTotalLen)
          {            
            md5sum(md5_temp, (uint8_t  *)APPLICATION_ADDR, stFwF103Head.uiTotalLen);
            if(0 == memcmp(md5_temp, stFwF103Head.ucBinMd5, MD5_LEN))
            {
              strcpy((char *)g_stBootConfig.ucNewFWVer, (char *)stFwF103Head.ucFWVer);
              g_stBootConfig.ucBootSeccessFlg = 0;
              g_stBootConfig.ucTryTimes = 0;
              g_stBootConfig.ucBootType = TYPE_LOCAL;

              HAL_IWDG_Refresh(&hiwdg);
              FLASH_If_Erase_Config();
              
              HAL_IWDG_Refresh(&hiwdg);
              FLASH_If_Write(BOOT_CONFIG_ADDR, (uint32_t*)(&g_stBootConfig), sizeof(T_BootConfig));      

              NVIC_SystemReset();
            }
            else
            {
              HAL_IWDG_Refresh(&hiwdg);
              FLASH_If_Erase_App();

              T_Resp_Info stReqRec;
              
              stReqRec.resp_code = 132;
              stReqRec.endpointId = 0;
              stReqRec.identity = 0;
              stReqRec.data = 0;      
              stReqRec.crc = crc8_chk_value((uint8_t *)&stReqRec, 11);
                      
              while((&huart2)->gState != HAL_UART_STATE_READY);
              HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&stReqRec, sizeof(T_Resp_Info));                  
            }
            
          }          
        }
      }
    }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

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

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 2500;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
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

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M1_R_Pin|M0_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, M0_S_Pin|M1_S_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : AUX_R_Pin AUX_S_Pin */
  GPIO_InitStruct.Pin = AUX_R_Pin|AUX_S_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : M1_R_Pin M0_R_Pin */
  GPIO_InitStruct.Pin = M1_R_Pin|M0_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : M0_S_Pin M1_S_Pin */
  GPIO_InitStruct.Pin = M0_S_Pin|M1_S_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
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

  tocopy = UART_BUFFER_SIZE - Uart_Write;      /* Get number of bytes we can copy to the end of buffer */


  /* Check how many bytes to copy */
  if (tocopy > length)
  {
    tocopy = length;
  }
    
  /* Uart_Write received data for UART main buffer for manipulation later */
  ptr = DMA_RX_Buffer+start;
  memcpy(&UART_Buffer[Uart_Write], ptr, tocopy);   /* Copy first part */
  
  /* Correct values for remaining data */
  Uart_Write += tocopy;
  length -= tocopy;
  ptr += tocopy;
  
  /* If still data to write for beginning of buffer */
  if (length)
  {
    memcpy(&UART_Buffer[0], ptr, length);      /* Don't care if we override Uart_Read pointer now */
    Uart_Write = length;
  }


  if(UART_BUFFER_SIZE == Uart_Write)
  {
    Uart_Write = 0;
  } 
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
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
