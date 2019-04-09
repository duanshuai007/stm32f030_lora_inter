/**
  ******************************************************************************
  * @file    IAP_Main/Inc/flash_if.h 
  * @author  MCD Application Team
  * @version 1.0.0
  * @date    8-April-2015
  * @brief   This file provides all the headers of the flash_if functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_IF_H
#define __FLASH_IF_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Base address of the Flash sectors */
#define ADDR_FLASH_PAGE_0     ((uint32_t)0x08000000) /* Base @ of Page 0, 1 Kbytes */
#define ADDR_FLASH_PAGE_1     ((uint32_t)0x08000400) /* Base @ of Page 1, 1 Kbytes */
#define ADDR_FLASH_PAGE_2     ((uint32_t)0x08000800) /* Base @ of Page 2, 1 Kbytes */
#define ADDR_FLASH_PAGE_3     ((uint32_t)0x08000c00) /* Base @ of Page 3, 1 Kbytes */
#define ADDR_FLASH_PAGE_4     ((uint32_t)0x08001000) /* Base @ of Page 4, 1 Kbytes */
#define ADDR_FLASH_PAGE_5     ((uint32_t)0x08001400) /* Base @ of Page 5, 1 Kbytes */
#define ADDR_FLASH_PAGE_6     ((uint32_t)0x08001800) /* Base @ of Page 6, 1 Kbytes */
#define ADDR_FLASH_PAGE_7     ((uint32_t)0x08001c00) /* Base @ of Page 7, 1 Kbytes */
#define ADDR_FLASH_PAGE_8     ((uint32_t)0x08002000) /* Base @ of Page 8, 1 Kbytes */
#define ADDR_FLASH_PAGE_9     ((uint32_t)0x08002400) /* Base @ of Page 9, 1 Kbytes */
#define ADDR_FLASH_PAGE_10    ((uint32_t)0x08002800) /* Base @ of Page 10, 1 Kbytes */
#define ADDR_FLASH_PAGE_11    ((uint32_t)0x08002c00) /* Base @ of Page 11, 1 Kbytes */
#define ADDR_FLASH_PAGE_12    ((uint32_t)0x08003000) /* Base @ of Page 12, 1 Kbytes */
#define ADDR_FLASH_PAGE_13    ((uint32_t)0x08003400) /* Base @ of Page 13, 1 Kbytes */
#define ADDR_FLASH_PAGE_14    ((uint32_t)0x08003800) /* Base @ of Page 14, 1 Kbytes */
#define ADDR_FLASH_PAGE_15    ((uint32_t)0x08003c00) /* Base @ of Page 15, 1 Kbytes */
#define ADDR_FLASH_PAGE_16    ((uint32_t)0x08004000) /* Base @ of Page 16, 1 Kbytes */
#define ADDR_FLASH_PAGE_17    ((uint32_t)0x08004400) /* Base @ of Page 17, 1 Kbytes */
#define ADDR_FLASH_PAGE_18    ((uint32_t)0x08004800) /* Base @ of Page 18, 1 Kbytes */
#define ADDR_FLASH_PAGE_19    ((uint32_t)0x08004c00) /* Base @ of Page 19, 1 Kbytes */
#define ADDR_FLASH_PAGE_20    ((uint32_t)0x08005000) /* Base @ of Page 20, 1 Kbytes */
#define ADDR_FLASH_PAGE_21    ((uint32_t)0x08005400) /* Base @ of Page 21, 1 Kbytes  */
#define ADDR_FLASH_PAGE_22    ((uint32_t)0x08005800) /* Base @ of Page 22, 1 Kbytes  */
#define ADDR_FLASH_PAGE_23    ((uint32_t)0x08005c00) /* Base @ of Page 23, 1 Kbytes */
#define ADDR_FLASH_PAGE_24    ((uint32_t)0x08006000) /* Base @ of Page 24, 1 Kbytes */
#define ADDR_FLASH_PAGE_25    ((uint32_t)0x08006400) /* Base @ of Page 25, 1 Kbytes */
#define ADDR_FLASH_PAGE_26    ((uint32_t)0x08006800) /* Base @ of Page 26, 1 Kbytes */
#define ADDR_FLASH_PAGE_27    ((uint32_t)0x08006c00) /* Base @ of Page 27, 1 Kbytes */
#define ADDR_FLASH_PAGE_28    ((uint32_t)0x08007000) /* Base @ of Page 28, 1 Kbytes */
#define ADDR_FLASH_PAGE_29    ((uint32_t)0x08007400) /* Base @ of Page 29, 1 Kbytes */
#define ADDR_FLASH_PAGE_30    ((uint32_t)0x08007800) /* Base @ of Page 30, 1 Kbytes */
#define ADDR_FLASH_PAGE_31    ((uint32_t)0x08007c00) /* Base @ of Page 31, 1 Kbytes */
#define ADDR_FLASH_PAGE_32    ((uint32_t)0x08008000) /* Base @ of Page 32, 1 Kbytes */
#define ADDR_FLASH_PAGE_33    ((uint32_t)0x08008400) /* Base @ of Page 33, 1 Kbytes */
#define ADDR_FLASH_PAGE_34    ((uint32_t)0x08008800) /* Base @ of Page 34, 1 Kbytes */
#define ADDR_FLASH_PAGE_35    ((uint32_t)0x08008c00) /* Base @ of Page 35, 1 Kbytes */
#define ADDR_FLASH_PAGE_36    ((uint32_t)0x08009000) /* Base @ of Page 36, 1 Kbytes */
#define ADDR_FLASH_PAGE_37    ((uint32_t)0x08009400) /* Base @ of Page 37, 1 Kbytes */
#define ADDR_FLASH_PAGE_38    ((uint32_t)0x08009800) /* Base @ of Page 38, 1 Kbytes */
#define ADDR_FLASH_PAGE_39    ((uint32_t)0x08009c00) /* Base @ of Page 39, 1 Kbytes */
#define ADDR_FLASH_PAGE_40    ((uint32_t)0x0800a000) /* Base @ of Page 40, 1 Kbytes */
#define ADDR_FLASH_PAGE_41    ((uint32_t)0x0800a400) /* Base @ of Page 41, 1 Kbytes */
#define ADDR_FLASH_PAGE_42    ((uint32_t)0x0800a800) /* Base @ of Page 42, 1 Kbytes */
#define ADDR_FLASH_PAGE_43    ((uint32_t)0x0800ac00) /* Base @ of Page 43, 1 Kbytes */
#define ADDR_FLASH_PAGE_44    ((uint32_t)0x0800b000) /* Base @ of Page 44, 1 Kbytes */
#define ADDR_FLASH_PAGE_45    ((uint32_t)0x0800b400) /* Base @ of Page 45, 1 Kbytes */
#define ADDR_FLASH_PAGE_46    ((uint32_t)0x0800b800) /* Base @ of Page 46, 1 Kbytes */
#define ADDR_FLASH_PAGE_47    ((uint32_t)0x0800bc00) /* Base @ of Page 47, 1 Kbytes */
#define ADDR_FLASH_PAGE_48    ((uint32_t)0x0800c000) /* Base @ of Page 48, 1 Kbytes */
#define ADDR_FLASH_PAGE_49    ((uint32_t)0x0800c400) /* Base @ of Page 49, 1 Kbytes */
#define ADDR_FLASH_PAGE_50    ((uint32_t)0x0800c800) /* Base @ of Page 50, 1 Kbytes */
#define ADDR_FLASH_PAGE_51    ((uint32_t)0x0800cc00) /* Base @ of Page 51, 1 Kbytes */
#define ADDR_FLASH_PAGE_52    ((uint32_t)0x0800d000) /* Base @ of Page 52, 1 Kbytes */
#define ADDR_FLASH_PAGE_53    ((uint32_t)0x0800d400) /* Base @ of Page 53, 1 Kbytes */
#define ADDR_FLASH_PAGE_54    ((uint32_t)0x0800d800) /* Base @ of Page 54, 1 Kbytes */
#define ADDR_FLASH_PAGE_55    ((uint32_t)0x0800dc00) /* Base @ of Page 55, 1 Kbytes */
#define ADDR_FLASH_PAGE_56    ((uint32_t)0x0800e000) /* Base @ of Page 56, 1 Kbytes */
#define ADDR_FLASH_PAGE_57    ((uint32_t)0x0800e400) /* Base @ of Page 57, 1 Kbytes */
#define ADDR_FLASH_PAGE_58    ((uint32_t)0x0800e800) /* Base @ of Page 58, 1 Kbytes */
#define ADDR_FLASH_PAGE_59    ((uint32_t)0x0800ec00) /* Base @ of Page 59, 1 Kbytes */
#define ADDR_FLASH_PAGE_60    ((uint32_t)0x0800f000) /* Base @ of Page 60, 1 Kbytes */
#define ADDR_FLASH_PAGE_61    ((uint32_t)0x0800f400) /* Base @ of Page 61, 1 Kbytes */
#define ADDR_FLASH_PAGE_62    ((uint32_t)0x0800f800) /* Base @ of Page 62, 1 Kbytes */
#define ADDR_FLASH_PAGE_63    ((uint32_t)0x0800fc00) /* Base @ of Page 63, 1 Kbytes */

#define APPLICATION_ADDR              ADDR_FLASH_PAGE_25
#define BOOT_CONFIG_ADDR              ADDR_FLASH_PAGE_62
#define USER_FLASH_END_ADDRESS        ADDR_FLASH_PAGE_63

#define FLASH_SAVE_SECTION            ADDR_FLASH_PAGE_63
#define FLASH_SAVE_DEVICEID_ADDR      ADDR_FLASH_PAGE_63+0x30
#define SAVE_MESSAGE_HEAD             0x474f5246        //"FROG"



/* Error code */
enum 
{
  FLASHIF_OK = 0,
  FLASHIF_ERASEKO,
  FLASHIF_WRITINGCTRL_ERROR,
  FLASHIF_WRITING_ERROR,
  FLASHIF_PROTECTION_ERRROR
};

/* protection type */  
enum{
  FLASHIF_PROTECTION_NONE         = 0,
  FLASHIF_PROTECTION_PCROPENABLED = 0x1,
  FLASHIF_PROTECTION_WRPENABLED   = 0x2,
  FLASHIF_PROTECTION_RDPENABLED   = 0x4,
};

/* protection update */
enum {
	FLASHIF_WRP_ENABLE,
	FLASHIF_WRP_DISABLE
};


#define FLASH_PAGE_TO_BE_PROTECTED (OB_WRP_PAGES0TO3 | OB_WRP_PAGES4TO7 | OB_WRP_PAGES8TO11 | OB_WRP_PAGES12TO15 | \
                                    OB_WRP_PAGES16TO19 | OB_WRP_PAGES20TO23 | OB_WRP_PAGES24TO27 | OB_WRP_PAGES28TO31 )  

/* Exported functions ------------------------------------------------------- */
#define READ_FLASH_WORD(addr) (*(volatile uint32_t *)(addr))


void FLASH_If_Init(void);
uint32_t FLASH_If_Erase_Config(void);
uint32_t FLASH_If_GetWriteProtectionStatus(void);
uint32_t FLASH_If_Write(uint32_t destination, uint32_t *p_source, uint32_t length);
uint32_t FLASH_If_WriteProtectionConfig(uint32_t modifier);
uint32_t FLASH_If_Write_Halfword(uint32_t destination, uint16_t data);


#endif  /* __FLASH_IF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/