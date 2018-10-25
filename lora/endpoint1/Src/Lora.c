#include "Lora.h"
#include "stdint.h"
#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_def.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_hal_uart.h"
#include "crc16.h"
#include "crc_stm32f0.h"
#include "hardware.h"
#include "user_config.h"

extern UART_HandleTypeDef huart1;
extern LoraPacket       gLoraPacket;
extern Device           gDevice;

/*
*
*       aux状态：       
*               在模块上电后，aux输出低电平并进行硬件自检。
*               在此过程中aux保持低电平，完成后输出高电平。
*               并按照M1，M0组合而成的工作模式开始工作。
*               所以用户需要等待aux上升沿，作为模块正常工作的起点。
*
*       用于无线收发缓冲提示和自检提示
*               AUX=1时，用户可以连续发起小于512字节的数据
*               AUX=0时，缓冲区不为空。
*
*       AUX高电平表示空闲状态，低电平表示忙状态
*/
uint8_t Lora_is_idle(void)
{
//    GPIO_PinState ret;
//    
//    ret = HAL_GPIO_ReadPin(GPIO_Lora_AUX, GPIO_Lora_AUX_Pin);
//    if (GPIO_PIN_SET == ret) {
//        return 1;
//    } else {
//        return 0;
//    }
    
    return gLoraPacket.isIdle;
}

bool Lora_Module_Set_Mode(LoraPacket *lp, Lora_Mode lm)
{
#define LORA_SET_MAX_TIME 	500
    
    uint16_t delay = 0;
    
    lp->mode = lm;
    
    HAL_Delay(5);       //必须的延时，至少5ms

    switch(lp->mode)
    {
    case LoraMode_Normal:
        HAL_GPIO_WritePin(GPIO_Lora_M0, GPIO_Lora_M0_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_Lora_M1, GPIO_Lora_M1_Pin, GPIO_PIN_RESET);
        break;
    case LoraMode_WakeUp:
        HAL_GPIO_WritePin(GPIO_Lora_M0, GPIO_Lora_M0_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIO_Lora_M1, GPIO_Lora_M1_Pin, GPIO_PIN_RESET);
        break;
    case LoraMode_LowPower:
        HAL_GPIO_WritePin(GPIO_Lora_M0, GPIO_Lora_M0_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_Lora_M1, GPIO_Lora_M1_Pin, GPIO_PIN_SET);
        break;
    case LoraMode_Sleep:
        HAL_GPIO_WritePin(GPIO_Lora_M0, GPIO_Lora_M0_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIO_Lora_M1, GPIO_Lora_M1_Pin, GPIO_PIN_SET);
        break;
    default:
        return false;
    }

    do {
        HAL_Delay(2);   //必须的do{}while(...)循环，先执行一遍延时
        delay++;
    } while((!Lora_is_idle()) && (delay < LORA_SET_MAX_TIME));

    if(delay >= LORA_SET_MAX_TIME)
        return false;
    else
        return true;
}

/*
*       Generate Lora Paramter
*	根据参数生成数据：成功返回0，失败返回1
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
        return 1;
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
        return 1;
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
        return 1;
    }
    
    if(lp->u8Channel > CHAN_441MHZ)
        return 1;
    
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
        return 1;
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
        return 1;
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
        return 1;
    }
    
    if(lp->u8FEC)
    {
        buff[5] |= 1 << 2;
    }
    else
    {
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
        return 1;
    }
    
    return 0;
}

/*
*       Set Lora Paramter
*/
uint8_t Lora_SetPar(uint8_t flag, LoraPacket *lp)
{
#define LORA_SET_PAR_MAX_TIME 	100
    uint8_t delay = 0;
    uint8_t config[6];
    
    if(lp == NULL || &lp->paramter == NULL)
        return 1;
    
    if (Lora_GeneratePar(&lp->paramter, config, SAVE_IN_FLASH))
        return 1;
    
    //必要的延时函数
    HAL_Delay(10);
    
    for(uint8_t i = 0; i < 6; i++)
    {
        lp->dma_sbuff[i] = config[i];
    }
    
    if (HAL_UART_Transmit_DMA(&huart1, lp->dma_sbuff, 6) == HAL_OK) {
        
	lp->setflag = 1;
        
        while(lp->setflag && (delay < LORA_SET_PAR_MAX_TIME)) {
            delay++;
            HAL_Delay(5);
        }
        if(delay >= LORA_SET_PAR_MAX_TIME) {
            lp->setflag = 0;
            return 1;
        } else {
            return 0;
        }
    } else {
        return 1;
    }
}
//从接收到的数据包中分析配置参数
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
*       PS:在发送了读取版本号命令之后不能再发送命令
*/
uint8_t Lora_Read(uint8_t no, LoraPacket *lp)
{
#define LORA_READ_PAR_MAX_TIME 100
    uint8_t delay = 0;
    
    if (LoraReadPar == no) {
        *(lp->dma_sbuff + 0) = 0xC1;
        *(lp->dma_sbuff + 1) = 0xC1;
        *(lp->dma_sbuff + 2) = 0xC1;
    } else if (LoraReadVer == no) {
        *(lp->dma_sbuff + 0) = 0xC3;
        *(lp->dma_sbuff + 1) = 0xC3;
        *(lp->dma_sbuff + 2) = 0xC3;
    } else {
        return 1;
    }
    
    if (HAL_UART_Transmit_DMA(&huart1, lp->dma_sbuff, 3) == HAL_OK) {
        
	lp->setflag = 1;
        
	while(lp->setflag && (delay < LORA_READ_PAR_MAX_TIME))
        {
            delay++;
            HAL_Delay(5);
        }
        if ( delay >= LORA_READ_PAR_MAX_TIME) {
            lp->setflag = 0;
            return 1;
        } 
	else
            return 0;
    } else {
        return 1;
    }
}
//reset lora module
uint8_t Lora_Reset(LoraPacket *lp)
{
#define LORA_RESET_MAX_TIME 	200
    uint8_t delay = 0;
    
    *(lp->dma_sbuff + 0) = 0xC4;
    *(lp->dma_sbuff + 1) = 0xC4;
    *(lp->dma_sbuff + 2) = 0xC4;
    
    if (HAL_UART_Transmit_DMA(&huart1, lp->dma_sbuff, 3) == HAL_OK) {
        while ((!Lora_is_idle()) && (delay < LORA_RESET_MAX_TIME))
        {
            delay++;
            HAL_Delay(5);
        }
        
        if (delay >= LORA_RESET_MAX_TIME) {
            return 1;
        } else {
            return 0;
        }
    } else {
        return 1;
    }
}

void LoraSetAndReadParamter(UART_HandleTypeDef *huart)
{
    //进入配置模式,在配置模式下，不能带校验
    Lora_Module_Set_Mode(&gLoraPacket, LoraMode_Sleep);
    //使能DMA接收，设置接收缓冲区
    HAL_UART_Receive_DMA(huart, gLoraPacket.dma_rbuff, MAX_DMA_LEN);
    //使能空闲中断，仅对接收起作用
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    
#if 0
    //写入配置参数
    gLoraPacket.paramter.u16Addr      = 0x000f;
    gLoraPacket.paramter.u8Baud       = BAUD_57600;
    gLoraPacket.paramter.u8Channel    = CHAN_440MHZ;
    gLoraPacket.paramter.u8FEC        = FEC_ENABLE;
    gLoraPacket.paramter.u8Parity     = PARITY_8O1;
    gLoraPacket.paramter.u8Speedinair = SPEED_IN_AIR_19_2K;
    gLoraPacket.paramter.u8TranferMode = TRANSFER_MODE_DINGDIAN; //TRANSFER_MODE_TOUMING;
    gLoraPacket.paramter.u8IOMode     = IOMODE_PP;
    gLoraPacket.paramter.u8WakeUpTime = WAKEUP_TIME_500MS;
    gLoraPacket.paramter.u8SendDB     = SEND_20DB;
    while(Lora_SetPar(SAVE_IN_FLASH, &gLoraPacket));
#endif
    
    //读取配置参数
    while(Lora_Read(LoraReadPar, &gLoraPacket));
    
    Lora_Module_Set_Mode(&gLoraPacket, LoraMode_LowPower);
    
    //cur APB CLOCK SET TO 1mhz,uart max speed is 57600
    __HAL_UART_DISABLE(huart);
    huart->Init.BaudRate = 57600;
    huart->Init.WordLength = UART_WORDLENGTH_9B;
    huart->Init.Parity = UART_PARITY_ODD;
    if (HAL_UART_Init(huart) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}

//通过lora发送一串数据
/*
*       id:目标设备的地址
*       len：buff的信息长度，如果长度超过58则需要分段发送
*/
bool LoraTransfer(Device *d)
{
    bool ret = false;
#define LORA_MAX_IDLE_WAIT      200
#define UART_MAX_SEND_WAIT      200
    
    uint8_t delay = 0;
    uint8_t pos;        //如果是定点模式，则在信息头部需要附加目标地址和信道共三个字节数据
    
    while((!Lora_is_idle()) && (delay < LORA_MAX_IDLE_WAIT))
    {
        delay++;
        HAL_Delay(1);
    }
    
    if (delay >= LORA_MAX_IDLE_WAIT) {
        return false;
    }
    
    if ( LoraMode_Normal != gLoraPacket.mode) {
        Lora_Module_Set_Mode(&gLoraPacket, LoraMode_Normal);
    }
    
    if ( TRANSFER_MODE_DINGDIAN == gLoraPacket.paramter.u8TranferMode) {
        gLoraPacket.dma_sbuff[0] = (uint8_t)((TARGET_ID & 0xff00) >> 8);
        gLoraPacket.dma_sbuff[1] = (uint8_t)(TARGET_ID & 0x00ff);
        gLoraPacket.dma_sbuff[2] = gLoraPacket.paramter.u8Channel;
        pos = 3;
    } else {
        pos = 0;
    }
    
    RespDataPacket *ptr = (RespDataPacket *)&gLoraPacket.dma_sbuff[pos];
    ptr->u8Head = 0x85;
    ptr->u8Len  = 6;
    ptr->u16Id  = gLoraPacket.paramter.u16Addr;
    ptr->u8Cmd  = d->u8Cmd;
    ptr->u8Resp = d->u8Resp;
    ptr->u16Crc = CRC16_IBM(&gLoraPacket.dma_sbuff[pos], 6);

    if ( HAL_OK == HAL_UART_Transmit_DMA(&huart1, gLoraPacket.dma_sbuff, ptr->u8Len + 2 + pos)) {
         
        gLoraPacket.sflag = 1;
        delay = 0;
        while((gLoraPacket.sflag) && (delay < UART_MAX_SEND_WAIT))
        {
            delay++;
            HAL_Delay(2);
        }
        
        if (delay >= UART_MAX_SEND_WAIT) {
            gLoraPacket.sflag = 0;
            ret = false;
        } else {
            ret = true;
        }
    }
    
    while(!Lora_is_idle());

    Lora_Module_Set_Mode(&gLoraPacket, LoraMode_LowPower);

    return ret;
}

//在普通模式下，中断中接收数据
int8_t LoraReceiveData(DMA_HandleTypeDef *hdma, LoraPacket *lp)
{
    uint8_t i, len;

    //len 等于已经用于存放数据的长度(空间)
    len = (MAX_DMA_LEN - (uint8_t)__HAL_DMA_GET_COUNTER(hdma));
    
    if (0 == len)
        return -1;
    
    if (len >= lp->pos) {
	//已经的空间减去pos的位置就是本次的数据长度	
        len -= lp->pos;
    } else {
    	//当剩余空间不足以存放接收到数据时，pos指针会重新移动到
    	//0位置开始存放剩余数据
    	//因为pos移动到0的时候用于存放数据的长度就等于len
    	//此时len对应的是没能在buffer尾部存放下而移动至0位置的剩余长度
    	//所以本次的数据长度就等于len + MAX_DMA_LEN - lp->pos
    	// 	当pos 到 MAX_DMA_LEN 之间剩余5个字节空间，接收到10个字节时 1,2,3,4,5,6,7,8,9,a
    	//	[0----------------------------------------------pos-------MAX_DMA_LEN] 
    	//	[0----------------------------------------------1 2 3 4 5]
    	//	次数pos指向了MAX_DMA_LEN，pos被移动到0位置
    	//	[6 7 8 9 a ---------------------------------------1 2 3 4 5]
        len += (MAX_DMA_LEN - lp->pos);
    }
    
    //从dma接收缓冲区提取数据
    uint8_t buffer[MAX_DMA_LEN];
    
    for (i = 0; i < len; i++) {
        buffer[i] = lp->dma_rbuff[lp->pos++];
        if (MAX_DMA_LEN == lp->pos) {
            lp->pos = 0;
        }
    }
    
    if (lp->setflag) {
        //如果是设置或读取参数的数据包
	uint8_t rec_data;
	
        lp->setflag = 0;
        rec_data = buffer[0];
        
        if (0xC0 == rec_data) {
         
            Lora_GetPar(&lp->paramter, buffer);
            
            return 0;
        } else if (0xC3 == rec_data) {
            
            uint8_t *p = (uint8_t *)&lp->version;
            
            for (i=0; i < 4; i++) {
                *p++ = buffer[i];
            }
            
            p = NULL;
            
            return 0;
        } else{
            return -1;
        }
        /*end if(lp->setflag) true*/
    } else {
        //传输数据包接收方式
        CmdDataPacket *ptr = (CmdDataPacket *)buffer;
        
        if (0x58 == ptr->u8Head) {
            if (5 == ptr->u8Len) {
                uint16_t crc = CRC16_IBM((uint8_t *)ptr, 5);
                if (crc == ptr->u16Crc) {
                    //判断目标设备id是否是本设备
                    if (lp->paramter.u16Addr == ptr->u16Id) {
                        //如果当前有指令在运行，则返回忙信息
                        if ((gDevice.u8Cmd != HW_CMD_NONE) && (gDevice.u8CmdRunning == CMD_RUN)) {
                            gDevice.u8ReCMD = 1;
                        } else {
                            gDevice.u8Cmd = ptr->u8Cmd;
                        }
                    }
                    
                    return ptr->u8Len;
                }
            }
        }
        
        return -1;
        /*end if...else{}*/
    }
}
