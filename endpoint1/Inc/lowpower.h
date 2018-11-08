#ifndef __LOWPOWER_H__
#define __LOWPOWER_H__

#include "stdint.h"
#include "stm32f0xx_hal.h"

/*
*   ÿ�ν���͹���ģʽ֮ǰ��Ҫ���ã�
*   �رղ�ʹ�õ����裬��ʡ����
*/
void LowPowerInit(uint8_t mode);

/*
*   ϵͳ�ϵ������
*   ��ϵͳ�д�δ�õ�����������Ž�������
*/
void CloseNotUsedPeriphClock(void);

/*
*   ����������ͬ�������0
*   �첽�����1
*/
uint8_t ProcessTheData(void);

/*
*   ÿ�δӵ͹���ģʽ�����Ѻ���Ҫ���³�ʼ������
*/
void UART_ReInit(UART_HandleTypeDef *huart);

/*
*   �첽�����ִ�н������ͨ��lora����resp��Ϣ
*/
uint8_t SyncCMDDone(void);

#endif
