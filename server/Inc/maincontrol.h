#ifndef _MAINCONTROL_H_
#define _MAINCONTROL_H_

#include "stdint.h"
#include "stm32f1xx.h"
#include "user_config.h"
#include "lora.h"

void UARTF405_Init(UartModule *um, UART_HandleTypeDef *uart);

void F405CmdProcess(UartModule *um);

void UartSendRespToF405(UartModule *um, DeviceNode *devn);

void UartSendOnLineToF405(UartModule *um, uint16_t id);
void UartSendOffLineToF405(UartModule *um, uint16_t id);

//�����豸æ��Ϣ��f405
void UartSendDeviceBusyToF405(UartModule *um, uint32_t identify);
//����cmd��ʱ��Ӧ��f405
void UartSendCMDTimeOutToF405(UartModule *um, uint32_t identify);

#endif



