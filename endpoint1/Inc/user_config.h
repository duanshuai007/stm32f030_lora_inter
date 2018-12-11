#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

#include "stdint.h"
#include "stm32f0xx.h"
#include "lora_paramter.h"

#define DEBUG

typedef enum {
  false,
  true,
}bool;

enum {
  CMD_STOP,
  CMD_RUN,
};

#define LORA_MSG_HEAD 0xA5
#define LORA_MSG_TAIL 0x5A
#define POOL_CLEAN    0x00

#define SERVER_SEND_CMD_LEN     12
#define SERVER_REC_RESP_LEN     13

//������ݵ����汾����������
#define MODE_OLD

#pragma pack(1)
typedef struct {
  uint8_t u8Cmd;
  uint8_t u8ReCmd;       //���յ����ظ�ָ��
  
  uint8_t u8CmdRunning;   //��ǰ��ָ��������
  volatile uint8_t u8Resp;
  volatile uint8_t u8CmdDone;      //�첽ָ��ִ����ɱ�־λ
  uint8_t u8ReCmdFlag;        //ָ��ִ�й����нӵ���ָ���־λ
  
  volatile uint8_t u8InteFlag;   //�жϱ�־
  volatile uint8_t u8InterDone;
  
  uint32_t u32Identify;
  uint32_t u32ReIdentify;
} Device;
#pragma pack()

#define GPIO_BEEP                       GPIOB
#define GPIO_BEEP_Pin                   GPIO_PIN_10
//LORA��ռ�õ�����
#define GPIO_Lora_M0 	                  GPIOA
#define GPIO_Lora_M1	                  GPIOB
#define GPIO_Lora_AUX	                  GPIOB
#define GPIO_Lora_M0_Pin	              GPIO_PIN_8
#define GPIO_Lora_M1_Pin	              GPIO_PIN_15
#define GPIO_Lora_AUX_Pin	              GPIO_PIN_14
//PA4 �������룬���λ�ü���ź�K1
#define GPIO_SENSOR_UP                  GPIOA
#define GPIO_SENSOR_UP_PIN              GPIO_PIN_4
//PA5 �������룬���λ�ü���ź�k2
#define GPIO_SENSOR_DOWN                GPIOA
#define GPIO_SENSOR_DOWN_PIN            GPIO_PIN_5
//PB4 ���������������������ź�P
#define GPIO_SENSOR_FORWARE             GPIOB
#define GPIO_SENSOR_FORWARE_PIN         GPIO_PIN_4
//PB5 ���������������������ź�N
#define GPIO_SENSOR_BACKWARD            GPIOB
#define GPIO_SENSOR_BACKWARD_PIN        GPIO_PIN_5
//PA6 ������������λ�ô���������ʹ���ź�K3
#define GPIO_SENSOR_SWITCH              GPIOA
#define GPIO_SENSOR_SWITCH_PIN          GPIO_PIN_6

#endif