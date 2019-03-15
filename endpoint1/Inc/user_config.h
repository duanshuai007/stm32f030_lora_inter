#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

#include "stdint.h"
#include "stm32f0xx.h"
#include "lora_paramter.h"

typedef enum {
  false,
  true,
}bool;

enum {
  CMD_STOP,
  CMD_RUN,
};

enum {
  CMD_EXEC_NONE = 0,
  CMD_EXEC_DOING,
  CMD_EXEC_DONE
};

#define LORA_MSG_HEAD 0xA5
#define LORA_MSG_TAIL 0x5A

//��λ���ף�ԭ��λ�Ǻ��ף�ֵ400
#define ULTRASONIC_MIN_SAFE_DISTANCE  40

//���ջ���
#define SERVER_SEND_CMD_LEN     (12)
//���ͻ���
#define SERVER_REC_RESP_LEN     (13)

//������ݵ����汾����������
#define MODE_OLD

#define RECMD_MAX_NUMBER         10

#pragma pack(1)

typedef struct {
  uint8_t u8Cmd;
  uint32_t u32Identify;
} reCmd;

typedef struct {
  uint8_t u8Cmd;
  uint8_t u8CmdRunning;   //��ǰ��ָ��������
  volatile uint8_t u8MotorResp;         //ר���ڵ�����첽����
  volatile uint8_t u8CmdDone;      //�첽ָ��ִ����ɱ�־λ
  
  volatile bool bInteFlag;   //�жϱ�־
  bool bInterDone;
  volatile uint8_t u8ReCmdNumber;  //���յ����ظ�ָ��ĸ���
  uint8_t u8LPUseRTC;         //�ڵ͹���ģʽ���Ƿ�ʹ��rtc����,ÿ�и������������+1��ʹ�������-1.
  
  volatile bool bHasRtcInter;
  volatile bool bHasLoraInter;
  volatile bool bHasMotorNormalInter;
  volatile bool bHasMotorAbnmalInter;
  
  volatile uint8_t u8ReCmdPos;
  uint16_t u16UltraDistance;
  uint32_t u32Identify;
  uint8_t u8UltraSafeDistance;
  
  reCmd sReCmd[RECMD_MAX_NUMBER];
} Device;

typedef struct {
  uint8_t u8Cmd;
  uint8_t u8Resp;
  uint32_t u32Identify;
} MsgDevice;
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
//PB13 ������ʹ������
#define GPIO_ULTRASONIC                 GPIOB
#define GPIO_ULTRASONIC_PIN             GPIO_PIN_13

#endif