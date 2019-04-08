#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "hardware.h"
#include "stdint.h"
#include "stm32f0xx.h"
#include "stm32f0xx_hal_def.h"
#include "stm32f0xx_hal_gpio.h"
#include "user_config.h"

typedef enum {
    MOTOR_CMD_IDLE    = 0,
    MOTOR_CMD_DOWN    = 1,
    MOTOR_CMD_DOWN_INTERNAL = 2,
    MOTOR_CMD_UP      = 3,
} MOTOR_CMD;

typedef void (* motor_ctrl_callback)(MOTOR_CB_TYPE m);
typedef void (* motor_gpio_callback)(uint16_t pin);
typedef void (* timer_ctrl_timeout_callback)(void);

typedef struct {
    //������ִ��ָ��֮���״̬
    volatile MOTOR_STATUS status;     //�����ǰ״̬ 1=�Ե�,2=ǰ��,3=ֱ��,4=����
    volatile MOTOR_CMD action;     //����
    volatile MOTOR_STATUS realStatus; //������ʵ�ĵ���״̬
    
    MOTOR_STATUS can_stop; //��ʼ�˶�ʱ��λ��״̬�������Ե������ʱ����ֹͣ�жϡ�

    motor_ctrl_callback         ctrl_cb;
    motor_gpio_callback         gpio_cb;
    timer_ctrl_timeout_callback ctrl_timer_cb;
} Motor;

void GPIOMotorSenserInit(void);
void GPIOMotorInit(uint8_t cmd);
//������ƺ���
MOTOR_STATUS motor_conctrl(MOTOR_CMD cmd);
//��ȡ���λ��״̬�ĺ���
MOTOR_STATUS MotorGetStatus(void);

void MotorInit(void);

//���λ�ü�������жϿ����͹ر�
void setMotorGpioNormal(void);
void setMotorGpioInterrupt(void);

void MotorSenceSwitch(bool b);
//����쳣��������
bool MotorAbnormalCheck(Device *d, bool flag);

MOTOR_STATUS getMotorStatus(void); //��ѭ����ȡʵʱ���λ��
void resetMotorStatus(void);
void setMotorInterrupt(void);
void cancelMotorInterrupt(void);
MOTOR_STATUS getMotorStatusInt(void); //�жϻ�ȡʵʱ���λ��

#endif