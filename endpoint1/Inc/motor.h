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
typedef void (* motor_check_callback)(MOTOR_CB_TYPE m);
typedef void (* motor_gpio_callback)(uint16_t pin);
typedef void (* timer_gpio_xd_callback)(void);
typedef void (* timer_ctrl_timeout_callback)(void);

typedef struct {
    volatile MOTOR_STATUS status;     //�����ǰ״̬ 1=�Ե�,2=ǰ��,3=ֱ��,4=����
    volatile MOTOR_CMD action;     //����

//    volatile MOTOR_STATUS last_status;    //������һ�ε�����λ��״̬
    MOTOR_STATUS can_stop; //��ʼ�˶�ʱ��λ��״̬�������Ե������ʱ����ֹͣ�жϡ�

    motor_ctrl_callback         ctrl_cb;
    motor_gpio_callback         gpio_cb;
    timer_ctrl_timeout_callback ctrl_timer_cb;
} Motor;

void GPIO_MotorSenserInit(void);
void GPIO_Motor_Init(uint8_t cmd);
//������ƺ���
MOTOR_STATUS motor_conctrl(MOTOR_CMD cmd);
//��ȡ���λ��״̬�ĺ���
MOTOR_STATUS MotorGetStatus(void);

void motor_init(void);

//���λ�ü�������жϿ����͹ر�
void set_motor_gpio_normal(void);
void set_motor_gpio_interrupt(void);

void MotorSenceSwitch(bool b);
//����쳣��������
void MotorAbnormalCheck(Device *d);
void MotorAbnormalProcess(Device *d);

#endif

