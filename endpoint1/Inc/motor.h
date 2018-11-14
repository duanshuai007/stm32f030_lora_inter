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
    MOTOR_CMD_UP      = 3,
} MOTOR_CMD;

typedef void (* motor_ctrl_callback)(MOTOR_CB_TYPE m);
typedef void (* motor_check_callback)(MOTOR_CB_TYPE m);
typedef void (* motor_gpio_callback)(uint16_t pin);
typedef void (* timer_gpio_xd_callback)(void);
typedef void (* timer_ctrl_timeout_callback)(void);
typedef struct {                  
    bool        enable;
    uint8_t    max_count;       //���ʱʱ��Ϊ 100��255 = 25.5S
    uint8_t    cur_count;
} motor_timer;

typedef struct {
    volatile MOTOR_STATUS status;     //�����ǰ״̬ 1=�Ե�,2=ǰ��,3=ֱ��,4=����
    volatile MOTOR_CMD action;     //����

    volatile MOTOR_STATUS last_status;    //������һ�ε�����λ��״̬
    MOTOR_STATUS can_stop; //��ʼ�˶�ʱ��λ��״̬�������Ե������ʱ����ֹͣ�жϡ�

    motor_ctrl_callback         ctrl_cb;
    motor_check_callback        check_cb;
    motor_gpio_callback         gpio_cb;
    //htim6
    motor_timer                 xiaodou_timer;
//    timer_gpio_xd_callback      xiaodou_timer_cb;
    //rtc alarm
    timer_ctrl_timeout_callback ctrl_timer_cb;
} Motor;

//ϵͳ�ӿ�
//������ƺ���
MOTOR_STATUS motor_conctrl(MOTOR_CMD cmd);
//��ȡ���λ��״̬�ĺ���
MOTOR_STATUS motor_get_status(void);

void motor_init(void);

//���λ�ü�������жϿ����͹ر�
void motor_input_pin_off_interrupt(bool b);

//����쳣��������
uint8_t motor_abnormal(void);

#endif