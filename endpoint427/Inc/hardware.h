#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#include "stdint.h"
#include "user_config.h"

typedef enum {
  HW_CMD_NONE = 0,
  HW_MOTOR_UP,
  //����ֵ��  0   UP �ɹ�
  //          1   UP ʧ�ܣ�������Ȼ�����Ե�״̬
  //          2   UP ʧ�ܣ���������ǰ���״̬
  //          4   UP ʧ�ܣ��������ں����״̬
  HW_MOTOR_DOWN,
  //����ֵ    0   DOWN �ɹ�
  //          2   DOWN ʧ�ܣ���������ǰ��״̬
  //          3   DOWN ʧ�ܣ���������ֱ��״̬
  //          4   DOWN ʧ�ܣ��������ں���״̬
  HW_MOTOR_GET,
  //����ֵ    1   ���������Ե�״̬
  //          2   ��Ŷ����ǰ��״̬
  //          3   ��������ֱ��״̬
  //          4   �������ں���״̬
  HW_BEEP_ON,   //4
  //����ֵ    0   �������򿪳ɹ�
  //          1   ��������ʧ��
  HW_BEEP_OFF,
  //����ֵ    0   �������رճɹ�
  //          1   �������ر�ʧ��
  HW_BEEP_GET,
  //����ֵ    1   ���������ڴ�״̬
  //          0   ���������ڹر�״̬
  HW_ADC_GET,
  //����ֵ    ��ص�ѹת����Ĵ����������ֵ(0-100)
  HW_ULTRA_GET, //8
  //����ֵ    ���������õ��ľ���ֵ 0-255֮�䣬��λ����
  //������ 0xff
  //����/��ȡ��������ȫ����ֵ
  HW_ULTRA_SAFE_SET,  //9
  HW_ULTRA_SAFE_GET,  //0x0a
  HW_ULTRA_CHECKTIME_SET, //0xb
  HW_ULTRA_CHECKTIME_GET, //0xc
  HW_SYSTEM_RESET,        //0xd
  HW_CMD_MAXVALUE,
} HW_CMD;

#define HW_DEVICE_HEART     101   //����
#define HW_MOTOR_ABNORMAL   102   //����쳣
#define HW_DEVICE_BUSY      9   //�豸��æ

typedef enum {
  MOTOR_OK        = 0,
  MOTOR_DOWN      = 1,
  MOTOR_QIANQING  = 2,
  MOTOR_UP        = 3,
  MOTOR_HOUQING   = 4,
  MOTOR_ERROR     = 5,  //�������ж���������̧����
  MOTOR_ERROR_ULTRA = 6,
  
  MOTOR_RUNNING   = HW_DEVICE_BUSY,
  MOTOR_RUN = 99,

//  MOTOR_STATUS_UNKNOW = 0xfe,
  MOTOR_DONTDO = 0xff,
} MOTOR_STATUS;

typedef enum {
  ACTION_OK               = 0,
  ACTION_FAIL_DOWN        = 1,
  ACTION_FAIL_QIANQING    = 2,
  ACTION_FAIL_UP          = 3,
  ACTION_FAIL_HOUQING     = 4,
} MOTOR_CB_TYPE;

void hardware_ctrl(Device *d);

#endif
