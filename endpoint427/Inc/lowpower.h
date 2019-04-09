#ifndef __LOWPOWER_H__
#define __LOWPOWER_H__

#include "stdint.h"
#include "stm32f0xx_hal.h"
#include "motor.h"
#include "user_config.h"

/*
*   ÿ�ν���͹���ģʽ֮ǰ��Ҫ���ã�
*   �رղ�ʹ�õ����裬��ʡ����
*   mode:��Ӧ��ͬ��ָ��ǵ���˶���������ܹ���ʱ���أ�������˶�����
*   ��Ҫ����һ��ʱ����ܷ��ء�mode = 0 ��Ӧ��ʱ���� = 1��Ӧ�Ǽ�ʱ����
*   flag����Ӧ�쳣������־�������쳣������ͬ��������� =1 ��Ӧ���쳣��������
*/
//bool LowPowerInit(Device *d);
void EnterLowPower(void);
/*
*   ϵͳ�ϵ������
*   ��ϵͳ�д�δ�õ�����������Ž�������
*/
void CloseNotUsedPeriphClock(void);

/*
*   ����������ͬ�������0
*   �첽�����1
*/
void ExecCMD(Device *d);

/*
*   �첽�����ִ�н������ͨ��lora����resp��Ϣ
*/
void SyncCMDDone(Device *d);

/*
*   ����ָ�������ڽ��г�������⣬��ͨ��lora����
*/
void doCheckUltraByTimer(uint8_t u8TimeOut);

/*
*   ����ָ�������ڽ���������Ϣ����
*/
void doHeartByTimer(uint8_t u8TimeOut, MOTOR_STATUS mStatus);

#endif
