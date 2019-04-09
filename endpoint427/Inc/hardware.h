#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#include "stdint.h"
#include "user_config.h"

typedef enum {
  HW_CMD_NONE = 0,
  HW_MOTOR_UP,
  //返回值：  0   UP 成功
  //          1   UP 失败，地锁依然处于卧倒状态
  //          2   UP 失败，地锁处于前倾的状态
  //          4   UP 失败，地锁处于后倾的状态
  HW_MOTOR_DOWN,
  //返回值    0   DOWN 成功
  //          2   DOWN 失败，地锁处于前倾状态
  //          3   DOWN 失败，地锁处于直立状态
  //          4   DOWN 失败，地锁处于后倾状态
  HW_MOTOR_GET,
  //返回值    1   地锁处于卧倒状态
  //          2   丢哦处于前倾状态
  //          3   地锁处于直立状态
  //          4   地锁处于后倾状态
  HW_BEEP_ON,   //4
  //返回值    0   蜂鸣器打开成功
  //          1   蜂鸣器打开失败
  HW_BEEP_OFF,
  //返回值    0   蜂鸣器关闭成功
  //          1   蜂鸣器关闭失败
  HW_BEEP_GET,
  //返回值    1   蜂鸣器处于打开状态
  //          0   蜂鸣器处于关闭状态
  HW_ADC_GET,
  //返回值    电池电压转换后的代表电量的数值(0-100)
  HW_ULTRA_GET, //8
  //返回值    超声波测距得到的距离值 0-255之间，单位厘米
  //出错返回 0xff
  //设置/获取超声波安全距离值
  HW_ULTRA_SAFE_SET,  //9
  HW_ULTRA_SAFE_GET,  //0x0a
  HW_ULTRA_CHECKTIME_SET, //0xb
  HW_ULTRA_CHECKTIME_GET, //0xc
  HW_SYSTEM_RESET,        //0xd
  HW_CMD_MAXVALUE,
} HW_CMD;

#define HW_DEVICE_HEART     101   //心跳
#define HW_MOTOR_ABNORMAL   102   //电机异常
#define HW_DEVICE_BUSY      9   //设备繁忙

typedef enum {
  MOTOR_OK        = 0,
  MOTOR_DOWN      = 1,
  MOTOR_QIANQING  = 2,
  MOTOR_UP        = 3,
  MOTOR_HOUQING   = 4,
  MOTOR_ERROR     = 5,  //地锁上有东西，不能抬起来
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
