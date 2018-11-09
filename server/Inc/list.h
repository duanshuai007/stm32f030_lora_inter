#ifndef __MY_LIST_H__
#define __MY_LIST_H__

#include "stdint.h"
#include "user_config.h"

//typedef struct CmdNode {
//  struct CmdNode *nextCMD;
//  uint32_t u32Identify;
//  uint32_t u32TimeStamp;  //指令超时计时 
//  uint8_t u8Cmd;
//  uint8_t u8Resp;
//  uint8_t u8CmdStatus;
//} CmdNode;
//
//typedef struct ListNode {
//  struct ListNode *next;
//  struct CmdNode *nextCMD;
//  uint32_t  u32HeartTime;   //设备心跳计时
//  uint16_t  u16ID;
//  uint8_t   u8CHAN;
//  uint8_t   u8CmdCount;     //记录等待执行的命令个数
//  uint8_t   u8RespCount;    //记录等待处理的resp个数  
//  uint8_t   u8SendCmdCount; //记录已经发出但是没有收到resp的命令个数
//} ListNode;

typedef struct DeviceNode {
  uint32_t u32Identify;     //指令identify
  uint32_t u32Time;         //超时计时：1:心跳时间，2:指令执行超时
  uint16_t u16ID;           //设备ID
  uint8_t u8CHAN;           //信道
  uint8_t u8CMD;
  uint8_t u8CMDSTATUS;
  uint8_t u8RESP;
} DeviceNode;

typedef struct List {
  struct List *next;
  struct DeviceNode *Device;
} List;

/*
*   链表初始化
*/
List *list_init(void);

/*
*   添加设备到链表中
*/
void add_device_to_list(List *list, uint16_t id);

/*
*   从链表中删除一个设备
*/
uint8_t delete_device_from_list(List *list, uint16_t id);

/*
*   设置链表中对应id的设备命令
*/
void set_device_of_list_cmd(List *list, uint16_t id, uint8_t cmd, uint32_t identify);

/*
*   打印整个表
*/
void LookAllList(List *lcn);

/*
*   链表总任务
*/
void ListProcess(List *LCmdNode);

/*
*   向链表发送设备的响应信息
*/
void SendCMDRespToList(List *lcn, uint16_t id, uint8_t cmd, uint32_t time, uint8_t resp);


#endif



