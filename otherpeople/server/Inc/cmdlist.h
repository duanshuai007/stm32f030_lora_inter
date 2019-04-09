#ifndef __CMD_LIST_H__
#define __CMD_LIST_H__

#include "stdint.h"
#include "user_config.h"


typedef struct CmdList
{
  uint32_t u32Identify;
  
  uint16_t u16Id;
  uint8_t  u8Cmd;
  uint8_t  u32Data;
  
  struct CmdList *next;
  
}T_CmdList;

T_CmdList *initCmdList(T_CmdList *head);
T_CmdList *insertNode(T_CmdList *head, uint16_t id, uint32_t u32Identify, uint8_t u8Cmd, uint32_t data);
void delNode(T_CmdList *head, uint16_t id);
T_CmdList *popFrontNode(T_CmdList *head);
bool isExist(T_CmdList *head, uint16_t id);
bool isNull(T_CmdList *head);
#endif




