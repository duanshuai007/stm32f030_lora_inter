#include "stdlib.h"
#include "string.h"
#include "cmdlist.h"

T_CmdList *initCmdList(T_CmdList *head)
{

  head = (T_CmdList *)malloc(sizeof(T_CmdList));
  if(NULL == head)
  {
    return NULL;
  }

  head->next = NULL;
  head->u16Id = 0;
  head->u32Identify = 0;
  head->u8Cmd = 0;

  return head;
}

static T_CmdList * createNode(uint16_t id, uint32_t u32Identify, uint8_t u8Cmd, uint32_t data)
{
  T_CmdList *new_Node = (T_CmdList *)malloc(sizeof(T_CmdList));
  if(NULL == new_Node)
  {
    return NULL;
  }

  new_Node->u16Id = id;
  new_Node->u32Identify = u32Identify;
  new_Node->u8Cmd = u8Cmd;
  new_Node->u32Data = data;
  
  new_Node->next = NULL;

  return new_Node;
}


T_CmdList *insertNode(T_CmdList *head, uint16_t id, uint32_t u32Identify, uint8_t u8Cmd, uint32_t data)
{
  
  if(NULL == head)
  {
    return NULL;
  }
  
  T_CmdList *cur = head;

  while(NULL != cur->next)
  {
    cur = cur->next;
  }

  T_CmdList *newNode = createNode(id, u32Identify, u8Cmd, data);
  if(NULL == newNode)
  {
    return NULL;
  }

  cur->next = newNode;

  return newNode;
}

void delNode(T_CmdList *head, uint16_t id)
{
  if(isNull(head))
  {
    return;
  }

  T_CmdList *cur = head->next;
  T_CmdList *pre = head;
  while(NULL != cur)
  {
    if(id == cur->u16Id)
    {
      break;
    }

    pre = cur;
    cur = cur->next;
  }

  if(NULL == cur)
  {
    return;
  }
      
  pre->next = cur->next;
  free(cur);      
}

T_CmdList *popFrontNode(T_CmdList *head)
{
  if(isNull(head))
  {
    return NULL;
  }

  return head->next;  
}

bool isNull(T_CmdList *head)
{
  if(NULL == head || NULL == head->next )
  {
    return true;
  }

  return false;
}

bool isExist(T_CmdList *head, uint16_t id)
{
  if(isNull(head))
  {
    return false;
  }

  T_CmdList *cur = head->next;

  while(NULL != cur)
  {
    if(cur->u16Id == id)
    {
      return true;
    }
    
    cur = cur->next;
  }

  return false;
  
}
