#ifndef _DEBUGLOG_H
#define _DEBUGLOG_H

#include "flash_if.h"
#include <stdint.h>

//#define _MSG_INFO
#ifdef _MSG_INFO 
#define Msg_Info(format, args...) (uart3_msg_output("[I][%d]:" format, __LINE__, ##args))
#else
#define Msg_Info(format, args...)  
#endif 


//#define _MSG_DEBUG
#ifdef _MSG_DEBUG
#define Msg_Debug(format, args...) (uart3_msg_output("[D][%d]:" format, __LINE__, ##args))
#else
#define Msg_Debug(format, args...)
#endif

//#define _MSG_WARN
#ifdef _MSG_WARN
#define Msg_Warn(format, args...) (uart3_msg_output("[W][%d]:" format, __LINE__, ##args))
#else
#define Msg_Warn(format, args...)
#endif


#define _MSG_ERROR
#ifdef _MSG_ERROR
#define Msg_Error(format, args...) \
{\
  char *logBuf = (char *)malloc(256);\
  sprintf(logBuf, "[ERROR][%s][%d][%s]:" format, __FUNCTION__, __LINE__, mqtt_channel, ##args);\
  FLASH_If_Erase(LOG_ADDRESS);\
  FLASH_If_Write_Byte(LOG_ADDRESS, (uint8_t *)logBuf, strlen(logBuf)+1);\
  if(NULL != logBuf)\
  {\
    free(logBuf);\
    logBuf=NULL;\
  }\
}
#else
#define Msg_Error(format, args...)
#endif

#endif

