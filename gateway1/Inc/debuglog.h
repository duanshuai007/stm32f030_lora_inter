#ifndef _DEBUGLOG_H
#define _DEBUGLOG_H
 
#define _MSG_INFO
#ifdef _MSG_INFO 
#define Msg_Info(format, args...) (printf("[INFO][%s][%d]:" format, __FUNCTION__ , __LINE__, ##args))
#else
#define Msg_Info(format, args...)  
#endif 


//#define _MSG_DEBUG
#ifdef _MSG_DEBUG
#define Msg_Debug(format, args...) (printf("[DEBUG][%s][%d]:" format, __FUNCTION__ , __LINE__, ##args))
#else
#define Msg_Debug(format, args...)
#endif

//#define _MSG_WARN
#ifdef _MSG_WARN
#define Msg_Warn(format, args...) (printf("[WARN][%s][%d]:" format, __FUNCTION__ , __LINE__, ##args))
#else
#define Msg_Warn(format, args...)
#endif


#define _MSG_ERROR
#ifdef _MSG_ERROR
#define Msg_Error(format, args...) (printf("[ERROR][%s][%d]:" format, __FUNCTION__ , __LINE__, ##args))
#else
#define Msg_Error(format, args...)
#endif

#endif

