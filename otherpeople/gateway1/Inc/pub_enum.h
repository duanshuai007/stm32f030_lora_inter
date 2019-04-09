#ifndef PUB_ENUM_H_
#define PUB_ENUM_H_
typedef enum
{
  MQTT_INIT = 0,
  MQTT_RDY,
  MQTT_CPIN,
  MQTT_SMS,
  MQTT_PB,
  
  MQTT_ATE0 = 5,
  MQTT_ATE0_RSP,
  MQTT_CCID,
  MQTT_CCID_RSP,
  MQTT_START,
  
  MQTT_START_RSP = 10,
  MQTT_ACCQ,
  MQTT_ACCQ_RSP,
  MQTT_WILLTOPIC,
  MQTT_WILLTOPIC_RSP,
  
  MQTT_WILLTOPIC_CONTENT = 15,
  MQTT_WILLTOPIC_CONTENT_RSP,
  MQTT_WILLMSG,
  MQTT_WILLMSG_RSP,
  MQTT_WILLMSG_CONTENT,
  
  MQTT_WILLMSG_CONTENT_RSP = 20,
  MQTT_CONNECT,
  MQTT_CONNECT_RSP,
  MQTT_PUB_TOPIC,
  MQTT_PUB_TOPIC_RSP,
  
  MQTT_PUB_TOPIC_CONTENT = 25,
  MQTT_PUB_TOPIC_CONTENT_RSP,
  MQTT_PUB_PAYLOAD,
  MQTT_PUB_PAYLOAD_RSP,
  MQTT_PUB_PAYLOAD_CONTENT,
  
  MQTT_PUB_PAYLOAD_CONTENT_RSP = 30,
  MQTT_PUB,
  MQTT_PUB_RSP,
  MQTT_SUB,
  MQTT_SUB_RSP,
  
  MQTT_SUB_UPDATE = 35,
  MQTT_SUB_UPDATE_RSP,
  MQTT_SUB_UPDATE_CONTENT,
  MQTT_SUB_UPDATE_CONTENT_RSP,
  MQTT_SUB_CONTENT,

  MQTT_OK = 40 // pub ok
}MQTT_PHASE;

typedef enum
{
  MQTT_RCV_INIT = 0,
  MQTT_RCV_SUM,
  MQTT_RCV_TOPIC_LEN,
  MQTT_RCV_TOPIC_CONTENT,
  MQTT_RCV_PAYLOAD_LEN,
  MQTT_RCV_PAYLOAD_CONTENT,
}MQTT_RCV_PHASE;

typedef enum
{
  MQTT_RCV_TOPIC_CONTROL = 0,
  MQTT_RCV_TOPIC_UPDATE
}MQTT_RCV_TOPIC_TYPE;


typedef enum
{
  MQTT_SND_INIT = 0,
  MQTT_SND_TOPIC,
  MQTT_SND_TOPIC_RSP,
  MQTT_SND_TOPIC_CONTENT,
  MQTT_SND_TOPIC_CONTENT_RSP,
  MQTT_SND_PAYLOAD = 5,
  MQTT_SND_PAYLOAD_RSP,
  MQTT_SND_PAYLOAD_CONTENT,
  MQTT_SND_PAYLOAD_CONTENT_RSP,
  MQTT_SND_CMD,
  MQTT_SND_CMD_RSP1 = 10,
  MQTT_SND_CMD_RSP2,
}MQTT_SND_PHASE;

typedef enum 
{
  CMD_MOTOR_UP = 1,
  CMD_MOTOR_DOWN = 2,
  CMD_MOTOR_STATUS_GET = 3,
  CMD_BEEP_ON = 4,
  CMD_BEEP_OFF = 5,
  CMD_BEEP_STATUS_GET = 6,
  CMD_ADC_GET = 7,
  CMD_MODIFY_DISTANCE = 9,
  CMD_GET_DISTANCE = 10,
  CMD_MODIFY_SCANCYCLE = 11,
  CMD_GET_SCANCYCLE = 12,
  
  CMD_F405_ONLINE = 31,  
  CMD_UPDATE = 32,   
  CMD_ADD_ENDPOINT = 33,
  CMD_DEL_ENDPOINT = 34,
  CMD_MODIFY_CHANNEL = 35,
  CMD_MODIFY_SERVER_DISTANCE    = 36,
  CMD_GET_SERVER_DISTANCE    = 37,

}CMD_TYPE;

typedef enum
{
  //Normal
  NORMAL_SUCCESS = 101,
  NORMAL_DOWN = 102,
  NORMAL_FORWARD = 103,    
  NORMAL_UP = 104,
  NORMAL_BACK = 105,
  NORMAL_BUSY = 106,
  NORMAL_BEEP_OPEN_FAILED = 107,
  NORMAL_BEEP_CLOSE_FAILED = 108,
  NORMAL_BEEP_STATUS_OPEN = 109,
  NORMAL_BEEP_STATUS_CLOSED = 110,
  NORMAL_MOTOR_RUNNING      = 111,
  NORMAL_HAVE_CAR           = 112,
  NORMAL_UNKNOWN_ERROR      = 113,  
  NORMAL_CAR_CHECK_ERROR    = 114,

  //child online or offline
  CHILD_ONLINE              = 120,
  CHILD_OFFLINE             = 121,
  NODE_NOTEXIST             = 122,
  NODE_NO_SUPPORT_CMD       = 123,

  //get firmware
  GET_RUN_BIN = 131,  
  GET_RECOVERY_BIN = 132,  
  GET_RUN_BIN_FIN = 133,  
  GET_RECOVERY_BIN_FIN = 134,  
  ENDPOINT_ADD_SUCCESS = 135,  
  ENDPOINT_DEL_SUCCESS = 136,  
  CHANNEL_MODIFY_SUCCESS = 137,
  SOUND_DIST_SET_SUCCESS = 138, 
  SOUND_DIST_SET_FAILED = 139, 
  SOUND_DIST_GET_SUCCESS = 140, 
  SOUND_SCANCYCLE_SET_SUCCESS = 141, 
  SOUND_SCANCYCLE_SET_FAILED = 142, 
  SOUND_SCANCYCLE_GET_SUCCESS = 143,


  //通知服务器是否有车
  UPLOAD_HAVE_CAR = 150,
  UPLOAD_NO_HAVE_CAR = 151,
  UPLOAD_SOUNDWAVE_TROUBLE = 152,  

  //interupt
  INTERUPT_DOWN = 201,
  INTERUPT_FORWARD = 202,    
  INTERUPT_UP = 203,
  INTERUPT_BACK = 204,  

  INTERUPT_TIMEOUT = 210,  
  
}RESP_CODE;


typedef enum
{
  FTP_INIT = 0,    
  FTP_PORT = 1,    
  FTP_SERV = 2,
  FTP_TYPE = 3,
  FTP_MODE = 4,  
  FTP_USER = 5,
  FTP_PASSWD = 6,
  FTP_OK = 7
}FTP_INIT_PHASE;


#if 0
typedef enum
{
  UPDATE_STEP_NONE = 0,
  UPDATE_STEP_MD5,
  UPDATE_STEP_SIZE,
  UPDATE_STEP_FW
    
}UPDATE_STEP;
typedef enum
{
  UPDATE_MD5_SEND,
  UPDATE_MD5_RCV_OK,
  UPDATE_MD5_RCV_DATA_LEN,
  UPDATE_MD5_RCV_DATA,
  UPDATE_MD5_FIN  
}UPDATE_MD5_STEP;
#endif

typedef enum
{
  UPDATE_STEP_NONE = 0,
    
  UPDATE_MD5_SEND,
  UPDATE_MD5_RCV_OK,
  UPDATE_MD5_RCV_DATA_LEN,
  UPDATE_MD5_RCV_DATA,
  UPDATE_MD5_FIN,
  
  UPDATE_SIZE_SEND,
  UPDATE_SIZE_RCV_OK,
  UPDATE_SIZE_RCV_DATA_LEN,
  UPDATE_SIZE_RCV_DATA,
  UPDATE_SIZE_FIN, 

  UPDATE_FW_SEND,
  UPDATE_FW_RCV_OK,
  UPDATE_FW_RCV_EVENT,  
  UPDATE_FW_RCV_CACHE_FIN,

  UPDATE_FW_FTPCACHERD_SIZE_SEND,  
  UPDATE_FW_FTPCACHERD_SIZE_RES,
  UPDATE_FW_FTPCACHERD_SIZE_OK,

  UPDATE_FW_FTPCACHERD_DATA_SEND,  
  UPDATE_FW_FTPCACHERD_DATA_LEN,  
  UPDATE_FW_FTPCACHERD_DATA_LEN_END,
  UPDATE_FW_FTPCACHERD_DATA,  
  UPDATE_FW_FTPCACHERD_DATA_OK,  
  
  
}UPDATE_SIZE_STEP;

#if 0
typedef enum 
{
  CODE_NO_ERROR = 0,
  CODE_CMD_LACK_PARAM_HEAD = 1,
  CODE_CMD_LACK_PARAM_TYPE = 2,
  CODE_CMD_LACK_PARAM_SIMID = 3,
  CODE_CMD_LACK_PARAM_IDENTIFY = 4,
  CODE_CMD_LACK_PARAM_ENDPOITID = 5,
  CODE_CMD_LACK_PARAM_DATA = 6,
  CODE_CMD_LACK_PARAM_VERSION = 7,
  CODE_CMD_LACK_PARAM_PATH = 8,  
}UPDATE_ERROR_CODE;
#endif

#if 0
typedef enum
{
  UPDATE_FW_SEND,
  UPDATE_FW_RCV_OK,
  UPDATE_FW_RCV_DATA_LEN,
  UPDATE_FW_RCV_DATA,
  UPDATE_FW_FIN  
}UPDATE_FW_STEP;
#endif

#endif
