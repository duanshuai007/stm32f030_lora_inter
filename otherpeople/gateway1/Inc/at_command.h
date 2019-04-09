#ifndef AT_COMMAND_H_
#define AT_COMMAND_H_

#define MQTT_TOPIC_CHILD_ONLINE "AT+CMQTTTOPIC=0,13\r\n"
#define MQTT_TOPIC_CHILD_ONLINE_CONTENT "/child_online\r\n"
#define MQTT_TOPIC_CHILD_OFFLINE "AT+CMQTTTOPIC=0,14\r\n"
#define MQTT_TOPIC_CHILD_OFFLINE_CONTENT "/child_offline\r\n"
#define MQTT_TOPIC_ACTION_RSP "AT+CMQTTTOPIC=0,12\r\n"
#define MQTT_TOPIC_ACTION_RSP_CONTENT "/action_resp\r\n"
#define MQTT_TOPIC_INTERRUPT_RSP "AT+CMQTTTOPIC=0,15\r\n"
#define MQTT_TOPIC_INTERRUPT_RSP_CONTENT "/interrupt_resp\r\n"
#define MQTT_TOPIC_FW_UPDATE_RSP "AT+CMQTTTOPIC=0,16\r\n"
#define MQTT_TOPIC_FW_UPDATE_RSP_CONTENT "/firmware_update\r\n"

#define MQTT_TOPIC_HAVE_CAR_RSP "AT+CMQTTTOPIC=0,10\r\n"
#define MQTT_TOPIC_HAVE_CAR_RSP_CONTENT "/car_check\r\n"


#define ATE0 "ATE0\r\n"
#define CMQTTSTART "AT+CMQTTSTART\r\n"
#define CCID "AT+CICCID\r\n"
#define ACCQ_PREFIX "AT+CMQTTACCQ=0,"
#define WILLTOPIC "AT+CMQTTWILLTOPIC=0,13\r\n"
#define WILLTOPIC_CONTENT "/root_offline\r\n"
#define WILLMSG "AT+CMQTTWILLMSG=0,%d,1\r\n"
#define TIMEOUT "AT+CMQTTCNCTTIMEOUT=0,10\r\n"
//#define CONNECT_TEST "AT+CMQTTCONNECT=0,\"tcp://119.119.52.253:1883\",60,1\r\n"
#define CONNECT_PRODUCT "AT+CMQTTCONNECT=0,\"tcp://faye.weixinxk.net:2883\",120,1,\"disuo\",\"Di@Suo#321\"\r\n"
#define SUB "AT+CMQTTSUBTOPIC=0,%d,1\r\n"
#define SUB_UPDATE "AT+CMQTTSUB=0,15,1,0\r\n"
#define SUB_UPDATE_CONTENT "fireware_update"
#define PUBTOPIC "AT+CMQTTTOPIC=0,12\r\n"
#define PUBTOPIC_CONTENT "/root_online\r\n"
#define PUBPAYLOAD "AT+CMQTTPAYLOAD=0,%d\r\n"
#define PUB "AT+CMQTTPUB=0,1,60\r\n"
#define RCVMSG1 "+CMQTTRXSTART: 0,"
#define RCVMSG2 "+CMQTTRXTOPIC: 0,"
#define RCVMSG3 "+CMQTTRXPAYLOAD: 0,"
#define RCVMSG4 "+CMQTTRXEND: 0"


//FTP AT COMMAND
#define AT_FTPPORT "AT+CFTPPORT=21\r\n"
#define AT_FTPTYPE "AT+CFTPTYPE=I\r\n"
#define AT_FTPMODE "AT+CFTPMODE=1\r\n"

#define AT_FTPSERV "AT+CFTPSERV=\"ota.update.weixinxk.net\"\r\n"
#define AT_FTPUSER "AT+CFTPUN=\"otatest\"\r\n"
#define AT_FTPPWD "AT+CFTPPW=\"otatest.321\"\r\n"

#define AT_FTPGETMD5 "AT+CFTPGET=\"%s/%s/%s/md5\"\r\n"
#define RES_MD5_LEN "+CFTPGET: DATA,"
#define RES_MD5_FINISH "+CFTPGET: 0"

#define AT_FTPGETLIST "AT+CFTPLIST=\"%s/%s/%s/%s.bin\"\r\n"
#define RES_SIZE_LEN "+CFTPLIST: DATA,"
#define RES_SIZE_PREFIX "-rw-r--r--   1 otatest  otatest     "

#define RES_SIZE_FINISH "+CFTPLIST: 0"

#define AT_FTPGET "AT+CFTPGET=\"%s/%s/%s/%s.bin\",0,1\r\n"

#define RES_FTPGET_RECV_EVENT "+CFTP: RECV EVENT"
#define RES_FTPGET_FIN "+CFTPGET: 0"

#define AT_FTPCACHERD_SIZE "AT+CFTPCACHERD?\r\n"
#define RES_FTPCACHERD_SIZE "+CFTPCACHERD: "

#define AT_FTPCACHERD_DATA "AT+CFTPCACHERD\r\n"
#define RES_FTPCACHERD_DATA "+CFTPGET: DATA,"


#endif
