#ifndef __BEEP_H__
#define __BEEP_H__

typedef enum {
    BEEP_OFF,
    BEEP_ON,
    BEEP_GET,
} BEEP_CMD;

typedef enum {
    BEEP_OK             = 0,
    BEEP_FAIL           = 1,
    BEEP_GET_RESP_OFF   = 0,
    BEEP_GET_RESP_ON    = 1,
} BEEP_RESP;

BEEP_RESP beep_ctrl(BEEP_CMD cmd);

#endif
