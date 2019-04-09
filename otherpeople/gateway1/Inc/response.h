#ifndef _RESPONSE_H_
#define _RESPONSE_H_

#include "cjson.h"
#include <stdint.h>


void firmware_resp(uint8_t *buff, uint8_t *simid, int cmdType, int result, int identify, uint8_t *info);
void normal_resp(uint8_t *buff, int action_resp, int identify_resp);
void root_online_resp(uint8_t *buff, uint8_t *GWID, uint8_t *simId, uint8_t *F405_VER, uint8_t *F103_VER);
void child_online_resp(uint8_t *buff, uint8_t *GWID, int EPID, int status, int haveCar);
void child_offline_resp(uint8_t *buff, int EPID, uint8_t *GWID);
void interupt_resp(uint8_t *buff, uint8_t *GWID, int EPID, int status);
void sound_check_resp(uint8_t *buff, uint8_t *GWID, int EPID, int haveCar);



#endif
