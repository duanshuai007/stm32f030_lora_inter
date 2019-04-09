
#include "string.h"
#include <stdlib.h>
#include "stdarg.h"
#include "response.h"
#include "stm32f4xx_hal.h"


void firmware_resp(uint8_t *buff, uint8_t *simid, int cmdType, int result, int identify, uint8_t *info)
{
  if(NULL == buff)
  {
    return;
  }
  
  cJSON *root = cJSON_CreateObject();
  
  cJSON_AddStringToObject(root, "SIMID", (char *)simid);
  cJSON_AddNumberToObject(root,"FWType",   cmdType);
  cJSON_AddNumberToObject(root,"result",   result);
  cJSON_AddNumberToObject(root,"identify",   identify); 
  cJSON_AddStringToObject(root,"INFO",   (char *)info);

  char *out = cJSON_PrintUnformatted(root);

  strcpy((char *)buff, (char *)out);

  cJSON_Delete(root); 
  free(out);

  return;
}

void normal_resp(uint8_t *buff, int action_resp, int identify_resp)
{
  if(NULL == buff)
  {
    return;
  }

  cJSON *root = cJSON_CreateObject();
  
  cJSON_AddNumberToObject(root,"action_resp",   action_resp);
  cJSON_AddNumberToObject(root,"identify_resp",   identify_resp);

  char *out = cJSON_PrintUnformatted(root);

  strcpy((char *)buff, (char *)out);

  cJSON_Delete(root); 
  free(out);

  return;
}

void root_online_resp(uint8_t *buff, uint8_t *GWID, uint8_t *simId, uint8_t *F405_VER, uint8_t *F103_VER)
{
  if(NULL == buff)
  {
    return;
  }

  cJSON *root = cJSON_CreateObject();

  cJSON_AddStringToObject(root, "GWID", (char *)GWID);  
  cJSON_AddStringToObject(root, "simId", (char *)simId);  
  cJSON_AddStringToObject(root, "F405_VER", (char *)F405_VER);  
  cJSON_AddStringToObject(root, "F103_VER", (char *)F103_VER);  

  char *out = cJSON_PrintUnformatted(root);

  strcpy((char *)buff, (char *)out);

  cJSON_Delete(root); 
  free(out);

  return;
}

void child_online_resp(uint8_t *buff, uint8_t *GWID, int EPID, int status, int haveCar)
{
  if(NULL == buff)
  {
    return;
  }

  cJSON *root = cJSON_CreateObject();

  cJSON_AddStringToObject(root, "GWID", (char *)GWID);  
  cJSON_AddNumberToObject(root,"EPID",   EPID);
  cJSON_AddNumberToObject(root,"status",   status);
  cJSON_AddNumberToObject(root,"haveCar",   haveCar);

  char *out = cJSON_PrintUnformatted(root);

  strcpy((char *)buff, (char *)out);

  cJSON_Delete(root); 
  free(out);

  return;
}

void child_offline_resp(uint8_t *buff, int EPID, uint8_t *GWID)
{
  if(NULL == buff)
  {
    return;
  }

  sprintf((char *)buff, "%d:%s", EPID, GWID);

  return;
}

void interupt_resp(uint8_t *buff, uint8_t *GWID, int EPID, int status)
{
  if(NULL == buff)
  {
    return;
  }

  cJSON *root = cJSON_CreateObject();

  cJSON_AddStringToObject(root, "GWID", (char *)GWID);  
  cJSON_AddNumberToObject(root,"EPID",   EPID);
  cJSON_AddNumberToObject(root,"status",   status);

  char *out = cJSON_PrintUnformatted(root);

  strcpy((char *)buff, (char *)out);

  cJSON_Delete(root); 
  free(out);

  return;
}

void sound_check_resp(uint8_t *buff, uint8_t *GWID, int EPID, int haveCar)
{
  if(NULL == buff)
  {
    return;
  }

  cJSON *root = cJSON_CreateObject();

  cJSON_AddStringToObject(root, "GWID", (char *)GWID);  
  cJSON_AddNumberToObject(root,"EPID",   EPID);
  cJSON_AddNumberToObject(root,"haveCar",   haveCar);

  char *out = cJSON_PrintUnformatted(root);

  strcpy((char *)buff, (char *)out);

  cJSON_Delete(root); 
  free(out);

  return;
}


