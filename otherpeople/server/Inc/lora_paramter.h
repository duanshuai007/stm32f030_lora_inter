#ifndef __LORA_PARAMTER_H_
#define __LORA_PARAMTER_H_

typedef enum {
  PARITY_8N1,
  PARITY_8O1,
  PARITY_8E1,
  PARITY_8N1_S,
} ParityType;

typedef enum {
  BAUD_1200,
  BAUD_2400,
  BAUD_4800,
  BAUD_9600,
  BAUD_19200,
  BAUD_38400,
  BAUD_57600,
  BAUD_115200,
} BaudType;

typedef enum {
  SPEED_IN_AIR_0_3K,
  SPEED_IN_AIR_1_2K,
  SPEED_IN_AIR_2_4K,
  SPEED_IN_AIR_4_8K,
  SPEED_IN_AIR_9_6K,
  SPEED_IN_AIR_19_2K,
  SPEED_IN_AIR_19_2K_1,
  SPEED_IN_AIR_19_2K_2,
} SpeedInAirType;

typedef enum {
  CHAN_410MHZ, CHAN_411MHZ, CHAN_412MHZ, CHAN_413MHZ, CHAN_414MHZ,
  CHAN_415MHZ, CHAN_416MHZ, CHAN_417MHZ, CHAN_418MHZ, CHAN_419MHZ,
  CHAN_420MHZ, CHAN_421MHZ, CHAN_422MHZ, CHAN_423MHZ, CHAN_424MHZ,
  CHAN_425MHZ, CHAN_426MHZ, CHAN_427MHZ, CHAN_428MHZ, CHAN_429MHZ,
  CHAN_430MHZ, CHAN_431MHZ, CHAN_432MHZ, CHAN_433MHZ, CHAN_434MHZ,
  CHAN_435MHZ, CHAN_436MHZ, CHAN_437MHZ, CHAN_438MHZ, CHAN_439MHZ,
  CHAN_440MHZ, CHAN_441MHZ,
} CHANType;

typedef enum {
  TRANSFER_MODE_TOUMING,
  TRANSFER_MODE_DINGDIAN,	
} TransferModeType;

typedef enum {
  IOMODE_OD,
  IOMODE_PP,
} IOType;

typedef enum {
  WAKEUP_TIME_250MS,
  WAKEUP_TIME_500MS,
  WAKEUP_TIME_750MS,
  WAKEUP_TIME_1000MS,
  WAKEUP_TIME_1250MS,
  WAKEUP_TIME_1500MS,
  WAKEUP_TIME_1750MS,
  WAKEUP_TIME_2000MS,
} WakeUpTimeType;

typedef enum {
  FEC_DISABLE,
  FEC_ENABLE,
} FECType;

typedef enum {
  SEND_20DB,
  SEND_17DB,
  SEND_14DB,
  SEND_10DB,
} SendDBType;

typedef enum {
  SAVE_NONE,
  SAVE_IN_FLASH,
} SaveType;

typedef enum {
  LoraReadPar,
  LoraReadVer,
} ReadType;

#endif



