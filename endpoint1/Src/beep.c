#include "beep.h"
#include "stdint.h"
#include "stm32f0xx.h"
#include "stm32f0xx_hal_def.h"
#include "stm32f0xx_hal_gpio.h"
#include "user_config.h"

BEEP_RESP beep_ctrl(BEEP_CMD cmd)
{
    GPIO_PinState res;

    if (BEEP_GET != cmd) {
        HAL_GPIO_WritePin(GPIO_BEEP, GPIO_BEEP_Pin, (GPIO_PinState)cmd);
        HAL_Delay(5);
    }

    res = HAL_GPIO_ReadPin(GPIO_BEEP, GPIO_BEEP_Pin);

    switch(cmd)
    {
        case BEEP_OFF:
            if(GPIO_PIN_SET == res)
                return BEEP_FAIL;
            else
                return BEEP_OK;
            break;
        case BEEP_ON:
            if(GPIO_PIN_SET == res)
                return BEEP_OK;
            else
                return BEEP_FAIL;
            break;
        case BEEP_GET:
            if(GPIO_PIN_SET == res)
                return BEEP_GET_RESP_ON;
            else
                return BEEP_GET_RESP_OFF;
            break;
        default:
            return BEEP_FAIL;
    }
}
