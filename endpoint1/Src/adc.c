#include "adc.h"
#include "stdint.h"
#include "stm32f0xx.h"
#include "stm32f0xx_hal_def.h"
#include "stm32f0xx_hal_gpio.h"

HAL_StatusTypeDef get_adc_value(ADC_HandleTypeDef *hadc)
{
//    HAL_ADC_Start_DMA(&hadc, &battery, 1);
    return HAL_ADC_Start_IT(hadc);
}

