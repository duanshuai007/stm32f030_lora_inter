#include "adc.h"
#include "stdint.h"
#include "stm32f0xx.h"

extern ADC_HandleTypeDef hadc;

static void adc_normanchannel_init(void)
{
  ADC_ChannelConfTypeDef sConfig;

  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  HAL_ADC_Init(&hadc);

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);
}

static void adc_verfint_init(void)
{
  ADC_ChannelConfTypeDef sConfig;
  
  __HAL_RCC_ADC1_CLK_ENABLE();

  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_BACKWARD;
  HAL_ADC_Init(&hadc);

  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);
}

uint8_t get_adc_value(void)
{
  uint32_t u32ADC_Value;
  uint32_t VREFINT_DATA;
  uint16_t VREF_CAL;
  float VDDA_VAL;
  float read_adc;
  uint8_t ret;
  
  adc_verfint_init();
  VREF_CAL = *(volatile uint16_t *)(0x1ffff7ba);
  HAL_ADCEx_Calibration_Start(&hadc);
  HAL_ADC_Start(&hadc);
  while(!__HAL_ADC_GET_FLAG(&hadc, ADC_FLAG_EOC));
  VREFINT_DATA = HAL_ADC_GetValue(&hadc);
  VDDA_VAL = VREF_CAL * 3.3f / VREFINT_DATA;
  
  adc_normanchannel_init();
  HAL_ADC_Start(&hadc);
  while(!__HAL_ADC_GET_FLAG(&hadc, ADC_FLAG_EOC));
  u32ADC_Value = HAL_ADC_GetValue(&hadc);
  read_adc = u32ADC_Value * VDDA_VAL / 4095;
  read_adc -= 1.264977f; //4V-0.2 = 3.8 , 3.8 * 1/3 = 1.26666666666
  ret = (uint8_t)(read_adc * 150.200394f); //6-0.2=5.8,5.8/3=1.9333333,1.93-1.27=0.66
  if(ret > 100)
    ret = 100;

  HAL_Delay(10);
  
//  HAL_ADC_DeInit(&hadc);
  __HAL_RCC_ADC1_CLK_DISABLE();
  
  return ret;
}

