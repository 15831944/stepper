#include "temp.h"

#include "stm32f4xx_conf.h"

#include "gcode.h"
#include "math.h"

float convertAdcTemperature(uint16_t raw);

void TIM3_IRQHandler(void)
{

    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) == SET)
    {
        //puts("TIM3_CC1\n");
        ADC_SoftwareStartConv(ADC1);
    }
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
}


void ADC_IRQHandler(void)
{
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC))
    {
        float temp = convertAdcTemperature(ADC_GetConversionValue(ADC1));
        gcode_setExtruderTempMeasure(temp);
        //printf("adc=%d\n", ADC_GetConversionValue(ADC1));

        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    }

    if (ADC_GetITStatus(ADC1, ADC_IT_OVR))
    {
        //gcode_setExtruderTempMeasure(ADC_GetConversionValue(ADC1));
        puts("adc_ovr\n");

        ADC_ClearITPendingBit(ADC1, ADC_IT_OVR);
    }
}

void temp_init(void)
{
    // Analog Channel 1 on PA1
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef   GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    // Timer 3 : stepper
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM3EN, ENABLE);

    TIM_TimeBaseInitTypeDef timeBaseInitStruct;
    timeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    timeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    timeBaseInitStruct.TIM_Prescaler = 0xFFFF;
    timeBaseInitStruct.TIM_Period = 2563;
    timeBaseInitStruct.TIM_RepetitionCounter = 0; // Only TIM1 & TIM8

    TIM_TimeBaseInit(TIM3, &timeBaseInitStruct);


    TIM_OCInitTypeDef ocInitStruct;
    TIM_OCStructInit(&ocInitStruct);

    ocInitStruct.TIM_OCMode = TIM_OCMode_Active;
//    ocInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    ocInitStruct.TIM_Pulse = 1;
//    ocInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &ocInitStruct);

    TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
    NVIC_EnableIRQ(TIM3_IRQn);

    TIM_Cmd(TIM3, ENABLE);

    // Extruder ADC
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // General Configuration (shared between all ADCx)
    ADC_CommonInitTypeDef ADC_CommonInitStruct;
    ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div8;
    ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;

    ADC_CommonInit(&ADC_CommonInitStruct);

    // Configuration of ADC1
    ADC_InitTypeDef adc_config;
    adc_config.ADC_Resolution = ADC_Resolution_12b;
    adc_config.ADC_ScanConvMode = DISABLE;
    adc_config.ADC_ContinuousConvMode = DISABLE;
    adc_config.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
    adc_config.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_CC1;//ADC_ExternalTrigConv_T1_CC1;
    adc_config.ADC_DataAlign = ADC_DataAlign_Right;
    adc_config.ADC_NbrOfConversion = 1;

    ADC_Init(ADC1, &adc_config);

    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
    NVIC_SetPriority(ADC_IRQn, 8);
    NVIC_EnableIRQ(ADC_IRQn);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_480Cycles);
//    ADC_SoftwareStartConv(ADC1);
    ADC_EOCOnEachRegularChannelCmd(ADC1, ENABLE);

    ADC_Cmd(ADC1, ENABLE);



//    /* Enable ADC1 reset calibaration register */
//    ADC_ResetCalibration(ADC1);
//    /* Check the end of ADC1 reset calibration register */
//    while(ADC_GetResetCalibrationStatus(ADC1));
//    /* Start ADC1 calibaration */
//    ADC_StartCalibration(ADC1);
//    /* Check the end of ADC1 calibration */
//    while(ADC_GetCalibrationStatus(ADC1));
}

void temp_set_extruder(int temp)
{

}

void temp_set_bed(int temp)
{
    /// \todo add bed
}

int temp_get_extruder(void)
{
    // Uses a EPCOS B57550G104J﻿PTN resistor (100k @ 25°C)
    // R/T curve 8304
    // B25/100 = 4092 K

    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_480Cycles);
    // Start the conversion
    ADC_SoftwareStartConv(ADC1);
    // Wait until conversion completion
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    // Reset the flag
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    // Get the conversion value
    return ADC_GetConversionValue(ADC1)/10;
}

int temp_get_bed(void)
{
    /// \todo add bed
}

float convertAdcTemperature(uint16_t raw)
{
    const float ref_res = 11000; // Ohms
    const float ref_voltage = 3.3; // Volts
    float voltage = ref_voltage*raw/4096; // Volts
    float res = voltage*ref_res/(ref_voltage-voltage); // Ohms

    const float B = 4088.1547092533;
    const float ln_r_inf = -2.3087367665;

    float temp = B / (logf(res) - ln_r_inf) - 273; // °C

    return temp;
}
