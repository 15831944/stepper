#include "temp.h"

#include "stm32f4xx_conf.h"

#include "gcode.h"
#include "math.h"


float extruder_setpoint = 0.0;

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
        if (temp < extruder_setpoint)
        {
            GPIO_WriteBit(GPIOA, GPIO_Pin_3,  Bit_SET); // Turn heater on if below temp
        }
        else
        {
            GPIO_WriteBit(GPIOA, GPIO_Pin_3,  Bit_RESET); // Turn heater off if temperature reached
        }

        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    }

    if (ADC_GetITStatus(ADC1, ADC_IT_OVR))
    {
        puts("adc_ovr\n");

        ADC_ClearITPendingBit(ADC1, ADC_IT_OVR);
    }
}

void temp_init(void)
{

    // Extruder Analog Channel 1 on PC1 / ADC123_IN11
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef   GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOC, &GPIO_InitStructure);



    // Extruder Heater Output on PA3
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef   GPIO_InitStructureHeater;
    GPIO_InitStructureHeater.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructureHeater.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructureHeater.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructureHeater.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructureHeater.GPIO_Pin = GPIO_Pin_3;

    GPIO_Init(GPIOA, &GPIO_InitStructureHeater);

    GPIO_WriteBit(GPIOA, GPIO_Pin_3,  Bit_RESET); // Turn heater off at startup


    // Timer 3 : temp measurements
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

    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_480Cycles);
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
    extruder_setpoint = temp;
}

void temp_set_bed(int temp)
{
    /// \todo add bed
}


float convertAdcTemperature(uint16_t raw)
{
    const float ref_res = 1000; // Ohms
    const float ref_voltage = 3.3; // Volts
    float voltage = ref_voltage*raw/4096; // Volts
    float res = voltage*ref_res/(ref_voltage-voltage); // Ohms

// Beta Equation
//    const float B = 4088.1547092533;
//    const float ln_r_inf = -2.3087367665;
//    float temp = B / (logf(res) - ln_r_inf) - 273; // °C

    // Full Steinhart Hart equation
    float A = 0.000722752911679;
    float B = 0.000216691739579;
    float C = 8.91392905583e-08;
    float temp = 1/ (A + B * logf(res) + C * powf(logf(res),3) ) - 273.15;

    return temp;
}
