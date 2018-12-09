#include "sk_adc.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_adc.h"


#define ADC1_DR_Address    ((u32)0x40012400+0x4c)
__IO uint16_t ADC_ConvertedValue;

static void SK_ADC1GPIOInit(void)
{
    GPIO_InitTypeDef stGpioInit;

    /* Enable ADC1 and GPIOC clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    /* Configure PC.01  as analog input */
    stGpioInit.GPIO_Pin = GPIO_Pin_1;
    stGpioInit.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &stGpioInit);
}

static void SK_ADC1DMAInit(void)
{
    DMA_InitTypeDef stDMA_Init;

    /* Enable DMA clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* DMA channel1 configuration */
    DMA_DeInit(DMA1_Channel1);

    stDMA_Init.DMA_PeripheralBaseAddr = ADC1_DR_Address;
    stDMA_Init.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;
    stDMA_Init.DMA_DIR = DMA_DIR_PeripheralSRC;
    stDMA_Init.DMA_BufferSize = 1;
    stDMA_Init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    stDMA_Init.DMA_MemoryInc = DMA_MemoryInc_Disable;
    stDMA_Init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    stDMA_Init.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    stDMA_Init.DMA_Mode = DMA_Mode_Circular;
    stDMA_Init.DMA_Priority = DMA_Priority_High;
    stDMA_Init.DMA_M2M = DMA_M2M_Disable;

    DMA_Init(DMA1_Channel1, &stDMA_Init);
    /* Enable DMA channel1 */
    DMA_Cmd(DMA1_Channel1, ENABLE);
}

void SK_ADC1Init(void)
{
    ADC_InitTypeDef stADC_Init;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    /// Step 1 : Configure I/O Pin First
    SK_ADC1GPIOInit();

    /// Step 2 : Configure DMA
    SK_ADC1DMAInit();

    /// Step 3 : PCLK2 div in 8, ADC CLK => 9Mhz
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);

    /// Step 4 : Configure Basic function of ADC1
    stADC_Init.ADC_Mode = ADC_Mode_Independent;
    stADC_Init.ADC_ScanConvMode = DISABLE;
    stADC_Init.ADC_ContinuousConvMode = ENABLE;
    stADC_Init.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    stADC_Init.ADC_DataAlign = ADC_DataAlign_Right;
    stADC_Init.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &stADC_Init);

    /// Step 5 : Configure Ch11 sample rate
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_55Cycles5);

    /// Step 6 : Enable ADC1 DMA
    ADC_DMACmd(ADC1, ENABLE);

    /// Step 7 : Enable ADC1
    ADC_Cmd(ADC1, ENABLE);

    /// Step 8 : Reset Calibration Register
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));

    /// Step 9 : Start Calibration
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

    /// Step 10 : Software trigger ADC1
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
