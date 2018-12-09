#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_iwdg.h"
#include "stm32f10x_wwdg.h"

#include "delay.h"
#include "SK_Sys.h"

typedef struct {
    volatile uint32_t hdr_found;
    volatile uint32_t data_len;
    volatile uint32_t data_ready;
}SK_USART_RX_CTL_t;

SK_USART_RX_CTL_t g_stUsartRxCtl;
uint8_t g_SK_UsartRxDataBuf[SK_USART_RX_BUF_LEN] = {0};
uint8_t g_SK_UsartTxDataBuf[SK_USART_TX_BUF_LEN] = {0};

/*
 **********************************************************
 **************** GPIO Function Defination ****************
 **********************************************************
*/
void SK_GPIOInit(void)
{
    GPIO_InitTypeDef  stGpioInit;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);

    stGpioInit.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
    stGpioInit.GPIO_Mode = GPIO_Mode_Out_PP;
    stGpioInit.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOG, &stGpioInit);
}

void SK_SetLedStatus(SK_LED_e led, SK_LED_STAT_e stat)
{
    if (SK_LED_1 == led)
    {
        if (stat == SK_LED_ON)
            GPIO_SetBits(GPIOG, GPIO_Pin_13);
        else
            GPIO_ResetBits(GPIOG, GPIO_Pin_13);
    }

    if (SK_LED_2 == led)
    {
        if (stat == SK_LED_ON)
            GPIO_SetBits(GPIOG, GPIO_Pin_14);
        else
            GPIO_ResetBits(GPIOG, GPIO_Pin_14);
    }
}

/*
 ************************************************************
 **************** USART1 Function Defination ****************
 ************************************************************
*/
void SK_UsartClearRxBufStat(void)
{
    uint32_t i = 0;

    g_stUsartRxCtl.hdr_found = 0;
    g_stUsartRxCtl.data_len = 0;
    g_stUsartRxCtl.data_ready = 0;

    for (i = 0; i < SK_USART_RX_BUF_LEN; i++)
        g_SK_UsartRxDataBuf[i] = 0x00;
}

void SK_UartInit(void)
{
    GPIO_InitTypeDef stGpioInit;
    USART_InitTypeDef stUsartInit;
    NVIC_InitTypeDef stNVIC;

    /* Step1: Open USART1 And GPIOA Clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    USART_DeInit(USART1);

    /* Step2: Config The Pin Remap */
    // According to the hardware diagram, just use UART1 On PA9 And PA10 in default
    GPIO_PinRemapConfig(GPIO_Remap_USART1, DISABLE);

    /* Step3: Config RXD/TXD Mode */
    // PA9 As TXD
    stGpioInit.GPIO_Pin = GPIO_Pin_9;
    stGpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    stGpioInit.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_Init(GPIOA, &stGpioInit);
   
    // PA10 As RXD
    stGpioInit.GPIO_Pin = GPIO_Pin_10;
    stGpioInit.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &stGpioInit);

    /* Step4: Reset USART1 before use it */
    USART_DeInit(USART1);

    /* Step5: Configure the UART Basic Settings */
    stUsartInit.USART_BaudRate = 9600;
    stUsartInit.USART_WordLength = USART_WordLength_8b;
    stUsartInit.USART_StopBits = USART_StopBits_1;
    stUsartInit.USART_Parity = USART_Parity_No;
    stUsartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    stUsartInit.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &stUsartInit);

    // Configure RX Interrupt
    stNVIC.NVIC_IRQChannel = USART1_IRQn;
    stNVIC.NVIC_IRQChannelPreemptionPriority= 3 ;
    stNVIC.NVIC_IRQChannelSubPriority = 3;
    stNVIC.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&stNVIC);

#ifndef USART_USE_DMA
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
#endif
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);

    // Enable Usart1
    USART_Cmd(USART1, ENABLE);
}

void SK_UsartSendChar(uint8_t ch)
{
    while(USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);
    USART_SendData(USART1, ch);
}

void SK_UsartSendData(uint8_t *buf, uint32_t len)
{
    uint32_t i = 0;

    for (i = 0; i < len; i++)
        SK_UsartSendChar(buf[i]);
}

/*
 * Protocol: 1st. Frame start at 0x5A
 *           2nd. Second is data length
 *           3rd. Data
*/
/*
void USART1_IRQHandler(void)
{
    uint8_t rx_data = 0;
    static uint8_t cnt = 0;

    if (USART_GetITStatus(USART1, USART_IT_RXNE))
    {
        rx_data = USART_ReceiveData(USART1);
    }

    if (SK_USART_RX_FRM_HEADER == rx_data && !g_stUsartRxCtl.hdr_found)
    {
        g_stUsartRxCtl.hdr_found++;
        return;
    }

    if (g_stUsartRxCtl.hdr_found && g_stUsartRxCtl.data_len == 0)
    {
        g_stUsartRxCtl.data_len = rx_data;
        if (g_stUsartRxCtl.data_len > SK_USART_RX_BUF_LEN)
            g_stUsartRxCtl.data_len = SK_USART_RX_BUF_LEN;
        return;
    }

    if (cnt < g_stUsartRxCtl.data_len)
    {
        g_SK_UsartRxDataBuf[cnt++] = rx_data;
        if (g_stUsartRxCtl.data_len == cnt)
        {
            g_stUsartRxCtl.data_ready = 1;
            cnt = 0;
            SK_SetLedStatus(SK_LED_1, SK_LED_ON);
            delay_ms(2000);
            SK_SetLedStatus(SK_LED_1, SK_LED_OFF);
            SK_UsartSendData(g_SK_UsartRxDataBuf, g_stUsartRxCtl.data_len);
        }
    }
}
*/

/*
 ************************************************************
 **************** DMA Function Defination *******************
 ************************************************************
*/
static void DMA_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
    NVIC_Init(&NVIC_InitStructure);
}

#ifdef USART_USE_DMA
void SK_UsartDmaInit(void)
{
    DMA_InitTypeDef stDMA_InitStructCh4;
    DMA_InitTypeDef stDMA_InitStructCh5;

    /// Step 1 : Open the DMA1 Clock
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // NVIC Config
    DMA_NVIC_Config();

    /// Step 2 : Reset DMA1_CH4(For USART1 TX) and DMA1_CH5(For USART1 RX)
    DMA_DeInit(DMA1_Channel4);

    // Configure the USART1 TX DMA Transfer for DMA CH4
    // Configure the Peripheral address
    stDMA_InitStructCh4.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    // Configure the TX Data Buffer address
    stDMA_InitStructCh4.DMA_MemoryBaseAddr     = (uint32_t)g_SK_UsartTxDataBuf;
    // Configure the data direct: Memory to Peripheral
    stDMA_InitStructCh4.DMA_DIR                = DMA_DIR_PeripheralDST;
    // Configure the data Len
    stDMA_InitStructCh4.DMA_BufferSize         = SK_USART_TX_BUF_LEN;
    // Configure the Peripheral Address auto add (disable)
    stDMA_InitStructCh4.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    // Configure the Memory Address auto add (enable)
    stDMA_InitStructCh4.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    // Configure the Peripheral Data Size = 1 byte
    stDMA_InitStructCh4.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    // Configure the Memory Data Size = 1 byte
    stDMA_InitStructCh4.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    // Configure Normal mode
    stDMA_InitStructCh4.DMA_Mode               = DMA_Mode_Normal;
    // Configure Priority as medium
    stDMA_InitStructCh4.DMA_Priority           = DMA_Priority_Medium;
    // Disable memory to memory
    stDMA_InitStructCh4.DMA_M2M                = DMA_M2M_Disable;
    // Config DMA1 CH4
    DMA_Init(DMA1_Channel4, &stDMA_InitStructCh4);
    // Enable Ch4
    DMA_Cmd(DMA1_Channel4, ENABLE);
    // Enable IRQ when transfer finished
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

    /// Step 3: Reset DMA1_CH5(For USART1 RX)
    DMA_DeInit(DMA1_Channel5);
    // Configure the USART1 RX DMA for DMA CH5
    // Configure the Peripheral address
    stDMA_InitStructCh5.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    // Configure the TX Data Buffer address
    stDMA_InitStructCh5.DMA_MemoryBaseAddr     = (uint32_t)g_SK_UsartRxDataBuf;
    // Configure the data direct: Memory to Peripheral
    stDMA_InitStructCh5.DMA_DIR                = DMA_DIR_PeripheralSRC;
    // Configure the data Len
    stDMA_InitStructCh5.DMA_BufferSize         = SK_USART_RX_BUF_LEN;
    // Configure the Peripheral Address auto add (disable)
    stDMA_InitStructCh5.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    // Configure the Memory Address auto add (enable)
    stDMA_InitStructCh5.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    // Configure the Peripheral Data Size = 1 byte
    stDMA_InitStructCh5.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    // Configure the Memory Data Size = 1 byte
    stDMA_InitStructCh5.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    // Configure Normal mode
    stDMA_InitStructCh5.DMA_Mode               = DMA_Mode_Normal;
    // Configure Priority as medium
    stDMA_InitStructCh5.DMA_Priority           = DMA_Priority_Medium;
    // Disable memory to memory
    stDMA_InitStructCh5.DMA_M2M                = DMA_M2M_Disable;
    // Config DMA1 CH5
    DMA_Init(DMA1_Channel5, &stDMA_InitStructCh5);
    // Enable Ch5
    DMA_Cmd(DMA1_Channel5, ENABLE);
    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);
}
#endif

/*
 ************************************************************
 **************** EXTI Function Defination *******************
 ************************************************************
*/
// PE5 Connect with K1
// PE6 Connect with K2
static SK_SK_EXITNVICConfig(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* PE5/PE6 */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void SK_EXITInit(void)
{
    GPIO_InitTypeDef stGpioInit;
    EXTI_InitTypeDef stEXTIInit;

    /// Step 1 : Power On the AFIO and GPIOE
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);

    /// Step 2 : Config the NVIC
    SK_SK_EXITNVICConfig();

    /// Step 3 : Config (PE5/PE6) As pull up input
    stGpioInit.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    stGpioInit.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOE, &stGpioInit);

    /// Step 4 : Select the output interrupt source
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource5);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource6);

    /// Step 5 : Select the output interrupt source
    stEXTIInit.EXTI_Line = EXTI_Line5 | EXTI_Line6;
    stEXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    stEXTIInit.EXTI_Trigger = EXTI_Trigger_Falling;
    stEXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&stEXTIInit);
}


/*
 ***************************************************************
 **************** ADC1 Function Defination *******************
 ***************************************************************
*/
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

/*
 ***************************************************************
 **************** TIM6&TIM7 Function Defination ****************
 ***************************************************************
*/
#define TIM6_CNT    (10000 - 1)
#define TIM6_PSC    (7200 - 1)

#define TIM7_CNT    (10000 - 1)
#define TIM7_PSC    (7200*2 - 1)

static SK_TIM6_NVICConifg(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* PE5/PE6 */
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static SK_TIM7_NVICConifg(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* PE5/PE6 */
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void SK_TIM6_Init(void)
{   
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    SK_TIM6_NVICConifg();

    /// Step 1 : Open TIM6&TIM7 Clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    /// Step 2 : Set basic settings
    // Input CLOCK = 72MHz
    TIM_TimeBaseStructure.TIM_Period = TIM6_CNT;
    TIM_TimeBaseStructure.TIM_Prescaler = TIM6_PSC;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

    TIM_ARRPreloadConfig(TIM6, ENABLE);

    TIM_SelectOnePulseMode(TIM6, TIM_OPMode_Repetitive);

    TIM_UpdateRequestConfig(TIM6, TIM_UpdateSource_Global);

    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    TIM_Cmd(TIM6, ENABLE);
}

void SK_TIM7_Init(void)
{   
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    SK_TIM7_NVICConifg();

    /// Step 1 : Open TIM6&TIM7 Clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

    /// Step 2 : Set basic settings
    // Input CLOCK = 72MHz
    TIM_TimeBaseStructure.TIM_Period = TIM7_CNT;
    TIM_TimeBaseStructure.TIM_Prescaler = TIM7_PSC;
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

    TIM_ARRPreloadConfig(TIM7, ENABLE);

    TIM_SelectOnePulseMode(TIM7, TIM_OPMode_Repetitive);

    TIM_UpdateRequestConfig(TIM7, TIM_UpdateSource_Global);

    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

    TIM_Cmd(TIM7, ENABLE);
}

/*
 ***************************************************************
 ******************* RTC Function Defination *******************
 ***************************************************************
*/
#define LSE_CLK_32768KHZ     32768
#define RTC_CFG_DONE        0xAAAA
#define RTC_CFG_DEINIT      0xBBBB
#define RTC_YEAR_START       1970 // Start at 1971.1.1
#define RTC_YREA_END         2099

#define LEAP_YEAR_SEC       31622400
#define NORMAL_YEAR_SEC     31536000
#define A_DAY_SEC           86400
#define A_HOUR_SEC          3600
#define A_MIN_SEC           60

#define MAX_DAY             10
#define ONE_DAY_HOURS       24
#define ONE_HOUR_MIN        60
#define ONE_MIN_SEC         60
#define MAX_SEC             (MAX_DAY * ONE_DAY_HOURS * ONE_HOUR_MIN * ONE_MIN_SEC)

typedef struct {
    uint32_t day;
    uint32_t hour;
    uint32_t min;
    uint32_t sec;
} SK_TIME_t;

SK_TIME_t stTestRTCTime = {13, 00, 30, 00};

SK_TIME_t g_stCurrentTime;

const uint32_t mon_table[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

//static uint8_t _isLeapYear(uint32_t year)
//{
//    if(((year % 4 == 0) && (year % 100 != 0)) || year % 400 == 0)
//        return 1;
//    else
//        return 0;
//}

static void _setCurrentTime(SK_TIME_t *cur_time)
{
#if 0
    uint16_t t = 0;
    uint32_t secCount=0;

    // Calculate for Year
    for(t = RTC_YEAR_START; t < cur_time->year; t++)
    {
        if(_isLeapYear(t))
            secCount += LEAP_YEAR_SEC;
        else
            secCount += NORMAL_YEAR_SEC;
    }

    cur_time->month -= 1;
    // Calculate for Month
    for(t = 0; t < cur_time->month; t++)
    {
        secCount += (uint32_t)mon_table[t] * A_DAY_SEC;
        if(_isLeapYear(cur_time->year) && t == 1)
            secCount += A_DAY_SEC;
    }

    // Calculate for Day
    secCount += (cur_time->day - 1) * A_DAY_SEC;

    // Calculate for hours
    secCount += cur_time->hour * A_HOUR_SEC;

    // Calculate for min
    secCount += cur_time->min * A_MIN_SEC;

    // Calculate for seconds
    secCount += cur_time->sec;

    RTC_SetCounter(secCount);
    RTC_WaitForLastTask();
#endif
    // Test RTC At 2018.7.13 -- 00:26:00
    RTC_SetCounter(0x00);
    RTC_WaitForLastTask();
}

void SK_getCurrentTime(SK_TIME_t *cur_time)
{
#if 0
    static uint32_t dayCount = 0;

    uint32_t secCount = 0;
    uint32_t tmp = 0;
    uint32_t tmp1 = 0;

    secCount = RTC_GetCounter();
    // Get the total day
    tmp = secCount / A_DAY_SEC;

    if(dayCount != tmp)
    {
        dayCount = tmp;
        tmp1 = RTC_YEAR_START;

        while(tmp >= 365)
        {
            if(_isLeapYear(tmp1))
            {
                if(tmp >= 366)
                    tmp -= 366;
                else
                    break;
            }
            else
            {
                tmp -= 365;
            }
            tmp1++;
        }
        // Get the year
        cur_time->year = tmp1;
        tmp1 = 0;
        while(tmp >= 28)
        {
            if(_isLeapYear(cur_time->year) && tmp1 == 1)
            {
                if(tmp >= 29)
                    tmp -= 29;
                else
                    break;
            }
            else
            {
                if(tmp >= mon_table[tmp1])
                    tmp -= mon_table[tmp1];
                else
                    break;
            }
        }
        tmp1++;
        // Get the month
        cur_time->month = tmp1 + 1;
        // Get the day
        cur_time->day = tmp + 1;
    }

    tmp = secCount % A_DAY_SEC;
    cur_time->hour = tmp / 3600;
    cur_time->min=(tmp % 3600) / 60;
    cur_time->sec=(tmp % 3600) % 60;
#endif
    uint32_t secCount = RTC_GetCounter();
    uint32_t sec = secCount % A_DAY_SEC;

    cur_time->day   = secCount / A_DAY_SEC ;
    cur_time->hour  = sec / 3600;
    cur_time->min   = (sec % 3600) / 60;
    cur_time->sec   = (sec % 3600) % 60;
}


static uint8_t SK_RTCIsConfiged(void)
{
    return ((BKP_ReadBackupRegister(BKP_DR1) == RTC_CFG_DONE) ? 1 : 0);
}

static void SK_RTCNVICConfig(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
    NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static void SK_RTC_Configuration(void)
{
    // Step 1 : Open Power & Backup zone Clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

    // Step 2 : Set Power register to allow to access backup domain
    PWR_BackupAccessCmd(ENABLE);

    // Step 3 : Rest BackUp domain
    BKP_DeInit();

    // Step 4 : Enable LSE Clock and wait for ready
    RCC_LSEConfig(RCC_LSE_ON);
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);

    // Step 5 : Configure the LSE as RTC Clock input
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

    // Step 6 : Enable RTC Clock
    RCC_RTCCLKCmd(ENABLE);

    // Step 7 : Because APB1 was reset when power on, follow datasheet, must wait RSF
    RTC_WaitForSynchro();

    // Step 8 : Wait until last write was finished
    RTC_WaitForLastTask();

    // Step 9 : Set prescaler as 32767
    // RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1)
    RTC_SetPrescaler(LSE_CLK_32768KHZ - 1);
    RTC_WaitForLastTask();

    // Step 10 : Enable second interrupt
    RTC_ITConfig(RTC_IT_SEC, ENABLE);
    RTC_WaitForLastTask();
}


void SK_RTCInit(void)
{
    // First Configure RTC
    if (!SK_RTCIsConfiged())
    {
        SK_RTC_Configuration();
        _setCurrentTime(&stTestRTCTime);
        BKP_WriteBackupRegister(BKP_DR1, RTC_CFG_DONE);
    }
    else
    {
        RTC_WaitForSynchro();
        RTC_ITConfig(RTC_IT_SEC, ENABLE);
        RTC_WaitForLastTask();
    }
    SK_RTCNVICConfig();
    SK_getCurrentTime(&g_stCurrentTime);
}

void SK_RTCDeInit(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    PWR_BackupAccessCmd(ENABLE);
    BKP_DeInit();
}

void RTC_IRQHandler(void)
{
    if (RTC_GetITStatus(RTC_IT_SEC) != RESET)
    {
        if (RTC_GetCounter() >= MAX_SEC)
        {
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
            PWR_BackupAccessCmd(ENABLE);
            RTC_WaitForLastTask();
            RTC_SetCounter(0x0);
            RTC_WaitForLastTask();
        }
        SK_getCurrentTime(&g_stCurrentTime);
    }
    if(RTC_GetITStatus(RTC_IT_ALR)!= RESET)
    {
        RTC_ClearITPendingBit(RTC_IT_ALR);
    }
    RTC_ClearITPendingBit(RTC_IT_SEC | RTC_IT_OW);
    RTC_WaitForLastTask();
}

/*
 ***************************************************************
 ******************* IWDG Function Defination *******************
 ***************************************************************
*/
void SK_IWDGInit(uint16_t prer, uint16_t wdg_value)
{
    // First Open LSI Clock for IWDG
    RCC_LSICmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);

    // If use IWDG, LSI will be opened force
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    // Set Prescaler
    IWDG_SetPrescaler(prer);
    // Set reload value
    IWDG_SetReload(wdg_value);
    // Set 0xAAAA To make sure not go into reset
    IWDG_ReloadCounter();
    // Enable WDG
    IWDG_Enable();
}

void SK_IWDGFeed(void)
{
    IWDG_ReloadCounter();
}

/*
 ***************************************************************
 ******************* WWDG Function Defination *******************
 ***************************************************************
*/
static void SK_WWDGNVICConfig(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = WWDG_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_Init(&NVIC_InitStructure);

}
void SK_WWDGInit(uint8_t target, uint8_t window, uint32_t fprer)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);

    target = target & MAX_WWDG_CNT;

    WWDG_SetPrescaler(fprer);

    WWDG_SetWindowValue(window);

    SK_WWDGNVICConfig();

    WWDG_Enable(target);

    WWDG_ClearFlag();

    WWDG_EnableIT();
}

void SK_WWDGFeed(uint8_t target)
{
    WWDG_Enable(target);
}


void WWDG_IRQHandler(void)
{
    //WWDG_SetCounter(MAX_WWDG_CNT); // If open this, it will never reset

    WWDG_ClearFlag();

    //SK_SetLedStatus(SK_LED_1, SK_LED_ON);
    //SK_SetLedStatus(SK_LED_1, SK_LED_OFF);
}


/*
 ***************************************************************
 *************** SPI Flash Function Defination *****************
 ***************************************************************
*/



/*
 ***************************************************************
 **************** Vector Function Defination *******************
 ***************************************************************
*/
#ifdef USART_USE_DMA
uint32_t Usart1_Rec_Cnt = 0;

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_IDLE))
    {
        uint8_t clr = 0;
        clr = USART1->SR;
        clr = USART1->DR;

        Usart1_Rec_Cnt = DMA_GetCurrDataCounter(DMA1_Channel5);

        SK_SetLedStatus(SK_LED_1, SK_LED_ON);
        delay_ms(2000);
        SK_SetLedStatus(SK_LED_1, SK_LED_OFF);
    }
}

void DMA1_Channel4_IRQHandler(void)
{
    DMA_ClearITPendingBit(DMA1_IT_TC4);
    SK_SetLedStatus(SK_LED_2, SK_LED_ON);
}

void DMA1_Channel5_IRQHandler(void)
{
    DMA_ClearITPendingBit(DMA1_IT_TC5);
    SK_SetLedStatus(SK_LED_2, SK_LED_OFF);
}

#else
void USART1_IRQHandler(void)
{
    uint8_t rx_data = 0;
    static uint8_t cnt = 0;
    uint8_t clr = 0;
    if (USART_GetITStatus(USART1, USART_IT_RXNE))
    {
        rx_data = USART_ReceiveData(USART1);
        g_SK_UsartRxDataBuf[cnt++] = rx_data;
    }

    if (USART_GetITStatus(USART1, USART_IT_IDLE))
    {
        clr = USART1->SR;
        clr = USART1->DR;
        SK_SetLedStatus(SK_LED_1, SK_LED_ON);
        delay_ms(2000);
        SK_SetLedStatus(SK_LED_1, SK_LED_OFF);
        SK_UsartSendData(&rx_data, 1);
    }
}
#endif

void EXTI9_5_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line5) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line5);
        SK_SetLedStatus(SK_LED_1, SK_LED_ON);
        delay_ms(5000);
        SK_SetLedStatus(SK_LED_1, SK_LED_OFF);
    }

    if(EXTI_GetITStatus(EXTI_Line6) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line6);
        SK_SetLedStatus(SK_LED_2, SK_LED_ON);
        delay_ms(5000);
        SK_SetLedStatus(SK_LED_2, SK_LED_OFF);
    }

}

void TIM6_IRQHandler(void)
{
    static uint8_t cnt = 0;
    if (TIM_GetITStatus(TIM6, TIM_IT_Update))
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        if (!cnt)
        {
            SK_SetLedStatus(SK_LED_1, SK_LED_ON);
            cnt++;
        }
        else
        {
            SK_SetLedStatus(SK_LED_1, SK_LED_OFF);
            cnt--;
        }
    }
}

void TIM7_IRQHandler(void)
{
    static uint8_t cnt = 0;
    if (TIM_GetITStatus(TIM7, TIM_IT_Update))
    {
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
        if (!cnt)
        {
            SK_SetLedStatus(SK_LED_2, SK_LED_ON);
            cnt++;
        }
        else
        {
            SK_SetLedStatus(SK_LED_2, SK_LED_OFF);
            cnt--;
        }
    }
}


