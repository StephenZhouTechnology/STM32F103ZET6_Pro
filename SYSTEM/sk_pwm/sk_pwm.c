#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_pwr.h"

#include "sk_pwm.h"

#define TIMx_DEFAULT_PERIOD     (1000)
#define CCRx_DEFAULT_VALUE      (TIMx_DEFAULT_PERIOD / 2)
#define TIMx_DEFAULT_PRE        (72)

static void SK_PWM_PortInit(void)
{
    GPIO_InitTypeDef  stGpioInit;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    stGpioInit.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    stGpioInit.GPIO_Mode = GPIO_Mode_AF_PP;
    stGpioInit.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOA, &stGpioInit);

    stGpioInit.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &stGpioInit);
}

static void SK_PWM_NvicInit(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static void SK_PWM_ClockInit(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8, ENABLE);
}

static void SK_PWM_ModeInit(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_BDTRInitTypeDef      TIM1_BDTRInitStruct;

    // Configure the period and prescaler
    TIM_TimeBaseStructure.TIM_Period = (TIMx_DEFAULT_PERIOD - 1);
    TIM_TimeBaseStructure.TIM_Prescaler = (TIMx_DEFAULT_PRE - 1);
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // Configure the Output mode
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCRx_DEFAULT_VALUE;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity= TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCRx_DEFAULT_VALUE / 2;
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCRx_DEFAULT_VALUE / 4;
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCRx_DEFAULT_VALUE / 10;
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
#if 1
    TIM1_BDTRInitStruct.TIM_OSSRState = TIM_OSSRState_Disable;
    TIM1_BDTRInitStruct.TIM_OSSIState = TIM_OSSIState_Disable;
    TIM1_BDTRInitStruct.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
    TIM1_BDTRInitStruct.TIM_DeadTime = 205;
    TIM_BDTRConfig(TIM1, &TIM1_BDTRInitStruct);
#endif
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
}

static void SK_PWM_Enable(void)
{
    TIM_Cmd(TIM1, ENABLE);
}

void SK_PWM_Init(void)
{
    SK_PWM_PortInit();
    SK_PWM_ClockInit();
    SK_PWM_NvicInit();
    SK_PWM_ModeInit();
    SK_PWM_Enable();
}

void TIM1_UP_IRQHandler(void)
{
    static uint32_t cnt = 0;
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}

