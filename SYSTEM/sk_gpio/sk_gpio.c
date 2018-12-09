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
#include "sk_gpio.h"

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

