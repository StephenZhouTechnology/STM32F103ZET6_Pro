#include "sk_exit.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "delay.h"
#include "sk_gpio.h"

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
