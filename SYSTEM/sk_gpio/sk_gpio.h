#ifndef __SK_GPIO_H__
#define __SK_GPIO_H__

typedef enum {
    SK_LED_1,
    SK_LED_2,
    SK_LED_MAX
} SK_LED_e;

typedef enum {
    SK_LED_OFF,
    SK_LED_ON,
    SK_LED_NONE
} SK_LED_STAT_e;

void SK_GPIOInit(void);
void SK_SetLedStatus(SK_LED_e led, SK_LED_STAT_e stat);


#endif // __SK_GPIO_H__