#ifndef __SK_SYS_H__
#define __SK_SYS_H__

#define SK_USART_RX_BUF_LEN     256
#define SK_USART_TX_BUF_LEN     256
#define SK_USART_RX_FRM_HEADER  0x5A

#define DMA_MAX_SUPPORT_DATA_LEN 65535

#define SK_IWDG_PER             4   // 64 Prescaler divider to 625
#define SK_IWDG_CNT             625 // 1s timeout

#define MAX_WWDG_CNT    0x7F
#define WWDG_TEST_WIN   0x5F

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
void SK_UartInit(void);
void SK_UsartSendChar(uint8_t ch);
void SK_UsartSendData(uint8_t *buf, uint32_t len);
void SK_SetLedStatus(SK_LED_e led, SK_LED_STAT_e stat);
void SK_EXITInit(void);
void SK_ADC1Init(void);
void SK_TIM6_Init(void);
void SK_TIM7_Init(void);
void SK_RTCInit(void);
void SK_RTCDeInit(void);
void SK_IWDGInit(uint16_t prer, uint16_t wdg_value);
void SK_IWDGFeed(void);
void SK_WWDGInit(uint8_t target, uint8_t window, uint32_t fprer);
void SK_WWDGFeed(uint8_t target);


#ifdef USART_USE_DMA
void SK_UsartDmaInit(void);
#endif

#endif // __SK_SYS_H__

