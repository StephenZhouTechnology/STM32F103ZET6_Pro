#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "SK_Sys.h"
#include "stm32f10x_wwdg.h"
#include "sk_spi_flash.h"
#include "sk_mirco_sd_card.h"
#include "sk_print.h"
#include "sk_pwm.h"

#define DATA_LEN                10
#define SPI_FLASH_DATA_LEN      100
#define SPI_FLASH_START_ADDR    0x00

float ADC_ConvertedValueLocal = 0;
extern __IO uint16_t ADC_ConvertedValue;
extern void SK_USB_Init(void);
//W25Q64_ID_t g_stW25Q64_ID;
//uint8_t g_spi_flash_rd_data[SPI_FLASH_DATA_LEN] = {0};
//uint8_t g_spi_flash_wr_data[SPI_FLASH_DATA_LEN] = {0};

//ALIENTEK战舰STM32开发板实验4
//串口实验  
//技术支持：www.openedv.com
//广州市星翼电子科技有限公司
 int main(void)
 {
    //uint8_t data[DATA_LEN] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
    //uint32_t i = 0;

    delay_init();
    NVIC_Configuration();   //设置NVIC中断分组2:2位抢占优先级，2位响应优先级

    // SK Interface
    /* GPIO Test */
    SK_GPIOInit();
    /* UART Test */
    //SK_UartInit();
    /* UART DMA Test */
    //SK_UsartDmaInit();
    /* EXIT Test */
    //SK_EXITInit();
    /* ADC Test */
    //SK_ADC1Init();

    SK_SetLedStatus(SK_LED_2, SK_LED_ON);
    SK_SetLedStatus(SK_LED_1, SK_LED_ON);
    delay_ms(1000);
    SK_SetLedStatus(SK_LED_2, SK_LED_OFF);
    SK_SetLedStatus(SK_LED_1, SK_LED_OFF);
    
    SK_SetLedStatus(SK_LED_2, SK_LED_ON);
    SK_SetLedStatus(SK_LED_1, SK_LED_ON);
    /* TIM Test */
    //SK_TIM6_Init();
    //SK_TIM7_Init();

    /* RTC Test */
    //SK_RTCInit();
    //SK_RTCDeInit();

    /* IWDG Test */
    //SK_IWDGInit(SK_IWDG_PER, SK_IWDG_CNT);

    /* WWDG Test */
    //SK_WWDGInit(MAX_WWDG_CNT, WWDG_TEST_WIN, WWDG_Prescaler_8);

    /* SPI FLASH TEST */
    /*
    SK_SPIFlashInit();
    SK_GetFlashID(&g_stW25Q64_ID);
    SK_GetFlashDeviceID(&g_stW25Q64_ID);
    SK_SPI_FLASH_Erase(SPI_FLASH_START_ADDR, W25X_4K_SectorErase);
    SK_SPI_FLASH_BufferRead(g_spi_flash_rd_data, SPI_FLASH_START_ADDR, SPI_FLASH_DATA_LEN);
    for (i = 0; i < SPI_FLASH_DATA_LEN; i++)
        g_spi_flash_wr_data[i] = i;
    SK_SPI_FLASH_PageWrite(g_spi_flash_wr_data, SPI_FLASH_START_ADDR, SPI_FLASH_DATA_LEN);
    SK_SPI_FLASH_BufferRead(g_spi_flash_rd_data, SPI_FLASH_START_ADDR, SPI_FLASH_DATA_LEN);
    */

    /* SDCARD TEST */
    //SK_SDIO_SDCARD_Init();
    //SD_EraseTest();
    //SD_SingleBlockTest();
    //SD_MultiBlockTest();

    printf("** Stephen.Zhou STM32 System Start Running ...\n");
    //printf("** PWM Configuration starting ...\n");
    //SK_PWM_Init(); // Must without UART1 Init,because the port conflict

    while(1)
    {
        //ADC_ConvertedValueLocal = (float) ADC_ConvertedValue/4096*3.3;
        //delay_ms(5000);
        //SK_IWDGFeed();
    }
}

