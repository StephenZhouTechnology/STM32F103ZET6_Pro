#include "sk_print.h"
#include "stm32f10x_usart.h"

#pragma import(__use_no_semihosting)
//标准库需要的支持函数                 
struct __FILE 
{ 
    int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
    x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
    while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); 
    USART_SendData(USART1,(uint8_t)ch);   
    return ch;
}
