
#ifndef STM32TIME_H
#define STM32TIME_H

#ifdef __cplusplus
extern "C"
{
#endif


#include <stm32f10x.h>
 
void SysTick_Init(void);
//void delay_nus(uint32_t n);
void delay(uint32_t n);
void udelay(uint32_t n);
uint32_t millis(void);
uint32_t micros(void);
void SysTick_Handler(void);
void RCC_Configuration(void);


#ifdef __cplusplus
}
#endif


#endif


