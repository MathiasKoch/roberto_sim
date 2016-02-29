
#include <stm32f10x.h>
#include "stm32_time.h"

static __IO uint32_t sysTickCounter;
 
void SysTick_Init(void) {
	/****************************************
	 *SystemFrequency/1000      1ms         *
	 *SystemFrequency/100000    10us        *
	 *SystemFrequency/1000000   1us         *
	 *****************************************/
	while (SysTick_Config(SystemCoreClock / 1000) != 0); // One SysTick interrupt now equals 1us
 
}
 
/**
 * This method needs to be called in the SysTick_Handler
 */

void SysTick_Handler(void) {
	sysTickCounter++;
}
 
/*void delay_nus(uint32_t n) {
	uint32_t i = sysTickCounter;
	while (sysTickCounter - i < n);
}*/
 
void delay(uint32_t n) {
	uint32_t i = sysTickCounter;
	while ((sysTickCounter - i) < (n));
}

uint32_t millis(void){
	return sysTickCounter;
}

void RCC_Configuration(void){
  /* PCLK1 = HCLK/4 */
  RCC_PCLK1Config(RCC_HCLK_Div4);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
 
 
  /* GPIOA, GPIOB and GPIOC clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
}