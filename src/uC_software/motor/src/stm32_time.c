
#include <stm32f10x.h>
#include "stm32_time.h"

static __IO uint32_t sysTickCounter;
 
void SysTick_Init(void) {
	/****************************************
	 *SystemFrequency/1000      1ms         *
	 *SystemFrequency/100000    10us        *
	 *SystemFrequency/1000000   1us         *
	 *****************************************/
	SystemCoreClockUpdate();
	while (SysTick_Config(SystemCoreClock / 1000000) != 0); // One SysTick interrupt now equals 1us
 
}
 
/**
 * This method needs to be called in the SysTick_Handler
 */

void SysTick_Handler(void) {
	__disable_irq();
	sysTickCounter++;
	__enable_irq();
}
 
/*void delay_nus(uint32_t n) {
	uint32_t i = sysTickCounter;
	while (sysTickCounter - i < n);
}*/
 
void delay(uint32_t n) {
	uint32_t i = millis();
	while ((millis() - i) < (n));
}

void udelay(uint32_t n) {
	uint32_t i = micros();
	while ((micros() - i) < (n));
}


uint32_t millis(void){
	return sysTickCounter/1000;
}

uint32_t micros(void){
	return sysTickCounter;
}

void RCC_Configuration(void){
//#if 0
	ErrorStatus HSEStartUpStatus;
	RCC_DeInit();

	/* Enable HSE */
 	RCC_HSEConfig(RCC_HSE_ON);
 	/* Wait till HSE is ready */
 	HSEStartUpStatus = RCC_WaitForHSEStartUp();
 
 	if(HSEStartUpStatus == SUCCESS){
		 /* HCLK = SYSCLK */
		 RCC_HCLKConfig(RCC_SYSCLK_Div1);

		 /* PCLK2 = HCLK */
		 RCC_PCLK2Config(RCC_HCLK_Div1);

		 /* PCLK1 = HCLK/2 */
		 RCC_PCLK1Config(RCC_HCLK_Div4);

		 /* ADCCLK = PCLK2/4 */
		 RCC_ADCCLKConfig(RCC_PCLK2_Div4);

		 /* Flash 2 wait state */
		 FLASH_SetLatency(FLASH_Latency_2);

		 /* Enable Prefetch Buffer */
		 FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		 /* PLLCLK = 8MHz * 9 = 72 MHz */
		 RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

		 /* Enable PLL */
		 RCC_PLLCmd(ENABLE);

		 /* Wait till PLL is ready */
		 while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

		 /* Select PLL as system clock source */
		 RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		 /* Wait till PLL is used as system clock source */
		 while(RCC_GetSYSCLKSource() != 0x08);
 	}
//#endif
  /* PCLK1 = HCLK/4 */
  //RCC_PCLK1Config(RCC_HCLK_Div2);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
 
 
  /* GPIOA, GPIOB and GPIOC clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
}