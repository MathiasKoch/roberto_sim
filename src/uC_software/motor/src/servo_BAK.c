
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include <stm32f10x.h>
#include "servo.h"


#define DEBUG_PRINT(args...) printf(args)


TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

#define MAX 6800
#define MIN 2000


void SERVO_Init(){

	GPIO_InitTypeDef GPIO_InitStructure;

	/*Configure GPIO pin : PB8 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*Configure GPIO pin : PB9 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	TIM_TimeBaseStructure.TIM_Period = 59999; // ie 0..29999
	TIM_TimeBaseStructure.TIM_Prescaler = 23;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_OCStructInit(&TIM_OCInitStructure); // clears the other 4 fields not used here

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0x0000;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);

	/* TIM Main Output Enable */
	TIM_CtrlPWMOutputs(TIM4, ENABLE);

	/* TIM enable counter */
	TIM_Cmd(TIM4, ENABLE);
}


void servo_set(uint8_t id, uint8_t val){
	if(val > 180){
		val = 180;
	}else if(val < 0){
		val = 0;
	}
	if(id == 0)
		TIM4->CCR3 = val * (MAX - MIN) / 180 + MIN;
	else if(id == 1)
		TIM4->CCR4 = val * (MAX - MIN) / 180 + MIN;
}