

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include <stm32f10x.h>
#include "motors.h"


#define DEBUG_PRINT(args...) printf(args)

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

#define PERIOD 14399
#define PRESCALER 0
#define CLOCKDIV 0

void MOTOR_Init(void){

	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	TIM_TimeBaseStructure.TIM_Period = PERIOD;
	TIM_TimeBaseStructure.TIM_Prescaler = PRESCALER;
	TIM_TimeBaseStructure.TIM_ClockDivision = CLOCKDIV;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_OCStructInit(&TIM_OCInitStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0x0000;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM1, ENABLE);

	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	TIM_Cmd(TIM1, ENABLE);




	// IN2_A
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// IN2_B
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// EN2_A
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// EN2_B
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	TIM_TimeBaseStructure.TIM_Period = PERIOD;
	TIM_TimeBaseStructure.TIM_Prescaler = PRESCALER;
	TIM_TimeBaseStructure.TIM_ClockDivision = CLOCKDIV;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_OCStructInit(&TIM_OCInitStructure); 

	/* PWM1 Mode configuration: Channel4 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0x0000;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);

	TIM_CtrlPWMOutputs(TIM3, ENABLE);

	TIM_Cmd(TIM3, ENABLE);




	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	TIM_TimeBaseStructure.TIM_Period = PERIOD; 
	TIM_TimeBaseStructure.TIM_Prescaler = PRESCALER;
	TIM_TimeBaseStructure.TIM_ClockDivision = CLOCKDIV;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_OCStructInit(&TIM_OCInitStructure); 

	/* PWM1 Mode configuration: Channel4 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0x0000;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);

	TIM_CtrlPWMOutputs(TIM2, ENABLE);

	TIM_Cmd(TIM2, ENABLE);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	TIM_TimeBaseStructure.TIM_Period = PERIOD*2; 
	TIM_TimeBaseStructure.TIM_Prescaler = PRESCALER;
	TIM_TimeBaseStructure.TIM_ClockDivision = CLOCKDIV;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_OCStructInit(&TIM_OCInitStructure);

	/* PWM1 Mode configuration: Channel4 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0x0000;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM1, ENABLE);

	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	TIM_Cmd(TIM1, ENABLE);


	motor_set_speed(0, 0, DIR_FORWARD);
	motor_set_speed(1, 0, DIR_FORWARD);
	motor_set_speed(2, 0, DIR_FORWARD);
	motor_set_speed(3, 0, DIR_FORWARD);
}

uint8_t motor_set_speed(int motor_id, uint16_t speed, int dir){
	if(speed < 0)
		speed = 0;
	if(speed > PERIOD)
		speed = PERIOD;

	if(motor_id == 0){
		if(speed > 0){
			GPIO_SetBits(GPIOC, GPIO_Pin_14);
			GPIO_SetBits(GPIOA, GPIO_Pin_0);

			if(dir == DIR_FORWARD){
				GPIO_SetBits(GPIOC, GPIO_Pin_13);
				GPIO_ResetBits(GPIOC, GPIO_Pin_15);
			}else if(dir == DIR_REVERSE){
				GPIO_ResetBits(GPIOC, GPIO_Pin_13);
				GPIO_SetBits(GPIOC, GPIO_Pin_15);
			}else{
				return ERROR_DIR_INAVAILABLE;
			}
		}else{
			GPIO_SetBits(GPIOC, GPIO_Pin_14);
			GPIO_SetBits(GPIOA, GPIO_Pin_0);

			// Break to GND
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);
			GPIO_ResetBits(GPIOC, GPIO_Pin_15);
		}


		TIM1->CCR1 = (uint32_t)speed*2;
	}else if(motor_id == 1){
		if(speed != 0){
			GPIO_SetBits(GPIOA, GPIO_Pin_5);
			GPIO_SetBits(GPIOB, GPIO_Pin_10);

			if(dir == DIR_FORWARD){
				GPIO_SetBits(GPIOA, GPIO_Pin_4);
				GPIO_ResetBits(GPIOB, GPIO_Pin_2);
			}else if(dir == DIR_REVERSE){
				GPIO_ResetBits(GPIOA, GPIO_Pin_4);
				GPIO_SetBits(GPIOB, GPIO_Pin_2);
			}else{
				return ERROR_DIR_INAVAILABLE;
			}	
		}else{
			GPIO_SetBits(GPIOA, GPIO_Pin_5);
			GPIO_SetBits(GPIOB, GPIO_Pin_10);

			// Break to GND
			GPIO_ResetBits(GPIOA, GPIO_Pin_4);
			GPIO_ResetBits(GPIOB, GPIO_Pin_2);
		}

		TIM3->CCR4 = (uint32_t)speed;
	}else if(motor_id == 2){
		if(speed != 0){
			GPIO_SetBits(GPIOB, GPIO_Pin_12);
			GPIO_SetBits(GPIOB, GPIO_Pin_14);

			if(dir == DIR_FORWARD){
				GPIO_SetBits(GPIOB, GPIO_Pin_11);
				GPIO_ResetBits(GPIOB, GPIO_Pin_13);
			}else if(dir == DIR_REVERSE){
				GPIO_ResetBits(GPIOB, GPIO_Pin_11);
				GPIO_SetBits(GPIOB, GPIO_Pin_13);
			}else{
				return ERROR_DIR_INAVAILABLE;
			}	
		}else{
			GPIO_SetBits(GPIOB, GPIO_Pin_12);
			GPIO_SetBits(GPIOB, GPIO_Pin_14);

			// Break to GND
			GPIO_ResetBits(GPIOB, GPIO_Pin_11);
			GPIO_ResetBits(GPIOB, GPIO_Pin_13);
		}

		TIM2->CCR4 = (uint32_t)speed;
	}else if(motor_id == 3){
		if(speed != 0){
			GPIO_SetBits(GPIOA, GPIO_Pin_12);
			GPIO_SetBits(GPIOB, GPIO_Pin_3);

			if(dir == DIR_FORWARD){
				GPIO_SetBits(GPIOB, GPIO_Pin_15);
				GPIO_ResetBits(GPIOA, GPIO_Pin_15);
			}else if(dir == DIR_REVERSE){
				GPIO_ResetBits(GPIOB, GPIO_Pin_15);
				GPIO_SetBits(GPIOA, GPIO_Pin_15);
			}else{
				return ERROR_DIR_INAVAILABLE;
			}
		}else{
			GPIO_SetBits(GPIOA, GPIO_Pin_12);
			GPIO_SetBits(GPIOB, GPIO_Pin_3);

			// Break to GND
			GPIO_ResetBits(GPIOB, GPIO_Pin_15);
			GPIO_ResetBits(GPIOA, GPIO_Pin_15);
		}	

		TIM1->CCR4 = (uint32_t)speed*2;
	}else{
		return ERROR_MOTOR_NOT_EXISTING;
	}

	return SUCCESS;
}