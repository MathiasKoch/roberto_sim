
#include "servo.h"
#include "motorSettings.h"


servo::servo(motorSettings *settings) : motor(settings)
{

}

servo::~servo()
{
}

bool servo::motorInit()
{

	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = m_settings->m_ServoPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(m_settings->m_ServoPort, &GPIO_InitStructure);




	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 59999; // ie 0..29999
	TIM_TimeBaseStructure.TIM_Prescaler = 23;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(m_settings->m_Timer, &TIM_TimeBaseStructure);


	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure); // clears the other 4 fields not used here

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0x0000;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	switch(m_settings->m_TimerChannel){
		case 1:
			TIM_OC1Init(m_settings->m_Timer, &TIM_OCInitStructure);
			TIM_OC1PreloadConfig(m_settings->m_Timer, TIM_OCPreload_Enable);
			break;
		case 2:
			TIM_OC2Init(m_settings->m_Timer, &TIM_OCInitStructure);
			TIM_OC2PreloadConfig(m_settings->m_Timer, TIM_OCPreload_Enable);
			break;
		case 3:
			TIM_OC3Init(m_settings->m_Timer, &TIM_OCInitStructure);
			TIM_OC3PreloadConfig(m_settings->m_Timer, TIM_OCPreload_Enable);
			break;
		case 4:
			TIM_OC4Init(m_settings->m_Timer, &TIM_OCInitStructure);
			TIM_OC4PreloadConfig(m_settings->m_Timer, TIM_OCPreload_Enable);
			break;
		default:
			return false;
	}


	TIM_ARRPreloadConfig(m_settings->m_Timer, ENABLE);

	/* TIM Main Output Enable */
	TIM_CtrlPWMOutputs(m_settings->m_Timer, ENABLE);

	/* TIM enable counter */
	TIM_Cmd(m_settings->m_Timer, ENABLE);

	return true;
}

void servo::setReference(float setPoint){
	pos = (int)setPoint;
	if(pos > 180){
		pos = 180;
	}else if(pos < 0){
		pos = 0;
	}
}

float servo::getReference(){
	return (float)pos;
}
char* servo::motorName(){
	return m_settings->m_motorName;
}

float servo::update(float dt, bool connected){
	float LPF_Beta = 0.06;
	int sp = pos * (m_settings->m_ServoLimitMax - m_settings->m_ServoLimitMin) / 180 + m_settings->m_ServoLimitMin;
	int curr, newVal;
	switch(m_settings->m_TimerChannel){
		case 1:
			curr = (m_settings->m_Timer)->CCR1;
			newVal = (int)(curr - (LPF_Beta * (curr - sp)));
			(m_settings->m_Timer)->CCR1 = newVal;
			break;
		case 2:
			curr = (m_settings->m_Timer)->CCR2;
			newVal = (int)(curr - (LPF_Beta * (curr - sp)));
			(m_settings->m_Timer)->CCR2 = newVal;
			break;
		case 3:
			curr = (m_settings->m_Timer)->CCR3;
			newVal = (int)(curr - (LPF_Beta * (curr - sp)));
			(m_settings->m_Timer)->CCR3 = newVal;
			break;
		case 4:
			curr = (m_settings->m_Timer)->CCR4;
			newVal = (int)(curr - (LPF_Beta * (curr - sp)));
			(m_settings->m_Timer)->CCR4 = newVal;
			break;
	}
	return newVal;
}