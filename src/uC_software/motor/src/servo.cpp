
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

	// Setup Controller
	float b_[] = {0.0029,    0.0087,    0.0087,    0.0029};
	float a_[] = {1.0000,   -2.3741,    1.9294,   -0.5321};
	std::copy(b_, b_ + 5, b);
	std::copy(a_, a_ + 5, a);

	return true;
}

void servo::setReference(float setPoint){
	pos = (int) setPoint;
	//pos = (int)setPoint > 90? 90 : ( (int)setPoint < -MAX_ANGLE_PIVOT? -MAX_ANGLE_PIVOT : (int)setPoint);
}

float servo::getReference(){
	return (float)pos;
}
const char* servo::motorName(){
	return m_settings->m_motorName;
}

std::tuple<float, float, int, int> servo::update(float dt, bool connected){
	int sp = pos * (m_settings->m_ServoLimitMax - m_settings->m_ServoLimitMin) / 180 + m_settings->m_ServoLimitMin;	
	sp = (int) filterUpdate(sp);

	switch(m_settings->m_TimerChannel){
		case 1:
			(m_settings->m_Timer)->CCR1 = sp;
			break;
		case 2:
			(m_settings->m_Timer)->CCR2 = sp;
			break;
		case 3:
			(m_settings->m_Timer)->CCR3 = sp;
			break;
		case 4:
			(m_settings->m_Timer)->CCR4 = sp;
			break;
	}
	return std::make_tuple(0, 0.0, sp, 0);
}

float servo::filterUpdate(float sp){
	int i;
	for(i = 3; i>0; i++){
		u[i-1] = u[i]; 
	}
	u[0] = sp; 
	
	float Y = b[0]*u[0];
	for(i = 0; i<4; i++){
		Y += b[1]*u[1];
		Y -= a[1]*y[1];
	}
	Y /= a[0];

	for(int j = 3; j>0; j++){
		u[i-1] = y[i]; 
	}
	y[0] = Y;
	return y[0];
}