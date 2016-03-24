
#include "DCMotor.h"
#include "motorSettings.h"

#define PERIOD 14399
#define PRESCALER 0
#define CLOCKDIV 0


DCMotor::DCMotor(motorSettings *settings) : motor(settings)
{

}

DCMotor::~DCMotor()
{
}

bool DCMotor::motorInit()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = m_settings->m_DCInAPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(m_settings->m_DCInAPort, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = m_settings->m_DCInBPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(m_settings->m_DCInBPort, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = m_settings->m_DCEnAPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(m_settings->m_DCEnAPort, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = m_settings->m_DCEnBPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(m_settings->m_DCEnBPort, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = m_settings->m_DCPWMPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(m_settings->m_DCPWMPort, &GPIO_InitStructure);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	if(m_settings->m_Timer == TIM1)
		TIM_TimeBaseStructure.TIM_Period = PERIOD*2;
	else
		TIM_TimeBaseStructure.TIM_Period = PERIOD;
	TIM_TimeBaseStructure.TIM_Prescaler = PRESCALER;
	TIM_TimeBaseStructure.TIM_ClockDivision = CLOCKDIV;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(m_settings->m_Timer, &TIM_TimeBaseStructure);

	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0x00;
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

	TIM_CtrlPWMOutputs(m_settings->m_Timer, ENABLE);

	TIM_Cmd(m_settings->m_Timer, ENABLE);

	// Initialize encoder
	initEncoder(m_settings->encoderAddr);

	// Initialize PID regulator
	KP = m_settings->Kp;
	KI = m_settings->Ki;
	KD = m_settings->Kd;
	max = m_settings->MAX;
	min = m_settings->MIN;
	integral = 0;
	error = 0;

	return true;
}

void DCMotor::setReference(float setPoint){
	speed = setPoint;
	if(speed < 0)
		speed = 0;

	if(speed > PERIOD)
		speed = PERIOD;
}

bool DCMotor::setSpeed(int s){

	if(abs(s) > 0){
		GPIO_SetBits(m_settings->m_DCEnAPort, m_settings->m_DCEnAPin);
		GPIO_SetBits(m_settings->m_DCEnBPort, m_settings->m_DCEnBPin);

		if(s > 0){
			GPIO_SetBits(m_settings->m_DCInAPort, m_settings->m_DCInAPin);
			GPIO_ResetBits(m_settings->m_DCInBPort, m_settings->m_DCInBPin);
		}else if(s < 0){
			GPIO_ResetBits(m_settings->m_DCInAPort, m_settings->m_DCInAPin);
			GPIO_SetBits(m_settings->m_DCInBPort, m_settings->m_DCInBPin);
		}else{
			return false;
		}
	}else{
		GPIO_SetBits(m_settings->m_DCEnAPort, m_settings->m_DCEnAPin);
		GPIO_SetBits(m_settings->m_DCEnBPort, m_settings->m_DCEnBPin);

		// Break to GND
		GPIO_ResetBits(m_settings->m_DCInAPort, m_settings->m_DCInAPin);
		GPIO_ResetBits(m_settings->m_DCInBPort, m_settings->m_DCInBPin);
	}

	if(m_settings->m_Timer == TIM1)
		s = s*2;

	switch(m_settings->m_TimerChannel){
		case 1:
			(m_settings->m_Timer)->CCR1 = (uint32_t)abs(s);
			break;
		case 2:
			(m_settings->m_Timer)->CCR2 = (uint32_t)abs(s);
			break;
		case 3:
			(m_settings->m_Timer)->CCR3 = (uint32_t)abs(s);
			break;
		case 4:
			(m_settings->m_Timer)->CCR4 = (uint32_t)abs(s);
			break;
		default:
			return false;
	}

	return true;
}

float DCMotor::getReference(){
	return speed;
}

char* DCMotor::motorName(){
	return m_settings->m_motorName;
}

void DCMotor::initEncoder(uint16_t addr){
	encAddr = addr;
}

uint16_t DCMotor::readEncoder(){
	NumberOfByteToReceive = RXBUFFERSIZE;
    Rx_Idx = 0x00;

    slaveAddress = encAddr;
    
    I2C_ITConfig(I2C1, I2C_IT_EVT , ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    I2C_GenerateSTART(I2C1, ENABLE);


    // TODO: Add timeout here
    while ((Rx_Idx < RXBUFFERSIZE)); 

    return ((int16_t)((RxBuffer[0] << 8) | RxBuffer[1]));
}

float DCMotor::updateRegulator(float enc, float dt){
	float error_new = speed-enc;
	integral += error_new*dt;
	float derivative = (error_new-error)/dt;
	float output = (KP*error + KI*integral + KD*derivative);
	if(output > max){
		output = max;
	}else if(output < min){
		output = min;
	}
	error = error_new;
	return output;
}

uint16_t DCMotor::update(float dt){
	// Read encoder
	uint16_t encSpeed = readEncoder();		// rad/s
	//float speed_si = encSpeed * wheelRadius;	// m/s
	// Update PID regulator
	//int s = updateRegulator(speed_si, dt);		// m/s
	// Set motor speed to process value
	setSpeed(0);		// m/s
	// Return encoder values for publishing to localization
	return encSpeed;
}