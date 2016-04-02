
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

	integral = 0;
	error = 0;

	wheelRadius = m_settings->wheelRadius;

	return true;
}

void DCMotor::setReference(float setPoint){
	speed = setPoint;

}

bool DCMotor::setSpeed(int s){
	if(s < -PERIOD)
		s = -PERIOD;

	if(s > PERIOD)
		s = PERIOD;

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
		s = 0;

		GPIO_SetBits(m_settings->m_DCEnAPort, m_settings->m_DCEnAPin);
		GPIO_SetBits(m_settings->m_DCEnBPort, m_settings->m_DCEnBPin);

		// Break to GND
		GPIO_ResetBits(m_settings->m_DCInAPort, m_settings->m_DCInAPin);
		GPIO_ResetBits(m_settings->m_DCInBPort, m_settings->m_DCInBPin);
	}
	

	if(m_settings->m_Timer == TIM1)
		s = s * 2;

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

int32_t DCMotor::readEncoder(){
	NumberOfByteToReceive = RXBUFFERSIZE;
    Rx_Idx = 0x00;

    slaveAddress = encAddr;
    
    I2C_ITConfig(I2C1, I2C_IT_EVT , ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    I2C_GenerateSTART(I2C1, ENABLE);

    // TODO: Implement actual error handling on timeout and retry count.. At the very least throw an error message!
    uint32_t start = millis();
    while ((Rx_Idx < RXBUFFERSIZE)){
    	if((millis() - start) > (uint32_t)(m_settings->encoder_timeout)){
    		return 0;
    	}
    } 

    if(RxBuffer[4] > 5){
    	return 0;
    }

    return ((int32_t)((RxBuffer[0] << 24)|(RxBuffer[1] << 16)|(RxBuffer[2] << 8) | RxBuffer[3]));
}

float DCMotor::updateRegulator(float enc, float dt){
	float error_new = speed-enc;
	
	integral += error_new*dt;
	
	if (integral > m_settings->integralSaturation){
		integral = m_settings->integralSaturation;
	}else if (integral < -(m_settings->integralSaturation)){
		integral = -(m_settings->integralSaturation);
	}
	
	float derivative = (error_new-error)/dt;
	float output = (m_settings->KP*error + m_settings->KI*integral + m_settings->KD*derivative);
	error = error_new;
	return output;
}

float DCMotor::update(float dt, bool connected){
	// Read encoder
	int s = 0;
	float speed_si = -1;
	if(connected){
		float encSpeed = readEncoder()*0.04793689962;		// rad/s
		speed_si = encSpeed * wheelRadius;	// m/s
		if(strstr(m_settings->m_motorName, "left") > 0)
			speed_si = speed_si * -1;
		// Update PID regulator
		s = (int) updateRegulator(speed_si, dt);		// m/s*/
		// Set motor speed to process value
	}
	setSpeed(s);		// m/s
	// Return encoder values for publishing to localization
	return speed_si;
}