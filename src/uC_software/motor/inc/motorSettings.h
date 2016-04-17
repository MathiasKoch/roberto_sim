
#ifndef _MOTORSETTINGS_H
#define _MOTORSETTINGS_H

#include <stm32f10x.h>


class motorSettings
{
public:
    motorSettings();
    motorSettings(int type, const char* name, TIM_TypeDef* timer, int timerChannel);
    //  These are the local variables

    void setDCPins(int m_DCInAPin_, GPIO_TypeDef* m_DCInAPort_, int m_DCEnAPin_, GPIO_TypeDef* m_DCEnAPort_, int m_DCInBPin_, GPIO_TypeDef* m_DCInBPort_, int m_DCEnBPin_, GPIO_TypeDef* m_DCEnBPort_, int m_DCPWMPin_, GPIO_TypeDef* m_DCPWMPort_);
    void setRegulator(float FF_, float KP_, float KI_, float KD_, float integralSaturation_);

    int m_motorType;                                          // type code of motor in use
    const char *m_motorName;

    TIM_TypeDef* m_Timer;
    int m_TimerChannel;

    //  motor-specific vars

    //  Servo motor

    int m_ServoLimitMax;
    int m_ServoLimitMin;
    int m_ServoPin;
    GPIO_TypeDef* m_ServoPort;

    //  DC motor

    int m_DCInAPin;
    GPIO_TypeDef* m_DCInAPort;
    int m_DCEnAPin;
    GPIO_TypeDef* m_DCEnAPort;
    int m_DCInBPin;
    GPIO_TypeDef* m_DCInBPort;
    int m_DCEnBPin;
    GPIO_TypeDef* m_DCEnBPort;
    int m_DCPWMPin;
    GPIO_TypeDef* m_DCPWMPort;
    uint16_t encoderAddr;
    int encoder_timeout;
    
    float FF;
    float KP;
    float KI;
    float KD;
    float integralSaturation;

    float wheelRadius;
};

#endif // _MOTORSETTINGS_H

