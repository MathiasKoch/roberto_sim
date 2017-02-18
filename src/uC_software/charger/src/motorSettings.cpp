#include "motorSettings.h"
#include "servo.h"
//#include "dcMotor.h"


motorSettings::motorSettings()
{
    //  preset general defaults

    m_motorType = 0;

    //  motor defaults

    /*m_MPU9150GyroAccelSampleRate = 50;
    m_MPU9150CompassSampleRate = 25;
    m_MPU9150GyroAccelLpf = MPU9150_LPF_20;
    m_MPU9150GyroFsr = MPU9150_GYROFSR_1000;
    m_MPU9150AccelFsr = MPU9150_ACCELFSR_8;*/
}

motorSettings::motorSettings(int type, const char* name, TIM_TypeDef* timer, int timerChannel)
{
    //  preset general defaults

    m_motorType = type;
    m_motorName = name;
    m_Timer = timer;
    m_TimerChannel = timerChannel;

    //  motor defaults

    /*m_MPU9150GyroAccelSampleRate = 50;
    m_MPU9150CompassSampleRate = 25;
    m_MPU9150GyroAccelLpf = MPU9150_LPF_20;
    m_MPU9150GyroFsr = MPU9150_GYROFSR_1000;
    m_MPU9150AccelFsr = MPU9150_ACCELFSR_8;*/
}

void motorSettings::setDCPins(int m_DCInAPin_, GPIO_TypeDef* m_DCInAPort_, int m_DCEnAPin_, GPIO_TypeDef* m_DCEnAPort_,
                                 int m_DCInBPin_, GPIO_TypeDef* m_DCInBPort_, int m_DCEnBPin_, 
                                 GPIO_TypeDef* m_DCEnBPort_, int m_DCPWMPin_, GPIO_TypeDef* m_DCPWMPort_){

    m_DCInAPin = m_DCInAPin_;
    m_DCInAPort = m_DCInAPort_;
    m_DCEnAPin = m_DCEnAPin_;
    m_DCEnAPort = m_DCEnAPort_;
    m_DCInBPin = m_DCInBPin_;
    m_DCInBPort = m_DCInBPort_;
    m_DCEnBPin = m_DCEnBPin_;
    m_DCEnBPort = m_DCEnBPort_;
    m_DCPWMPin = m_DCPWMPin_;
    m_DCPWMPort = m_DCPWMPort_;
}

void motorSettings::setRegulator(float FF_, float KP_, float KI_, float KD_, float integralSaturation_){
    FF = FF_;
    KP = KP_;
    KI = KI_;
    KD = KD_;
    integralSaturation = integralSaturation_;
}
