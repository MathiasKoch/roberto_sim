
#ifndef _DCMotor_H
#define	_DCMotor_H

#include <stm32f10x.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "motor.h"
#include "encoder.h"
#include "stm32_time.h"


class DCMotor : public motor
{
public:
    DCMotor(motorSettings *settings);
    ~DCMotor();

    virtual void setReference(float setPoint);
    virtual float getReference();
    virtual const char* motorName();
    //virtual std::tuple<float, int, int, int> update(float dt, bool connected);
    virtual float update(float dt, bool connected, bool enable);

    virtual int motorType() { return MOTOR_TYPE_DC_MOTOR; }
    virtual bool motorInit();

private:
	bool setSpeed(int s, bool enable);
    void initEncoder(uint16_t addr);
    //std::tuple<int32_t, uint8_t, uint16_t> readEncoder();
    int32_t readEncoder();
    float updateRegulator(float enc, float sp, float dt);
	float speed;
    uint8_t encAddr;
    float wheelRadius;

    // PID
    float FF;
    float KP;
    float KI;
    float KD;
    float integralSaturation;
    float error;
    float integral;

};

#endif // _DCMotor_H
