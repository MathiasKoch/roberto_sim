
#ifndef _DCMotor_H
#define	_DCMotor_H

#include <stm32f10x.h>
#include <stdlib.h>
#include "motor.h"
#include "encoder.h"


class DCMotor : public motor
{
public:
    DCMotor(motorSettings *settings);
    ~DCMotor();

    virtual void setReference(float setPoint);
    virtual float getReference();
    virtual char* motorName();
    virtual uint16_t update(float dt);

    virtual int motorType() { return MOTOR_TYPE_DC_MOTOR; }
    virtual bool motorInit();

private:
	bool setSpeed(int s);
    void initEncoder(uint16_t addr);
    uint16_t readEncoder();
    float updateRegulator(float enc, float dt);
	float speed;
    uint8_t encAddr;
    float wheelRadius;

    // PID
    float KP;
    float KI;
    float KD;
    float max;
    float min;
    float error;
    float integral;

};

#endif // _DCMotor_H
