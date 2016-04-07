
#ifndef _servo_H
#define	_servo_H

#include <stm32f10x.h>
#include "motor.h"

class servo : public motor
{
public:
    servo(motorSettings *settings);
    ~servo();

    virtual void setReference(float setPoint);
    virtual float getReference();
    virtual const char* motorName();
    virtual std::tuple<float, float, int, int> update(float dt, bool connected);

    virtual int motorType() { return MOTOR_TYPE_SERVO; }
    virtual bool motorInit();

private:
	int pos;

};

#endif // _servo_H
