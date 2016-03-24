
#ifndef _MOTOR_H
#define	_MOTOR_H


#include <stdio.h>

#define MOTOR_TYPE_NOT_SET             0
#define MOTOR_TYPE_SERVO               1
#define MOTOR_TYPE_DC_MOTOR            2

class motorSettings;

class motor
{
public:
    //  motors should always be created with the following call

    static motor *createMotor(motorSettings *settings);

    //  Constructor/destructor

    motor(motorSettings *settings);
    virtual ~motor();

    //  These functions must be provided by sub classes

    virtual int motorType() = 0;                              // the type code of the motor
    virtual bool motorInit() = 0;                              // set up the motor
    virtual char* motorName() = 0;                     // the name of the motor
    virtual void setReference(float setPoint) = 0;
    virtual float getReference() = 0;
    virtual float update(float dt) = 0;


protected:
    motorSettings *m_settings;                              // the settings object pointer
};

#endif // _RTmotor_H
