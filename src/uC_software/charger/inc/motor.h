
#ifndef _MOTOR_H
#define	_MOTOR_H


#include <stdio.h>
#include <tuple>

#define MOTOR_TYPE_NOT_SET             0
#define MOTOR_TYPE_SERVO               1
#define MOTOR_TYPE_DC_MOTOR            2


// SERVO INFO

#define M_PI           3.14159265358979323846  /* pi */

#define MAX_ANGLE_PIVOT 13

/*#define SERVO_PWM_LIMIT_MAX                 // MAX as in hardware limit of full range
#define SERVO_PWM_LIMIT_MIN                 // MIN as in hardware limit of full range

#define SERVO_FULL_RANGE_DEG                // Degrees of full range (eg 180)


#define DEG2PWM (SERVO_PWM_LIMIT_MAX-SERVO_PWM_LIMIT_MIN)/(SERVO_FULL_RANGE_DEG)
#define PWM2DEG SERVO_FULL_RANGE_DEG/(SERVO_PWM_LIMIT_MAX-SERVO_PWM_LIMIT_MIN)*/


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
    virtual const char* motorName() = 0;                     // the name of the motor
    virtual void setReference(float setPoint) = 0;
    virtual float getReference() = 0;
    //virtual std::tuple<float, int, int, int> update(float dt, bool connected) = 0;
    virtual float update(float dt, bool connected, bool enable) = 0;

protected:
    motorSettings *m_settings;                              // the settings object pointer
};

#endif // _RTmotor_H
