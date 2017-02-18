
#include "motor.h"
#include "motorSettings.h"

#include "servo.h"
#include "DCMotor.h"

motor *motor::createMotor(motorSettings *settings){
    switch (settings->m_motorType) {
	    case MOTOR_TYPE_SERVO:
	    	/*settings->m_ServoLimitMax = 7000;
  			settings->m_ServoLimitMin = 2000;*/
	        return new servo(settings);
			
	    case MOTOR_TYPE_DC_MOTOR:
	        return new DCMotor(settings);

	    default:
	        return 0;
    }
}


motor::motor(motorSettings *settings){
    m_settings = settings;
}

motor::~motor()
{
}