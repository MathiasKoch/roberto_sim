#ifndef _ROS_rosserial_arduino_Servo_h
#define _ROS_rosserial_arduino_Servo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rosserial_arduino
{

  class Servo : public ros::Msg
  {
    public:
      uint8_t servo0;
      uint8_t servo1;
      uint8_t servo2;
      uint8_t servo3;
      uint8_t servo4;

    Servo():
      servo0(0),
      servo1(0),
      servo2(0),
      servo3(0),
      servo4(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->servo0 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->servo0 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->servo0);
      *(outbuffer + offset + 0) = (this->servo1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->servo1 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->servo1);
      *(outbuffer + offset + 0) = (this->servo2 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->servo2 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->servo2);
      *(outbuffer + offset + 0) = (this->servo3 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->servo3 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->servo3);
      *(outbuffer + offset + 0) = (this->servo4 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->servo4 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->servo4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->servo0 =  ((uint8_t) (*(inbuffer + offset)));
      this->servo0 |= ((uint8_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->servo0);
      this->servo1 =  ((uint8_t) (*(inbuffer + offset)));
      this->servo1 |= ((uint8_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->servo1);
      this->servo2 =  ((uint8_t) (*(inbuffer + offset)));
      this->servo2 |= ((uint8_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->servo2);
      this->servo3 =  ((uint8_t) (*(inbuffer + offset)));
      this->servo3 |= ((uint8_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->servo3);
      this->servo4 =  ((uint8_t) (*(inbuffer + offset)));
      this->servo4 |= ((uint8_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->servo4);
     return offset;
    }

    const char * getType(){ return "rosserial_arduino/Servo"; };
    const char * getMD5(){ return "6d7853a614e2e821319068311f2af25b"; };

  };

}
#endif