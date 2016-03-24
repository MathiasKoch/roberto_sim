#ifndef _ROS_roberto_msgs_MotorState_h
#define _ROS_roberto_msgs_MotorState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace roberto_msgs
{

  class MotorState : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float heading_angle;
      float heading_velocity;
      float speed;
      float acceleration;
      uint8_t mode;
      enum { DRIVE_MODE_AUTO = 0 };
      enum { DRIVE_MODE_PIVOT = 1 };
      enum { DRIVE_MODE_SPIN = 2 };
      enum { DRIVE_MODE_SIDEWAYS = 3 };

    MotorState():
      header(),
      heading_angle(0),
      heading_velocity(0),
      speed(0),
      acceleration(0),
      mode(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->heading_angle);
      offset += serializeAvrFloat64(outbuffer + offset, this->heading_velocity);
      offset += serializeAvrFloat64(outbuffer + offset, this->speed);
      offset += serializeAvrFloat64(outbuffer + offset, this->acceleration);
      *(outbuffer + offset + 0) = (this->mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mode);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->heading_angle));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->heading_velocity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->speed));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->acceleration));
      this->mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mode);
     return offset;
    }

    const char * getType(){ return "roberto_msgs/MotorState"; };
    const char * getMD5(){ return "e8b5d1162b85151c49e02b8224796c0e"; };

  };

}
#endif