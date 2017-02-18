#ifndef _ROS_roberto_msgs_JointCommand_h
#define _ROS_roberto_msgs_JointCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace roberto_msgs
{

  class JointCommand : public ros::Msg
  {
    public:
      uint8_t command_length;
      float st_command;
      float * command;
      uint8_t names_length;
      char* st_names;
      char* * names;

    JointCommand():
      command_length(0), command(NULL),
      names_length(0), names(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = command_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < command_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->command[i]);
      }
      *(outbuffer + offset++) = names_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < names_length; i++){
      uint32_t length_namesi = strlen(this->names[i]);
      memcpy(outbuffer + offset, &length_namesi, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->names[i], length_namesi);
      offset += length_namesi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t command_lengthT = *(inbuffer + offset++);
      if(command_lengthT > command_length)
        this->command = (float*)realloc(this->command, command_lengthT * sizeof(float));
      offset += 3;
      command_length = command_lengthT;
      for( uint8_t i = 0; i < command_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_command));
        memcpy( &(this->command[i]), &(this->st_command), sizeof(float));
      }
      uint8_t names_lengthT = *(inbuffer + offset++);
      if(names_lengthT > names_length)
        this->names = (char**)realloc(this->names, names_lengthT * sizeof(char*));
      offset += 3;
      names_length = names_lengthT;
      for( uint8_t i = 0; i < names_length; i++){
      uint32_t length_st_names;
      memcpy(&length_st_names, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_names-1]=0;
      this->st_names = (char *)(inbuffer + offset-1);
      offset += length_st_names;
        memcpy( &(this->names[i]), &(this->st_names), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return "roberto_msgs/JointCommand"; };
    const char * getMD5(){ return "e611b9a44b456fac312ea981140e46a4"; };

  };

}
#endif