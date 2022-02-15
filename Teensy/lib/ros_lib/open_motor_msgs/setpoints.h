#ifndef _ROS_open_motor_msgs_setpoints_h
#define _ROS_open_motor_msgs_setpoints_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace open_motor_msgs
{

  class setpoints : public ros::Msg
  {
    public:
      typedef int32_t _command_type;
      _command_type command;
      int32_t position_setpoint[4];
      float velocity_setpoint[4];

    setpoints():
      command(0),
      position_setpoint(),
      velocity_setpoint()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_command;
      u_command.real = this->command;
      *(outbuffer + offset + 0) = (u_command.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_command.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_command.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_command.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->command);
      for( uint32_t i = 0; i < 4; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_position_setpointi;
      u_position_setpointi.real = this->position_setpoint[i];
      *(outbuffer + offset + 0) = (u_position_setpointi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_setpointi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_setpointi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_setpointi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_setpoint[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_velocity_setpointi;
      u_velocity_setpointi.real = this->velocity_setpoint[i];
      *(outbuffer + offset + 0) = (u_velocity_setpointi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_setpointi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_setpointi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_setpointi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_setpoint[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_command;
      u_command.base = 0;
      u_command.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_command.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_command.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_command.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->command = u_command.real;
      offset += sizeof(this->command);
      for( uint32_t i = 0; i < 4; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_position_setpointi;
      u_position_setpointi.base = 0;
      u_position_setpointi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_setpointi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position_setpointi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position_setpointi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position_setpoint[i] = u_position_setpointi.real;
      offset += sizeof(this->position_setpoint[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_velocity_setpointi;
      u_velocity_setpointi.base = 0;
      u_velocity_setpointi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_setpointi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_setpointi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_setpointi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity_setpoint[i] = u_velocity_setpointi.real;
      offset += sizeof(this->velocity_setpoint[i]);
      }
     return offset;
    }

    const char * getType(){ return "open_motor_msgs/setpoints"; };
    const char * getMD5(){ return "dc7f8d5bac1d165478b1e20e9fa53d87"; };

  };

}
#endif
