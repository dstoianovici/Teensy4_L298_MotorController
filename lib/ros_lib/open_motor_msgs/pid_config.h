#ifndef _ROS_open_motor_msgs_pid_config_h
#define _ROS_open_motor_msgs_pid_config_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace open_motor_msgs
{

  class pid_config : public ros::Msg
  {
    public:
      typedef bool _update_type;
      _update_type update;
      typedef float _kP_pos_type;
      _kP_pos_type kP_pos;
      typedef float _kI_pos_type;
      _kI_pos_type kI_pos;
      typedef float _kD_pos_type;
      _kD_pos_type kD_pos;
      typedef float _pid_update_position_type;
      _pid_update_position_type pid_update_position;
      typedef float _kP_vel_type;
      _kP_vel_type kP_vel;
      typedef float _kI_vel_type;
      _kI_vel_type kI_vel;
      typedef float _kD_vel_type;
      _kD_vel_type kD_vel;

    pid_config():
      update(0),
      kP_pos(0),
      kI_pos(0),
      kD_pos(0),
      pid_update_position(0),
      kP_vel(0),
      kI_vel(0),
      kD_vel(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_update;
      u_update.real = this->update;
      *(outbuffer + offset + 0) = (u_update.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->update);
      union {
        float real;
        uint32_t base;
      } u_kP_pos;
      u_kP_pos.real = this->kP_pos;
      *(outbuffer + offset + 0) = (u_kP_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kP_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kP_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kP_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kP_pos);
      union {
        float real;
        uint32_t base;
      } u_kI_pos;
      u_kI_pos.real = this->kI_pos;
      *(outbuffer + offset + 0) = (u_kI_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kI_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kI_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kI_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kI_pos);
      union {
        float real;
        uint32_t base;
      } u_kD_pos;
      u_kD_pos.real = this->kD_pos;
      *(outbuffer + offset + 0) = (u_kD_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kD_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kD_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kD_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kD_pos);
      union {
        float real;
        uint32_t base;
      } u_pid_update_position;
      u_pid_update_position.real = this->pid_update_position;
      *(outbuffer + offset + 0) = (u_pid_update_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pid_update_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pid_update_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pid_update_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pid_update_position);
      union {
        float real;
        uint32_t base;
      } u_kP_vel;
      u_kP_vel.real = this->kP_vel;
      *(outbuffer + offset + 0) = (u_kP_vel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kP_vel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kP_vel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kP_vel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kP_vel);
      union {
        float real;
        uint32_t base;
      } u_kI_vel;
      u_kI_vel.real = this->kI_vel;
      *(outbuffer + offset + 0) = (u_kI_vel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kI_vel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kI_vel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kI_vel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kI_vel);
      union {
        float real;
        uint32_t base;
      } u_kD_vel;
      u_kD_vel.real = this->kD_vel;
      *(outbuffer + offset + 0) = (u_kD_vel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kD_vel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kD_vel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kD_vel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kD_vel);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_update;
      u_update.base = 0;
      u_update.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->update = u_update.real;
      offset += sizeof(this->update);
      union {
        float real;
        uint32_t base;
      } u_kP_pos;
      u_kP_pos.base = 0;
      u_kP_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kP_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kP_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kP_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kP_pos = u_kP_pos.real;
      offset += sizeof(this->kP_pos);
      union {
        float real;
        uint32_t base;
      } u_kI_pos;
      u_kI_pos.base = 0;
      u_kI_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kI_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kI_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kI_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kI_pos = u_kI_pos.real;
      offset += sizeof(this->kI_pos);
      union {
        float real;
        uint32_t base;
      } u_kD_pos;
      u_kD_pos.base = 0;
      u_kD_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kD_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kD_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kD_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kD_pos = u_kD_pos.real;
      offset += sizeof(this->kD_pos);
      union {
        float real;
        uint32_t base;
      } u_pid_update_position;
      u_pid_update_position.base = 0;
      u_pid_update_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pid_update_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pid_update_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pid_update_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pid_update_position = u_pid_update_position.real;
      offset += sizeof(this->pid_update_position);
      union {
        float real;
        uint32_t base;
      } u_kP_vel;
      u_kP_vel.base = 0;
      u_kP_vel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kP_vel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kP_vel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kP_vel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kP_vel = u_kP_vel.real;
      offset += sizeof(this->kP_vel);
      union {
        float real;
        uint32_t base;
      } u_kI_vel;
      u_kI_vel.base = 0;
      u_kI_vel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kI_vel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kI_vel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kI_vel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kI_vel = u_kI_vel.real;
      offset += sizeof(this->kI_vel);
      union {
        float real;
        uint32_t base;
      } u_kD_vel;
      u_kD_vel.base = 0;
      u_kD_vel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kD_vel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kD_vel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kD_vel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kD_vel = u_kD_vel.real;
      offset += sizeof(this->kD_vel);
     return offset;
    }

    const char * getType(){ return "open_motor_msgs/pid_config"; };
    const char * getMD5(){ return "e86fc9db45258146486378e7e8c8682c"; };

  };

}
#endif
