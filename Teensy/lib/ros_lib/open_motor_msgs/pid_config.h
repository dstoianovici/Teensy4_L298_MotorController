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
      float kP_pos[4];
      float kI_pos[4];
      float kD_pos[4];
      typedef float _pid_update_position_type;
      _pid_update_position_type pid_update_position;
      float kP_vel[4];
      float kI_vel[4];
      float kD_vel[4];
      typedef float _pid_update_velocity_type;
      _pid_update_velocity_type pid_update_velocity;

    pid_config():
      update(0),
      kP_pos(),
      kI_pos(),
      kD_pos(),
      pid_update_position(0),
      kP_vel(),
      kI_vel(),
      kD_vel(),
      pid_update_velocity(0)
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
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_kP_posi;
      u_kP_posi.real = this->kP_pos[i];
      *(outbuffer + offset + 0) = (u_kP_posi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kP_posi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kP_posi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kP_posi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kP_pos[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_kI_posi;
      u_kI_posi.real = this->kI_pos[i];
      *(outbuffer + offset + 0) = (u_kI_posi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kI_posi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kI_posi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kI_posi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kI_pos[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_kD_posi;
      u_kD_posi.real = this->kD_pos[i];
      *(outbuffer + offset + 0) = (u_kD_posi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kD_posi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kD_posi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kD_posi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kD_pos[i]);
      }
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
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_kP_veli;
      u_kP_veli.real = this->kP_vel[i];
      *(outbuffer + offset + 0) = (u_kP_veli.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kP_veli.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kP_veli.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kP_veli.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kP_vel[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_kI_veli;
      u_kI_veli.real = this->kI_vel[i];
      *(outbuffer + offset + 0) = (u_kI_veli.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kI_veli.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kI_veli.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kI_veli.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kI_vel[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_kD_veli;
      u_kD_veli.real = this->kD_vel[i];
      *(outbuffer + offset + 0) = (u_kD_veli.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kD_veli.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kD_veli.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kD_veli.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kD_vel[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_pid_update_velocity;
      u_pid_update_velocity.real = this->pid_update_velocity;
      *(outbuffer + offset + 0) = (u_pid_update_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pid_update_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pid_update_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pid_update_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pid_update_velocity);
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
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_kP_posi;
      u_kP_posi.base = 0;
      u_kP_posi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kP_posi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kP_posi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kP_posi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kP_pos[i] = u_kP_posi.real;
      offset += sizeof(this->kP_pos[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_kI_posi;
      u_kI_posi.base = 0;
      u_kI_posi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kI_posi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kI_posi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kI_posi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kI_pos[i] = u_kI_posi.real;
      offset += sizeof(this->kI_pos[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_kD_posi;
      u_kD_posi.base = 0;
      u_kD_posi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kD_posi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kD_posi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kD_posi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kD_pos[i] = u_kD_posi.real;
      offset += sizeof(this->kD_pos[i]);
      }
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
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_kP_veli;
      u_kP_veli.base = 0;
      u_kP_veli.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kP_veli.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kP_veli.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kP_veli.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kP_vel[i] = u_kP_veli.real;
      offset += sizeof(this->kP_vel[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_kI_veli;
      u_kI_veli.base = 0;
      u_kI_veli.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kI_veli.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kI_veli.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kI_veli.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kI_vel[i] = u_kI_veli.real;
      offset += sizeof(this->kI_vel[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_kD_veli;
      u_kD_veli.base = 0;
      u_kD_veli.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kD_veli.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kD_veli.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kD_veli.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kD_vel[i] = u_kD_veli.real;
      offset += sizeof(this->kD_vel[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_pid_update_velocity;
      u_pid_update_velocity.base = 0;
      u_pid_update_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pid_update_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pid_update_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pid_update_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pid_update_velocity = u_pid_update_velocity.real;
      offset += sizeof(this->pid_update_velocity);
     return offset;
    }

    const char * getType(){ return "open_motor_msgs/pid_config"; };
    const char * getMD5(){ return "cd4fcf7971c73c09a2b3aefbc2513515"; };

  };

}
#endif
