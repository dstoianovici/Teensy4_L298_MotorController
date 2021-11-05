#ifndef _ROS_ryan_msgs_MotionCommand_h
#define _ROS_ryan_msgs_MotionCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "ryan_msgs/GestureCommand.h"

namespace ryan_msgs
{

  class MotionCommand : public ros::Msg
  {
    public:
      typedef int32_t _type_type;
      _type_type type;
      typedef bool _merge_type;
      _merge_type merge;
      typedef trajectory_msgs::JointTrajectoryPoint _posture_type;
      _posture_type posture;
      typedef ryan_msgs::GestureCommand _gesture_type;
      _gesture_type gesture;
      enum { POSTURE =  1 };
      enum { GESTURE =  2 };
      enum { BOTH =  3 };

    MotionCommand():
      type(0),
      merge(0),
      posture(),
      gesture()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_type;
      u_type.real = this->type;
      *(outbuffer + offset + 0) = (u_type.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_type.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_type.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_type.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->type);
      union {
        bool real;
        uint8_t base;
      } u_merge;
      u_merge.real = this->merge;
      *(outbuffer + offset + 0) = (u_merge.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->merge);
      offset += this->posture.serialize(outbuffer + offset);
      offset += this->gesture.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_type;
      u_type.base = 0;
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->type = u_type.real;
      offset += sizeof(this->type);
      union {
        bool real;
        uint8_t base;
      } u_merge;
      u_merge.base = 0;
      u_merge.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->merge = u_merge.real;
      offset += sizeof(this->merge);
      offset += this->posture.deserialize(inbuffer + offset);
      offset += this->gesture.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "ryan_msgs/MotionCommand"; };
    const char * getMD5(){ return "d8569228b376fb0676f6beddd688e42c"; };

  };

}
#endif
