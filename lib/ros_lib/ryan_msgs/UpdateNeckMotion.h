#ifndef _ROS_SERVICE_UpdateNeckMotion_h
#define _ROS_SERVICE_UpdateNeckMotion_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ryan_msgs/MotionCommand.h"

namespace ryan_msgs
{

static const char UPDATENECKMOTION[] = "ryan_msgs/UpdateNeckMotion";

  class UpdateNeckMotionRequest : public ros::Msg
  {
    public:
      typedef ryan_msgs::MotionCommand _desired_motion_type;
      _desired_motion_type desired_motion;

    UpdateNeckMotionRequest():
      desired_motion()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->desired_motion.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->desired_motion.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return UPDATENECKMOTION; };
    const char * getMD5(){ return "d6e353577eb7552ebab661448279f651"; };

  };

  class UpdateNeckMotionResponse : public ros::Msg
  {
    public:
      typedef int32_t _error_code_type;
      _error_code_type error_code;
      enum { SUCCESSFUL =  0 };
      enum { ERROR =  -1 };
      enum { INVALID_TRAJECTORIES =  -2 };
      enum { INVALID_REQUEST =  -3 };
      enum { INVALID_RESPONSE =  -4 };

    UpdateNeckMotionResponse():
      error_code(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_error_code;
      u_error_code.real = this->error_code;
      *(outbuffer + offset + 0) = (u_error_code.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_error_code.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_error_code.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_error_code.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->error_code);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_error_code;
      u_error_code.base = 0;
      u_error_code.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_error_code.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_error_code.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_error_code.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->error_code = u_error_code.real;
      offset += sizeof(this->error_code);
     return offset;
    }

    const char * getType(){ return UPDATENECKMOTION; };
    const char * getMD5(){ return "d4e308ee985fa161b042d237ac412785"; };

  };

  class UpdateNeckMotion {
    public:
    typedef UpdateNeckMotionRequest Request;
    typedef UpdateNeckMotionResponse Response;
  };

}
#endif
