#ifndef _ROS_SERVICE_GetMood_h
#define _ROS_SERVICE_GetMood_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ryan_msgs
{

static const char GETMOOD[] = "ryan_msgs/GetMood";

  class GetMoodRequest : public ros::Msg
  {
    public:

    GetMoodRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return GETMOOD; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetMoodResponse : public ros::Msg
  {
    public:
      typedef float _mood_type;
      _mood_type mood;

    GetMoodResponse():
      mood(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_mood;
      u_mood.real = this->mood;
      *(outbuffer + offset + 0) = (u_mood.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mood.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mood.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mood.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mood);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_mood;
      u_mood.base = 0;
      u_mood.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mood.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mood.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mood.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mood = u_mood.real;
      offset += sizeof(this->mood);
     return offset;
    }

    const char * getType(){ return GETMOOD; };
    const char * getMD5(){ return "763335e4015e6012142330e4939bb87a"; };

  };

  class GetMood {
    public:
    typedef GetMoodRequest Request;
    typedef GetMoodResponse Response;
  };

}
#endif
