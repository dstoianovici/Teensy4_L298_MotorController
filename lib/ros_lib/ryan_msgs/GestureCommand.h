#ifndef _ROS_ryan_msgs_GestureCommand_h
#define _ROS_ryan_msgs_GestureCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ryan_msgs
{

  class GestureCommand : public ros::Msg
  {
    public:
      typedef int32_t _type_type;
      _type_type type;
      typedef double _duration_type;
      _duration_type duration;
      typedef double _intensity_type;
      _intensity_type intensity;
      typedef double _rate_type;
      _rate_type rate;
      typedef double _delay_type;
      _delay_type delay;
      typedef bool _invert_type;
      _invert_type invert;
      enum { NOD =  1 };
      enum { HEADBANG =  2 };
      enum { EX_NOD =  3 };
      enum { TILT =  4 };
      enum { SURPRISE =  5 };
      enum { HOLD_SURPRISE =  6 };
      enum { CIRCLE =  7 };
      enum { SHAKE =  8 };
      enum { GLANCE =  9 };

    GestureCommand():
      type(0),
      duration(0),
      intensity(0),
      rate(0),
      delay(0),
      invert(0)
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
        double real;
        uint64_t base;
      } u_duration;
      u_duration.real = this->duration;
      *(outbuffer + offset + 0) = (u_duration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_duration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_duration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_duration.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_duration.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_duration.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_duration.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_duration.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->duration);
      union {
        double real;
        uint64_t base;
      } u_intensity;
      u_intensity.real = this->intensity;
      *(outbuffer + offset + 0) = (u_intensity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_intensity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_intensity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_intensity.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_intensity.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_intensity.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_intensity.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_intensity.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->intensity);
      union {
        double real;
        uint64_t base;
      } u_rate;
      u_rate.real = this->rate;
      *(outbuffer + offset + 0) = (u_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rate.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_rate.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_rate.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_rate.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_rate.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->rate);
      union {
        double real;
        uint64_t base;
      } u_delay;
      u_delay.real = this->delay;
      *(outbuffer + offset + 0) = (u_delay.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_delay.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_delay.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_delay.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_delay.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_delay.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_delay.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_delay.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->delay);
      union {
        bool real;
        uint8_t base;
      } u_invert;
      u_invert.real = this->invert;
      *(outbuffer + offset + 0) = (u_invert.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->invert);
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
        double real;
        uint64_t base;
      } u_duration;
      u_duration.base = 0;
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->duration = u_duration.real;
      offset += sizeof(this->duration);
      union {
        double real;
        uint64_t base;
      } u_intensity;
      u_intensity.base = 0;
      u_intensity.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_intensity.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_intensity.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_intensity.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_intensity.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_intensity.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_intensity.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_intensity.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->intensity = u_intensity.real;
      offset += sizeof(this->intensity);
      union {
        double real;
        uint64_t base;
      } u_rate;
      u_rate.base = 0;
      u_rate.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rate.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rate.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rate.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_rate.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_rate.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_rate.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_rate.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->rate = u_rate.real;
      offset += sizeof(this->rate);
      union {
        double real;
        uint64_t base;
      } u_delay;
      u_delay.base = 0;
      u_delay.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_delay.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_delay.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_delay.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_delay.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_delay.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_delay.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_delay.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->delay = u_delay.real;
      offset += sizeof(this->delay);
      union {
        bool real;
        uint8_t base;
      } u_invert;
      u_invert.base = 0;
      u_invert.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->invert = u_invert.real;
      offset += sizeof(this->invert);
     return offset;
    }

    const char * getType(){ return "ryan_msgs/GestureCommand"; };
    const char * getMD5(){ return "95b1e57bfb7382cd61ee21bab15ab4b4"; };

  };

}
#endif
