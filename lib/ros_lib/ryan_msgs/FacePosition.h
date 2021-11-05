#ifndef _ROS_ryan_msgs_FacePosition_h
#define _ROS_ryan_msgs_FacePosition_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ryan_msgs
{

  class FacePosition : public ros::Msg
  {
    public:
      typedef double _x_type;
      _x_type x;
      typedef double _y_type;
      _y_type y;
      typedef double _w_type;
      _w_type w;
      typedef double _h_type;
      _h_type h;

    FacePosition():
      x(0),
      y(0),
      w(0),
      h(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_x.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_x.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_x.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_x.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->x);
      union {
        double real;
        uint64_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_y.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_y.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_y.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_y.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->y);
      union {
        double real;
        uint64_t base;
      } u_w;
      u_w.real = this->w;
      *(outbuffer + offset + 0) = (u_w.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_w.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_w.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_w.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_w.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_w.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_w.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_w.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->w);
      union {
        double real;
        uint64_t base;
      } u_h;
      u_h.real = this->h;
      *(outbuffer + offset + 0) = (u_h.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_h.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_h.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_h.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_h.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_h.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_h.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_h.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->h);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        double real;
        uint64_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        double real;
        uint64_t base;
      } u_w;
      u_w.base = 0;
      u_w.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_w.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_w.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_w.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_w.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_w.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_w.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_w.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->w = u_w.real;
      offset += sizeof(this->w);
      union {
        double real;
        uint64_t base;
      } u_h;
      u_h.base = 0;
      u_h.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_h.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_h.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_h.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_h.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_h.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_h.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_h.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->h = u_h.real;
      offset += sizeof(this->h);
     return offset;
    }

    const char * getType(){ return "ryan_msgs/FacePosition"; };
    const char * getMD5(){ return "336f5560f18c4f799af453369d639f87"; };

  };

}
#endif
