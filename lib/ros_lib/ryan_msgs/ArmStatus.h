#ifndef _ROS_ryan_msgs_ArmStatus_h
#define _ROS_ryan_msgs_ArmStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ryan_msgs
{

  class ArmStatus : public ros::Msg
  {
    public:
      typedef int32_t _status_type;
      _status_type status;
      enum { READY =  1            };
      enum { ACTIVE =  0           };
      enum { BLOCKED =  -1         };
      enum { COLLISION =  -2       };
      enum { DISABLED =  -3        };
      enum { ERROR =  -4           };
      enum { DISCONNECTED =  -5    };

    ArmStatus():
      status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_status.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_status.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_status.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->status = u_status.real;
      offset += sizeof(this->status);
     return offset;
    }

    const char * getType(){ return "ryan_msgs/ArmStatus"; };
    const char * getMD5(){ return "d6df8d710d0520ed56e2d90d802f7d7e"; };

  };

}
#endif
