#ifndef _ROS_ryan_msgs_IntrRequestResult_h
#define _ROS_ryan_msgs_IntrRequestResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ryan_msgs
{

  class IntrRequestResult : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      typedef int32_t _refId_type;
      _refId_type refId;

    IntrRequestResult():
      result(0),
      refId(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_result;
      u_result.real = this->result;
      *(outbuffer + offset + 0) = (u_result.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_result.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_result.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_result.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->result);
      union {
        int32_t real;
        uint32_t base;
      } u_refId;
      u_refId.real = this->refId;
      *(outbuffer + offset + 0) = (u_refId.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_refId.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_refId.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_refId.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->refId);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_result;
      u_result.base = 0;
      u_result.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_result.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_result.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_result.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->result = u_result.real;
      offset += sizeof(this->result);
      union {
        int32_t real;
        uint32_t base;
      } u_refId;
      u_refId.base = 0;
      u_refId.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_refId.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_refId.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_refId.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->refId = u_refId.real;
      offset += sizeof(this->refId);
     return offset;
    }

    const char * getType(){ return "ryan_msgs/IntrRequestResult"; };
    const char * getMD5(){ return "2baf6ee73b443c6cf09e691d3069e500"; };

  };

}
#endif
