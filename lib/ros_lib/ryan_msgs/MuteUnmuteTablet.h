#ifndef _ROS_ryan_msgs_MuteUnmuteTablet_h
#define _ROS_ryan_msgs_MuteUnmuteTablet_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ryan_msgs
{

  class MuteUnmuteTablet : public ros::Msg
  {
    public:
      typedef int32_t _refId_type;
      _refId_type refId;
      typedef int8_t _commandRequest_type;
      _commandRequest_type commandRequest;
      typedef int8_t _commandMute_type;
      _commandMute_type commandMute;
      typedef int32_t _statusCode_type;
      _statusCode_type statusCode;

    MuteUnmuteTablet():
      refId(0),
      commandRequest(0),
      commandMute(0),
      statusCode(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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
      union {
        int8_t real;
        uint8_t base;
      } u_commandRequest;
      u_commandRequest.real = this->commandRequest;
      *(outbuffer + offset + 0) = (u_commandRequest.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->commandRequest);
      union {
        int8_t real;
        uint8_t base;
      } u_commandMute;
      u_commandMute.real = this->commandMute;
      *(outbuffer + offset + 0) = (u_commandMute.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->commandMute);
      union {
        int32_t real;
        uint32_t base;
      } u_statusCode;
      u_statusCode.real = this->statusCode;
      *(outbuffer + offset + 0) = (u_statusCode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_statusCode.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_statusCode.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_statusCode.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->statusCode);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
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
      union {
        int8_t real;
        uint8_t base;
      } u_commandRequest;
      u_commandRequest.base = 0;
      u_commandRequest.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->commandRequest = u_commandRequest.real;
      offset += sizeof(this->commandRequest);
      union {
        int8_t real;
        uint8_t base;
      } u_commandMute;
      u_commandMute.base = 0;
      u_commandMute.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->commandMute = u_commandMute.real;
      offset += sizeof(this->commandMute);
      union {
        int32_t real;
        uint32_t base;
      } u_statusCode;
      u_statusCode.base = 0;
      u_statusCode.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_statusCode.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_statusCode.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_statusCode.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->statusCode = u_statusCode.real;
      offset += sizeof(this->statusCode);
     return offset;
    }

    const char * getType(){ return "ryan_msgs/MuteUnmuteTablet"; };
    const char * getMD5(){ return "7fc5dfc3f51882e1779f270c8e565897"; };

  };

}
#endif
