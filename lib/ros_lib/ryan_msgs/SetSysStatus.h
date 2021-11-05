#ifndef _ROS_SERVICE_SetSysStatus_h
#define _ROS_SERVICE_SetSysStatus_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ryan_msgs
{

static const char SETSYSSTATUS[] = "ryan_msgs/SetSysStatus";

  class SetSysStatusRequest : public ros::Msg
  {
    public:
      typedef int32_t _status_type;
      _status_type status;
      typedef int32_t _reason_type;
      _reason_type reason;

    SetSysStatusRequest():
      status(0),
      reason(0)
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
      union {
        int32_t real;
        uint32_t base;
      } u_reason;
      u_reason.real = this->reason;
      *(outbuffer + offset + 0) = (u_reason.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_reason.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_reason.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_reason.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->reason);
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
      union {
        int32_t real;
        uint32_t base;
      } u_reason;
      u_reason.base = 0;
      u_reason.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_reason.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_reason.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_reason.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->reason = u_reason.real;
      offset += sizeof(this->reason);
     return offset;
    }

    const char * getType(){ return SETSYSSTATUS; };
    const char * getMD5(){ return "99e94ce1ae88f5803fe2b9e147be88aa"; };

  };

  class SetSysStatusResponse : public ros::Msg
  {
    public:
      typedef bool _result_type;
      _result_type result;

    SetSysStatusResponse():
      result(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.real = this->result;
      *(outbuffer + offset + 0) = (u_result.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.base = 0;
      u_result.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->result = u_result.real;
      offset += sizeof(this->result);
     return offset;
    }

    const char * getType(){ return SETSYSSTATUS; };
    const char * getMD5(){ return "eb13ac1f1354ccecb7941ee8fa2192e8"; };

  };

  class SetSysStatus {
    public:
    typedef SetSysStatusRequest Request;
    typedef SetSysStatusResponse Response;
  };

}
#endif
