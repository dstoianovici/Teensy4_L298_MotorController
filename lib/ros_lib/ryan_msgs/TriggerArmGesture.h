#ifndef _ROS_SERVICE_TriggerArmGesture_h
#define _ROS_SERVICE_TriggerArmGesture_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ryan_msgs/ArmStatus.h"

namespace ryan_msgs
{

static const char TRIGGERARMGESTURE[] = "ryan_msgs/TriggerArmGesture";

  class TriggerArmGestureRequest : public ros::Msg
  {
    public:
      typedef int32_t _gesture_type;
      _gesture_type gesture;
      enum { HOME =  100 };
      enum { WAVE_LEFT =  101 };
      enum { WAVE_RIGHT =  102 };
      enum { TAI_CHI =  103 };

    TriggerArmGestureRequest():
      gesture(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_gesture;
      u_gesture.real = this->gesture;
      *(outbuffer + offset + 0) = (u_gesture.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gesture.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gesture.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gesture.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gesture);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_gesture;
      u_gesture.base = 0;
      u_gesture.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gesture.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gesture.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gesture.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gesture = u_gesture.real;
      offset += sizeof(this->gesture);
     return offset;
    }

    const char * getType(){ return TRIGGERARMGESTURE; };
    const char * getMD5(){ return "33952ee218c76963d697300198f46e71"; };

  };

  class TriggerArmGestureResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      typedef ryan_msgs::ArmStatus _status_type;
      _status_type status;
      enum { ACK =  0                };
      enum { BUSY =  -1              };
      enum { INVALID_GESTURE =  -2   };

    TriggerArmGestureResponse():
      result(0),
      status()
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
      offset += this->status.serialize(outbuffer + offset);
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
      offset += this->status.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return TRIGGERARMGESTURE; };
    const char * getMD5(){ return "ad06775cd69e320e3a51e7fc3b8bb9b6"; };

  };

  class TriggerArmGesture {
    public:
    typedef TriggerArmGestureRequest Request;
    typedef TriggerArmGestureResponse Response;
  };

}
#endif
