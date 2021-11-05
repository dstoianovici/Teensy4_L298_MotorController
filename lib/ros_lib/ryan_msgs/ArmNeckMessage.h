#ifndef _ROS_ryan_msgs_ArmNeckMessage_h
#define _ROS_ryan_msgs_ArmNeckMessage_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ryan_msgs
{

  class ArmNeckMessage : public ros::Msg
  {
    public:
      typedef int32_t _refId_type;
      _refId_type refId;
      typedef int32_t _gesture_type;
      _gesture_type gesture;
      typedef int32_t _statusCode_type;
      _statusCode_type statusCode;
      typedef float _pitch_type;
      _pitch_type pitch;
      typedef float _yaw_type;
      _yaw_type yaw;
      enum { WAVE_LEFT =  100 };
      enum { WAVE_RIGHT =  101 };
      enum { TAI_CHI =  102 };
      enum { HOME =  103 };
      enum { NOD =  200 };
      enum { SHAKE =  201 };
      enum { GLANCE =  202 };
      enum { POSE =  203 };
      enum { REQUEST =  2 };
      enum { ACK =  1 };
      enum { SUCCESS =  0 };
      enum { ABORT =  -1 };
      enum { ERROR =  -2 };

    ArmNeckMessage():
      refId(0),
      gesture(0),
      statusCode(0),
      pitch(0),
      yaw(0)
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
        int32_t real;
        uint32_t base;
      } u_gesture;
      u_gesture.real = this->gesture;
      *(outbuffer + offset + 0) = (u_gesture.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gesture.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gesture.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gesture.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gesture);
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
      union {
        float real;
        uint32_t base;
      } u_pitch;
      u_pitch.real = this->pitch;
      *(outbuffer + offset + 0) = (u_pitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pitch.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pitch.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pitch.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pitch);
      union {
        float real;
        uint32_t base;
      } u_yaw;
      u_yaw.real = this->yaw;
      *(outbuffer + offset + 0) = (u_yaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw);
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
      union {
        float real;
        uint32_t base;
      } u_pitch;
      u_pitch.base = 0;
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pitch = u_pitch.real;
      offset += sizeof(this->pitch);
      union {
        float real;
        uint32_t base;
      } u_yaw;
      u_yaw.base = 0;
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yaw = u_yaw.real;
      offset += sizeof(this->yaw);
     return offset;
    }

    const char * getType(){ return "ryan_msgs/ArmNeckMessage"; };
    const char * getMD5(){ return "2617b75e2c8362598cf70abc92189fe4"; };

  };

}
#endif
