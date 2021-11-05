#ifndef _ROS_ryan_msgs_FER_h
#define _ROS_ryan_msgs_FER_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ryan_msgs/FacePosition.h"

namespace ryan_msgs
{

  class FER : public ros::Msg
  {
    public:
      typedef int32_t _Emotion_type;
      _Emotion_type Emotion;
      typedef ryan_msgs::FacePosition _Position_type;
      _Position_type Position;
      enum { Neutral =  0 };
      enum { Happy =  1 };
      enum { Sad =  2 };
      enum { Anger =  3 };
      enum { Surprise =  4 };
      enum { Disgust =  5 };
      enum { Contempt =  6 };

    FER():
      Emotion(0),
      Position()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_Emotion;
      u_Emotion.real = this->Emotion;
      *(outbuffer + offset + 0) = (u_Emotion.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Emotion.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Emotion.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Emotion.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Emotion);
      offset += this->Position.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_Emotion;
      u_Emotion.base = 0;
      u_Emotion.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Emotion.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Emotion.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Emotion.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Emotion = u_Emotion.real;
      offset += sizeof(this->Emotion);
      offset += this->Position.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "ryan_msgs/FER"; };
    const char * getMD5(){ return "d67d28ec1f418ab95e6633b13b4245c4"; };

  };

}
#endif
