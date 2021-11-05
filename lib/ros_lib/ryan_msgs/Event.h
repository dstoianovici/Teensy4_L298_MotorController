#ifndef _ROS_ryan_msgs_Event_h
#define _ROS_ryan_msgs_Event_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace ryan_msgs
{

  class Event : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _refId_type;
      _refId_type refId;
      typedef int32_t _code_type;
      _code_type code;
      typedef const char* _data_type;
      _data_type data;
      typedef int32_t _source_type;
      _source_type source;
      typedef const char* _additionalData_type;
      _additionalData_type additionalData;

    Event():
      header(),
      refId(0),
      code(0),
      data(""),
      source(0),
      additionalData("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
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
      } u_code;
      u_code.real = this->code;
      *(outbuffer + offset + 0) = (u_code.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_code.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_code.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_code.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->code);
      uint32_t length_data = strlen(this->data);
      varToArr(outbuffer + offset, length_data);
      offset += 4;
      memcpy(outbuffer + offset, this->data, length_data);
      offset += length_data;
      union {
        int32_t real;
        uint32_t base;
      } u_source;
      u_source.real = this->source;
      *(outbuffer + offset + 0) = (u_source.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_source.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_source.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_source.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->source);
      uint32_t length_additionalData = strlen(this->additionalData);
      varToArr(outbuffer + offset, length_additionalData);
      offset += 4;
      memcpy(outbuffer + offset, this->additionalData, length_additionalData);
      offset += length_additionalData;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
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
      } u_code;
      u_code.base = 0;
      u_code.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_code.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_code.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_code.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->code = u_code.real;
      offset += sizeof(this->code);
      uint32_t length_data;
      arrToVar(length_data, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_data; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_data-1]=0;
      this->data = (char *)(inbuffer + offset-1);
      offset += length_data;
      union {
        int32_t real;
        uint32_t base;
      } u_source;
      u_source.base = 0;
      u_source.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_source.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_source.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_source.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->source = u_source.real;
      offset += sizeof(this->source);
      uint32_t length_additionalData;
      arrToVar(length_additionalData, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_additionalData; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_additionalData-1]=0;
      this->additionalData = (char *)(inbuffer + offset-1);
      offset += length_additionalData;
     return offset;
    }

    const char * getType(){ return "ryan_msgs/Event"; };
    const char * getMD5(){ return "f969e81d3673e4e3c97044f2cf420e0e"; };

  };

}
#endif
