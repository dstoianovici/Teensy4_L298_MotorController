#ifndef _ROS_ryan_msgs_GetSchedulesResponseTemplate_h
#define _ROS_ryan_msgs_GetSchedulesResponseTemplate_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ryan_msgs
{

  class GetSchedulesResponseTemplate : public ros::Msg
  {
    public:
      typedef int8_t _response_result_type;
      _response_result_type response_result;
      typedef const char* _success_result_type;
      _success_result_type success_result;
      typedef const char* _errors_type;
      _errors_type errors;

    GetSchedulesResponseTemplate():
      response_result(0),
      success_result(""),
      errors("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_response_result;
      u_response_result.real = this->response_result;
      *(outbuffer + offset + 0) = (u_response_result.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->response_result);
      uint32_t length_success_result = strlen(this->success_result);
      varToArr(outbuffer + offset, length_success_result);
      offset += 4;
      memcpy(outbuffer + offset, this->success_result, length_success_result);
      offset += length_success_result;
      uint32_t length_errors = strlen(this->errors);
      varToArr(outbuffer + offset, length_errors);
      offset += 4;
      memcpy(outbuffer + offset, this->errors, length_errors);
      offset += length_errors;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_response_result;
      u_response_result.base = 0;
      u_response_result.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->response_result = u_response_result.real;
      offset += sizeof(this->response_result);
      uint32_t length_success_result;
      arrToVar(length_success_result, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_success_result; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_success_result-1]=0;
      this->success_result = (char *)(inbuffer + offset-1);
      offset += length_success_result;
      uint32_t length_errors;
      arrToVar(length_errors, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_errors; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_errors-1]=0;
      this->errors = (char *)(inbuffer + offset-1);
      offset += length_errors;
     return offset;
    }

    const char * getType(){ return "ryan_msgs/GetSchedulesResponseTemplate"; };
    const char * getMD5(){ return "5bf3ec8b5e1d4ba24fbfed83faad5ef8"; };

  };

}
#endif
