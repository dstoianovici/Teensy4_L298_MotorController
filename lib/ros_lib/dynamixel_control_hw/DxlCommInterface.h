#ifndef _ROS_SERVICE_DxlCommInterface_h
#define _ROS_SERVICE_DxlCommInterface_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_control_hw
{

static const char DXLCOMMINTERFACE[] = "dynamixel_control_hw/DxlCommInterface";

  class DxlCommInterfaceRequest : public ros::Msg
  {
    public:
      typedef int32_t _cmd_type;
      _cmd_type cmd;
      uint32_t servos_length;
      typedef char* _servos_type;
      _servos_type st_servos;
      _servos_type * servos;
      enum { PING = 1 };
      enum { ENABLE = 2 };
      enum { DISABLE = 3 };
      enum { RESET_ERROR = 4 };
      enum { SET_MIN_TORQUE = 5 };
      enum { SET_MAX_TORQUE = 6 };
      enum { READ_TEMP = 7 };
      enum { READ_CURRENT = 8 };

    DxlCommInterfaceRequest():
      cmd(0),
      servos_length(0), servos(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_cmd;
      u_cmd.real = this->cmd;
      *(outbuffer + offset + 0) = (u_cmd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cmd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cmd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cmd);
      *(outbuffer + offset + 0) = (this->servos_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->servos_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->servos_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->servos_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->servos_length);
      for( uint32_t i = 0; i < servos_length; i++){
      uint32_t length_servosi = strlen(this->servos[i]);
      varToArr(outbuffer + offset, length_servosi);
      offset += 4;
      memcpy(outbuffer + offset, this->servos[i], length_servosi);
      offset += length_servosi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_cmd;
      u_cmd.base = 0;
      u_cmd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cmd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cmd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cmd = u_cmd.real;
      offset += sizeof(this->cmd);
      uint32_t servos_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      servos_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      servos_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      servos_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->servos_length);
      if(servos_lengthT > servos_length)
        this->servos = (char**)realloc(this->servos, servos_lengthT * sizeof(char*));
      servos_length = servos_lengthT;
      for( uint32_t i = 0; i < servos_length; i++){
      uint32_t length_st_servos;
      arrToVar(length_st_servos, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_servos; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_servos-1]=0;
      this->st_servos = (char *)(inbuffer + offset-1);
      offset += length_st_servos;
        memcpy( &(this->servos[i]), &(this->st_servos), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return DXLCOMMINTERFACE; };
    const char * getMD5(){ return "2107e496839d6ffae5756d3dd7d8a520"; };

  };

  class DxlCommInterfaceResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      typedef float _data_type;
      _data_type data;
      enum { SUCCESS = 0 };
      enum { ERROR = -1 };

    DxlCommInterfaceResponse():
      result(0),
      data(0)
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
        float real;
        uint32_t base;
      } u_data;
      u_data.real = this->data;
      *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data);
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
        float real;
        uint32_t base;
      } u_data;
      u_data.base = 0;
      u_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->data = u_data.real;
      offset += sizeof(this->data);
     return offset;
    }

    const char * getType(){ return DXLCOMMINTERFACE; };
    const char * getMD5(){ return "ebb4420d7e974147b90613c262b01a6e"; };

  };

  class DxlCommInterface {
    public:
    typedef DxlCommInterfaceRequest Request;
    typedef DxlCommInterfaceResponse Response;
  };

}
#endif
