#ifndef _ROS_marvelmind_nav_beacon_pos_a_h
#define _ROS_marvelmind_nav_beacon_pos_a_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace marvelmind_nav
{

  class beacon_pos_a : public ros::Msg
  {
    public:
      typedef uint8_t _address_type;
      _address_type address;
      typedef double _x_m_type;
      _x_m_type x_m;
      typedef double _y_m_type;
      _y_m_type y_m;
      typedef double _z_m_type;
      _z_m_type z_m;

    beacon_pos_a():
      address(0),
      x_m(0),
      y_m(0),
      z_m(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->address >> (8 * 0)) & 0xFF;
      offset += sizeof(this->address);
      union {
        double real;
        uint64_t base;
      } u_x_m;
      u_x_m.real = this->x_m;
      *(outbuffer + offset + 0) = (u_x_m.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x_m.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x_m.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x_m.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_x_m.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_x_m.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_x_m.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_x_m.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->x_m);
      union {
        double real;
        uint64_t base;
      } u_y_m;
      u_y_m.real = this->y_m;
      *(outbuffer + offset + 0) = (u_y_m.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y_m.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y_m.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y_m.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_y_m.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_y_m.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_y_m.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_y_m.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->y_m);
      union {
        double real;
        uint64_t base;
      } u_z_m;
      u_z_m.real = this->z_m;
      *(outbuffer + offset + 0) = (u_z_m.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_z_m.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_z_m.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_z_m.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_z_m.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_z_m.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_z_m.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_z_m.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->z_m);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->address =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->address);
      union {
        double real;
        uint64_t base;
      } u_x_m;
      u_x_m.base = 0;
      u_x_m.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x_m.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x_m.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x_m.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_x_m.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_x_m.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_x_m.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_x_m.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->x_m = u_x_m.real;
      offset += sizeof(this->x_m);
      union {
        double real;
        uint64_t base;
      } u_y_m;
      u_y_m.base = 0;
      u_y_m.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y_m.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y_m.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y_m.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_y_m.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_y_m.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_y_m.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_y_m.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->y_m = u_y_m.real;
      offset += sizeof(this->y_m);
      union {
        double real;
        uint64_t base;
      } u_z_m;
      u_z_m.base = 0;
      u_z_m.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_z_m.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_z_m.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_z_m.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_z_m.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_z_m.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_z_m.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_z_m.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->z_m = u_z_m.real;
      offset += sizeof(this->z_m);
     return offset;
    }

    const char * getType(){ return "marvelmind_nav/beacon_pos_a"; };
    const char * getMD5(){ return "2ebe9b8512406c92c2dbfed7a627f03c"; };

  };

}
#endif
