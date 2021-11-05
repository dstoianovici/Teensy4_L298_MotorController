#ifndef _ROS_marvelmind_nav_beacon_distance_h
#define _ROS_marvelmind_nav_beacon_distance_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace marvelmind_nav
{

  class beacon_distance : public ros::Msg
  {
    public:
      typedef uint8_t _address_hedge_type;
      _address_hedge_type address_hedge;
      typedef uint8_t _address_beacon_type;
      _address_beacon_type address_beacon;
      typedef double _distance_m_type;
      _distance_m_type distance_m;

    beacon_distance():
      address_hedge(0),
      address_beacon(0),
      distance_m(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->address_hedge >> (8 * 0)) & 0xFF;
      offset += sizeof(this->address_hedge);
      *(outbuffer + offset + 0) = (this->address_beacon >> (8 * 0)) & 0xFF;
      offset += sizeof(this->address_beacon);
      union {
        double real;
        uint64_t base;
      } u_distance_m;
      u_distance_m.real = this->distance_m;
      *(outbuffer + offset + 0) = (u_distance_m.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distance_m.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distance_m.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distance_m.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_distance_m.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_distance_m.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_distance_m.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_distance_m.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->distance_m);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->address_hedge =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->address_hedge);
      this->address_beacon =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->address_beacon);
      union {
        double real;
        uint64_t base;
      } u_distance_m;
      u_distance_m.base = 0;
      u_distance_m.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_distance_m.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_distance_m.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_distance_m.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_distance_m.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_distance_m.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_distance_m.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_distance_m.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->distance_m = u_distance_m.real;
      offset += sizeof(this->distance_m);
     return offset;
    }

    const char * getType(){ return "marvelmind_nav/beacon_distance"; };
    const char * getMD5(){ return "0961792211a42c14a3b38a49e24931f3"; };

  };

}
#endif
