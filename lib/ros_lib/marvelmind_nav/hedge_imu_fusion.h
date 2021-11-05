#ifndef _ROS_marvelmind_nav_hedge_imu_fusion_h
#define _ROS_marvelmind_nav_hedge_imu_fusion_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace marvelmind_nav
{

  class hedge_imu_fusion : public ros::Msg
  {
    public:
      typedef int64_t _timestamp_ms_type;
      _timestamp_ms_type timestamp_ms;
      typedef double _x_m_type;
      _x_m_type x_m;
      typedef double _y_m_type;
      _y_m_type y_m;
      typedef double _z_m_type;
      _z_m_type z_m;
      typedef double _qw_type;
      _qw_type qw;
      typedef double _qx_type;
      _qx_type qx;
      typedef double _qy_type;
      _qy_type qy;
      typedef double _qz_type;
      _qz_type qz;
      typedef double _vx_type;
      _vx_type vx;
      typedef double _vy_type;
      _vy_type vy;
      typedef double _vz_type;
      _vz_type vz;
      typedef double _ax_type;
      _ax_type ax;
      typedef double _ay_type;
      _ay_type ay;
      typedef double _az_type;
      _az_type az;

    hedge_imu_fusion():
      timestamp_ms(0),
      x_m(0),
      y_m(0),
      z_m(0),
      qw(0),
      qx(0),
      qy(0),
      qz(0),
      vx(0),
      vy(0),
      vz(0),
      ax(0),
      ay(0),
      az(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_timestamp_ms;
      u_timestamp_ms.real = this->timestamp_ms;
      *(outbuffer + offset + 0) = (u_timestamp_ms.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_timestamp_ms.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_timestamp_ms.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_timestamp_ms.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_timestamp_ms.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_timestamp_ms.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_timestamp_ms.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_timestamp_ms.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->timestamp_ms);
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
      union {
        double real;
        uint64_t base;
      } u_qw;
      u_qw.real = this->qw;
      *(outbuffer + offset + 0) = (u_qw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_qw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_qw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_qw.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_qw.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_qw.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_qw.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_qw.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->qw);
      union {
        double real;
        uint64_t base;
      } u_qx;
      u_qx.real = this->qx;
      *(outbuffer + offset + 0) = (u_qx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_qx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_qx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_qx.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_qx.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_qx.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_qx.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_qx.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->qx);
      union {
        double real;
        uint64_t base;
      } u_qy;
      u_qy.real = this->qy;
      *(outbuffer + offset + 0) = (u_qy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_qy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_qy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_qy.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_qy.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_qy.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_qy.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_qy.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->qy);
      union {
        double real;
        uint64_t base;
      } u_qz;
      u_qz.real = this->qz;
      *(outbuffer + offset + 0) = (u_qz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_qz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_qz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_qz.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_qz.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_qz.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_qz.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_qz.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->qz);
      union {
        double real;
        uint64_t base;
      } u_vx;
      u_vx.real = this->vx;
      *(outbuffer + offset + 0) = (u_vx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vx.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_vx.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_vx.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_vx.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_vx.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->vx);
      union {
        double real;
        uint64_t base;
      } u_vy;
      u_vy.real = this->vy;
      *(outbuffer + offset + 0) = (u_vy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vy.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_vy.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_vy.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_vy.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_vy.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->vy);
      union {
        double real;
        uint64_t base;
      } u_vz;
      u_vz.real = this->vz;
      *(outbuffer + offset + 0) = (u_vz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vz.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_vz.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_vz.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_vz.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_vz.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->vz);
      union {
        double real;
        uint64_t base;
      } u_ax;
      u_ax.real = this->ax;
      *(outbuffer + offset + 0) = (u_ax.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ax.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ax.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ax.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_ax.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_ax.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_ax.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_ax.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->ax);
      union {
        double real;
        uint64_t base;
      } u_ay;
      u_ay.real = this->ay;
      *(outbuffer + offset + 0) = (u_ay.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ay.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ay.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ay.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_ay.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_ay.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_ay.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_ay.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->ay);
      union {
        double real;
        uint64_t base;
      } u_az;
      u_az.real = this->az;
      *(outbuffer + offset + 0) = (u_az.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_az.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_az.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_az.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_az.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_az.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_az.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_az.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->az);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_timestamp_ms;
      u_timestamp_ms.base = 0;
      u_timestamp_ms.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_timestamp_ms.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_timestamp_ms.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_timestamp_ms.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_timestamp_ms.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_timestamp_ms.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_timestamp_ms.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_timestamp_ms.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->timestamp_ms = u_timestamp_ms.real;
      offset += sizeof(this->timestamp_ms);
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
      union {
        double real;
        uint64_t base;
      } u_qw;
      u_qw.base = 0;
      u_qw.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_qw.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_qw.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_qw.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_qw.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_qw.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_qw.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_qw.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->qw = u_qw.real;
      offset += sizeof(this->qw);
      union {
        double real;
        uint64_t base;
      } u_qx;
      u_qx.base = 0;
      u_qx.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_qx.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_qx.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_qx.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_qx.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_qx.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_qx.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_qx.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->qx = u_qx.real;
      offset += sizeof(this->qx);
      union {
        double real;
        uint64_t base;
      } u_qy;
      u_qy.base = 0;
      u_qy.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_qy.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_qy.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_qy.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_qy.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_qy.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_qy.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_qy.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->qy = u_qy.real;
      offset += sizeof(this->qy);
      union {
        double real;
        uint64_t base;
      } u_qz;
      u_qz.base = 0;
      u_qz.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_qz.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_qz.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_qz.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_qz.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_qz.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_qz.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_qz.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->qz = u_qz.real;
      offset += sizeof(this->qz);
      union {
        double real;
        uint64_t base;
      } u_vx;
      u_vx.base = 0;
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->vx = u_vx.real;
      offset += sizeof(this->vx);
      union {
        double real;
        uint64_t base;
      } u_vy;
      u_vy.base = 0;
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->vy = u_vy.real;
      offset += sizeof(this->vy);
      union {
        double real;
        uint64_t base;
      } u_vz;
      u_vz.base = 0;
      u_vz.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vz.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vz.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vz.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_vz.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_vz.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_vz.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_vz.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->vz = u_vz.real;
      offset += sizeof(this->vz);
      union {
        double real;
        uint64_t base;
      } u_ax;
      u_ax.base = 0;
      u_ax.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ax.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ax.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ax.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_ax.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_ax.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_ax.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_ax.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->ax = u_ax.real;
      offset += sizeof(this->ax);
      union {
        double real;
        uint64_t base;
      } u_ay;
      u_ay.base = 0;
      u_ay.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ay.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ay.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ay.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_ay.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_ay.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_ay.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_ay.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->ay = u_ay.real;
      offset += sizeof(this->ay);
      union {
        double real;
        uint64_t base;
      } u_az;
      u_az.base = 0;
      u_az.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_az.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_az.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_az.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_az.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_az.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_az.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_az.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->az = u_az.real;
      offset += sizeof(this->az);
     return offset;
    }

    const char * getType(){ return "marvelmind_nav/hedge_imu_fusion"; };
    const char * getMD5(){ return "80fa4231724bd716826855f463bf5400"; };

  };

}
#endif
