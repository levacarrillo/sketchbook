#ifndef _ROS_SERVICE_CalculateIK_h
#define _ROS_SERVICE_CalculateIK_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ctrl_msgs
{

static const char CALCULATEIK[] = "ctrl_msgs/CalculateIK";

  class CalculateIKRequest : public ros::Msg
  {
    public:
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _z_type;
      _z_type z;
      typedef float _roll_type;
      _roll_type roll;
      typedef float _pitch_type;
      _pitch_type pitch;
      typedef float _yaw_type;
      _yaw_type yaw;

    CalculateIKRequest():
      x(0),
      y(0),
      z(0),
      roll(0),
      pitch(0),
      yaw(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_z;
      u_z.real = this->z;
      *(outbuffer + offset + 0) = (u_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z);
      union {
        float real;
        uint32_t base;
      } u_roll;
      u_roll.real = this->roll;
      *(outbuffer + offset + 0) = (u_roll.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll);
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
        float real;
        uint32_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_z;
      u_z.base = 0;
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->z = u_z.real;
      offset += sizeof(this->z);
      union {
        float real;
        uint32_t base;
      } u_roll;
      u_roll.base = 0;
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->roll = u_roll.real;
      offset += sizeof(this->roll);
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

    const char * getType(){ return CALCULATEIK; };
    const char * getMD5(){ return "3d5697f0c9e8859325238b57521f8f4e"; };

  };

  class CalculateIKResponse : public ros::Msg
  {
    public:
      uint32_t joint_values_length;
      typedef float _joint_values_type;
      _joint_values_type st_joint_values;
      _joint_values_type * joint_values;

    CalculateIKResponse():
      joint_values_length(0), joint_values(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->joint_values_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_values_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_values_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_values_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_values_length);
      for( uint32_t i = 0; i < joint_values_length; i++){
      union {
        float real;
        uint32_t base;
      } u_joint_valuesi;
      u_joint_valuesi.real = this->joint_values[i];
      *(outbuffer + offset + 0) = (u_joint_valuesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_valuesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_valuesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_valuesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_values[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t joint_values_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_values_length);
      if(joint_values_lengthT > joint_values_length)
        this->joint_values = (float*)realloc(this->joint_values, joint_values_lengthT * sizeof(float));
      joint_values_length = joint_values_lengthT;
      for( uint32_t i = 0; i < joint_values_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_joint_values;
      u_st_joint_values.base = 0;
      u_st_joint_values.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_joint_values.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_joint_values.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_joint_values.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_joint_values = u_st_joint_values.real;
      offset += sizeof(this->st_joint_values);
        memcpy( &(this->joint_values[i]), &(this->st_joint_values), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return CALCULATEIK; };
    const char * getMD5(){ return "50b3293f6e585f2a02888f333d3c3e8a"; };

  };

  class CalculateIK {
    public:
    typedef CalculateIKRequest Request;
    typedef CalculateIKResponse Response;
  };

}
#endif
