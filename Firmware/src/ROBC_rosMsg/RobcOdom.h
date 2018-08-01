#ifndef _ROS_robc_controller_RobcOdom_h
#define _ROS_robc_controller_RobcOdom_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "ArduinoIncludes.h"

namespace robc_controller
{

  class RobcOdom : public ros::Msg
  {
    public:
      typedef ros::Time _timestamp_type;
      _timestamp_type timestamp;
      typedef int16_t _steeringAngle_type;
      _steeringAngle_type steeringAngle;
      typedef int16_t _encoderCounts_type;
      _encoderCounts_type encoderCounts;

    RobcOdom():
      timestamp(),
      steeringAngle(0),
      encoderCounts(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->timestamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp.sec);
      *(outbuffer + offset + 0) = (this->timestamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp.nsec);
      union {
        int16_t real;
        uint16_t base;
      } u_steeringAngle;
      u_steeringAngle.real = this->steeringAngle;
      *(outbuffer + offset + 0) = (u_steeringAngle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_steeringAngle.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->steeringAngle);
      union {
        int16_t real;
        uint16_t base;
      } u_encoderCounts;
      u_encoderCounts.real = this->encoderCounts;
      *(outbuffer + offset + 0) = (u_encoderCounts.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoderCounts.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->encoderCounts);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->timestamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp.sec);
      this->timestamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp.nsec);
      union {
        int16_t real;
        uint16_t base;
      } u_steeringAngle;
      u_steeringAngle.base = 0;
      u_steeringAngle.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_steeringAngle.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->steeringAngle = u_steeringAngle.real;
      offset += sizeof(this->steeringAngle);
      union {
        int16_t real;
        uint16_t base;
      } u_encoderCounts;
      u_encoderCounts.base = 0;
      u_encoderCounts.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoderCounts.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->encoderCounts = u_encoderCounts.real;
      offset += sizeof(this->encoderCounts);
     return offset;
    }

    const char * getType(){ return PSTR("robc_controller/RobcOdom"); };
    const char * getMD5(){ return PSTR("9566c05f15a81ad42c6a30f158c88b24"); };

  };

}
#endif
