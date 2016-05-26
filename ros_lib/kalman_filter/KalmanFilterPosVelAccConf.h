#ifndef _ROS_kalman_filter_KalmanFilterPosVelAccConf_h
#define _ROS_kalman_filter_KalmanFilterPosVelAccConf_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace kalman_filter
{

  class KalmanFilterPosVelAccConf : public ros::Msg
  {
    public:
      enum { RECONFIGURE_KALMAN_FILTER_FLAG =  0 };
      enum { RECONFIGURE_INPUT_SIGMA =  1 };
      enum { RECONFIGURE_BIAS_SIGMA =  2 };
      enum { RECONFIGURE_MEASURE_SIGMA =  3 };

    KalmanFilterPosVelAccConf()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return "kalman_filter/KalmanFilterPosVelAccConf"; };
    const char * getMD5(){ return "d55dd5c79cfe34ed526dcadf76972449"; };

  };

}
#endif