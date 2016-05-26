#ifndef _ROS_SERVICE_SetTransformableMarkerColor_h
#define _ROS_SERVICE_SetTransformableMarkerColor_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/ColorRGBA.h"

namespace jsk_interactive_marker
{

static const char SETTRANSFORMABLEMARKERCOLOR[] = "jsk_interactive_marker/SetTransformableMarkerColor";

  class SetTransformableMarkerColorRequest : public ros::Msg
  {
    public:
      const char* target_name;
      std_msgs::ColorRGBA color;

    SetTransformableMarkerColorRequest():
      target_name(""),
      color()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_target_name = strlen(this->target_name);
      memcpy(outbuffer + offset, &length_target_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->target_name, length_target_name);
      offset += length_target_name;
      offset += this->color.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_target_name;
      memcpy(&length_target_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_target_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_target_name-1]=0;
      this->target_name = (char *)(inbuffer + offset-1);
      offset += length_target_name;
      offset += this->color.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SETTRANSFORMABLEMARKERCOLOR; };
    const char * getMD5(){ return "6da9e77546dd19426d1dc251fb18d20e"; };

  };

  class SetTransformableMarkerColorResponse : public ros::Msg
  {
    public:

    SetTransformableMarkerColorResponse()
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

    const char * getType(){ return SETTRANSFORMABLEMARKERCOLOR; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetTransformableMarkerColor {
    public:
    typedef SetTransformableMarkerColorRequest Request;
    typedef SetTransformableMarkerColorResponse Response;
  };

}
#endif
