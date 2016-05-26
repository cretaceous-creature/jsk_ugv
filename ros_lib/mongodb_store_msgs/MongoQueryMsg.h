#ifndef _ROS_SERVICE_MongoQueryMsg_h
#define _ROS_SERVICE_MongoQueryMsg_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mongodb_store_msgs/SerialisedMessage.h"
#include "mongodb_store_msgs/StringPairList.h"

namespace mongodb_store_msgs
{

static const char MONGOQUERYMSG[] = "mongodb_store_msgs/MongoQueryMsg";

  class MongoQueryMsgRequest : public ros::Msg
  {
    public:
      const char* database;
      const char* collection;
      const char* type;
      bool single;
      uint16_t limit;
      mongodb_store_msgs::StringPairList message_query;
      mongodb_store_msgs::StringPairList meta_query;
      mongodb_store_msgs::StringPairList sort_query;
      enum { JSON_QUERY = "jnsdfskajd_fmgs.dlf" };

    MongoQueryMsgRequest():
      database(""),
      collection(""),
      type(""),
      single(0),
      limit(0),
      message_query(),
      meta_query(),
      sort_query()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_database = strlen(this->database);
      memcpy(outbuffer + offset, &length_database, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->database, length_database);
      offset += length_database;
      uint32_t length_collection = strlen(this->collection);
      memcpy(outbuffer + offset, &length_collection, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->collection, length_collection);
      offset += length_collection;
      uint32_t length_type = strlen(this->type);
      memcpy(outbuffer + offset, &length_type, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->type, length_type);
      offset += length_type;
      union {
        bool real;
        uint8_t base;
      } u_single;
      u_single.real = this->single;
      *(outbuffer + offset + 0) = (u_single.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->single);
      *(outbuffer + offset + 0) = (this->limit >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->limit >> (8 * 1)) & 0xFF;
      offset += sizeof(this->limit);
      offset += this->message_query.serialize(outbuffer + offset);
      offset += this->meta_query.serialize(outbuffer + offset);
      offset += this->sort_query.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_database;
      memcpy(&length_database, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_database; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_database-1]=0;
      this->database = (char *)(inbuffer + offset-1);
      offset += length_database;
      uint32_t length_collection;
      memcpy(&length_collection, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_collection; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_collection-1]=0;
      this->collection = (char *)(inbuffer + offset-1);
      offset += length_collection;
      uint32_t length_type;
      memcpy(&length_type, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_type-1]=0;
      this->type = (char *)(inbuffer + offset-1);
      offset += length_type;
      union {
        bool real;
        uint8_t base;
      } u_single;
      u_single.base = 0;
      u_single.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->single = u_single.real;
      offset += sizeof(this->single);
      this->limit =  ((uint16_t) (*(inbuffer + offset)));
      this->limit |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->limit);
      offset += this->message_query.deserialize(inbuffer + offset);
      offset += this->meta_query.deserialize(inbuffer + offset);
      offset += this->sort_query.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return MONGOQUERYMSG; };
    const char * getMD5(){ return "3d0490f3c1d9b6e336a11bb2fd62eab9"; };

  };

  class MongoQueryMsgResponse : public ros::Msg
  {
    public:
      uint8_t messages_length;
      mongodb_store_msgs::SerialisedMessage st_messages;
      mongodb_store_msgs::SerialisedMessage * messages;
      uint8_t metas_length;
      mongodb_store_msgs::StringPairList st_metas;
      mongodb_store_msgs::StringPairList * metas;

    MongoQueryMsgResponse():
      messages_length(0), messages(NULL),
      metas_length(0), metas(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = messages_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < messages_length; i++){
      offset += this->messages[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = metas_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < metas_length; i++){
      offset += this->metas[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t messages_lengthT = *(inbuffer + offset++);
      if(messages_lengthT > messages_length)
        this->messages = (mongodb_store_msgs::SerialisedMessage*)realloc(this->messages, messages_lengthT * sizeof(mongodb_store_msgs::SerialisedMessage));
      offset += 3;
      messages_length = messages_lengthT;
      for( uint8_t i = 0; i < messages_length; i++){
      offset += this->st_messages.deserialize(inbuffer + offset);
        memcpy( &(this->messages[i]), &(this->st_messages), sizeof(mongodb_store_msgs::SerialisedMessage));
      }
      uint8_t metas_lengthT = *(inbuffer + offset++);
      if(metas_lengthT > metas_length)
        this->metas = (mongodb_store_msgs::StringPairList*)realloc(this->metas, metas_lengthT * sizeof(mongodb_store_msgs::StringPairList));
      offset += 3;
      metas_length = metas_lengthT;
      for( uint8_t i = 0; i < metas_length; i++){
      offset += this->st_metas.deserialize(inbuffer + offset);
        memcpy( &(this->metas[i]), &(this->st_metas), sizeof(mongodb_store_msgs::StringPairList));
      }
     return offset;
    }

    const char * getType(){ return MONGOQUERYMSG; };
    const char * getMD5(){ return "f348d453c2d7347807f66360b61cd0ef"; };

  };

  class MongoQueryMsg {
    public:
    typedef MongoQueryMsgRequest Request;
    typedef MongoQueryMsgResponse Response;
  };

}
#endif
