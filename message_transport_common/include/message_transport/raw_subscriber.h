#ifndef MESSAGE_TRANSPORT_RAW_SUBSCRIBER_H
#define MESSAGE_TRANSPORT_RAW_SUBSCRIBER_H

#include "message_transport/simple_subscriber_plugin.h"

namespace message_transport {

/**
 * \brief The default SubscriberPlugin.
 *
 * RawSubscriber is a simple wrapper for ros::Subscriber which listens for Message messages
 * and passes them through to the callback.
 */
template <class M>
class RawSubscriber : public SimpleSubscriberPlugin<M,M>
{
public:
  virtual ~RawSubscriber() {}

  virtual std::string getTransportName() const
  {
    return "raw";
  }

protected:
  virtual void internalCallback(const typename M::ConstPtr& message, 
		  const typename SimpleSubscriberPlugin<M,M>::Callback& user_cb)
  {
    user_cb(message);
  }

  virtual std::string getTopicToSubscribe(const std::string& base_topic) const
  {
    return base_topic;
  }
};

} //namespace message_transport

#endif
