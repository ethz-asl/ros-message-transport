#ifndef MESSAGE_TRANSPORT_RAW_PUBLISHER_H
#define MESSAGE_TRANSPORT_RAW_PUBLISHER_H

#include "message_transport/simple_publisher_plugin.h"

namespace message_transport {

/**
 * \brief The default PublisherPlugin.
 *
 * RawPublisher is a simple wrapper for ros::Publisher, publishing unaltered Image
 * messages on the base topic.
 */
template <class M>
class RawPublisher : public SimplePublisherPlugin<M,M>
{
public:
  virtual ~RawPublisher() {}

  virtual std::string getTransportName() const
  {
    return "raw";
  }

protected:
  virtual void publish(const M& message, 
		  const typename SimplePublisherPlugin<M,M>::PublishFn& publish_fn) const
  {
    publish_fn(message);
  }

  virtual std::string getTopicToAdvertise(const std::string& base_topic) const
  {
    return base_topic;
  }
};

} //namespace message_transport

#endif
