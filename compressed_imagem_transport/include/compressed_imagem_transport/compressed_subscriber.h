#include "message_transport/simple_subscriber_plugin.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

namespace compressed_imagem_transport {

class CompressedSubscriber : public message_transport::SimpleSubscriberPlugin<sensor_msgs::Image,sensor_msgs::CompressedImage>
{
public:
  virtual ~CompressedSubscriber() {}

  virtual std::string getTransportName() const
  {
    return "compressed";
  }

protected:
  virtual void internalCallback(const sensor_msgs::CompressedImageConstPtr& message,
		  const message_transport::SimpleSubscriberPlugin<sensor_msgs::Image,sensor_msgs::CompressedImage>::Callback& user_cb);
};

} //namespace message_transport
