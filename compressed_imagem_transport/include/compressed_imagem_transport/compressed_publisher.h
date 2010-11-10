#ifndef IMAGEM_TRANSPORT_COMPRESSED_PUBLISHER_H
#define IMAGEM_TRANSPORT_COMPRESSED_PUBLISHER_H

#include <message_transport/simple_publisher_plugin.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

namespace compressed_imagem_transport {

class CompressedPublisher : public message_transport::SimplePublisherPlugin<sensor_msgs::Image,sensor_msgs::CompressedImage>
{
public:
  virtual ~CompressedPublisher() {}

  virtual std::string getTransportName() const
  {
    return "compressed";
  }

protected:
  virtual void publish(const sensor_msgs::Image& message,
		  const message_transport::SimplePublisherPlugin<sensor_msgs::Image,sensor_msgs::CompressedImage>::PublishFn& publish_fn) const ;
};

} //namespace compressed_imagem_transport

#endif // IMAGEM_TRANSPORT_COMPRESSED_PUBLISHER_H
