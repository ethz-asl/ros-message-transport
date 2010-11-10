#include <message_transport/raw_subscriber.h>
#include <sensor_msgs/PointCloud.h>

namespace decimated_transport {

class DecimatedSubscriber : public message_transport::RawSubscriber<sensor_msgs::PointCloud>
{
public:
  virtual ~DecimatedSubscriber() {}

  virtual std::string getTransportName() const
  {
    return "decimated";
  }
};

} //namespace message_transport
