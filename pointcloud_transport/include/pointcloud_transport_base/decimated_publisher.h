#ifndef POINTCLOUD_TRANSPORT_DECIMATED_PUBLISHER_H
#define POINTCLOUD_TRANSPORT_DECIMATED_PUBLISHER_H

#include <message_transport/simple_publisher_plugin.h>
#include <sensor_msgs/PointCloud.h>

namespace decimated_transport {

	class DecimatedPublisher : public message_transport::SimplePublisherPlugin<sensor_msgs::PointCloud,sensor_msgs::PointCloud>
	{
		public:
			DecimatedPublisher() {decimation=10;}
			virtual ~DecimatedPublisher() {}

			virtual std::string getTransportName() const
			{
				return "decimated";
			}

		protected:
			virtual void publish(const sensor_msgs::PointCloud& pointcloud,
					const message_transport::SimplePublisherPlugin<sensor_msgs::PointCloud,sensor_msgs::PointCloud>::PublishFn& publish_fn) const ;

			mutable unsigned int decimation;
	};

} //namespace decimated_transport

#endif // POINTCLOUD_TRANSPORT_DECIMATED_PUBLISHER_H
