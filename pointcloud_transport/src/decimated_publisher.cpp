#include "pointcloud_transport_base/decimated_publisher.h"

namespace decimated_transport {

void DecimatedPublisher::publish(const sensor_msgs::PointCloud& pointcloud,
		const message_transport::SimplePublisherPlugin<sensor_msgs::PointCloud,sensor_msgs::PointCloud>::PublishFn& publish_fn) const
{
	// Set up message and publish
	unsigned int n = pointcloud.points.size();
	unsigned int m = pointcloud.channels.size();

	sensor_msgs::PointCloud decimated;
	decimated.header = pointcloud.header;
	decimated.points.resize(n/decimation);
	decimated.channels.resize(m);
	for (unsigned int i=0;i<m;i++) {
		decimated.channels[i].name = pointcloud.channels[i].name;
		decimated.channels[i].values.resize(n/decimation);
	}
	for (unsigned int i=0;i<n/decimation;i++) {
		decimated.points[i] = pointcloud.points[i*decimation];
		for (unsigned int j=0;j<m;j++) {
			decimated.channels[j].values[i] = pointcloud.channels[j].values[i*decimation];
		}
	}

	publish_fn(decimated);
}

} //namespace decimated_transport
