#include <message_transport/list_transport.h>
#include <sensor_msgs/PointCloud.h>

using namespace message_transport;

int main(int argc, char** argv)
{
	LIST_TRANSPORT("pointcloud_transport",sensor_msgs::PointCloud);

	return 0;
}
