#include <message_transport/list_transport.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

using namespace message_transport;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "list_transport_pointcloud"); 
    
    {
        printf("\n\n=========== sensor_msgs::PointCloud ===========\n");
        ListTransport<sensor_msgs::PointCloud> l_pc;
        l_pc.run("pointcloud_transport","sensor_msgs::PointCloud");
    }

    {
        printf("\n\n=========== sensor_msgs::PointCloud2 ===========\n");
        ListTransport<sensor_msgs::PointCloud2> l_pc2;
        l_pc2.run("pointcloud_transport","sensor_msgs::PointCloud2");
    }

    {
        printf("\n\n=========== sensor_msgs::LaserScan ===========\n");
        ListTransport<sensor_msgs::LaserScan> l_ls;
        l_ls.run("pointcloud_transport","sensor_msgs::LaserScan");
    }

	return 0;
}
