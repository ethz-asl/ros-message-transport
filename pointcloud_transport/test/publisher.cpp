#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <message_transport/message_transport.h>

class PointCloudPublisher {
	protected:

		ros::NodeHandle n_;
		message_transport::MessageTransport<sensor_msgs::PointCloud> it_;
		message_transport::Publisher pcmsg_pub_;
		sensor_msgs::PointCloud pointcloud;


	public:

		PointCloudPublisher(ros::NodeHandle &n) : n_(n), 
			it_(n_,"pointcloud_transport","sensor_msgs::PointCloud") {
			const unsigned int numpoints = 300000;
			pcmsg_pub_ = it_.advertise("pc_source",1);
			pointcloud.points.resize(numpoints);
			pointcloud.channels.resize(1);
			pointcloud.channels[0].name = "intensity";
			pointcloud.channels[0].values.resize(numpoints);

		}

		~PointCloudPublisher()
		{
		}

		int mainloop()
		{

			ros::Rate loop_rate(30);
			while (ros::ok())
			{
				pointcloud.header.stamp = ros::Time::now();
				pcmsg_pub_.publish(pointcloud);
				ROS_DEBUG("Published image at %f",pointcloud.header.stamp.toSec());
				ros::spinOnce();
				loop_rate.sleep();
			}

			return 0;
		}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_publisher");
	ros::NodeHandle n;
	PointCloudPublisher ic(n);
	ic.mainloop();
	return 0;
}

