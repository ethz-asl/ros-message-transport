#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/PointCloud.h>
#include <message_transport/message_transport.h>
#include "numpoints.h"

class PointCloudPublisher {
	protected:

		ros::NodeHandle n_;
		message_transport::MessageTransport<sensor_msgs::PointCloud> it_;
		message_transport::Publisher pcmsg_pub_;
		sensor_msgs::PointCloud pointcloud;


	public:

		PointCloudPublisher(ros::NodeHandle &n) : n_(n), 
			it_(n_,"pointcloud_transport","sensor_msgs::PointCloud") {
            unsigned int i;
			pcmsg_pub_ = it_.advertise("pc_source",1);
			pointcloud.points.resize(numpoints);
			pointcloud.channels.resize(1);
			pointcloud.channels[0].name = "intensity";
			pointcloud.channels[0].values.resize(numpoints);
            for (i=0;i<numpoints;i++) {
                pointcloud.points[i].x = i;
                pointcloud.points[i].y = i/10;
                pointcloud.points[i].z = i/100;
                pointcloud.channels[0].values[i] = i;
            }

		}

		~PointCloudPublisher()
		{
		}

		int mainloop()
		{

            ROS_INFO("Entering main loop");
			ros::Rate loop_rate(3);
			while (ros::ok())
			{
				pointcloud.header.stamp = ros::Time::now();
                // ROS_DEBUG("Trying to publish");
				pcmsg_pub_.publish(pointcloud);
				ROS_DEBUG("Published pointcloud at %f",pointcloud.header.stamp.toSec());
				ros::spinOnce();
				loop_rate.sleep();
			}

			return 0;
		}

};

void setDebugLevel(const std::string & logname) {
    log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(logname);
    logger->setLevel(log4cxx::Level::getDebug());
    ros::console::notifyLoggerLevelsChanged();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_publisher");
	ros::NodeHandle n;
	PointCloudPublisher ic(n);
    // setDebugLevel("ros.pointcloud_transport");
    // setDebugLevel("ros.sharedmem_transport");
	ic.mainloop();
	return 0;
}

