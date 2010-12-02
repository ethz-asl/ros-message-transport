#include <ros/ros.h>
#include <message_transport/message_transport.h>
#include <sensor_msgs/PointCloud.h>

std::string transport;
unsigned int npoints = 0;
FILE * fp=NULL;

void callback(const sensor_msgs::PointCloudConstPtr& pointcloud)
{
	double tnow = ros::Time::now().toSec();
	double tstamp = pointcloud->header.stamp.toSec();
	if (!fp) {
		char fname[512];
		sprintf(fname,"received_%d_%s_%d.txt",
				getpid(),transport.c_str(),pointcloud->points.size());
		ROS_INFO("Saving data in %s",fname);
		fp = fopen(fname,"w");
	}
	fprintf(fp,"%d %f %f %f\n",npoints,tnow,tstamp,tnow-tstamp);
	npoints ++;

	ROS_INFO("%d: Scan received at %f, delay %f",
			getpid(),tstamp,tnow-tstamp);

	if (npoints > 1000) {
		fclose(fp);
		ros::shutdown();
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_receiver", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  message_transport::MessageTransport<sensor_msgs::PointCloud> 
	  it(nh,"pointcloud_transport","sensor_msgs::PointCloud");

  transport = std::string((argc > 1) ? argv[1] : "raw");
  message_transport::Subscriber sub = it.subscribe("pc_source", 1, callback, 
		  transport);
  ROS_INFO("test_receiver started");

  ros::spin();
}

