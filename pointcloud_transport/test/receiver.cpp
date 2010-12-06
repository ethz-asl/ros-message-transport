#include <ros/ros.h>
#include <roscpp/SetLoggerLevel.h>
#include <message_transport/message_transport.h>
#include <sensor_msgs/PointCloud.h>
#include "numpoints.h"

std::string transport;
unsigned int npoints = 0;

// #define RECORD

#ifdef RECORD
FILE * fp=NULL;
#endif

void callback(const sensor_msgs::PointCloudConstPtr& pointcloud)
{
    unsigned int i;
    double tnow = ros::Time::now().toSec();
    double tstamp = pointcloud->header.stamp.toSec();
#ifdef RECORD
    if (!fp) {
        char fname[512];
        sprintf(fname,"received_%d_%s_%d.txt",
                getpid(),transport.c_str(),pointcloud->points.size());
        ROS_INFO("Saving data in %s",fname);
        fp = fopen(fname,"w");
    }
    fprintf(fp,"%d %f %f %f\n",npoints,tnow,tstamp,tnow-tstamp);
#endif
    npoints ++;

    assert(pointcloud->points.size() >= numpoints);
    assert(pointcloud->channels.size() == 1);
    assert(pointcloud->channels[0].values.size() >= numpoints);
    for (i=0;i<pointcloud->points.size();i++) {
        assert(round(pointcloud->points[i].x - i) < 1e-3);
        assert(round(pointcloud->points[i].y - i/10) < 1e-3);
        assert(round(pointcloud->points[i].z - i/100) < 1e-3);
        assert(round(pointcloud->channels[0].values[i] - i) < 1e-3);
    }

    ROS_INFO("%d: Scan received at %f, delay %f",
            getpid(),tstamp,tnow-tstamp);

#ifdef RECORD
    if (npoints > 1000) {
        fclose(fp);
        ros::shutdown();
    }
#endif
}

void setDebugLevel(const std::string & logname) {
    log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(logname);
    logger->setLevel(log4cxx::Level::getDebug());
    ros::console::notifyLoggerLevelsChanged();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_receiver", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    message_transport::MessageTransport<sensor_msgs::PointCloud> 
        it(nh,"pointcloud_transport","sensor_msgs::PointCloud");
    std::string pkgname("pointcloud_transport");
    transport = std::string((argc > 1) ? argv[1] : "pointcloud_transport/raw");
    if (transport.compare(0,pkgname.length(),pkgname)) {
        transport = pkgname + "/" + transport;
    }
    message_transport::Subscriber sub = it.subscribe("pc_source", 1, callback, 
            transport);
    ROS_INFO("test_receiver started");

    // setDebugLevel("ros.pointcloud_transport");
    // setDebugLevel("ros.sharedmem_transport");

    ros::spin();
}

