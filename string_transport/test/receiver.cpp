#include <ros/ros.h>
#include <roscpp/SetLoggerLevel.h>
#include <message_transport/message_transport.h>
#include <std_msgs/String.h>

std::string transport;
unsigned int npoints = 0;

void callback(const std_msgs::StringConstPtr& text)
{
    double tnow = ros::Time::now().toSec();


    ROS_INFO("%d: Text '%s' received at %f",
            getpid(),text->data.c_str(),tnow);

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

    message_transport::MessageTransport<std_msgs::String> 
        it(nh,"string_transport","std_msgs::String");
    std::string pkgname("string_transport");
    transport = std::string((argc > 1) ? argv[1] : "string_transport/raw");
    if (transport.compare(0,pkgname.length(),pkgname)) {
        transport = pkgname + "/" + transport;
    }
    message_transport::Subscriber sub = it.subscribe("str_source", 1, callback, 
            transport);
    ROS_INFO("test_receiver started");

    ros::spin();
}

