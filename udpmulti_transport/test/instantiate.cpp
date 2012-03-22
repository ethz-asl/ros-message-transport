

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <udpmulti_transport/udpmulti_publisher.h>
#include <udpmulti_transport/udpmulti_subscriber.h>

int main()
{
    udpmulti_transport::UDPMultiPublisher<roslib::Header> pub;
    udpmulti_transport::UDPMultiSubscriber<roslib::Header> sub;

    sleep(1);

    return 0;
}
