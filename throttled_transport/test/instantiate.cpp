

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <bz2_transport/bz2_publisher.h>
#include <bz2_transport/bz2_subscriber.h>

int main()
{
    bz2_transport::BZ2Publisher<roslib::Header> pub;
    bz2_transport::BZ2Subscriber<roslib::Header> sub;

    sleep(1);

    return 0;
}
