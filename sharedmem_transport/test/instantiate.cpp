

#include <ros/ros.h>
#include <roslib/Header.h>
#include <sharedmem_transport/sharedmem_publisher.h>
#include <sharedmem_transport/sharedmem_subscriber.h>

int main()
{
    sharedmem_transport::SharedmemPublisher<roslib::Header> pub;
    sharedmem_transport::SharedmemSubscriber<roslib::Header> sub;

    sleep(1);

    return 0;
}
