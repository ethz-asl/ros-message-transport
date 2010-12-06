#include <ros/ros.h>
#include <roslib/Header.h>
#include <pluginlib/class_list_macros.h>
#include "sharedmem_transport/sharedmem_publisher.h"
#include "sharedmem_transport/sharedmem_subscriber.h"

PLUGINLIB_DECLARE_CLASS(sharedmem_transport,sharedmem_pub, sharedmem_transport::SharedmemPublisher<roslib::Header>, message_transport::PublisherPlugin<roslib::Header>)

PLUGINLIB_DECLARE_CLASS(sharedmem_transport,sharedmem_sub, sharedmem_transport::SharedmemSubscriber<roslib::Header>, message_transport::SubscriberPlugin<roslib::Header>)
