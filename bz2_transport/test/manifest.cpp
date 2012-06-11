#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <pluginlib/class_list_macros.h>
#include "bz2_transport/bz2_publisher.h"
#include "bz2_transport/bz2_subscriber.h"

PLUGINLIB_DECLARE_CLASS(bz2_transport, bz2_pub, bz2_transport::BZ2Publisher<std_msgs::Header>, message_transport::PublisherPlugin<std_msgs::Header>)

PLUGINLIB_DECLARE_CLASS(bz2_transport, bz2_sub, bz2_transport::BZ2Subscriber<std_msgs::Header>, message_transport::SubscriberPlugin<std_msgs::Header>)
