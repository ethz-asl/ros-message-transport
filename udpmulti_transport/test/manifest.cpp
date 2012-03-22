#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <pluginlib/class_list_macros.h>
#include "udpmulti_transport/udpmulti_publisher.h"
#include "udpmulti_transport/udpmulti_subscriber.h"

PLUGINLIB_DECLARE_CLASS(udpmulti_transport, udpmulti_pub, udpmulti_transport::UDPMultiPublisher<std_msgs::Header>, message_transport::PublisherPlugin<std_msgs::Header>)

PLUGINLIB_DECLARE_CLASS(udpmulti_transport, udpmulti_sub, udpmulti_transport::UDPMultiSubscriber<std_msgs::Header>, message_transport::SubscriberPlugin<std_msgs::Header>)
