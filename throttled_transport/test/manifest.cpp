#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <pluginlib/class_list_macros.h>
#include "throttled_transport/throttled_publisher.h"
#include "throttled_transport/throttled_subscriber.h"

PLUGINLIB_DECLARE_CLASS(throttled_transport, throttled_pub, throttled_transport::ThrottledPublisher<std_msgs::Header>, message_transport::PublisherPlugin<std_msgs::Header>)

PLUGINLIB_DECLARE_CLASS(throttled_transport, throttled_sub, throttled_transport::ThrottledSubscriber<std_msgs::Header>, message_transport::SubscriberPlugin<std_msgs::Header>)
