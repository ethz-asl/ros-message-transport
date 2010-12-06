#include <ros/ros.h>
#include <roslib/Header.h>
#include <pluginlib/class_list_macros.h>
#include "udpmulti_transport/udpmulti_publisher.h"
#include "udpmulti_transport/udpmulti_subscriber.h"

PLUGINLIB_DECLARE_CLASS(udpmulti_transport, udpmulti_pub, udpmulti_transport::UDPMultiPublisher<roslib::Header>, message_transport::PublisherPlugin<roslib::Header>)

PLUGINLIB_DECLARE_CLASS(udpmulti_transport, udpmulti_sub, udpmulti_transport::UDPMultiSubscriber<roslib::Header>, message_transport::SubscriberPlugin<roslib::Header>)
