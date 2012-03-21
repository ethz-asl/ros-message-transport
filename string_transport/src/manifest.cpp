#include <pluginlib/class_list_macros.h>
#include <message_transport/raw_publisher.h>
#include <message_transport/raw_subscriber.h>
#include <sharedmem_transport/sharedmem_publisher.h>
#include <sharedmem_transport/sharedmem_subscriber.h>
#include <udpmulti_transport/udpmulti_publisher.h>
#include <udpmulti_transport/udpmulti_subscriber.h>
#include <std_msgs/String.h>

// Raw class
PLUGINLIB_DECLARE_CLASS(string_transport,raw_pub, message_transport::RawPublisher<std_msgs::String>, message_transport::PublisherPlugin<std_msgs::String>)
PLUGINLIB_DECLARE_CLASS(string_transport,raw_sub, message_transport::RawSubscriber<std_msgs::String>, message_transport::SubscriberPlugin<std_msgs::String>)

// Sharedmem class
PLUGINLIB_DECLARE_CLASS(string_transport,sharedmem_pub, sharedmem_transport::SharedmemPublisher<std_msgs::String>, message_transport::PublisherPlugin<std_msgs::String>)
PLUGINLIB_DECLARE_CLASS(string_transport,sharedmem_sub, sharedmem_transport::SharedmemSubscriber<std_msgs::String>, message_transport::SubscriberPlugin<std_msgs::String>)

// UDP multi-casting class
PLUGINLIB_DECLARE_CLASS(string_transport,udpmulti_pub, udpmulti_transport::UDPMultiPublisher<std_msgs::String>, message_transport::PublisherPlugin<std_msgs::String>)
PLUGINLIB_DECLARE_CLASS(string_transport,udpmulti_sub, udpmulti_transport::UDPMultiSubscriber<std_msgs::String>, message_transport::SubscriberPlugin<std_msgs::String>)

