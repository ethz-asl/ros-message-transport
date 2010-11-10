#include <pluginlib/class_list_macros.h>
#include <message_transport/raw_publisher.h>
#include <message_transport/raw_subscriber.h>
#include <sharedmem_transport/sharedmem_publisher.h>
#include <sharedmem_transport/sharedmem_subscriber.h>
#include <sensor_msgs/Image.h>

// Raw class
PLUGINLIB_REGISTER_CLASS(raw_pub, message_transport::RawPublisher<sensor_msgs::Image>, message_transport::PublisherPlugin<sensor_msgs::Image>)
PLUGINLIB_REGISTER_CLASS(raw_sub, message_transport::RawSubscriber<sensor_msgs::Image>, message_transport::SubscriberPlugin<sensor_msgs::Image>)

// Sharedmem class
PLUGINLIB_REGISTER_CLASS(sharedmem_pub, sharedmem_transport::SharedmemPublisherWithHeader<sensor_msgs::Image>, message_transport::PublisherPlugin<sensor_msgs::Image>)
PLUGINLIB_REGISTER_CLASS(sharedmem_sub, sharedmem_transport::SharedmemSubscriber<sensor_msgs::Image>, message_transport::SubscriberPlugin<sensor_msgs::Image>)
