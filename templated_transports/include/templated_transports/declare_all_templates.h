#ifndef DECLARE_ALL_TEMPLATES_H
#define DECLARE_ALL_TEMPLATES_H

#include <pluginlib/class_list_macros.h>
#include <message_transport/raw_publisher.h>
#include <message_transport/raw_subscriber.h>
#include <sharedmem_transport/sharedmem_publisher.h>
#include <sharedmem_transport/sharedmem_subscriber.h>
#include <udpmulti_transport/udpmulti_publisher.h>
#include <udpmulti_transport/udpmulti_subscriber.h>
#include <bz2_transport/bz2_publisher.h>
#include <bz2_transport/bz2_subscriber.h>
#include <throttled_transport/throttled_publisher.h>
#include <throttled_transport/throttled_subscriber.h>

// Declare all the templated classes for a given type
#define DECLARE_ALL_TEMPLATES(plugin_namespace,suffix,type) \
PLUGINLIB_DECLARE_CLASS(plugin_namespace,raw_##suffix##_pub, message_transport::RawPublisher<type>, message_transport::PublisherPlugin<type>)                  \
PLUGINLIB_DECLARE_CLASS(plugin_namespace,raw_##suffix##_sub, message_transport::RawSubscriber<type>, message_transport::SubscriberPlugin<type>)                \
PLUGINLIB_DECLARE_CLASS(plugin_namespace,sharedmem_##suffix##_pub, sharedmem_transport::SharedmemPublisher<type>, message_transport::PublisherPlugin<type>)    \
PLUGINLIB_DECLARE_CLASS(plugin_namespace,sharedmem_##suffix##_sub, sharedmem_transport::SharedmemSubscriber<type>, message_transport::SubscriberPlugin<type>)  \
PLUGINLIB_DECLARE_CLASS(plugin_namespace,udpmulti_##suffix##_pub, udpmulti_transport::UDPMultiPublisher<type>, message_transport::PublisherPlugin<type>)       \
PLUGINLIB_DECLARE_CLASS(plugin_namespace,udpmulti_##suffix##_sub, udpmulti_transport::UDPMultiSubscriber<type>, message_transport::SubscriberPlugin<type>)     \
PLUGINLIB_DECLARE_CLASS(plugin_namespace,bz2_##suffix##_pub, bz2_transport::BZ2Publisher<type>, message_transport::PublisherPlugin<type>)                      \
PLUGINLIB_DECLARE_CLASS(plugin_namespace,bz2_##suffix##_sub, bz2_transport::BZ2Subscriber<type>, message_transport::SubscriberPlugin<type>)                    \
PLUGINLIB_DECLARE_CLASS(plugin_namespace,throttled_##suffix##_pub, throttled_transport::ThrottledPublisher<type>, message_transport::PublisherPlugin<type>)    \
PLUGINLIB_DECLARE_CLASS(plugin_namespace,throttled_##suffix##_sub, throttled_transport::ThrottledSubscriber<type>, message_transport::SubscriberPlugin<type>)                    


#endif // DECLARE_ALL_TEMPLATES_H

