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

// Declare all the templated classes for a given type
#define DECLARE_ALL_TEMPLATES(plugin_namespace,suffix,type) \
PLUGINLIB_DECLARE_CLASS(plugin_namespace,raw_pub_##suffix, message_transport::RawPublisher<type>, message_transport::PublisherPlugin<type>)                  \
PLUGINLIB_DECLARE_CLASS(plugin_namespace,raw_sub_##suffix, message_transport::RawSubscriber<type>, message_transport::SubscriberPlugin<type>)                \
PLUGINLIB_DECLARE_CLASS(plugin_namespace,sharedmem_pub_##suffix, sharedmem_transport::SharedmemPublisher<type>, message_transport::PublisherPlugin<type>)    \
PLUGINLIB_DECLARE_CLASS(plugin_namespace,sharedmem_sub_##suffix, sharedmem_transport::SharedmemSubscriber<type>, message_transport::SubscriberPlugin<type>)  \
PLUGINLIB_DECLARE_CLASS(plugin_namespace,udpmulti_pub_##suffix, udpmulti_transport::UDPMultiPublisher<type>, message_transport::PublisherPlugin<type>)       \
PLUGINLIB_DECLARE_CLASS(plugin_namespace,udpmulti_sub_##suffix, udpmulti_transport::UDPMultiSubscriber<type>, message_transport::SubscriberPlugin<type>)     \
PLUGINLIB_DECLARE_CLASS(plugin_namespace,bz2_pub_##suffix, bz2_transport::BZ2Publisher<type>, message_transport::PublisherPlugin<type>)                      \
PLUGINLIB_DECLARE_CLASS(plugin_namespace,bz2_sub_##suffix, bz2_transport::BZ2Subscriber<type>, message_transport::SubscriberPlugin<type>)                    


#endif // DECLARE_ALL_TEMPLATES_H

