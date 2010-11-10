#include <pluginlib/class_list_macros.h>
#include "theora_imagem_transport/theora_publisher.h"
#include "theora_imagem_transport/theora_subscriber.h"

PLUGINLIB_REGISTER_CLASS(theora_pub, theora_imagem_transport::TheoraPublisher, message_transport::PublisherPlugin<sensor_msgs::Image>)

PLUGINLIB_REGISTER_CLASS(theora_sub, theora_imagem_transport::TheoraSubscriber, message_transport::SubscriberPlugin<sensor_msgs::Image>)
