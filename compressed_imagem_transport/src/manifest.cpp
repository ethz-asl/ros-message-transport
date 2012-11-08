#include <pluginlib/class_list_macros.h>
#include "compressed_imagem_transport/compressed_publisher.h"
#include "compressed_imagem_transport/compressed_subscriber.h"

PLUGINLIB_DECLARE_CLASS(imagem_transport, compressed_pub, compressed_imagem_transport::CompressedPublisher, message_transport::PublisherPlugin<sensor_msgs::Image>)

PLUGINLIB_DECLARE_CLASS(imagem_transport, compressed_sub, compressed_imagem_transport::CompressedSubscriber, message_transport::SubscriberPlugin<sensor_msgs::Image>)
