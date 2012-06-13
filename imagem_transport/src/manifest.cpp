#include <pluginlib/class_list_macros.h>
#include <templated_transports/declare_all_templates.h>
#include <sensor_msgs/Image.h>

// Brings in all templated transports (sharedmem, raw, udpmulti, bz2)
DECLARE_ALL_TEMPLATES(imagem_transport,img,sensor_msgs::Image)

