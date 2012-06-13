#include <pluginlib/class_list_macros.h>
#include <templated_transports/declare_all_templates.h>
#include <std_msgs/String.h>

// Brings in all templated transports (sharedmem, raw, udpmulti, bz2)
DECLARE_ALL_TEMPLATES(string_transport,str,std_msgs::String)

