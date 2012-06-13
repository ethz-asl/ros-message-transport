#include <pluginlib/class_list_macros.h>
#include <templated_transports/declare_all_templates.h>
#include <pointcloud_transport_base/decimated_publisher.h>
#include <pointcloud_transport_base/decimated_subscriber.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

    

/*****************************
 *
 * For PointCloud
 *
 * ***************************/

// Brings in all templated transports (sharedmem, raw, udpmulti, bz2)
DECLARE_ALL_TEMPLATES(pointcloud_transport,pc,sensor_msgs::PointCloud)

// Decimated class (only implemented here so far)
PLUGINLIB_DECLARE_CLASS(pointcloud_transport,decimated_pub_pc, decimated_transport::DecimatedPublisher, message_transport::PublisherPlugin<sensor_msgs::PointCloud>)
PLUGINLIB_DECLARE_CLASS(pointcloud_transport,decimated_sub_pc, decimated_transport::DecimatedSubscriber, message_transport::SubscriberPlugin<sensor_msgs::PointCloud>)


/*****************************
 *
 * For PointCloud2
 *
 * ***************************/

// Brings in all templated transports (sharedmem, raw, udpmulti, bz2)
DECLARE_ALL_TEMPLATES(pointcloud_transport,pc2,sensor_msgs::PointCloud2)

/*****************************
 *
 * For LaserScan
 *
 * ***************************/

// Brings in all templated transports (sharedmem, raw, udpmulti, bz2)
DECLARE_ALL_TEMPLATES(pointcloud_transport,ls,sensor_msgs::LaserScan)

