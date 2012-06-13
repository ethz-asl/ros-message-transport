

<library path="lib/libpointcloud_transport">
    

  <class name="pointcloud_transport/decimated_pub_pc" 
		type="decimated_transport::DecimatedPublisher" 
		base_class_type="message_transport::PublisherPlugin<sensor_msgs::PointCloud>">
    <description>
		This publisher only publishes 1/10th of the point cloud data
    </description>
  </class>

  <class name="pointcloud_transport/decimated_sub_pc" 
		type="decimated_transport::DecimatedSubscriber" 
		base_class_type="message_transport::SubscriberPlugin<sensor_msgs::PointCloud>">
    <description>
      This is the default pass-through subscriber for topics of type PointCloud
    </description>
  </class>

    DECLARE_ALL_TEMPLATES(pointcloud_transport,pc,sensor_msgs::PointCloud)
    DECLARE_ALL_TEMPLATES(pointcloud_transport,pc2,sensor_msgs::PointCloud2)
    DECLARE_ALL_TEMPLATES(pointcloud_transport,ls,sensor_msgs::LaserScan)
</library>
