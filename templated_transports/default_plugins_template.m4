
dnl DECLARE_ALL_TEMPLATES(PLUGIN_NAMESPACE,SUFFIX,TYPE) 
define(`DECLARE_ALL_TEMPLATES', `
    <class name="$1/raw_pub_$2" 
		type="message_transport::RawPublisher<$3>" 
		base_class_type="message_transport::PublisherPlugin<$3>">
    <description>
      This is the default publisher. It publishes the objects as-is on the base topic.
    </description>
  </class>

  <class name="$1/raw_sub_$2" 
		type="message_transport::RawSubscriber<$3>" 
		base_class_type="message_transport::SubscriberPlugin<$3>">
    <description>
      This is the default pass-through subscriber for topics of type PointCloud
    </description>
  </class>

  <class name="$1/sharedmem_pub_$2" 
		type="sharedmem_transport::SharedmemPublisher<$3>" 
		base_class_type="message_transport::PublisherPlugin<$3>">
    <description>
      This is the default publisher. It publishes the objects as-is on the base topic.
    </description>
  </class>

  <class name="$1/sharedmem_sub_$2" 
		type="sharedmem_transport::SharedmemSubscriber<$3>" 
		base_class_type="message_transport::SubscriberPlugin<$3>">
    <description>
      This is the default pass-through subscriber for topics of type PointCloud
    </description>
  </class>


  <class name="$1/udpmulti_pub_$2" 
		type="udpmulti_transport::UDPMultiPublisher<$3>" 
		base_class_type="message_transport::PublisherPlugin<$3>">
    <description>
        This publisher published this point cloud as UDP multicast telegram. 
        Size is limited to 8092 bytes
    </description>
  </class>

  <class name="$1/udpmulti_sub_$2" 
		type="udpmulti_transport::UDPMultiSubscriber<$3>" 
		base_class_type="message_transport::SubscriberPlugin<$3>">
    <description>
        This subscriber subscribes to point cloud UDP multicast telegrams. 
    </description>
  </class>

  <class name="$1/bz2_pub_$2" 
		type="bz2_transport::BZ2Publisher<$3>" 
		base_class_type="message_transport::PublisherPlugin<$3>">
    <description>
        This publisher published this point cloud as BZ2 compressed binary
        blobs
    </description>
  </class>

  <class name="$1/bz2_sub_$2" 
		type="bz2_transport::BZ2Subscriber<$3>" 
		base_class_type="message_transport::SubscriberPlugin<$3>">
    <description>
        This subscriber subscribes to point cloud BZ2 binary blobs
    </description>
  </class>
')

