
dnl DECLARE_ALL_TEMPLATES(PLUGIN_NAMESPACE,SUFFIX,TYPE) 
define(`DECLARE_ALL_TEMPLATES', `
    <class name="$1/raw_$2_pub" 
		type="message_transport::RawPublisher<$3>" 
		base_class_type="message_transport::PublisherPlugin<$3>">
    <description>
      This is the default publisher. It publishes the objects as-is on the base topic.
    </description>
  </class>

  <class name="$1/raw_$2_sub" 
		type="message_transport::RawSubscriber<$3>" 
		base_class_type="message_transport::SubscriberPlugin<$3>">
    <description>
      This is the default pass-through subscriber for topics of type $3
    </description>
  </class>

  <class name="$1/sharedmem_$2_pub" 
		type="sharedmem_transport::SharedmemPublisher<$3>" 
		base_class_type="message_transport::PublisherPlugin<$3>">
    <description>
      This publisher publish its data through a shared memory segment and
      shared memory condition.
    </description>
  </class>

  <class name="$1/sharedmem_$2_sub" 
		type="sharedmem_transport::SharedmemSubscriber<$3>" 
		base_class_type="message_transport::SubscriberPlugin<$3>">
    <description>
      This is the shared memory subscriber.
    </description>
  </class>


  <class name="$1/udpmulti_$2_pub" 
		type="udpmulti_transport::UDPMultiPublisher<$3>" 
		base_class_type="message_transport::PublisherPlugin<$3>">
    <description>
        This publisher published its data as UDP multicast telegram. 
        Size is limited to 8092 bytes
    </description>
  </class>

  <class name="$1/udpmulti_$2_sub" 
		type="udpmulti_transport::UDPMultiSubscriber<$3>" 
		base_class_type="message_transport::SubscriberPlugin<$3>">
    <description>
        This subscriber subscribes to UDP multicast telegrams. 
    </description>
  </class>

  <class name="$1/bz2_$2_pub" 
		type="bz2_transport::BZ2Publisher<$3>" 
		base_class_type="message_transport::PublisherPlugin<$3>">
    <description>
        This publisher publishes its type as BZ2 compressed binary
        blobs
    </description>
  </class>

  <class name="$1/bz2_$2_sub" 
		type="bz2_transport::BZ2Subscriber<$3>" 
		base_class_type="message_transport::SubscriberPlugin<$3>">
    <description>
        This subscriber subscribes to BZ2 binary blobs
    </description>
  </class>

  <class name="$1/throttled_$2_pub" 
		type="throttled_transport::ThrottledPublisher<$3>" 
		base_class_type="message_transport::PublisherPlugin<$3>">
    <description>
        This publisher publishes its message but provide a way to limit the
        number of message per second or the bandwidth used by the topic.
    </description>
  </class>

  <class name="$1/throttled_$2_sub" 
		type="throttled_transport::ThrottledSubscriber<$3>" 
		base_class_type="message_transport::SubscriberPlugin<$3>">
    <description>
        This subscriber subscribes to throttled topic. This is basically the
        same as the raw subscriber.
    </description>
  </class>
')

