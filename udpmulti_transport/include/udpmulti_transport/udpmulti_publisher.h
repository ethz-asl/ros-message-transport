#ifndef UDPMULTI_TRANSPORT_PUBLISHER_H
#define UDPMULTI_TRANSPORT_PUBLISHER_H


#include <roslib/Header.h>
#include <message_transport/simple_publisher_plugin.h>
#include <udpmulti_transport/UDPMultiHeader.h>

namespace udpmulti_transport {

	class UDPMultiPublisherImpl 
	{
		public:
			UDPMultiPublisherImpl();
			virtual ~UDPMultiPublisherImpl();

			uint32_t getPort();
			const std::string & getHostName();

			UDPMultHeader getUDPHeader() const;

			void multicast(const ros::Message & message);
		protected:
			// This will be modified after the first object is received, so we
			// mark them mutable and publish stays "const"
			
			uint32_t port_;
			std::string hostname_;
	};

	template <class Base>
	class UDPMultiPublisher : 
		public message_transport::SimplePublisherPlugin<Base,udpmulti_transport::UDPMultiHeader>
	{
		public:
			UDPMultiPublisher() : impl(), port(0)_, 
				datasize_(0), tlast_header_(-1) {}
			virtual ~UDPMultiPublisher() {}

			virtual std::string getTransportName() const
			{
				return "udpmulti";
			}

		protected:
			virtual void publish(const Base& message,
					const typename message_transport::SimplePublisherPlugin<Base,udpmulti_transport::UDPMultiHeader>::PublishFn& publish_fn) const {
				
				double tnow = ros::Time::now().toSec();
				
				datasize_ = ros::serialization::serializationLength(message);
				if (datasize_ > MAX_UDP_PACKET_SIZE) {
					ROS_ERROR("This type of message is too big (%d bytes) for UDP (max %d bytes)",
							datasize_, MAX_UDP_PACKET_SIZE);
					return;
				}

				if (tnow - tlast_header_ > 1.0) {
					publish_fn(impl.getUDPHeader());
					tlast_header_ = tnow;
				}
				impl.multicast(message);
			}

			mutable UDPMultiPublisherImpl impl;
			uint32_t port_;
			uint32_t datasize_;
			double tlast_header_;
			
	};


} //namespace udpmulti_transport


#endif // UDPMULTI_TRANSPORT_PUBLISHER_H
