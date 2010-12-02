#ifndef SHAREDMEM_TRANSPORT_PUBLISHER_H
#define SHAREDMEM_TRANSPORT_PUBLISHER_H


#include <roslib/Header.h>
#include "message_transport/simple_publisher_plugin.h"
#include "sharedmem_transport/SharedMemMessage.h"
#include <boost/interprocess/managed_shared_memory.hpp>

namespace sharedmem_transport {

	class SharedmemPublisherImpl 
	{
		public:
			SharedmemPublisherImpl();
			virtual ~SharedmemPublisherImpl();

			void registerServices();

            void setNodeHandle(ros::NodeHandle & nh) {
                nh_ = nh;
            }


			SharedMemMessage publish_msg(const ros::Message& message, const roslib::Header& header) ;
		protected:
			boost::interprocess::managed_shared_memory *segment_ ;
			bool clientRegistered;
			ros::ServiceClient registerMemoryClt;
			ros::ServiceClient requestMemoryClt;
			ros::ServiceClient releaseMemoryClt;

			// This will be modified after the first image is received, so we
			// mark them mutable and publish stays "const"
			uint8_t * ptr_;
			uint32_t alloc_length_;
			uint32_t handle_;
            ros::NodeHandle nh_;
			
	};

	template <class Base>
	class SharedmemPublisher : 
		public message_transport::SimplePublisherPlugin<Base,sharedmem_transport::SharedMemMessage>
	{
		public:
			SharedmemPublisher() {}
			virtual ~SharedmemPublisher() {}

			virtual std::string getTransportName() const
			{
				return "sharedmem";
			}

		protected:
			virtual void postAdvertiseInit() {
				impl.setNodeHandle(this->getNodeHandle());
			}

			virtual void publish(const Base& message,
					const typename message_transport::SimplePublisherPlugin<Base,sharedmem_transport::SharedMemMessage>::PublishFn& publish_fn) const {
				publish_fn(impl.publish_msg(message,this->getHeader(message)));
			}

			virtual roslib::Header getHeader(const Base & message) const {
				roslib::Header header;
				header.stamp = ros::Time::now();
				return header;
			}

			mutable SharedmemPublisherImpl impl;
			
	};

	template <class Base>
	class SharedmemPublisherWithHeader : public SharedmemPublisher<Base>
	{
		public:
			SharedmemPublisherWithHeader() {}
			virtual ~SharedmemPublisherWithHeader() {}

			virtual roslib::Header getHeader(const Base & message) const {
				return message.header;
			}
			
	};

} //namespace sharedmem_transport


#endif // SHAREDMEM_TRANSPORT_PUBLISHER_H
