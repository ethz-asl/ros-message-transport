#ifndef SHAREDMEM_TRANSPORT_PUBLISHER_H
#define SHAREDMEM_TRANSPORT_PUBLISHER_H


#include <std_msgs/Header.h>
#include "message_transport/simple_publisher_plugin.h"
#include "sharedmem_transport/SharedMemHeader.h"
#include "sharedmem_transport/SharedMemoryBlock.h"

namespace sharedmem_transport {

	class SharedmemPublisherImpl 
	{
		public:
			SharedmemPublisherImpl();
			virtual ~SharedmemPublisherImpl();

			uint32_t initialise(const std::string & topic);

            void setNodeHandle(ros::NodeHandle & nh) {
                nh_ = nh;
            }


            template <class M>
			void publish_msg(const M& message) {
                uint32_t serlen = ros::serialization::serializationLength(message);

                if (!shm_handle_.is_valid()) {
                    ROS_DEBUG("Ignoring publish request on an invalid handle");
                    return;
                }
                blockmgr_->reallocateBlock(*segment_,shm_handle_,serlen);
                if (shm_handle_.is_valid()) { // check again, in case reallocate failed
                    blockmgr_->serialize(*segment_,shm_handle_,message);
                }
            }
		protected:
			boost::interprocess::managed_shared_memory *segment_ ;
            SharedMemoryBlock *blockmgr_;
			bool clientRegistered;

			// This will be modified after the first image is received, so we
			// mark them mutable and publish stays "const"
            shm_handle shm_handle_;
            ros::NodeHandle nh_;
			
	};

	template <class Base>
	class SharedmemPublisher : 
		public message_transport::SimplePublisherPlugin<Base,sharedmem_transport::SharedMemHeader>
	{
		public:
			SharedmemPublisher() : 
                message_transport::SimplePublisherPlugin<Base,sharedmem_transport::SharedMemHeader>(true), // force latch
                first_run_(true) {}
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
					const typename message_transport::SimplePublisherPlugin<Base,sharedmem_transport::SharedMemHeader>::PublishFn& publish_fn) const {
                if (first_run_) {
                    ROS_INFO("First publish run");
                    SharedMemHeader header;
                    header.handle = impl.initialise(this->getTopic());
                    ROS_INFO("Publishing latched header");
                    publish_fn(header);
                    first_run_ = false;
                }
                ROS_DEBUG("Publishing shm message");
                impl.publish_msg(message);
			}

			mutable SharedmemPublisherImpl impl;
            mutable bool first_run_;
			
	};

} //namespace sharedmem_transport


#endif // SHAREDMEM_TRANSPORT_PUBLISHER_H
