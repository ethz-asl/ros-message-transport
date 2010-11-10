#ifndef SHAREDMEM_MESSAGE_TRANSPORT_SUBSCRIBER_H
#define SHAREDMEM_MESSAGE_TRANSPORT_SUBSCRIBER_H


#include <message_transport/simple_subscriber_plugin.h>
#include <sharedmem_transport/SharedMemoryBlock.h>
#include <sharedmem_transport/SharedMemMessage.h>
#include <boost/interprocess/managed_shared_memory.hpp>

namespace sharedmem_transport {

	template <class Base>
	class SharedmemSubscriber : public message_transport::SimpleSubscriberPlugin<Base,sharedmem_transport::SharedMemMessage>
	{
		public:
			SharedmemSubscriber() {
				ptr_ = NULL;
				segment_ = NULL;
				alloc_length_ = 0;
				handle_ = 0;
				try {
					segment_ = new boost::interprocess::managed_shared_memory(boost::interprocess::open_only,ROSSharedMemoryBlock);
				} catch (std::exception e) {
					ROS_ERROR("Could not open shared memory segment");
				}
			}

			virtual ~SharedmemSubscriber() {
				delete segment_;
			}

			virtual std::string getTransportName() const
			{
				return "sharedmem";
			}

		protected:
			virtual void internalCallback(const sharedmem_transport::SharedMemMessageConstPtr& message,
					const typename message_transport::SimpleSubscriberPlugin<Base,sharedmem_transport::SharedMemMessage>::Callback& user_cb)
			{
				if (!segment_) {
					ROS_ERROR("Sharedmem publisher cannot be used without sharedmem_manager running");
					return;
				}

				boost::shared_ptr<Base> message_ptr(new Base);
				if (message->blockid <= 0) {
					ROS_ERROR("Sharedmem handle is not valid");
					return;
				}
				ptr_ = (uint8_t*)(segment_->get_address_from_handle(message->blockid));
				alloc_length_ = message->blocksize;

				ros::serialization::IStream in(ptr_,alloc_length_);
				ros::serialization::deserialize(in, *message_ptr);

				user_cb(message_ptr);
			}


			boost::interprocess::managed_shared_memory *segment_ ;
			uint8_t * ptr_;
			uint32_t alloc_length_;
			uint32_t handle_;
	};

} //namespace transport

#endif // SHAREDMEM_MESSAGE_TRANSPORT_SUBSCRIBER_H
