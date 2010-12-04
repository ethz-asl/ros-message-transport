#ifndef SHAREDMEM_MESSAGE_TRANSPORT_SUBSCRIBER_H
#define SHAREDMEM_MESSAGE_TRANSPORT_SUBSCRIBER_H

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

#include <message_transport/simple_subscriber_plugin.h>
#include <sharedmem_transport/SharedMemoryBlock.h>
#include <sharedmem_transport/SharedMemHeader.h>

namespace sharedmem_transport {

	template <class Base>
	class SharedmemSubscriber : public message_transport::SimpleSubscriberPlugin<Base,sharedmem_transport::SharedMemHeader>
	{
		public:
			SharedmemSubscriber() {
				segment_ = NULL;
                blockmgr_ = NULL;
			}

			virtual ~SharedmemSubscriber() {
                ROS_INFO("Shutting down SharedmemSubscriber");
                if (rec_thread_) {
                    // We probably need to do something to clean up the
                    // cancelled thread here
                    rec_thread_->interrupt();
                    rec_thread_->join();
                    delete rec_thread_;
                }
                rec_thread_ = NULL;
				delete segment_;
			}

			virtual std::string getTransportName() const
			{
				return "sharedmem";
			}

		protected:
              void receiveThread() {
                while (ros::ok()) {
                    blockmgr_->wait_data_and_register_client(shm_handle_);
                    if (!ros::ok()) break;

                    boost::shared_ptr<Base> message_ptr(new Base);
                    blockmgr_->deserialize<Base>(*segment_,shm_handle_,*message_ptr);
                    if (user_cb_ && ros::ok()) {
                        (*user_cb_)(message_ptr);
                    }
                    blockmgr_->unregister_client(shm_handle_);
                }
            }

			virtual void internalCallback(const sharedmem_transport::SharedMemHeaderConstPtr& message,
					const typename message_transport::SimpleSubscriberPlugin<Base,sharedmem_transport::SharedMemHeader>::Callback& user_cb)
			{
				user_cb_ = &user_cb;
                if (!segment_) {
                    segment_ = new boost::interprocess::managed_shared_memory(boost::interprocess::open_only,ROSSharedMemoryDefaultBlock);
                    blockmgr_ = (segment_->find<SharedMemoryBlock>("Manager")).first;
                    shm_handle_ = blockmgr_->allocateBlock(*segment_,this->getTopic().c_str(),0);
                    rec_thread_ = new  boost::thread(&SharedmemSubscriber::receiveThread,this);
                }
			}


			const typename message_transport::SimpleSubscriberPlugin<Base,sharedmem_transport::SharedMemHeader>::Callback* user_cb_;
            boost::thread *rec_thread_;
			boost::interprocess::managed_shared_memory *segment_ ;
            SharedMemoryBlock *blockmgr_;
			shm_handle shm_handle_;
	};

} //namespace transport

#endif // SHAREDMEM_MESSAGE_TRANSPORT_SUBSCRIBER_H
