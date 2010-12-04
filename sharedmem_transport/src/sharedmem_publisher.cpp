

#include <boost/interprocess/managed_shared_memory.hpp>
#include <cstdlib> 
#include <cstddef>
#include <cassert>
#include <utility>

#include <ros/ros.h>

#include "sharedmem_transport/SharedMemoryBlock.h"
#include "sharedmem_transport/sharedmem_publisher.h"

using namespace boost::interprocess;


namespace sharedmem_transport {

	SharedmemPublisherImpl::SharedmemPublisherImpl()
	{
		segment_ = NULL;
		clientRegistered = false;
	}

	SharedmemPublisherImpl::~SharedmemPublisherImpl()
	{
        // This just disconnect from the segment, any subscriber can still
        // finish reading it.
		delete segment_;
	}

	uint32_t SharedmemPublisherImpl::initialise(const std::string & topic) {
		if (!clientRegistered) {
			clientRegistered = true;
            ROS_INFO("Waiting for service '/sharedmem_manager/get_blocks' to be ready");
            ros::service::waitForService("/sharedmem_manager/get_blocks");
            ROS_INFO("Connecting to shared memory segment");

            try {
                std::string segment_name = ROSSharedMemoryDefaultBlock;
                nh_.param<std::string>("/sharedmem_manager/segment_name",segment_name,ROSSharedMemoryDefaultBlock);
                segment_ = new managed_shared_memory(open_only,segment_name.c_str());
                blockmgr_ = (segment_->find<SharedMemoryBlock>("Manager")).first;
                shm_handle_ = blockmgr_->allocateBlock(*segment_,topic.c_str(),0);
            } catch (std::exception e) {
                ROS_ERROR("Could not open shared memory segment");
            }

		}
        return shm_handle_.handle;
	}

	void SharedmemPublisherImpl::publish_msg(const ros::Message& message) 
	{
		uint32_t serlen = ros::serialization::serializationLength(message);

        blockmgr_->reallocateBlock(*segment_,shm_handle_,serlen);
        blockmgr_->serialize(*segment_,shm_handle_,message);
	}

} //namespace sharedmem_transport
