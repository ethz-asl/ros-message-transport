

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
        if (segment_) {
            delete segment_;
        }
	}

	uint32_t SharedmemPublisherImpl::initialise(const std::string & topic) {
		if (!clientRegistered) {
			clientRegistered = true;
            ROS_INFO("Waiting for service '/sharedmem_manager/get_blocks' to be ready");
            ros::service::waitForService("/sharedmem_manager/get_blocks");
            ROS_INFO("Connecting to shared memory segment");

                std::string segment_name = ROSSharedMemoryDefaultBlock;
                nh_.param<std::string>("/sharedmem_manager/segment_name",segment_name,ROSSharedMemoryDefaultBlock);
                ROS_INFO("Got segment name: %s", segment_name.c_str());
                try {
                    segment_ = new managed_shared_memory(open_only,segment_name.c_str());
                    ROS_INFO("Got segment %p",segment_);
                } catch (boost::interprocess::bad_alloc e) {
                    segment_ = NULL;
                    ROS_ERROR("Could not open shared memory segment");
                    return -1;
                }
                blockmgr_ = (segment_->find<SharedMemoryBlock>("Manager")).first;
                if (!blockmgr_) {
                    delete segment_;
                    segment_ = NULL;
                    ROS_ERROR("Cannot find Manager block in shared memory segment");
                    return -1;
                }
                ROS_INFO("Got manager %p",blockmgr_);
                try {
                    shm_handle_ = blockmgr_->allocateBlock(*segment_,topic.c_str(),16);
                    ROS_INFO("Got shm handle");
                } catch (boost::interprocess::bad_alloc e) {
                    delete segment_;
                    segment_ = NULL;
                    ROS_ERROR("Could not open shared memory segment");
                    return -1;
                }
		}
        return shm_handle_.handle;
	}

} //namespace sharedmem_transport
