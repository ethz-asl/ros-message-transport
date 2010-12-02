

#include <boost/interprocess/managed_shared_memory.hpp>
#include <cstdlib> 
#include <cstddef>
#include <cassert>
#include <utility>

#include <ros/ros.h>

#include "sharedmem_transport/SharedMemoryBlock.h"
#include "sharedmem_transport/sharedmem_publisher.h"
#include "sharedmem_transport/SHMRequestMemory.h"
#include "sharedmem_transport/SHMReleaseMemory.h"
#include "sharedmem_transport/SHMRegisterMemory.h"

using namespace boost::interprocess;


namespace sharedmem_transport {

	SharedmemPublisherImpl::SharedmemPublisherImpl()
	{
		ptr_ = NULL;
		alloc_length_ = 0;
		segment_ = NULL;
		clientRegistered = false;
	}

	SharedmemPublisherImpl::~SharedmemPublisherImpl()
	{
		if (ptr_) {
#if 0
			// This does not work, because ros is already shutdown when the 
			// destructors are called
			sharedmem_transport::SHMReleaseMemory srv;
			srv.request.handle = handle_;
			if (releaseMemoryClt.call(srv)) {
				ROS_INFO("Release memory: %ld", (long int)srv.response.result);
			} else {
				ROS_ERROR("Failed to call service release_memory");
			}
#else
			segment_->deallocate(ptr_);
#endif
		}
		delete segment_;
	}

	void SharedmemPublisherImpl::registerServices() {
		if (!clientRegistered) {
			clientRegistered = true;
            ROS_INFO("Waiting for service '/sharedmem_manager/request_memory' to be ready");
            ros::service::waitForService("/sharedmem_manager/request_memory");
            ROS_INFO("Requesting shared memory segment");

            try {
                segment_ = new managed_shared_memory(open_only,ROSSharedMemoryBlock);
            } catch (std::exception e) {
                ROS_ERROR("Could not open shared memory segment");
            }

			requestMemoryClt = nh_.serviceClient<sharedmem_transport::SHMRequestMemory>("/sharedmem_manager/request_memory");
			releaseMemoryClt = nh_.serviceClient<sharedmem_transport::SHMReleaseMemory>("/sharedmem_manager/release_memory");
			registerMemoryClt = nh_.serviceClient<sharedmem_transport::SHMRegisterMemory>("/sharedmem_manager/register_memory");
			ROS_INFO("Sharedmem manager clients registered");
		}


		if (!alloc_length_) {
			// If the pointer has not been allocated yet, we check if there is 
			// a param telling us what to allocate
			int block_size = 0;
			nh_.param<int>("sharedmem_block_size",block_size,0);
			ROS_INFO("Looked for param 'sharedmem_block_size': %d",block_size);
			if (block_size) {
				alloc_length_ = block_size;
				ROS_INFO("Allocated memory based on param sharedmem_block_size: %d bytes", alloc_length_);
			}
		}
	}

	SharedMemMessage SharedmemPublisherImpl::publish_msg(const ros::Message& message,
			const roslib::Header& header) 
	{
		SharedMemMessage output;
		uint32_t serlen = ros::serialization::serializationLength(message);

		output.header = header;
		output.blockid = 0;
		output.blocksize = 0;

        registerServices();

		if (!segment_) {
			ROS_ERROR("Sharedmem publisher cannot be used without sharedmem_manager running");
			return output;
		}

		ROS_DEBUG("Publishing message %f",header.stamp.toSec());


		if (!ptr_) {
#if 0
			// This works, but the request for suppression of the block
			// cannot be sent
			sharedmem_transport::SHMRequestMemory srv;
			srv.request.size = serlen;
			if (requestMemoryClt.call(srv) && srv.response.handle) {
				ROS_DEBUG("Request memory: %ld", (long int)srv.response.handle);
				handle_ = srv.response.handle;
			} else {
				ROS_ERROR("Failed to call service request_memory");
				return output;
			}
			ptr_ = (uint8_t*)segment_->get_address_from_handle(handle_);
#else
			if (alloc_length_) {
				ptr_ = (uint8_t*)segment_->allocate(alloc_length_);
			} else {
				ptr_ = (uint8_t*)segment_->allocate(serlen);
			}
			handle_ = segment_->get_handle_from_address(ptr_);

			sharedmem_transport::SHMRegisterMemory srv;
			srv.request.size = serlen;
			srv.request.handle = handle_;
			srv.request.pid = getpid();
			if (!registerMemoryClt.call(srv)) {
				ROS_ERROR("Failed to call service register_memory");
			}
#endif 
			alloc_length_ = serlen;
			ROS_INFO("Allocated %d bytes of memory at handle %d/%p",
					alloc_length_,handle_,ptr_);
			assert(ptr_);
		} else if (serlen > alloc_length_) {
			throw boost::interprocess::bad_alloc();
		}
		ros::serialization::OStream out(ptr_,alloc_length_);
		ros::serialization::serialize(out,message);

		output.blockid = handle_;
		output.blocksize = alloc_length_;
		return output;
	}

} //namespace sharedmem_transport
