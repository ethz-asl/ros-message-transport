
#include <ros/ros.h>
#include <boost/thread.hpp>

#include "sharedmem_transport/SHMRegisterMemory.h"
#include "sharedmem_transport/SHMRequestMemory.h"
#include "sharedmem_transport/SHMReleaseMemory.h"
#include "sharedmem_transport/SHMClearAll.h"
#include "sharedmem_transport/SHMGetBlocks.h"
#include "sharedmem_transport/SharedMemoryBlock.h"

using namespace boost::interprocess;

struct BlockData {
	uint32_t size_;
	uint32_t pid_;
	BlockData(uint32_t size, uint32_t pid) : size_(size), pid_(pid) {}
};

typedef std::map< uint32_t, BlockData ,std::less<uint32_t> > HandleSet;
managed_shared_memory *segment = NULL;
HandleSet activeHandles;
boost::mutex main_mutex;

void clear_all_handles()
{
	HandleSet::iterator it;
	if (!segment) return;
	for (it=activeHandles.begin();it!=activeHandles.end();it++) {
		void *ptr = segment->get_address_from_handle(it->first);
		segment->deallocate(ptr);
	}
	activeHandles.clear();
}

bool register_memory(sharedmem_transport::SHMRegisterMemory::Request &req, 
		sharedmem_transport::SHMRegisterMemory::Response &res)
{
	boost::lock_guard<boost::mutex> guard(main_mutex);//auto-lock unlock, even on exception
	HandleSet::iterator it = activeHandles.find(req.handle);
	if (it != activeHandles.end()) {
		ROS_INFO("Re-registering block %d (pid %d->%d size %d->%d)",
				req.handle,it->second.pid_,req.pid,it->second.size_,req.size);
	}
	activeHandles.insert(std::pair<uint32_t,BlockData>(req.handle,
				BlockData(req.size,req.pid)));
	ROS_INFO("Registered %d bytes to handle %d for process %d",
			req.size,req.handle,req.pid);
	res.result = 0;
	
	return true;
}

bool request_memory(sharedmem_transport::SHMRequestMemory::Request &req, 
		sharedmem_transport::SHMRequestMemory::Response &res)
{
	boost::lock_guard<boost::mutex> guard(main_mutex);//auto-lock unlock, even on exception
	if (segment->get_free_memory() < req.size) {
		ROS_ERROR("Not enough shared memory in the segment (req %d, avail %d)",
				req.size,segment->get_free_memory());
		res.handle = 0;
	} else {
		void *ptr = segment->allocate(req.size);
		res.handle = segment->get_handle_from_address(ptr);
		activeHandles.insert(std::pair<uint32_t,BlockData>(res.handle,
					BlockData(req.size,req.pid)));
		ROS_INFO("Allocated %d bytes to handle %d for process %d",
				req.size,res.handle,req.pid);
	}
	return true;
}

bool release_memory(sharedmem_transport::SHMReleaseMemory::Request &req, 
		sharedmem_transport::SHMReleaseMemory::Response &res)
{
	boost::lock_guard<boost::mutex> guard(main_mutex);//auto-lock unlock, even on exception
	HandleSet::iterator it = activeHandles.find(req.handle);
	if (it == activeHandles.end()) {
		ROS_ERROR("Trying to release handle %d not managed by sharedmem_manager",
				req.handle);
		res.result = -1;
	} else {
		void *ptr = segment->get_address_from_handle(req.handle);
		segment->deallocate(ptr);
		activeHandles.erase(it);
		ROS_INFO("Released handle %d",req.handle);
	}
	return true;
}

bool clear_memory(sharedmem_transport::SHMClearAll::Request &req, 
		sharedmem_transport::SHMClearAll::Response &res)
{
	boost::lock_guard<boost::mutex> guard(main_mutex);//auto-lock unlock, even on exception
	clear_all_handles();
	ROS_INFO("Deleted all handles");
	res.result = 0;
	return true;
}

bool get_blocks(sharedmem_transport::SHMGetBlocks::Request &req, 
		sharedmem_transport::SHMGetBlocks::Response &res)
{
	HandleSet::iterator it;
	boost::lock_guard<boost::mutex> guard(main_mutex);//auto-lock unlock, even on exception
	unsigned int i=0;
	res.blocks.resize(activeHandles.size());
	res.sizes.resize(activeHandles.size());
	res.pids.resize(activeHandles.size());
	for (it=activeHandles.begin();it!=activeHandles.end();it++,i++) {
		res.blocks[i] = it->first;
		res.sizes[i] = it->second.size_;
		res.pids[i] = it->second.pid_;
	}
	return true;
}


int main(int argc,char *argv[])
{
	int segment_size = 1000000;
	ros::init(argc, argv, "sharedmem_manager");
	ros::NodeHandle n("sharedmem_manager");
	n.param<int>("segment_size",segment_size,1000000);

   //Remove shared memory on construction and destruction
   struct shm_remove
   {
      shm_remove() { shared_memory_object::remove(ROSSharedMemoryBlock); }
      ~shm_remove(){ 
		  clear_all_handles();
		  shared_memory_object::remove(ROSSharedMemoryBlock); 
	  }
   } remover;

   //Managed memory segment that allocates portions of a shared memory
   //segment with the default management algorithm
   managed_shared_memory managed_shm(create_only, ROSSharedMemoryBlock, segment_size);
   segment = &managed_shm;

   ros::ServiceServer regMemSrv = n.advertiseService("register_memory",register_memory);
   ros::ServiceServer reqMemSrv = n.advertiseService("request_memory",request_memory);
   ros::ServiceServer releaseMemSrv = n.advertiseService("release_memory",release_memory);
   ros::ServiceServer clearMemSrv = n.advertiseService("clear_memory",clear_memory);
   ros::ServiceServer getBlocksSrv = n.advertiseService("get_blocks",get_blocks);

   ROS_INFO("Created shared memory with %d bytes",segment_size);

   ros::spin();

}



