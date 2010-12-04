
#include <ros/ros.h>
#include <boost/thread.hpp>

#include "sharedmem_transport/SHMReleaseMemory.h"
#include "sharedmem_transport/SHMClearAll.h"
#include "sharedmem_transport/SHMGetBlocks.h"
#include "sharedmem_transport/SharedMemBlock.h"
#include "sharedmem_transport/SharedMemoryBlock.h"

using namespace boost::interprocess;
using namespace sharedmem_transport;

boost::mutex main_mutex;
managed_shared_memory *segment = NULL;
SharedMemoryBlock *blockmgr= NULL;



bool release_memory(sharedmem_transport::SHMReleaseMemory::Request &req, 
		sharedmem_transport::SHMReleaseMemory::Response &res)
{
	boost::lock_guard<boost::mutex> guard(main_mutex);//auto-lock unlock, even on exception
    shm_handle shm = blockmgr->findHandle(*segment,req.topic.c_str());
	if (!shm.is_valid()) {
		ROS_ERROR("Trying to release topic %s not managed by sharedmem_manager",
				req.topic.c_str());
		res.result = -1;
	} else {
        blockmgr->resetBlock(*segment, shm);
		ROS_INFO("Released topic %s",req.topic.c_str());
	}
	return true;
}

bool clear_memory(sharedmem_transport::SHMClearAll::Request &req, 
		sharedmem_transport::SHMClearAll::Response &res)
{
	boost::lock_guard<boost::mutex> guard(main_mutex);//auto-lock unlock, even on exception
    blockmgr->resetAllBlocks(*segment);
	ROS_INFO("Deleted all handles");
	res.result = 0;
	return true;
}

bool get_blocks(sharedmem_transport::SHMGetBlocks::Request &req, 
		sharedmem_transport::SHMGetBlocks::Response &res)
{
	boost::lock_guard<boost::mutex> guard(main_mutex);//auto-lock unlock, even on exception
	unsigned int i=0;
	res.blocks.clear();
    for (i=0;i<ROSSharedMemoryNumBlock;i++) {
        if (!blockmgr->descriptors[i].active_) continue;
        SharedMemBlock block;
        block.handle = i;
        block.size = blockmgr->descriptors[i].size_;
        block.allocated = blockmgr->descriptors[i].allocated_;
        block.name = blockmgr->descriptors[i].name_;
        res.blocks.push_back(block);
	}
	return true;
}


int main(int argc,char *argv[])
{
    std::string segment_name = ROSSharedMemoryDefaultBlock;
	int segment_size = 1000000;
	ros::init(argc, argv, "sharedmem_manager");
	ros::NodeHandle n("sharedmem_manager");
	n.param<int>("segment_size",segment_size,1000000);
	n.param<std::string>("segment_name",segment_name,ROSSharedMemoryDefaultBlock);

   //Remove shared memory on construction and destruction
    struct shm_remove {
        std::string name_;
        shm_remove(const std::string & name) : name_(name) { 
            shared_memory_object::remove(name_.c_str()); 
        }
        ~shm_remove(){ 
            if (blockmgr) {
                blockmgr->resetAllBlocks(*segment);
                blockmgr = NULL;
            }
            if (segment) {
                segment->destroy<SharedMemoryBlock>("Manager");
                segment = NULL;
            }
            shared_memory_object::remove(name_.c_str()); 
        }
    } remover(segment_name);

   //Managed memory segment that allocates portions of a shared memory
   //segment with the default management algorithm
   managed_shared_memory managed_shm(create_only, segment_name.c_str(), segment_size);
   segment = &managed_shm;
   blockmgr = segment->find_or_construct<SharedMemoryBlock>("Manager")();

   ros::ServiceServer releaseMemSrv = n.advertiseService("release_memory",release_memory);
   ros::ServiceServer clearMemSrv = n.advertiseService("clear_memory",clear_memory);
   ros::ServiceServer getBlocksSrv = n.advertiseService("get_blocks",get_blocks);

   ROS_INFO("Created shared memory with %d bytes",segment_size);

   ros::spin();

}



