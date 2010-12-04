#ifndef SHARED_MEMORY_BLOCK_H
#define SHARED_MEMORY_BLOCK_H

#include <cassert>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <ros/ros.h>
#include "sharedmem_transport/SharedMemoryBlockDescriptor.h"

namespace sharedmem_transport {

#define ROSSharedMemoryDefaultBlock "ROS::SharedMemoryBlock"
#define ROSSharedMemoryNumBlock 100



    struct shm_handle {
        uint32_t handle;
        uint8_t *ptr;
        uint32_t resize_count;
        shm_handle() : handle(-1), ptr(NULL), resize_count(0) {}
        shm_handle(uint32_t h, uint32_t rcount, uint8_t *p) : handle(h), ptr(p), resize_count(rcount) {}
        bool is_valid() const {return ptr != NULL;}
    };

    struct SharedMemoryBlock {
        //Mutex to protect access to the queue
        boost::interprocess::interprocess_mutex      mutex;
        boost::interprocess::interprocess_condition  cond;
        int32_t num_clients;

        SharedMemoryBlock() : num_clients(0) {}

        shm_handle findHandle(boost::interprocess::managed_shared_memory & segment, const char * name) ;

        SharedMemoryBlockDescriptor descriptors[ROSSharedMemoryNumBlock];

        shm_handle connectBlock(boost::interprocess::managed_shared_memory & segment, uint32_t handle) ;

        shm_handle allocateBlock(boost::interprocess::managed_shared_memory & segment, 
                const char * name, uint32_t size) ;

        void resetBlock(boost::interprocess::managed_shared_memory & segment, shm_handle & shm) ;
        void resetAllBlocks(boost::interprocess::managed_shared_memory & segment) ;

        void reallocateBlock(boost::interprocess::managed_shared_memory & segment, 
                shm_handle & shm, uint32_t size) ;

        void lock_global() {
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex);
            if (num_clients) {
                cond.wait(lock);
            }
        }

        void unlock_global() {
            mutex.unlock();
        }


        void register_global_client() {
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex); 
            num_clients ++;
        }

        void unregister_global_client() {
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex); 
            num_clients --;
            assert(num_clients >= 0);
            if (num_clients == 0) {
                cond.notify_one();
            }
        }

        void lock(const shm_handle & shm) {
            register_global_client();
            descriptors[shm.handle].lock();
        }

        void unlock(const shm_handle & shm) {
            descriptors[shm.handle].unlock();
            unregister_global_client();
        }


        void register_client(const shm_handle & shm) {
            register_global_client();
            descriptors[shm.handle].register_client();
        }

        void unregister_client(const shm_handle & shm) {
            descriptors[shm.handle].unregister_client();
            unregister_global_client();
        }

        void wait_data_and_register_client(const shm_handle & shm) {
            register_global_client();
            descriptors[shm.handle].wait_data_and_register_client();
        }

        void signal_data_and_unlock(const shm_handle & shm) {
            descriptors[shm.handle].signal_data_and_unlock();
            unregister_global_client();
        }


        template <class Base>
        void deserialize(boost::interprocess::managed_shared_memory & segment,
                shm_handle & src, Base & msg) {
            register_client(src);
            assert(src.handle < ROSSharedMemoryNumBlock);
            if (src.resize_count != descriptors[src.handle].resize_count_) {
                std::pair<uint8_t *, std::size_t> ret = segment.find<uint8_t>(descriptors[src.handle].name_);
                src.resize_count = descriptors[src.handle].resize_count_;
                src.ptr = ret.first;
            }
            ros::serialization::IStream in(src.ptr,descriptors[src.handle].size_);
            ros::serialization::deserialize(in, msg);
            unregister_client(src);
        }

        void serialize(boost::interprocess::managed_shared_memory & segment,
                shm_handle & dest, const ros::Message & msg) ;
    };
} //namespace image_transport

#endif // SHARED_MEMORY_BLOCK_H
