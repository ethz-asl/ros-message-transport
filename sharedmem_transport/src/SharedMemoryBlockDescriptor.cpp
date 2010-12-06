#include "sharedmem_transport/SharedMemoryBlockDescriptor.h"

namespace sharedmem_transport {

    SharedMemoryBlockDescriptor::SharedMemoryBlockDescriptor() :
        num_clients(0), size_(0), allocated_(0), resize_count_(0), active_(false)
    {
        name_[0] = 0;
    }

    uint32_t SharedMemoryBlockDescriptor::getSize() const {
        return size_;
    }

    void SharedMemoryBlockDescriptor::recordSize(uint32_t newsize, uint32_t alloc) {
        if (newsize > allocated_) {
            resize_count_++;
        }
        if (alloc) {
            allocated_ = alloc;
        }
        size_ = newsize;
    }

    void SharedMemoryBlockDescriptor::allocate(const char name[256]) {
        memcpy(name_,name,256);
        active_ = true;
    }

    void SharedMemoryBlockDescriptor::reset() {
        active_ = false;
        name_[0] = 0;
        size_ = 0;
        allocated_ = 0;
        resize_count_ = 0;
    }

    bool SharedMemoryBlockDescriptor::matchName(const char * name) const {
        return strncmp(name_,name, 256) == 0;
    }

}
