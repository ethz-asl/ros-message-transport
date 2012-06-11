#include <bzlib.h>
#include <boost/shared_array.hpp>
#include <ros/ros.h>


#include "bz2_transport/bz2_codec.h"

namespace bz2_transport {


    bool BZ2Codec::compress(const boost::shared_array<uint8_t> &buffer, size_t len, BZ2Packet & out) const  {
        int ret = 0;
        // Size following recommendation in bzip2 manual 101% + 600 bytes
        uint32_t destLen = (102*len)/100 + 600;
        boost::shared_array<uint8_t> buf(new uint8_t[destLen]);
        // Removed the first 4 bytes corresponding the data length
        ret = BZ2_bzBuffToBuffCompress((char*)buf.get(),&destLen, (char*)buffer.get(), len, 5, 0, 30);
        if (ret != BZ_OK) {
            ROS_ERROR("BZ2_bzBuffToBuffCompress return %d",ret);
            return false;
        }
        out.original_length = len;
        out.buffer.resize(destLen);
        // Could use memcpy, but probably less portable
        memcpy(&(out.buffer.front()),buf.get(),destLen);
        printf("Message compression: from %d to %d bytes\n",(int)len,(int)destLen);
        return true;
    }

    bool BZ2Codec::decompress(const BZ2Packet & in, 
            boost::shared_array<uint8_t> &buffer, size_t &len) const {
        int ret = 0;
        unsigned int destLen;
        buffer.reset(new uint8_t[in.original_length]);
        ret = BZ2_bzBuffToBuffDecompress((char*)buffer.get(),&destLen, (char*)&(in.buffer.front()), in.buffer.size(), 0, 0);
        if (ret != BZ_OK) {
            ROS_ERROR("BZ2_bzBuffToBuffDecompress return %d",ret);
            return false;
        }
        len = destLen;
        return true;
    }

};
