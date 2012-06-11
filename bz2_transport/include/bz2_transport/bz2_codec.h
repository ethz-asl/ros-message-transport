#ifndef BZ2_CODEC_H
#define BZ2_CODEC_H

#include <ros/ros.h>
#include <bz2_transport/BZ2Packet.h>

namespace bz2_transport {

    class BZ2Codec {
        public:
            BZ2Codec() {}
            ~BZ2Codec() {}

            bool compress(const boost::shared_array<uint8_t> &buffer, size_t len,
                    BZ2Packet & out) const ;

            bool decompress(const BZ2Packet & in, 
                    boost::shared_array<uint8_t> &buffer, size_t &len) const;
    };


}; // bz2_transport



#endif // BZ2_CODEC_H
