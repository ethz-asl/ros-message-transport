#ifndef BZ2_TRANSPORT_PUBLISHER_H
#define BZ2_TRANSPORT_PUBLISHER_H


#include <message_transport/simple_publisher_plugin.h>
#include <bz2_transport/bz2_codec.h>
#include <bz2_transport/BZ2Packet.h>

namespace bz2_transport {

	template <class Base>
	class BZ2Publisher : 
		public message_transport::SimplePublisherPlugin<Base,bz2_transport::BZ2Packet>
	{
		public:
			BZ2Publisher() : 
                message_transport::SimplePublisherPlugin<Base,bz2_transport::BZ2Packet>(){}
			virtual ~BZ2Publisher() {}

			virtual std::string getTransportName() const
			{
				return "bz2";
			}
		protected:
			virtual void publish(const Base& message,
					const typename message_transport::SimplePublisherPlugin<Base,bz2_transport::BZ2Packet>::PublishFn& publish_fn) const {
				
                BZ2Packet out;
                size_t datasize = ros::serialization::serializationLength(message);
                boost::shared_array<uint8_t> buffer(new uint8_t[datasize]);
                ros::serialization::OStream sout(buffer.get(),datasize);
                ros::serialization::serialize(sout,message);
                if (codec.compress(buffer, datasize, out)) {
                    publish_fn(out);
                }
			}

            BZ2Codec codec;
	};


} //namespace bz2_transport


#endif // BZ2_TRANSPORT_PUBLISHER_H
