#ifndef BZ2_MESSAGE_TRANSPORT_SUBSCRIBER_H
#define BZ2_MESSAGE_TRANSPORT_SUBSCRIBER_H


#include <string>

#include <message_transport/simple_subscriber_plugin.h>
#include <bz2_transport/BZ2Packet.h>
#include <bz2_transport/bz2_codec.h>

namespace bz2_transport {

	template <class Base>
	class BZ2Subscriber : public message_transport::SimpleSubscriberPlugin<Base,bz2_transport::BZ2Packet>
	{
		public:
			BZ2Subscriber() { }

			virtual ~BZ2Subscriber() {
				// close listener
                ROS_INFO("Shutting down BZ2Subscriber");
			}

			virtual std::string getTransportName() const
			{
				return "bz2";
			}

		protected:
			virtual void internalCallback(const bz2_transport::BZ2PacketConstPtr& message,
					const typename message_transport::SimpleSubscriberPlugin<Base,bz2_transport::BZ2Packet>::Callback& user_cb)
			{

                boost::shared_array<uint8_t> buffer;
                size_t len;
                if (codec.decompress(*message, buffer, len)) {
                    boost::shared_ptr<Base> message_ptr(new Base);
                    ros::serialization::IStream in(buffer.get(),len);
                    ros::serialization::deserialize(in, *message_ptr);
                    if (user_cb && ros::ok()) {
                        user_cb(message_ptr);
                    }
                }
			}


            BZ2Codec codec;
	};

} //namespace transport

#endif // BZ2_MESSAGE_TRANSPORT_SUBSCRIBER_H
