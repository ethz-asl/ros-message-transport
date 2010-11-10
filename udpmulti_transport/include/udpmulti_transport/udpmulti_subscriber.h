#ifndef UDPMULTI_MESSAGE_TRANSPORT_SUBSCRIBER_H
#define UDPMULTI_MESSAGE_TRANSPORT_SUBSCRIBER_H


#include <message_transport/simple_subscriber_plugin.h>
#include <udpmulti_transport/UDPMultiHeader.h>

namespace udpmulti_transport {

	template <class Base>
	class UDPMultiSubscriber : public message_transport::SimpleSubscriberPlugin<Base,udpmulti_transport::UDPMultiHeader>
	{
		public:
			UDPMultiSubscriber() {
				// Initialise listener but do not connect it
			}

			virtual ~UDPMultiSubscriber() {
				// close listener
			}

			virtual std::string getTransportName() const
			{
				return "udpmulti";
			}

		protected:
			virtual void internalCallback(const udpmulti_transport::UDPMultiHeaderConstPtr& message,
					const typename message_transport::SimpleSubscriberPlugin<Base,udpmulti_transport::UDPMultiHeader>::Callback& user_cb)
			{
				user_cb_ = user_cb;
				data_length_ = message.datasize;
				// Process the header, and get the listener started.
			}

			void udp_packet_callback(SomeBuffer buffer) {

				boost::shared_ptr<Base> message_ptr(new Base);
				ros::serialization::IStream in(buffer,data_length_);
				ros::serialization::deserialize(in, *message_ptr);

				user_cb_(message_ptr);
			}

			const typename message_transport::SimpleSubscriberPlugin<Base,udpmulti_transport::UDPMultiHeader>::Callback& user_cb_;
			uint32_t data_length_;
	};

} //namespace transport

#endif // UDPMULTI_MESSAGE_TRANSPORT_SUBSCRIBER_H
