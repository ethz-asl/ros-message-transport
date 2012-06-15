#ifndef THROTTLED_TRANSPORT_PUBLISHER_H
#define THROTTLED_TRANSPORT_PUBLISHER_H


#include <message_transport/simple_publisher_plugin.h>
#include <throttled_transport/throttler.h>

namespace throttled_transport {

	template <class Base>
	class ThrottledPublisher : 
		public message_transport::SimplePublisherPlugin<Base,Base>
	{
		public:
			ThrottledPublisher() : 
                message_transport::SimplePublisherPlugin<Base,Base>(){}
			virtual ~ThrottledPublisher() {}

			virtual std::string getTransportName() const
			{
				return "throttled";
			}
		protected:
			virtual void publish(const Base& message,
					const typename message_transport::SimplePublisherPlugin<Base,Base>::PublishFn& publish_fn) const {
				
                size_t datasize = ros::serialization::serializationLength(message);
                if (throttler.can_publish(datasize)) {
                    publish_fn(message);
                }
			}

            mutable Throttler throttler;
	};


} //namespace throttled_transport


#endif // THROTTLED_TRANSPORT_PUBLISHER_H
