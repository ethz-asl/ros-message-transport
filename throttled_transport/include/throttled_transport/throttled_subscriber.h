#ifndef THROTTLED_MESSAGE_TRANSPORT_SUBSCRIBER_H
#define THROTTLED_MESSAGE_TRANSPORT_SUBSCRIBER_H


#include <string>

#include <message_transport/simple_subscriber_plugin.h>
#include <message_transport/raw_subscriber.h>

namespace throttled_transport {

	template <class Base>
	class ThrottledSubscriber : public message_transport::SimpleSubscriberPlugin<Base,Base>
	{
		public:
			ThrottledSubscriber() { }

			virtual ~ThrottledSubscriber() {
			}

			virtual std::string getTransportName() const
			{
				return "throttled";
			}
        protected:
            virtual void internalCallback(const typename Base::ConstPtr& message, 
                    const typename message_transport::SimpleSubscriberPlugin<Base,Base>::Callback& user_cb)
            {
                user_cb(message);
            }
	};

} //namespace transport

#endif // THROTTLED_MESSAGE_TRANSPORT_SUBSCRIBER_H
