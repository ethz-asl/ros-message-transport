#ifndef MESSAGE_TRANSPORT_SINGLE_SUBSCRIBER_PUBLISHER
#define MESSAGE_TRANSPORT_SINGLE_SUBSCRIBER_PUBLISHER

#include <ros/ros.h>
#include <boost/noncopyable.hpp>
#include <boost/function.hpp>

namespace message_transport {

	/**
	 * \brief Allows publication of an image to a single subscriber. Only available inside
	 * subscriber connection callbacks.
	 */
	template <class M>
		class SingleSubscriberPublisher : boost::noncopyable
	{
		public:
			typedef boost::function<uint32_t()> GetNumSubscribersFn;
			typedef boost::function<void(const M&)> PublishFn;
			typedef boost::function<void(const SingleSubscriberPublisher<M>&)> StatusCB;

			SingleSubscriberPublisher(const std::string& caller_id, const std::string& topic,
					const GetNumSubscribersFn& num_subscribers_fn,
					const PublishFn& publish_fn)
				: caller_id_(caller_id), topic_(topic),
				num_subscribers_fn_(num_subscribers_fn),
				publish_fn_(publish_fn) { }

			std::string getSubscriberName() const
			{
				return caller_id_;
			}

			std::string getTopic() const
			{
				return topic_;
			}

			uint32_t getNumSubscribers() const
			{
				return num_subscribers_fn_();
			}

			void publish(const M& message) const
			{
				publish_fn_(message);
			}

			void publish(const typename M::ConstPtr& message) const
			{
				publish_fn_(*message);
			}

		private:
			std::string caller_id_;
			std::string topic_;
			GetNumSubscribersFn num_subscribers_fn_;
			PublishFn publish_fn_;

			friend class Publisher; // to get publish_fn_ directly
			friend class PublisherImplGen; // to get publish_fn_ directly
	};


} //namespace message_transport

#endif
