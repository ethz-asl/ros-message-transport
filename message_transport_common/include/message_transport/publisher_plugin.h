#ifndef MESSAGE_TRANSPORT_PUBLISHER_PLUGIN_H
#define MESSAGE_TRANSPORT_PUBLISHER_PLUGIN_H

#include <ros/ros.h>
#include "message_transport/single_subscriber_publisher.h"

namespace message_transport {

	/**
	 * \brief Base class for plugins to Publisher.
	 */
	class PublisherPluginGen : boost::noncopyable
	{
		public:
			virtual ~PublisherPluginGen() {}

			/**
			 * \brief Get a string identifier for the transport provided by
			 * this plugin.
			 */
			virtual std::string getTransportName() const = 0;

			/**
			 * \brief Advertise a topic, simple version.
			 */
			virtual void advertise(ros::NodeHandle & nh, const std::string& base_topic, uint32_t queue_size,
					bool latch = true) = 0;

			/**
			 * \brief Returns the number of subscribers that are currently connected to
			 * this PublisherPlugin.
			 */
			virtual uint32_t getNumSubscribers() const = 0;

			/**
			 * \brief Returns the communication topic that this PublisherPlugin will publish on.
			 */
			virtual std::string getTopic() const = 0;

			/**
			 * \brief Shutdown any advertisements associated with this PublisherPlugin.
			 */
			virtual void shutdown() = 0;

			/**
			 * \brief Return the lookup name of the PublisherPlugin associated with a specific
			 * transport identifier.
			 */
			static std::string getLookupName(const std::string& transport_name)
			{
				return transport_name + "_pub";
			}

		protected:
	};

	/**
	 * \brief Base class for plugins to Publisher.
	 */
	template <class M>
		class PublisherPlugin : public PublisherPluginGen
	{
		public:
			virtual ~PublisherPlugin() {}

			virtual void advertise(ros::NodeHandle & nh, const std::string& base_topic, uint32_t queue_size,
					bool latch = true) {
				advertiseImpl(nh,base_topic, queue_size, 
						typename message_transport::SingleSubscriberPublisher<M>::StatusCB(),
						typename message_transport::SingleSubscriberPublisher<M>::StatusCB(), ros::VoidPtr(), latch);

			}

			/**
			 * \brief Advertise a topic with subscriber status callbacks.
			 */

			void advertise(ros::NodeHandle & nh, const std::string& base_topic, uint32_t queue_size,
					const typename message_transport::SingleSubscriberPublisher<M>::StatusCB& connect_cb,
					const typename message_transport::SingleSubscriberPublisher<M>::StatusCB& disconnect_cb = SingleSubscriberPublisher<M>::StatusCB(),
					const ros::VoidPtr& tracked_object = ros::VoidPtr(), bool latch = true)
			{
				advertiseImpl(nh,base_topic, queue_size, 
						connect_cb, disconnect_cb, tracked_object, latch);
			}

			/**
			 * \brief Publish an image using the transport associated with this PublisherPlugin.
			 */
			virtual void publish(const M& message) const = 0;

			/**
			 * \brief Publish an image using the transport associated with this PublisherPlugin.
			 */
			virtual void publish(const typename M::ConstPtr& message) const
			{
				publish(*message);
			}

		protected:
			/**
			 * \brief Advertise a topic. Must be implemented by the subclass.
			 */
			virtual void advertiseImpl(ros::NodeHandle & nh, const std::string& base_topic, uint32_t queue_size,
					const typename message_transport::SingleSubscriberPublisher<M>::StatusCB& connect_cb,
					const typename message_transport::SingleSubscriberPublisher<M>::StatusCB& disconnect_cb,
					const ros::VoidPtr& tracked_object, bool latch) = 0;
	};

} //namespace message_transport

#endif
