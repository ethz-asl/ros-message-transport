#ifndef MESSAGE_TRANSPORT_SUBSCRIBER_PLUGIN_H
#define MESSAGE_TRANSPORT_SUBSCRIBER_PLUGIN_H

#include <ros/ros.h>
#include <boost/function.hpp>
#include <boost/noncopyable.hpp>
#include "message_transport/transport_hints.h"

namespace message_transport {

	/**
	 * \brief Base class for plugins to Subscriber.
	 */
	class SubscriberPluginGen : boost::noncopyable
	{
		public:
			virtual ~SubscriberPluginGen() {}

			/**
			 * \brief Get a string identifier for the transport provided by
			 * this plugin.
			 */
			virtual std::string getTransportName() const = 0;

			/**
			 * \brief Get the transport-specific communication topic.
			 */
			virtual std::string getTopic() const = 0;

			/**
			 * \brief Returns the number of publishers this subscriber is connected to.
			 */
			virtual uint32_t getNumPublishers() const = 0;

			/**
			 * \brief Unsubscribe the callback associated with this SubscriberPlugin.
			 */
			virtual void shutdown() = 0;

			/**
			 * \brief Return the lookup name of the SubscriberPlugin associated with a specific
			 * transport identifier.
			 */
			static std::string getLookupName(const std::string& transport_type)
			{
				return transport_type + "_sub";
			}

		protected:
	};

	template <class M>
		class SubscriberPlugin : public SubscriberPluginGen
	{
		public:
			typedef boost::function<void(const typename M::ConstPtr&)> Callback;

			virtual ~SubscriberPlugin() {}

			/**
			 * \brief Subscribe to an image topic, version for arbitrary boost::function object.
			 */
			void subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
					const Callback& callback, const ros::VoidPtr& tracked_object = ros::VoidPtr(),
					const TransportHints& transport_hints = TransportHints())
			{
				return subscribeImpl(nh, base_topic, queue_size, callback, tracked_object, transport_hints);
			}

			/**
			 * \brief Subscribe to an image topic, version for bare function.
			 */
			void subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
					void(*fp)(const typename M::ConstPtr&),
					const TransportHints& transport_hints = TransportHints())
			{
				return subscribe(nh, base_topic, queue_size,
						boost::function<void(const typename M::ConstPtr&)>(fp),
						ros::VoidPtr(), transport_hints);
			}

			/**
			 * \brief Subscribe to an image topic, version for class member function with bare pointer.
			 */
			template<class T>
				void subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
						void(T::*fp)(const typename M::ConstPtr&), T* obj,
						const TransportHints& transport_hints = TransportHints())
				{
					return subscribe(nh, base_topic, queue_size, boost::bind(fp, obj, _1), ros::VoidPtr(), transport_hints);
				}

			/**
			 * \brief Subscribe to an image topic, version for class member function with shared_ptr.
			 */
			template<class T>
				void subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
						void(T::*fp)(const typename M::ConstPtr&),
						const boost::shared_ptr<T>& obj,
						const TransportHints& transport_hints = TransportHints())
				{
					return subscribe(nh, base_topic, queue_size, boost::bind(fp, obj.get(), _1), obj, transport_hints);
				}

		protected:
			/**
			 * \brief Subscribe to an image transport topic. Must be implemented by the subclass.
			 */
			virtual void subscribeImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
					const Callback& callback, const ros::VoidPtr& tracked_object,
					const TransportHints& transport_hints) = 0;
	};

} //namespace message_transport

#endif
