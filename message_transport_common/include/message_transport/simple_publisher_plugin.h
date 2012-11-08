#ifndef MESSAGE_TRANSPORT_SIMPLE_PUBLISHER_PLUGIN_H
#define MESSAGE_TRANSPORT_SIMPLE_PUBLISHER_PLUGIN_H

#include "message_transport/publisher_plugin.h"
#include <boost/scoped_ptr.hpp>

namespace message_transport {

	/**
	 * \brief Base class to simplify implementing most plugins to Publisher.
	 *
	 * This base class vastly simplifies implementing a PublisherPlugin in the common
	 * case that all communication with the matching SubscriberPlugin happens over a
	 * single ROS topic using a transport-specific message type. SimplePublisherPlugin
	 * is templated on the transport-specific message type.
	 *
	 * A subclass need implement only two methods:
	 * - getTransportName() from PublisherPlugin
	 * - publish() with an extra PublishFn argument
	 *
	 * For access to the parameter server and name remappings, use nh().
	 *
	 * getTopicToAdvertise() controls the name of the internal communication topic.
	 * It defaults to \<base topic\>/\<transport name\>.
	 */
	template <class Base,class M>
		 class SimplePublisherPlugin : public PublisherPlugin<Base>
	{
		public:
            SimplePublisherPlugin(bool forceLatch = false) : forcelatch_(forceLatch) {}
			virtual ~SimplePublisherPlugin() {}

			virtual uint32_t getNumSubscribers() const
			{
				if (simple_impl_) return simple_impl_->pub_.getNumSubscribers();
				return 0;
			}

			virtual std::string getTopic() const
			{
				if (simple_impl_) return simple_impl_->pub_.getTopic();
				return std::string();
			}

			virtual void publish(const Base& message) const
			{
				if (!simple_impl_ || !simple_impl_->pub_) {
					ROS_ASSERT_MSG(false, "Call to publish() on an invalid message_transport::SimplePublisherPlugin");
					return;
				}

				publish(message, bindInternalPublisher(simple_impl_->pub_));
			}

			virtual void shutdown()
			{
				if (simple_impl_) simple_impl_->pub_.shutdown();
			}

		protected:
			virtual void advertiseImpl(ros::NodeHandle & nh, const std::string& base_topic, uint32_t queue_size,
					const typename SingleSubscriberPublisher<Base>::StatusCB& user_connect_cb,
					const typename SingleSubscriberPublisher<Base>::StatusCB& user_disconnect_cb,
					const ros::VoidPtr& tracked_object, bool latch)
			{
				/// @todo This does not work if base_topic is a global name.
				ros::NodeHandle param_nh(nh, getTopicToAdvertise(base_topic));
				simple_impl_.reset(new SimplePublisherPluginImpl(nh,param_nh));
				simple_impl_->pub_ = nh.advertise<M>(getTopicToAdvertise(base_topic), queue_size,
						bindCB(user_connect_cb, &SimplePublisherPlugin::connectCallbackHandle),
						bindCB(user_disconnect_cb, &SimplePublisherPlugin::disconnectCallbackHandle),
						tracked_object, latch | forcelatch_);
				this->postAdvertiseInit();
			}

			//! Can be overloaded by an implementation to call some
			//initialisation function once the node has been initialised
			virtual void postAdvertiseInit() {
                // ROS_WARN("Calling default postAdvertiseInit");
            }

			//! Generic function for publishing the internal message type.
			typedef boost::function<void(const M&)> PublishFn;

			/**
			 * \brief Publish an image using the specified publish function. Must be implemented by
			 * the subclass.
			 *
			 * The PublishFn publishes the transport-specific message type. This indirection allows
			 * SimpleSubscriberPlugin to use this function for both normal broadcast publishing and
			 * single subscriber publishing (in subscription callbacks).
			 */
			virtual void publish(const Base& message, const PublishFn& publish_fn) const = 0;

			void publishInternal(const M& message) const {
				if (simple_impl_) simple_impl_->pub_.publish(message);
            }

			/**
			 * \brief Return the communication topic name for a given base topic.
			 *
			 * Defaults to \<base topic\>/\<transport name\>.
			 */
			virtual std::string getTopicToAdvertise(const std::string& base_topic) const
			{
				return base_topic + "/" + this->getTransportName();
			}

			/**
			 * \brief Function called when a subscriber connects to the internal publisher.
			 *
			 * Defaults to noop.
			 */
			virtual void connectCallback(const ros::SingleSubscriberPublisher& pub) {}
			void connectCallbackHandle(const ros::SingleSubscriberPublisher& pub) {
                this->connectCallback(pub);
            }

			/**
			 * \brief Function called when a subscriber disconnects from the internal publisher.
			 *
			 * Defaults to noop.
			 */
			virtual void disconnectCallback(const ros::SingleSubscriberPublisher& pub) {}
			void disconnectCallbackHandle(const ros::SingleSubscriberPublisher& pub) {
                this->disconnectCallback(pub);
            }

			/**
			 * \brief Returns the ros::NodeHandle to be used for parameter lookup.
			 */
			const ros::NodeHandle& getParamNode() const
			{
				return simple_impl_->param_nh_;
			}

			ros::NodeHandle& getNodeHandle() 
			{
				return simple_impl_->nh_;
			}

		private:
			struct SimplePublisherPluginImpl
			{
				SimplePublisherPluginImpl(ros::NodeHandle& nh, const ros::NodeHandle& param_nh)
					: nh_(nh), param_nh_(param_nh)
				{
				}

				ros::NodeHandle nh_;
				const ros::NodeHandle param_nh_;
				ros::Publisher pub_;
			};

			boost::scoped_ptr<SimplePublisherPluginImpl> simple_impl_;
            bool forcelatch_;

			typedef void (SimplePublisherPlugin::*SubscriberStatusMemFn)(const ros::SingleSubscriberPublisher& pub);

			/**
			 * Binds the user callback to subscriberCB(), which acts as an intermediary to expose
			 * a publish(Message) interface to the user while publishing to an internal topic.
			 */
			ros::SubscriberStatusCallback bindCB(const typename SingleSubscriberPublisher<Base>::StatusCB& user_cb,
					SubscriberStatusMemFn internal_cb_fn)
			{
				ros::SubscriberStatusCallback internal_cb = boost::bind(internal_cb_fn, this, _1);
				if (user_cb)
					return boost::bind(&SimplePublisherPlugin::subscriberCB, this, _1, user_cb, internal_cb);
				else
					return internal_cb;
			}

			/**
			 * Forms the ros::SingleSubscriberPublisher for the internal communication topic into
			 * an message_transport::SingleSubscriberPublisher for Message messages and passes it
			 * to the user subscriber status callback.
			 */
			void subscriberCB(const ros::SingleSubscriberPublisher& ros_ssp,
					const typename SingleSubscriberPublisher<Base>::StatusCB& user_cb,
					const ros::SubscriberStatusCallback& internal_cb)
			{
				// First call the internal callback (for sending setup headers, etc.)
				internal_cb(ros_ssp);

				// Construct a function object for publishing Base through the
				// subclass-implemented publish() using the ros::SingleSubscriberPublisher to send
				// messages of the transport-specific type.
				typedef void (SimplePublisherPlugin::*PublishMemFn)(const Base&, const PublishFn&) const;
				PublishMemFn pub_mem_fn = &SimplePublisherPlugin::publish;
				MessagePublishFn message_publish_fn = boost::bind(pub_mem_fn, this, _1, bindInternalPublisher(ros_ssp));

				SingleSubscriberPublisher<Base> ssp(ros_ssp.getSubscriberName(), getTopic(),
						boost::bind(&SimplePublisherPlugin::getNumSubscribers, this),
						message_publish_fn);
				user_cb(ssp);
			}

			typedef boost::function<void(const Base&)> MessagePublishFn;

			/**
			 * Returns a function object for publishing the transport-specific message type
			 * through some ROS publisher type.
			 *
			 * @param pub An object with method void publish(const M&)
			 */
			template <class PubT>
				PublishFn bindInternalPublisher(const PubT& pub) const
				{
					// Bind PubT::publish(const Message&) as PublishFn
					typedef void (PubT::*InternalPublishMemFn)(const M&) const;
					InternalPublishMemFn internal_pub_mem_fn = &PubT::publish;
					return boost::bind(internal_pub_mem_fn, &pub, _1);
				}
	};

} //namespace message_transport

#endif
