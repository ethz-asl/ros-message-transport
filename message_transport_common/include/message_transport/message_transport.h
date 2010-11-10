#ifndef MESSAGE_TRANSPORT_MESSAGE_TRANSPORT_H
#define MESSAGE_TRANSPORT_MESSAGE_TRANSPORT_H

#include "message_transport/publisher.h"
#include "message_transport/subscriber.h"

namespace message_transport {

	/**
	 * \brief Advertise and subscribe to image topics.
	 *
	 * MessageTransport is analogous to ros::NodeHandle in that it contains advertise() and
	 * subscribe() functions for creating advertisements and subscriptions of image topics.
	 */
	template <class M>
		class MessageTransport
		{
			public:
				explicit MessageTransport(const ros::NodeHandle& nh, 
						const std::string & package_name,
						const std::string & class_name) 
					: nh_(nh), package_name_(package_name), class_name_(class_name) {
				}

				~MessageTransport() {
				}

				/*!
				 * \brief Advertise an image topic, simple version.
				 */
				Publisher advertise(const std::string& base_topic, uint32_t queue_size, bool latch = false)
				{
					return advertise(base_topic, queue_size, 
							typename SingleSubscriberPublisher<M>::StatusCB(),
							typename SingleSubscriberPublisher<M>::StatusCB(), 
							ros::VoidPtr(), latch);
				}

				/*!
				 * \brief Advertise an image topic with subcriber status callbacks.
				 */
				Publisher advertise(const std::string& base_topic, uint32_t queue_size,
						const typename SingleSubscriberPublisher<M>::StatusCB& connect_cb,
						const typename SingleSubscriberPublisher<M>::StatusCB& disconnect_cb = SingleSubscriberPublisher<M>::StatusCB(),
						const ros::VoidPtr& tracked_object = ros::VoidPtr(), bool latch = false)
				{
					Publisher p(nh_);
					p.do_initialise<M>(nh_, package_name_, class_name_,
							base_topic, queue_size, 
							connect_cb, disconnect_cb, tracked_object, latch);
					return p;
				}

				/**
				 * \brief Subscribe to an image topic, version for arbitrary boost::function object.
				 */
				Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
						const boost::function<void(const typename M::ConstPtr&)>& callback,
						const ros::VoidPtr& tracked_object = ros::VoidPtr(),
						const TransportHints& transport_hints = TransportHints())
				{
					Subscriber s(nh_);
					s.do_subscribe<M>(nh_, package_name_, class_name_,
							base_topic, queue_size, 
							callback, tracked_object, transport_hints);
					return s;
				}


				/**
				 * \brief Subscribe to an image topic, version for bare function.
				 */
				Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
						void(*fp)(const typename M::ConstPtr&),
						const TransportHints& transport_hints = TransportHints())
				{
					return subscribe(base_topic, queue_size,
							boost::function<void(const typename M::ConstPtr&)>(fp),
							ros::VoidPtr(), transport_hints);
				}

				/**
				 * \brief Subscribe to an image topic, version for class member function with bare pointer.
				 */
				template<class T>
					Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
							void(T::*fp)(const typename M::ConstPtr&), T* obj,
							const TransportHints& transport_hints = TransportHints())
					{
						return subscribe(base_topic, queue_size, boost::bind(fp, obj, _1), ros::VoidPtr(), transport_hints);
					}

				/**
				 * \brief Subscribe to an image topic, version for class member function with shared_ptr.
				 */
				template<class T>
					Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
							void(T::*fp)(const typename M::ConstPtr&),
							const boost::shared_ptr<T>& obj,
							const TransportHints& transport_hints = TransportHints())
					{
						return subscribe(base_topic, queue_size, boost::bind(fp, obj.get(), _1), obj, transport_hints);
					}

				/// @todo Implement shutdown() of all handles opened with this MessageTransport.
				//void shutdown();

			protected:
				ros::NodeHandle nh_;
				std::string package_name_;
				std::string class_name_;
		};

} //namespace message_transport

#endif
