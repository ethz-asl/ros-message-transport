/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef MESSAGE_TRANSPORT_PUBLISHER_H
#define MESSAGE_TRANSPORT_PUBLISHER_H

#include <ros/ros.h>
#include "message_transport/publisher_impl.h"
#include "message_transport/single_subscriber_publisher.h"

namespace message_transport {

	/**
	 * \brief Manages advertisements of multiple transport options on an Message topic.
	 *
	 * Publisher is a drop-in replacement for ros::Publisher when publishing
	 * Message topics. In a minimally built environment, they behave the same; however,
	 * Publisher is extensible via plugins to publish alternative representations of
	 * the image on related subtopics. This is especially useful for limiting bandwidth and
	 * latency over a network connection, when you might (for example) use the theora plugin
	 * to transport the images as streamed video. All topics are published only on demand
	 * (i.e. if there are subscribers).
	 *
	 * A Publisher should always be created through a call to MessageTransport::advertise(),
	 * or copied from one that was.
	 * Once all copies of a specific Publisher go out of scope, any subscriber callbacks
	 * associated with that handle will stop being called. Once all Publisher for a
	 * given base topic go out of scope the topic (and all subtopics) will be unadvertised.
	 */
	class Publisher
	{
		public:
			Publisher() {}
			Publisher(ros::NodeHandle& nh);

			/*!
			 * \brief Returns the number of subscribers that are currently connected to
			 * this Publisher.
			 *
			 * Returns the total number of subscribers to all advertised topics.
			 */
			uint32_t getNumSubscribers() const;

			/*!
			 * \brief Returns the base topic of this Publisher.
			 */
			std::string getTopic() const;

			/*!
			 * \brief Publish an image on the topics associated with this Publisher.
			 */
			template <class M>
				void publish(const M& message) const
				{
					if (!impl_ || !impl_->isValid()) {
						ROS_ASSERT_MSG(false, "Call to publish() on an invalid message_transport::Publisher");
						return;
					}

					impl_->publish<M>(message);
				}

			/*!
			 * \brief Publish an image on the topics associated with this Publisher.
			 */
			template <class M>
				void publish(const typename M::ConstPtr& message) const
				{
					if (!impl_ || !impl_->isValid()) {
						ROS_ASSERT_MSG(false, "Call to publish() on an invalid message_transport::Publisher");
						return;
					}

					impl_->publish<M>(message);
				}

			/*!
			 * \brief Shutdown the advertisements associated with this Publisher.
			 */
			void shutdown();

			operator void*() const;
			bool operator< (const Publisher& rhs) const { return impl_ <  rhs.impl_; }
			bool operator!=(const Publisher& rhs) const { return impl_ != rhs.impl_; }
			bool operator==(const Publisher& rhs) const { return impl_ == rhs.impl_; }


			template <class M>
				void do_initialise(ros::NodeHandle& nh, 
						const std::string & package_name,  const std::string & class_name, 
						const std::string& base_topic, uint32_t queue_size,
						const typename SingleSubscriberPublisher<M>::StatusCB& connect_cb,
						const typename SingleSubscriberPublisher<M>::StatusCB& disconnect_cb,
						const ros::VoidPtr& tracked_object, bool latch)
				{
					assert(impl_ == NULL);
					PublisherImpl<M>* impl = new PublisherImpl<M>(base_topic,package_name, class_name);
					impl_.reset(impl);

					BOOST_FOREACH(const std::string& lookup_name, impl->getDeclaredClasses()) {
						try {
							PublisherPlugin<M>* pub = impl->addInstance(lookup_name);
							pub->advertise(nh, impl->getTopic(), queue_size, 
									rebindCB<M>(connect_cb),
									rebindCB<M>(disconnect_cb), 
									tracked_object, latch);
						}
						catch (const std::runtime_error& e) {
							ROS_DEBUG("Failed to load plugin %s, error string: %s",
									lookup_name.c_str(), e.what());
						}
					}

					if (impl->getNumPublishers() == 0) {
						throw std::runtime_error("No plugins found! Does `rospack plugins --attrib=plugin "
								"message_transport` find any packages?");
					}
				}

		private:

			template <class M>
				typename SingleSubscriberPublisher<M>::StatusCB rebindCB(const typename SingleSubscriberPublisher<M>::StatusCB& user_cb)
				{
					// Note: the subscriber callback must be bound to the internal Impl object, not
					// 'this'. Due to copying behavior the Impl object may outlive this.
					if (user_cb)
						return boost::bind(&PublisherImplGen::subscriberCB<M>, impl_, _1, user_cb);
					else
						return typename SingleSubscriberPublisher<M>::StatusCB();
				}

			typedef boost::shared_ptr<PublisherImplGen> ImplPtr;

			ImplPtr impl_;

	};

} //namespace message_transport

#endif
