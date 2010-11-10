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

#ifndef MESSAGE_TRANSPORT_SUBSCRIBER_H
#define MESSAGE_TRANSPORT_SUBSCRIBER_H

#include <ros/ros.h>
#include "message_transport/transport_hints.h"
#include "message_transport/subscriber_impl.h"

namespace message_transport {

	/**
	 * \brief Manages a subscription callback on a specific topic that can be interpreted
	 * as an Message topic.
	 *
	 * Subscriber is the client-side counterpart to Publisher. By loading the
	 * appropriate plugin, it can subscribe to a base image topic using any available
	 * transport. The complexity of what transport is actually used is hidden from the user,
	 * who sees only a normal Message callback.
	 *
	 * A Subscriber should always be created through a call to MessageTransport::subscribe(),
	 * or copied from one that was.
	 * Once all copies of a specific Subscriber go out of scope, the subscription callback
	 * associated with that handle will stop being called. Once all Subscriber for a given
	 * topic go out of scope the topic will be unsubscribed.
	 */
	class Subscriber
	{
		public:
			Subscriber() {}
			Subscriber(ros::NodeHandle& nh);

			std::string getTopic() const;

			/**
			 * \brief Returns the number of publishers this subscriber is connected to.
			 */
			uint32_t getNumPublishers() const;

			/**
			 * \brief Unsubscribe the callback associated with this Subscriber.
			 */
			void shutdown();

			operator void*() const;
			bool operator< (const Subscriber& rhs) const { return impl_ <  rhs.impl_; }
			bool operator!=(const Subscriber& rhs) const { return impl_ != rhs.impl_; }
			bool operator==(const Subscriber& rhs) const { return impl_ == rhs.impl_; }

			template <class M>
				int do_subscribe(ros::NodeHandle& nh, 
						const std::string & package_name, const std::string & class_name, 
						const std::string& base_topic, uint32_t queue_size,
						const boost::function<void(const typename M::ConstPtr&)>& callback,
						const ros::VoidPtr& tracked_object, const TransportHints& transport_hints)
				{
					// Tell plugin to subscribe.
					impl_.reset(new SubscriberImpl<M>(package_name,class_name));
					// Load the plugin for the chosen transport.
					impl_.get()->reset(transport_hints);

					// Try to catch if user passed in a transport-specific topic as base_topic.
					std::string clean_topic = ros::names::clean(base_topic);
					size_t found = clean_topic.rfind('/');
					if (found != std::string::npos) {
						std::string transport = clean_topic.substr(found+1);
						std::string plugin_name = SubscriberPluginGen::getLookupName(transport);
						std::vector<std::string> plugins = impl_->getDeclaredClasses();
						if (std::find(plugins.begin(), plugins.end(), plugin_name) != plugins.end()) {
							std::string real_base_topic = clean_topic.substr(0, found);
							ROS_WARN("[message_transport] It looks like you are trying to subscribe directly to a "
									"transport-specific image topic '%s', in which case you will likely get a connection "
									"error. Try subscribing to the base topic '%s' instead with parameter ~message_transport "
									"set to '%s' (on the command line, _message_transport:=%s). "
									"See http://ros.org/wiki/message_transport for details.",
									clean_topic.c_str(), real_base_topic.c_str(), transport.c_str(), transport.c_str());
						}
					}

					impl_->getTemplateSubscriber<M>()->subscribe(nh, base_topic, queue_size, callback, tracked_object, transport_hints);
					return 0;
				}

		private:

			typedef boost::shared_ptr<SubscriberImplGen> ImplPtr;

			ImplPtr impl_;

	};

} //namespace message_transport

#endif
