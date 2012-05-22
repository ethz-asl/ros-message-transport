#ifndef MESSAGE_TRANSPORT_SUBSCRIBER_IMPL_H
#define MESSAGE_TRANSPORT_SUBSCRIBER_IMPL_H

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

#include "message_transport/subscriber_plugin.h"
#include <pluginlib/class_loader.h>
#include <boost/scoped_ptr.hpp>

namespace message_transport {

	class  SubscriberImplGen
	{
		public :
			SubscriberImplGen() : unsubscribed_(false) { }

			~SubscriberImplGen()
			{
			}

			bool isValid() const
			{
				return !unsubscribed_;
			}

			virtual void reset(const TransportHints& transport_hints) = 0;
			void shutdown() {
				this->shutdownImpl();
			}
			virtual void shutdownImpl() = 0;
			virtual boost::shared_ptr< SubscriberPluginGen > getSubscriber() = 0;

			template <class M> 
				boost::shared_ptr< SubscriberPlugin<M> > getTemplateSubscriber() {
                    return boost::dynamic_pointer_cast< SubscriberPlugin<M> >(getSubscriber());
				}


			virtual std::vector<std::string> getDeclaredClasses() = 0;
		protected :

			bool unsubscribed_;
	};

	template <class M>
		class  SubscriberImpl : public SubscriberImplGen
	{
		public:
			SubscriberImpl(const std::string & packageName,const std::string & className)
				: loader_(packageName, 
						std::string("message_transport::SubscriberPlugin<")+className+">") { }

			~SubscriberImpl() {
				shutdownImpl();
			}

			virtual void shutdownImpl()
			{
				if (!unsubscribed_) {
					unsubscribed_ = true;
					subscriber_->shutdown();
				}
			}

			virtual void reset(const TransportHints& transport_hints) {
				std::string lookup_name = SubscriberPluginGen::getLookupName(transport_hints.getTransport());
#if ROS_VERSION_MINIMUM(1, 7, 0) // if current ros version is >= 1.7.0
				subscriber_ = loader_.createInstance(lookup_name);
#else
				subscriber_.reset(loader_.createClassInstance(lookup_name));
#endif
			}

			virtual boost::shared_ptr< SubscriberPluginGen > getSubscriber() {
				return subscriber_;
			}

			virtual std::vector<std::string> getDeclaredClasses() {
				return loader_.getDeclaredClasses();
			}

		protected:

			pluginlib::ClassLoader< SubscriberPlugin<M> > loader_;
            boost::shared_ptr< SubscriberPlugin<M> > subscriber_;
	};
};

#endif // MESSAGE_TRANSPORT_SUBSCRIBER_IMPL_H
