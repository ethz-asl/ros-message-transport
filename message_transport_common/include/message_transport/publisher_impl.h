#ifndef MESSAGE_TRANSPORT_PUBLISHER_IMPL_H
#define MESSAGE_TRANSPORT_PUBLISHER_IMPL_H

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

#include "message_transport/publisher.h"
#include "message_transport/publisher_plugin.h"
#include <pluginlib/class_loader.h>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string/erase.hpp>

namespace message_transport {

	class PublisherImplGen
	{
		public:
			PublisherImplGen(const std::string & topic) : 
				base_topic_(topic), unadvertised_(false) { }

			virtual ~PublisherImplGen()
			{
			}

			virtual uint32_t getNumSubscribers() const = 0;
			uint32_t getNumSubscribersBindable() const {
				return this->getNumSubscribers();
			}

			std::string getTopic() const
			{
				return base_topic_;
			}

			bool isValid() const
			{
				return !unadvertised_;
			}

			void shutdown() {
				this->shutdownImpl();
			}

			virtual void shutdownImpl() = 0;

			template <class M>
				void subscriberCB(const SingleSubscriberPublisher<M>& plugin_pub,
						const typename SingleSubscriberPublisher<M>::StatusCB& user_cb)
				{
					SingleSubscriberPublisher<M> ssp(plugin_pub.getSubscriberName(), this->getTopic(),
							boost::bind(&PublisherImplGen::getNumSubscribersBindable, this),
							plugin_pub.publish_fn_);
					user_cb(ssp);
				}

			virtual std::vector<std::string> getDeclaredClasses() = 0;


			template <class M>
				void publish(const M& message) const;

			template <class M>
				void publish(const typename M::ConstPtr& message) const;

		protected :
			std::string base_topic_;
			bool unadvertised_;
			//double constructed_;
	};

	template <class M>
	class PublisherImpl : public PublisherImplGen
	{
		public :
			PublisherImpl(const std::string & topic, 
					const std::string & packageName,
					const std::string & className) : PublisherImplGen(topic),
			loader_(packageName, 
						std::string("message_transport::PublisherPlugin<")+className+">")
		{
		}

			virtual ~PublisherImpl() {
				shutdownImpl();
			}

			virtual uint32_t getNumSubscribers() const
			{
				uint32_t count = 0;
                for (unsigned int i=0;i<publishers_.size();i++) {
					count += publishers_[i]->getNumSubscribers();
                }
				return count;
			}

			virtual void shutdownImpl()
			{
				if (!unadvertised_) {
					unadvertised_ = true;
                    for (unsigned int i=0;i<publishers_.size();i++) {
						publishers_[i]->shutdown();
                    }
					publishers_.clear();
				}
			}

			virtual std::vector<std::string> getDeclaredClasses() {
				return loader_.getDeclaredClasses();
			}

            boost::shared_ptr< PublisherPlugin<M> > addInstance(const std::string & name) {
#if ROS_VERSION_MINIMUM(1, 7, 0) // if current ros version is >= 1.7.0
                boost::shared_ptr< PublisherPlugin<M> > pub = loader_.createInstance(name);
#else
                boost::shared_ptr< PublisherPlugin<M> > pub(loader_.createClassInstance(name));
#endif
				publishers_.push_back(pub);
				return pub;
			}

			uint32_t getNumPublishers() const {
				return publishers_.size();
			}

			void publish(const M& message) const {
                for (unsigned int i=0;i<publishers_.size();i++) {
					if (publishers_[i]->getNumSubscribers() > 0) {
						publishers_[i]->publish(message);
					}
				}
			}

			void publish(const typename M::ConstPtr& message) const {
                for (unsigned int i=0;i<publishers_.size();i++) {
					if (publishers_[i]->getNumSubscribers() > 0) {
						publishers_[i]->publish(message);
					}
				}
			}
		protected:
			pluginlib::ClassLoader< PublisherPlugin<M> > loader_;
            std::vector<boost::shared_ptr< PublisherPlugin<M> > > publishers_;
	};

	template <class M>
		void PublisherImplGen::publish(const M& message) const
		{
			(dynamic_cast<const PublisherImpl<M>* const>(this))->publish(message);
		}

	template <class M>
		void PublisherImplGen::publish(const typename M::ConstPtr& message) const
		{
			(dynamic_cast<const PublisherImpl<M>* const>(this))->publish(message);
		}
};
#endif // MESSAGE_TRANSPORT_PUBLISHER_IMPL_H
