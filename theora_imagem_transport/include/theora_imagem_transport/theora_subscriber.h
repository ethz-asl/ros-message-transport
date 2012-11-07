#ifndef IMAGEM_TRANSPORT_THEORA_SUBSCRIBER_H
#define IMAGEM_TRANSPORT_THEORA_SUBSCRIBER_H
#include <message_transport/simple_subscriber_plugin.h>
#include <dynamic_reconfigure/server.h>
#include <theora_imagem_transport/TheoraSubscriberConfig.h>
#include <theora_image_transport/Packet.h>
#include <cv_bridge/cv_bridge.h>

#include <theora/codec.h>
#include <theora/theoraenc.h>
#include <theora/theoradec.h>

namespace theora_imagem_transport {

    class TheoraSubscriber : public message_transport::SimpleSubscriberPlugin<sensor_msgs::Image,theora_image_transport::Packet>
    {
        public:
            TheoraSubscriber();
            virtual ~TheoraSubscriber();

            virtual std::string getTransportName() const
            {
                return "theora";
            }

        protected:
            //The function that does the actual decompression and calls a user supplied callback with the resulting image
            virtual void internalCallback(const theora_image_transport::Packet::ConstPtr &msg, 
                    const message_transport::SimpleSubscriberPlugin<sensor_msgs::Image,theora_image_transport::Packet>::Callback& user_cb);

            // Overridden to bump queue_size, otherwise we might lose headers
            // Overridden to tweak arguments and set up reconfigure server
			virtual void subscribeImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
					const typename SubscriberPlugin<sensor_msgs::Image>::Callback& callback, const ros::VoidPtr& tracked_object,
					const message_transport::TransportHints& transport_hints);

            // Dynamic reconfigure support
            typedef theora_imagem_transport::TheoraSubscriberConfig Config;
            typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
            boost::shared_ptr<ReconfigureServer> reconfigure_server_;
            int pplevel_; // Post-processing level

            void configCb(Config& config, uint32_t level);

            // Utility functions
            int updatePostProcessingLevel(int level);
            void msgToOggPacket(const theora_image_transport::Packet &msg, ogg_packet &ogg);

            bool received_header_;
            bool received_keyframe_;
            th_dec_ctx* decoding_context_;
            th_info header_info_;
            th_comment header_comment_;
            th_setup_info* setup_info_;
            sensor_msgs::ImagePtr latest_image_;
    };

} //namespace theora_imagem_transport

#endif // IMAGEM_TRANSPORT_THEORA_SUBSCRIBER_H
