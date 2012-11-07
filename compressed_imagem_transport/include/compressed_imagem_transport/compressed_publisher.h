#ifndef IMAGEM_TRANSPORT_COMPRESSED_PUBLISHER_H
#define IMAGEM_TRANSPORT_COMPRESSED_PUBLISHER_H

#include <message_transport/simple_publisher_plugin.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <dynamic_reconfigure/server.h>
#include <compressed_imagem_transport/CompressedPublisherConfig.h>


namespace compressed_imagem_transport {

    class CompressedPublisher : public message_transport::SimplePublisherPlugin<sensor_msgs::Image,sensor_msgs::CompressedImage>
    {
        public:
            virtual ~CompressedPublisher() {}

            virtual std::string getTransportName() const
            {
                return "compressed";
            }

        protected:
            virtual void publish(const sensor_msgs::Image& message,
                    const message_transport::SimplePublisherPlugin<sensor_msgs::Image,sensor_msgs::CompressedImage>::PublishFn& publish_fn) const ;

            virtual void postAdvertiseInit() {
                // Set up reconfigure server for this topic
                reconfigure_server_ = boost::make_shared<ReconfigureServer>(this->getNodeHandle());
                ReconfigureServer::CallbackType f = boost::bind(&CompressedPublisher::configCb, this, _1, _2);
                reconfigure_server_->setCallback(f);
            }


            typedef compressed_imagem_transport::CompressedPublisherConfig Config;
            typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
            boost::shared_ptr<ReconfigureServer> reconfigure_server_;
            Config config_;

            void configCb(Config& config, uint32_t level);
    };

} //namespace compressed_imagem_transport

#endif // IMAGEM_TRANSPORT_COMPRESSED_PUBLISHER_H
