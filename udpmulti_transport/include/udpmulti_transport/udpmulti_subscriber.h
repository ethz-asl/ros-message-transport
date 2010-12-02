#ifndef UDPMULTI_MESSAGE_TRANSPORT_SUBSCRIBER_H
#define UDPMULTI_MESSAGE_TRANSPORT_SUBSCRIBER_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <string>

#include <message_transport/simple_subscriber_plugin.h>
#include <udpmulti_transport/UDPMultHeader.h>
#include <udpmulti_transport/udpmulti_publisher.h>

namespace udpmulti_transport {

	template <class Base>
	class UDPMultiSubscriber : public message_transport::SimpleSubscriberPlugin<Base,udpmulti_transport::UDPMultHeader>
	{
		public:
			UDPMultiSubscriber() : io_service_(), socket_(io_service_), user_cb_(NULL), rec_thread_(NULL) {
			}

			virtual ~UDPMultiSubscriber() {
				// close listener
                ROS_INFO("Shutting down UDPMultiSubscriber");
                io_service_.stop();
                if (rec_thread_) {
                    try {
                        socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_receive);
                    } catch (boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::system::system_error> > e) {
                        // ignore
                    }
                    rec_thread_->interrupt();
                    rec_thread_->join();
                    delete rec_thread_;
                }
                rec_thread_ = NULL;
			}

			virtual std::string getTransportName() const
			{
				return "udpmulti";
			}

		protected:
            void receiveThread() {
                while (ros::ok()) {
                    std::size_t rec;
                    // ROS_INFO("Waiting for datagram");
                    rec = socket_.receive_from(
                            boost::asio::buffer(data_, MAX_UDP_PACKET_SIZE), endpoint_);
                    // ROS_INFO("Received datagram: %d bytes",rec);
                    if (!rec) continue;

                    boost::shared_ptr<Base> message_ptr(new Base);
                    ros::serialization::IStream in(data_,rec);
                    ros::serialization::deserialize(in, *message_ptr);

                    if (user_cb_ && ros::ok()) {
                        (*user_cb_)(message_ptr);
                    }
                }
            }
			virtual void internalCallback(const udpmulti_transport::UDPMultHeaderConstPtr& message,
					const typename message_transport::SimpleSubscriberPlugin<Base,udpmulti_transport::UDPMultHeader>::Callback& user_cb)
			{
				user_cb_ = &user_cb;
				// Process the header, and get the listener started.
                if (listening_interface_.empty()) {
                    // First message, launch the initialisation
                    ros::NodeHandle & nh = this->nh();
                    nh.param<std::string>("listening_interface",listening_interface_,"0.0.0.0");
                    multicast_address_ = message->multicast_address;
                    port_ = message->port;
                    ROS_INFO("Listening on %s, address '%s:%d'",listening_interface_.c_str(),multicast_address_.c_str(),port_);

                    // Create the socket so that multiple may be bound to the same address.
                    boost::asio::ip::udp::endpoint listen_endpoint(
                            boost::asio::ip::address::from_string(listening_interface_), port_);
                    socket_.open(listen_endpoint.protocol());
                    socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
                    socket_.bind(listen_endpoint);

                    // Join the multicast group.
                    socket_.set_option(boost::asio::ip::multicast::join_group(
                                boost::asio::ip::address::from_string(multicast_address_)));
                    // Allow loopback
                    socket_.set_option(boost::asio::ip::multicast::enable_loopback(true));

#if 0
                    socket_.async_receive_from(
                            boost::asio::buffer(data_, MAX_UDP_PACKET_SIZE), endpoint_,
                            boost::bind(&UDPMultiSubscriber::handle_receive_from, this,
                                boost::asio::placeholders::error,
                                boost::asio::placeholders::bytes_transferred));
#else
                    rec_thread_ = new  boost::thread(&UDPMultiSubscriber::receiveThread,this);
#endif
                }
			}

#if 0
            void handle_receive_from(const boost::system::error_code& error,
                    size_t bytes_recvd)
            {
                ROS_INFO("Received datagram");
                if (!error)
                {
                    ROS_INFO("Accepted datagram");
                    boost::shared_ptr<Base> message_ptr(new Base);
                    ros::serialization::IStream in(data_,MAX_UDP_PACKET_SIZE);
                    ros::serialization::deserialize(in, *message_ptr);

                    (*user_cb_)(message_ptr);

                    socket_.async_receive_from(
                            boost::asio::buffer(data_, MAX_UDP_PACKET_SIZE), endpoint_,
                            boost::bind(&UDPMultiSubscriber::handle_receive_from, this,
                                boost::asio::placeholders::error,
                                boost::asio::placeholders::bytes_transferred));
                }
            }
#endif

			uint32_t port_;
			std::string listening_interface_;
            boost::asio::io_service io_service_;
            boost::asio::ip::udp::endpoint endpoint_;
            boost::asio::ip::udp::socket socket_;
			const typename message_transport::SimpleSubscriberPlugin<Base,udpmulti_transport::UDPMultHeader>::Callback* user_cb_;
            boost::thread *rec_thread_;
			std::string multicast_address_;
            uint8_t data_[MAX_UDP_PACKET_SIZE];
	};

} //namespace transport

#endif // UDPMULTI_MESSAGE_TRANSPORT_SUBSCRIBER_H
