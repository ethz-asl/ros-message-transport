

#include <cstdlib> 
#include <cstddef>
#include <cassert>
#include <utility>

#include <ros/ros.h>

#include "udpmulti_transport/UDPMultHeader.h"
#include "udpmulti_transport/UDPMultRegisterTopic.h"
#include "udpmulti_transport/udpmulti_publisher.h"

namespace udpmulti_transport {

	UDPMultiPublisherImpl::UDPMultiPublisherImpl() : 
        io_service_()
    {
        endpoint_ = NULL;
        socket_ = NULL;
	}

	UDPMultiPublisherImpl::~UDPMultiPublisherImpl()
	{
        if (socket_) delete socket_;
        if (endpoint_) delete endpoint_;
        endpoint_ = NULL;
        socket_ = NULL;
	}

    void UDPMultiPublisherImpl::initialise(const std::string & topicname) {
        // TODO: Add a port management mechanism

        ROS_INFO("Waiting for service '/udpmulti_manager/register_topic' to be ready");
        ros::service::waitForService("/udpmulti_manager/register_topic");
        ROS_INFO("Requesting UDP Multicast port and address");

        ros::ServiceClient requestPortClt = nh_.serviceClient<udpmulti_transport::UDPMultRegisterTopic>("/udpmulti_manager/register_topic");
        udpmulti_transport::UDPMultRegisterTopic srv;
        srv.request.topic = topicname;
        if (!requestPortClt.call(srv)) {
            ROS_ERROR("Failed to call service register_memory");
        }
        multicast_address_ = srv.response.multicast_address;
        port_ = srv.response.port;
        ROS_INFO("Creating multicast connection on '%s:%d'",multicast_address_.c_str(),port_);

        endpoint_ = new boost::asio::ip::udp::endpoint(
                boost::asio::ip::address::from_string(multicast_address_),port_);
        socket_= new boost::asio::ip::udp::socket(io_service_,endpoint_-> protocol());
        printf("Endpoint %p / Socket %p\n",endpoint_,socket_);
    }

	const std::string & UDPMultiPublisherImpl::getMulticastAddress() const {
		return multicast_address_;
	}

	uint32_t UDPMultiPublisherImpl::getPort() const {
		return port_;
	}

	UDPMultHeader UDPMultiPublisherImpl::getUDPHeader() const
	{
		UDPMultHeader header;
		header.multicast_address = multicast_address_;
		header.port = port_; 
		return header;
	}

} //namespace udpmulti_transport
