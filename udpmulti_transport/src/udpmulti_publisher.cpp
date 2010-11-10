

#include <cstdlib> 
#include <cstddef>
#include <cassert>
#include <utility>

#include <ros/ros.h>

#include "udpmulti_transport/udpmulti_publisher.h"

using namespace boost::interprocess;


namespace udpmulti_transport {

	UDPMultiPublisherImpl::UDPMultiPublisherImpl()
	{
		//TODO
	}

	UDPMultiPublisherImpl::~UDPMultiPublisherImpl()
	{
		//TODO
	}

	const std::string & getHostName() {
		return hostname_;
	}

	uint32_t getPort() const {
		return port_;
	}

	UDPMultHeader UDPMultiPublisherImpl::getUDPHeader() const
	{
		UDPMultHeader header;
		header.host = getHostName();
		header.port = getPort(); 
		header.datasize = datasize_;
		return header;
	}

	void UDPMultiPublisherImpl::multicast(const ros::Message& message) const
	{
		char buffer[MAX_UDP_PACKET_SIZE];
		ros::serialization::OStream out(buffer,datasize_);
		ros::serialization::serialize(out,message);

		// TODO
		sock.write(buffer,datasize_);
	}

} //namespace udpmulti_transport
