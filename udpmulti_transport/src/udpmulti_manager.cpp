
#include <ros/ros.h>
#include <boost/thread.hpp>

#include "udpmulti_transport/UDPMultRegisterTopic.h"
#include "udpmulti_transport/UDPMultClearAll.h"
#include "udpmulti_transport/UDPMultGetTopicList.h"


typedef std::map< std::string, unsigned int ,std::less<std::string> > TopicPortMap;
unsigned int availablePort = 1024;
std::string multicast_address = "239.255.0.1";
TopicPortMap registeredTopic;
boost::mutex main_mutex;


bool register_topic(udpmulti_transport::UDPMultRegisterTopic::Request &req, 
		udpmulti_transport::UDPMultRegisterTopic::Response &res)
{
	boost::lock_guard<boost::mutex> guard(main_mutex);//auto-lock unlock, even on exception
	TopicPortMap::iterator it = registeredTopic.find(req.topic);
    if (it != registeredTopic.end()) {
        res.multicast_address = multicast_address;
        res.port = it->second;
    } else {
        res.multicast_address = multicast_address;
        res.port = availablePort++;
        registeredTopic.insert(std::pair<std::string,unsigned int>(req.topic,res.port));
    }
	ROS_INFO("Registered port %d for topic %s", res.port,req.topic.c_str());
	
	return true;
}

bool clear_all_topics(udpmulti_transport::UDPMultClearAll::Request &req, 
		udpmulti_transport::UDPMultClearAll::Response &res)
{
	boost::lock_guard<boost::mutex> guard(main_mutex);//auto-lock unlock, even on exception
    registeredTopic.clear();
    availablePort = 1024;
	ROS_INFO("Deleted all topic-port association");
	res.result = 0;
	return true;
}

bool get_topic_list(udpmulti_transport::UDPMultGetTopicList::Request &req, 
		udpmulti_transport::UDPMultGetTopicList::Response &res)
{
	boost::lock_guard<boost::mutex> guard(main_mutex);//auto-lock unlock, even on exception
    TopicPortMap::const_iterator it;
    unsigned int i=0;
    res.multicast_address = multicast_address;
    res.topics.resize(registeredTopic.size());
    for (it=registeredTopic.begin();it!=registeredTopic.end();it++,i++) {
        udpmulti_transport::UDPMultTopic tpc;
        tpc.topic = it->first;
        tpc.port = it->second;
        res.topics[i] = tpc;
    }
	return true;
}


int main(int argc,char *argv[])
{
	ros::init(argc, argv, "udpmulti_manager");
	ros::NodeHandle n("udpmulti_manager");
    n.param<std::string>("multicast_address",multicast_address,"239.255.0.1");

   ros::ServiceServer regTopicSrv = n.advertiseService("register_topic",register_topic);
   ros::ServiceServer clearAllSrv = n.advertiseService("clear_all_topics",clear_all_topics);
   ros::ServiceServer getListSrv = n.advertiseService("get_topics",get_topic_list);
   ROS_INFO("udpmulti_manager started");

   ros::spin();

}



