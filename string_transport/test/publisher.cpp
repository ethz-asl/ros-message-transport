#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/String.h>
#include <message_transport/message_transport.h>

class StringPublisher {
	protected:

		ros::NodeHandle n_;
		message_transport::MessageTransport<std_msgs::String> it_;
		message_transport::Publisher strmsg_pub_;
		std_msgs::String text;


	public:

		StringPublisher(ros::NodeHandle &n) : n_(n), 
			it_(n_,"string_transport","std_msgs::String") {
			strmsg_pub_ = it_.advertise("str_source",1);

		}

		~StringPublisher()
		{
		}

		int mainloop()
		{

            ROS_INFO("Entering main loop");
			ros::Rate loop_rate(3);
			while (ros::ok())
			{
                text.data = "Hello";
                ROS_INFO("Publishing '%s'",text.data.c_str());
				strmsg_pub_.publish(text);
				ros::spinOnce();
				loop_rate.sleep();
			}

			return 0;
		}

};

void setDebugLevel(const std::string & logname) {
    log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(logname);
    logger->setLevel(log4cxx::Level::getDebug());
    ros::console::notifyLoggerLevelsChanged();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_publisher");
	ros::NodeHandle n;
	StringPublisher ic(n);
	ic.mainloop();
	return 0;
}

