#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_transport/message_transport.h>
#include <cv_bridge/cv_bridge.h>

class ImagePublisher {
	protected:

		ros::NodeHandle n_;
		message_transport::MessageTransport<sensor_msgs::Image> it_;
		// sensor_msgs::CvBridge bridge_;
        cv_bridge::CvImagePtr cv_ptr;
		message_transport::Publisher image_pub_;
		sensor_msgs::Image image;


	public:

		ImagePublisher(ros::NodeHandle &n) : n_(n), 
			it_(n_,"imagem_transport","sensor_msgs::Image") {
			image_pub_ = it_.advertise("/image_source",1);
            cv_ptr.reset(new cv_bridge::CvImage);
            cv_ptr->image.create(cvSize(1500,1000),CV_8UC3);
            cv_ptr->toImageMsg(image);
            image.encoding = "rgb8";
		}

		~ImagePublisher()
		{
		}

		int mainloop()
		{

			ros::Rate loop_rate(30);
			while (ros::ok())
			{
				image.header.stamp = ros::Time::now();
				image_pub_.publish(image);
				ROS_DEBUG("Published image at %f",image.header.stamp.toSec());
				ros::spinOnce();
				loop_rate.sleep();
			}

			return 0;
		}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_publisher");
	ros::NodeHandle n("~");
	ImagePublisher ic(n);
	ic.mainloop();
	return 0;
}

