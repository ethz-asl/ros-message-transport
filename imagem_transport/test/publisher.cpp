#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_transport/message_transport.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

class ImagePublisher {
	protected:

		ros::NodeHandle n_;
		message_transport::MessageTransport<sensor_msgs::Image> it_;
		sensor_msgs::CvBridge bridge_;
		message_transport::Publisher image_pub_;
		IplImage *cv_image;
		sensor_msgs::Image image;


	public:

		ImagePublisher(ros::NodeHandle &n) : n_(n), 
			it_(n_,"imagem_transport","sensor_msgs::Image") {
			image_pub_ = it_.advertise("image_source",1);
			// cv_image = cvCreateImage(cvSize(640,480),8,1);
			// image = *(bridge_.cvToImgMsg(cv_image, "mono8"));
			// cv_image = cvCreateImage(cvSize(640,480),8,3);
			cv_image = cvCreateImage(cvSize(1500,1000),8,3);
			// cv_image = cvCreateImage(cvSize(3000,2000),8,3);
			image = *(bridge_.cvToImgMsg(cv_image, "bgr8"));
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
	ros::NodeHandle n;
	ImagePublisher ic(n);
	ic.mainloop();
	return 0;
}

