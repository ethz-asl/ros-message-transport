
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_transport/message_transport.h>

std::string transport;
unsigned int npoints = 0;
FILE * fp=NULL;

void callback(const sensor_msgs::ImageConstPtr& image)
{
	double tnow = ros::Time::now().toSec();
	double tstamp = image->header.stamp.toSec();
	if (!fp) {
		char fname[512];
		sprintf(fname,"received_%d_%s_%dx%dx%s.txt",
				getpid(),transport.c_str(),image->width,image->height,
				image->encoding.c_str());
        for (unsigned int i=0;i<strlen(fname);i++) {
            if (fname[i]=='/') fname[i] = '_';
        }
		ROS_INFO("Saving data in %s",fname);
		fp = fopen(fname,"w");
	}
    if (fp) {
        fprintf(fp,"%d %f %f %f\n",npoints,tnow,tstamp,tnow-tstamp);
    }
	npoints ++;

	ROS_INFO("%d: Image received at %f, delay %f",
			getpid(),tstamp,tnow-tstamp);

	if (npoints > 1000) {
        if (fp) {
            fclose(fp);
            fp = NULL;
        }
		ros::shutdown();
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_receiver", ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");
  message_transport::MessageTransport<sensor_msgs::Image> 
	  it(nh,"imagem_transport","sensor_msgs::Image");

  std::string pkgname("imagem_transport");
  transport = std::string((argc > 1) ? argv[1] : "imagem_transport/raw_img");
  if (transport.compare(0,pkgname.length(),pkgname)) {
      transport = pkgname + "/" + transport;
  }
  message_transport::Subscriber sub = it.subscribe("/image_source", 1, callback, 
		  transport);
  ROS_INFO("test_receiver started");

  ros::spin();
}

