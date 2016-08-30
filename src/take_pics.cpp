#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

int ct = 0;

void callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;

  try {
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
  }

  if (cv_ptr)
  {
	stringstream ss;

	string name = "/media/ubuntu/face_pictures/";
	string type = ".jpg";
	string title = "pic";

	ss << name << title << ct << type;

	string filename = ss.str();
	ss.str("");

	imwrite(filename, cv_ptr->image);

	ct++;

	imshow("pictures", cv_ptr->image);
	waitKey(3);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "take_pics");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub;
  image_sub = it.subscribe("/usb_cam/image_raw",1,callback);
  ros::spin();
  return 0;
}
