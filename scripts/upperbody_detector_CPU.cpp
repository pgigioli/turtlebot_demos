#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Upper body detection with CPU support";
string upperbody_cascade_name = "/home/ubuntu/haarcascades/haarcascade_upperbody.xml";
CascadeClassifier upperbody_cascade;

class upperbodyDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  upperbodyDetector() : it_(nh_)
  {
	image_sub_ = it_.subscribe("/usb_cam/image_raw",1,
	  &upperbodyDetector::callback, this);
	image_pub_ = it_.advertise("/upperbody_detection_CPU", 1);

	cv::namedWindow(OPENCV_WINDOW, WINDOW_NORMAL);
  }

  ~upperbodyDetector()
  {
	cv::destroyWindow(OPENCV_WINDOW);
  }

  //subscriber callback function
  void callback(const sensor_msgs::ImageConstPtr& msg)
  {
	double delay = (double)getTickCount();
        double totaldelay = 0.0;

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
	  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	  return;
	}

	// Load cascades
	if( !upperbody_cascade.load( upperbody_cascade_name ) ){ std::cout << "Error loading upper body cascade" << endl; };

	// Get image frame and apply classifier
	if (cv_ptr)
	{
	  long frmCnt = 0;
          double totalT = 0.0;
          double t = (double)getTickCount();

          std::vector<Rect> ubody;
	  Mat frame_gray;

  	  cvtColor( cv_ptr->image, frame_gray, CV_BGR2GRAY );
  	  equalizeHist( frame_gray, frame_gray );

          //-- Detect upper body
  	  upperbody_cascade.detectMultiScale( frame_gray, ubody, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

	  t=((double)getTickCount()-t)/getTickFrequency();
          totalT += t;
          frmCnt++;

          	for( size_t i = 0; i < ubody.size(); i++ )
          	{
            	  Point center( ubody[i].x + ubody[i].width*0.5, ubody[i].y + ubody[i].height*0.5 );
		  rectangle( cv_ptr->image, Point(ubody[i].x, ubody[i].y), Point(ubody[i].x+ubody[i].width,
				ubody[i].y+ubody[i].height), Scalar(255,0,255), 4, 8, 0);
            	  Mat faceROI = frame_gray( ubody[i] );
          	}
  	  //-- Show what you got
  	  imshow( OPENCV_WINDOW, cv_ptr->image );
	  cv::waitKey(3);

	  delay = ((double)getTickCount()-delay)/getTickFrequency();
          totaldelay += delay;

	  image_pub_.publish(cv_ptr->toImageMsg());

	  cout << "fps: " << 1.0/(totalT/(double)frmCnt) << endl;
//	  cout << "OpenCV delay: " << totaldelay << endl;
//          cout << " " << endl; 
	}
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "upperbody_detector_CPU");
  upperbodyDetector ubd;
  ros::spin();
  return 0;
}
