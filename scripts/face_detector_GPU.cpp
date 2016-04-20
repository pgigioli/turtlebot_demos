#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/gpu/gpu.hpp>

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;
using namespace cv::gpu;

static const std::string OPENCV_WINDOW = "Face detection with GPU support";
string face_cascade_name = "/home/ubuntu/haarcascades/GPU/haarcascade_frontalface_alt.xml";
//string profile_cascade_name = "/home/ubuntu/OpenCV-detection-models/haarcascades_GPU/haarcascade_profileface.xml";
//string eyes_cascade_name = "/home/ubuntu/OpenCV-detection-models/haarcascades_GPU/haarcascade_eye.xml";
CascadeClassifier_GPU face_cascade;
CascadeClassifier_GPU profile_cascade;
CascadeClassifier_GPU eyes_cascade;

class faceDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  faceDetector() : it_(nh_)
  {
	image_sub_ = it_.subscribe("/usb_cam/image_raw",1,
	  &faceDetector::callback, this);
	image_pub_ = it_.advertise("/face_detection_GPU", 1);

	cv::namedWindow(OPENCV_WINDOW);
  }

  ~faceDetector()
  {
	cv::destroyWindow(OPENCV_WINDOW);
  }

  //subscriber callback function
  void callback(const sensor_msgs::ImageConstPtr& msg)
  {
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
	if( !face_cascade.load( face_cascade_name ) ){ std::cout << "Error loading face cascade" << endl; };
  	// Leave out profile and eye detection for speed
	//if( !face_cascade.load( profile_cascade_name ) ){ std::cout << "Error loading profile cascade" << endl; };
	//if( !eyes_cascade.load( eyes_cascade_name ) ){ std::cout << "Error loading eyes cascade" << endl; };

	// Get image frame and apply classifier
	if (cv_ptr)
	{
	  long frmCnt = 0;
	  double totalT = 0.0;
	  double t = (double)getTickCount();
          //std::vector<Rect> faces;
	  GpuMat faces;
	  Mat frame_gray;

  	  cvtColor( cv_ptr->image, frame_gray, CV_BGR2GRAY );

	  GpuMat gray_gpu(frame_gray);
  	  equalizeHist( frame_gray, frame_gray );

          //-- Detect faces
  	  //face_cascade.detectMultiScale( gray_gpu, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
	  int detect_num = face_cascade.detectMultiScale(gray_gpu, faces, 1.1, 2, Size(30,30));

	  Mat obj_host;
	  faces.colRange(0, detect_num).download(obj_host);
	  Rect* cfaces = obj_host.ptr<Rect>();
	  Mat ROI_cropped;

	  //check how long it took
	  t=((double)getTickCount()-t)/getTickFrequency();
          totalT += t;
          frmCnt++;
          	for( size_t i = 0; i < detect_num; i++ )
          	{
            	  Point pt1 = Point(cfaces[i].x-10, cfaces[i].y-10);//cfaces[i].t1();
		  Size sz = cfaces[i].size();
		  Point pt2(pt1.x+sz.width+20, pt1.y+sz.height+20);
		  rectangle(cv_ptr->image, pt1, pt2, Scalar(255));
		  //ROI_cropped = cv_ptr->image(Rect(cfaces[i].x-10,cfaces[i].y-10,sz.width+20,sz.height+20));

		  //Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
		  //rectangle( cv_ptr->image, Point(faces[i].x, faces[i].y), Point(faces[i].x+faces[i].width,
		  //		faces[i].y+faces[i].height), Scalar(255,0,255), 4, 8, 0);
		  // Leave out eye detection
            	  //Mat faceROI = frame_gray( faces[i] );
            	  /*std::vector<Rect> eyes;

                  //-- In each face, detect eyes
            	  eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, Size(30, 30) );

           	  for( size_t j = 0; j < eyes.size(); j++ )
           	    {
            	      Point center( faces[i].x + eyes[j].x + eyes[j].width*0.5,
				faces[i].y + eyes[j].y + eyes[j].height*0.5 );
            	      int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
            	      circle( cv_ptr->image, center, radius, Scalar( 255, 0, 0 ), 4, 8, 0 );
           	    }*/
          	}
  	  //-- Show what you got
	  //if (ROI_cropped.empty() == 0)
	  //{
		//imshow("Face Window", ROI_cropped);
	  //}
  	  imshow( OPENCV_WINDOW, cv_ptr->image );
	  cv::waitKey(3);

	  image_pub_.publish(cv_ptr->toImageMsg());

	  cout << "fps: " << 1.0/(totalT/(double)frmCnt) << endl;
	 }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "face_detector_GPU");

  int gpuCnt = getCudaEnabledDeviceCount();
  if (gpuCnt==0)
  {
	cout << "no CUDA device found" << endl;
	return 0;
  }

  faceDetector fd;
  ros::spin();
  return 0;
}
