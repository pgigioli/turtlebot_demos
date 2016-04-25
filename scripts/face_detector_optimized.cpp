#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <geometry_msgs/Point.h>

#include <iostream>
#include <stdio.h>
#include <typeinfo>

using namespace std;
using namespace cv;
using namespace cv::gpu;

static const std::string OPENCV_WINDOW = "Face detection with GPU support";
static const std::string CROPPED_WINDOW = "Face cropped";
string face_cascade_name = "/home/ubuntu/haarcascades/GPU/haarcascade_frontalface_alt.xml";
CascadeClassifier_GPU face_cascade;
int ct = 0;
int iter = 0;

class faceDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  ros::Publisher ROI_coordinate_pub_;

public:
  faceDetector() : it_(nh_)
  {
	image_sub_ = it_.subscribe("/usb_cam/image_raw",1,
	  &faceDetector::callback, this);
	image_pub_ = it_.advertise("/face_detection_optimized", 1);

	ROI_coordinate_pub_ = nh_.advertise<geometry_msgs::Point>("ROI_coordinate", 1);

	cv::namedWindow(OPENCV_WINDOW, WINDOW_NORMAL);
	//cv::namedWindow(CROPPED_WINDOW, WINDOW_NORMAL);
  }

  ~faceDetector()
  {
	cv::destroyWindow(OPENCV_WINDOW);
	//cv::destroyWindow(CROPPED_WINDOW);
  }

  int saveCroppedImg(Mat ROI_cropped, int ct)
  {
	stringstream ss;

        string name = "/home/ubuntu/face_pictures/cropped_";
        string type = ".jpg";

        ss << name << ct <<type;

        string filename = ss.str();
        ss.str("");

        imwrite(filename, ROI_cropped);

        ct++;
	return ct;
  }

  void publishCroppedImg(Mat ROI_cropped)
  {
	cv_bridge::CvImage img_bridge;
        sensor_msgs::Image img_msg;
        std_msgs::Header header;
        header.seq = ct;
        header.stamp = ros::Time::now();
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, ROI_cropped);
        img_bridge.toImageMsg(img_msg);
        image_pub_.publish(img_msg);
  }

  cv_bridge::CvImagePtr drawCenterZone(cv_bridge::CvImagePtr cv_ptr)
  {
	int image_width;
        int image_height;
        ros::param::get("/usb_cam/image_width", image_width);
        ros::param::get("/usb_cam/image_height", image_height);

        float zone_x_min = (image_width * 0.5) - (image_width * 0.25 * 0.5);
        float zone_x_max = (image_width * 0.5) + (image_width * 0.25 * 0.5);
        float zone_y_min = (image_height * 0.5) - (image_height * 0.25 * 0.5);
        float zone_y_max = (image_height * 0.5) + (image_height * 0.25 * 0.5);

        rectangle(cv_ptr->image, Point(zone_x_min, zone_y_min),
                   Point(zone_x_max, zone_y_max), Scalar(255,0,255));
	return cv_ptr;
  }

  geometry_msgs::Point setCenterPoint(float ROI_center_x, float ROI_center_y,
					Rect* cfaces, int detect_num)
  {
	geometry_msgs::Point ROI_center;
	ROI_center.x = ROI_center_x;
        ROI_center.y = ROI_center_y;

        int x_total = 0;
        int y_total = 0;
        int total_points = 1;

        //get the mean of all points
        for ( size_t i = 0; i < detect_num; i++ )
        {
              Size get_size = cfaces[i].size();
              x_total = x_total + cfaces[i].x + get_size.width*0.5;
              y_total = y_total + cfaces[i].y + get_size.height*0.5;
              total_points = i + 1;
        }

        ROI_center.x = x_total / total_points;
        ROI_center.y = y_total / total_points;
  	return ROI_center;
  }

  //subscriber callback function
  void callback(const sensor_msgs::ImageConstPtr& msg)
  {
	//iter++;
	//cout << iter << endl;

	double delay = (double)getTickCount();
        double totaldelay = 0.0;

	//convert image msg to Mat format
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

	// Get image frame and apply classifier
	if (cv_ptr)
	{
	  long frmCnt = 0;
	  double totalT = 0.0;
	  double t = (double)getTickCount();

	  //create a copy of the image for the cropped window
	  Mat cv_ptr_copy = cv_ptr->image.clone();

          //std::vector<Rect> faces;
	  GpuMat faces;
	  Mat frame_gray;

  	  cvtColor( cv_ptr->image, frame_gray, CV_BGR2GRAY );

	  GpuMat gray_gpu(frame_gray);
  	  equalizeHist( frame_gray, frame_gray );

	  //catch every iter'th frame and detect number of faces identified
	  if (iter == 0)
	  {
	  	iter = 0;
	  	int detect_num = face_cascade.detectMultiScale(gray_gpu, faces, 1.1, 2, Size(30,30));

	  	Mat obj_host;
	  	faces.colRange(0, detect_num).download(obj_host);
	  	Rect* cfaces = obj_host.ptr<Rect>();
	  	Mat ROI_cropped;

	  	//check how long it took
	  	t=((double)getTickCount()-t)/getTickFrequency();
          	totalT += t;
          	frmCnt++;

	  	float ROI_center_x;
	  	float ROI_center_y;
	  	int face_exists = 0;

		//loop to draw ROI boxes
          	for( size_t i = 0; i < detect_num; i++ )
          	{
		  Point pt1 = Point(cfaces[i].x, cfaces[i].y);
		  Size sz = cfaces[i].size();
		  Point pt2(pt1.x+sz.width, pt1.y+sz.height);

		  Mat ROI_cropped = cv_ptr_copy(Rect(pt1.x, pt1.y, sz.width, sz.height));

		  rectangle(cv_ptr->image, pt1, pt2, Scalar(255));

		  if (ROI_cropped.empty() == false)
  	          {
		    //ct = saveCroppedImg(ROI_cropped, ct);

            	    //imshow(CROPPED_WINDOW, ROI_cropped);
               	    //cv::waitKey(3);

		    publishCroppedImg(ROI_cropped);
         	  }

		  float ROI_center_x = cfaces[i].x + sz.width*0.5;
		  float ROI_center_y = cfaces[i].y + sz.height*0.5;

		  face_exists = 1;

		  // define center zone for face tracking
		  drawCenterZone(cv_ptr);
          	}

	  	// Handle multiple faces by placing the center point at the mean of all ROIs
	  	geometry_msgs::Point ROI_center = setCenterPoint(ROI_center_x, ROI_center_y,
								cfaces, detect_num);

	  	Point face_center(ROI_center.x, ROI_center.y);
	  	circle(cv_ptr->image, face_center, 2, Scalar(255,0,255), 2, 8, 0 );

	  	if (face_exists == 1)
	  	{
		  face_exists = 0;
		  ROI_coordinate_pub_.publish(ROI_center);
	  	}
	  }

	  //-- Show what you got
	  imshow( OPENCV_WINDOW, cv_ptr->image );
	  cv::waitKey(3);

	  //image_pub_.publish(cv_ptr->toImageMsg());

	  cout << "Face Detection fps: " << 1.0/(totalT/(double)frmCnt) << endl;
	}
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "face_detector_optimized");

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
