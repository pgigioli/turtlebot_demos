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

using namespace std;
using namespace cv;
using namespace cv::gpu;

// define global variables
static const std::string OPENCV_WINDOW = "Face detection with GPU support";
static const std::string CROPPED_WINDOW = "Face cropped";
string face_cascade_name = "/home/ubuntu/haarcascades/GPU/haarcascade_frontalface_alt.xml";
CascadeClassifier_GPU face_cascade;
int ct = 0;
Mat ROI_cropped;
Mat cv_ptr_copy;
Point pt1;
Point pt2;
Size sz;
int waitForCascade = 1;
int tryCascade = 0;
int tryTemplateMatching = 0;
int TMsteps = 0;
Point searchROIpt1;
Point searchROIpt2;
Rect searchROI;
int frame_width;
int frame_height;

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

  //save a MAT image to the location /home/ubuntu/face_pictures
  int saveCroppedImg(Mat ROI_cropped, int ct, string title)
  {
	stringstream ss;

        string name = "/home/ubuntu/face_pictures/";
        string type = ".jpg";

        ss << name << title << ct << type;

        string filename = ss.str();
        ss.str("");

        imwrite(filename, ROI_cropped);

        ct++;
	return ct;
  }

  // publish the cropped ROI for a deep learning model
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

  // draw center zone for face tracker
  Mat drawCenterZone(Mat cv_ptr_copy)
  {
	float scale = 0.40;

        float zone_x_min = (frame_width * 0.5) - (frame_width * scale * 0.5);
        float zone_x_max = (frame_width * 0.5) + (frame_width * scale * 0.5);
        float zone_y_min = (frame_height * 0.5) - (frame_height * scale * 0.5);
        float zone_y_max = (frame_height * 0.5) + (frame_height * scale * 0.5);

        rectangle(cv_ptr_copy, Point(zone_x_min, zone_y_min),
                   Point(zone_x_max, zone_y_max), Scalar(255,0,255));
	return cv_ptr_copy;
  }

  // set the center point for n number of faces
  geometry_msgs::Point setCenterPoint(Rect* cfaces, int detect_num)
  {
	geometry_msgs::Point ROI_center;

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

  // set top left and bottom right corners of new ROI to draw face box
  void setROIcorners(float x1, float y1, float x2, float y2)
  {
	pt1 = Point(x1, y1);
	pt2 = Point(x2, y2);
  }

  // get a small piece of the face frame to use in template matching
  Mat getFaceTemplate(Mat ROI_cropped)
  {
	Size ROIsize = ROI_cropped.size();
	int ROIwidth = ROIsize.width;
	int ROIheight = ROIsize.height;
	int newFacex = ROIwidth * 0.25;
	int newFacey = ROIheight * 0.25;
	int newFaceWidth = ROIwidth * 0.5;
	int newFaceHeight = ROIheight * 0.5;

	Rect newFace = Rect(newFacex, newFacey, newFaceWidth, newFaceHeight);

	Mat faceTemplate = ROI_cropped(newFace);
	return faceTemplate;
  }

  // check if template matching is failing or not
  int checkThreshold(Mat matchingResult)
  {
	float threshold = 0.3;
	double minCheck, maxCheck;
	Point minLocCheck, maxLocCheck;
        minMaxLoc(matchingResult, &minCheck, &maxCheck, &minLocCheck, &maxLocCheck);
	if ( minCheck > threshold ) return 1;
	else return 0;
  }

  Rect setSearchROI(Mat ROI_cropped, Point pt1)
  {
	// define scale to increase ROI dimensions for the search ROI
	float scale_ratio = 0.5;
	int widthIncrease = scale_ratio*ROI_cropped.cols;
	int heightIncrease = scale_ratio*ROI_cropped.rows;

        // define new search ROI box corners
        searchROIpt1 = Point(pt1.x - widthIncrease*0.5, pt1.y - heightIncrease*0.5);
        searchROIpt2 = Point(searchROIpt1.x + ROI_cropped.cols + widthIncrease,
                                searchROIpt1.y + ROI_cropped.rows + heightIncrease);

	// redefine searchROI coordinates if they are outside frame dimensions
        if (searchROIpt1.x < 0) { searchROIpt1.x = 0; }
        if (searchROIpt1.y < 0) { searchROIpt1.y = 0; }
        if (searchROIpt1.x > frame_width) { searchROIpt1.x = frame_width; }
        if (searchROIpt1.y > frame_height) { searchROIpt1.y = frame_height; }

        if (searchROIpt2.x < 0) { searchROIpt2.x = 0; }
        if (searchROIpt2.y < 0) { searchROIpt2.y = 0; }
        if (searchROIpt2.x > frame_width) { searchROIpt2.x = frame_width; }
        if (searchROIpt2.y > frame_height) { searchROIpt2.y = frame_height; }

	// define search ROI dimensions and draw rectangle on frame
	searchROI = Rect(searchROIpt1.x, searchROIpt1.y,
             searchROIpt2.x - searchROIpt1.x, searchROIpt2.y - searchROIpt1.y);
	rectangle(cv_ptr_copy, searchROIpt1, searchROIpt2, Scalar(0,255,0), 1, 8, 0);
	return searchROI;
  }

  // template matching method
  void templateMatching(Mat ROI_cropped, Mat cv_ptr_copy, Point pt1)
  {
	// create copy of full frame
	Mat image = cv_ptr_copy;

	// run template matching TMsteps times and then flash cascade
	if (TMsteps < 10 && pt1.x > 0 && pt1.y > 0)
        {
	  // call function to set search ROI dimensions
	  searchROI = setSearchROI(ROI_cropped, pt1);

	  // cut out search ROI image from full frame
	  Mat searchImage = image(searchROI);

	  // call function to define face template as a small piece of the original ROI
	  Mat faceTemplate = getFaceTemplate(ROI_cropped);

	  // create blank Mat array to place results
	  Mat matchingResult;

	  // run template matching functions
	  matchTemplate(searchImage, faceTemplate, matchingResult, CV_TM_SQDIFF_NORMED);

	  // call function to determine any significant matches were found
	  int TMfailure = checkThreshold(matchingResult);

	  // if no good matches were found, turn off template matching and wait for
	  // cascade classifier
	  if (TMfailure == 1)
	  {
	    waitForCascade = 1;
	    return;
	  }

	  // normalize results between 0 and 1
	  normalize(matchingResult, matchingResult, 0, 1, NORM_MINMAX, -1, Mat());

	  // determine min and max value and find the locations of each
	  double min, max;
	  Point minLoc, maxLoc;
	  minMaxLoc(matchingResult, &min, &max, &minLoc, &maxLoc);

	  // adjust new ROI coordinates to full frame
	  minLoc.x += searchROI.x - faceTemplate.cols*0.5;
	  minLoc.y += searchROI.y - faceTemplate.rows*0.5;

	  // add smoother to ROI movement.  If the new ROI coordinates move too much
	  // limit their new location to a small buffer
	  if ((minLoc.x - pt1.x) > 20) {minLoc.x = pt1.x + 20;}
	  if ((minLoc.y - pt1.y) > 20) {minLoc.y = pt1.y + 20;}

	  // draw new boundary around estimate face position
	  Point vertex1 = Point(minLoc.x, minLoc.y);
	  Point vertex2 = Point(minLoc.x + ROI_cropped.cols, minLoc.y + ROI_cropped.rows);
	  rectangle(cv_ptr_copy, vertex1, vertex2, Scalar(255));

	  // draw center zone for face tracker
	  drawCenterZone(cv_ptr_copy);

	  // define and draw center point of new ROI for face tracker
	  Point templateMatchingCenter = Point(vertex1.x + ROI_cropped.cols*0.5,
						vertex1.y + ROI_cropped.rows*0.5);
	  geometry_msgs::Point faceTrackingPoint;
	  faceTrackingPoint.x = templateMatchingCenter.x;
	  faceTrackingPoint.y = templateMatchingCenter.y;

	  circle(cv_ptr_copy, templateMatchingCenter, 2, Scalar(255,0,255), 2, 8, 0 );

	  // publish center point for the face tracker
	  ROI_coordinate_pub_.publish(faceTrackingPoint);

	  // show new frame with annotations
	  imshow( OPENCV_WINDOW, cv_ptr_copy );
          cv::waitKey(3);

	  // set new ROI corners as global values
	  setROIcorners(vertex1.x, vertex1.y, vertex2.x, vertex2.y);

	  // increment template matching interval
	  TMsteps++;
	}
	else
	{
	  // turn off template matching and try cascade classifier
	  TMsteps = 0;
	  tryTemplateMatching = 0;
	  tryCascade = 1;
	}
  }

  // cascade classifier method
  void cascadeClassifier(Mat cv_ptr_copy)
  {
          // convert frame into grayscale and detect the number of faces identified
	  GpuMat faces;
          Mat frame_gray;

          cvtColor( cv_ptr_copy, frame_gray, CV_BGR2GRAY );

          GpuMat gray_gpu(frame_gray);
          equalizeHist( frame_gray, frame_gray );

          int detect_num = face_cascade.detectMultiScale(gray_gpu, faces, 1.1, 2, Size(30,30));

	  // if no faces detected, either keep trying or switch to template matching
	  if (detect_num == 0)
	  {
	    if (waitForCascade == 1)
	    {
		tryCascade = 1;
		tryTemplateMatching = 0;
	    }
	    else
	    {
		tryCascade = 0;
		tryTemplateMatching = 1;
	    }
	  }
	  // if a face is detected, keep trying for the next frame
	  else
	  {
	    waitForCascade = 0;
	    tryCascade = 1;
	    tryTemplateMatching = 0;
	  }

	  // retrieve detected faces and place dimensions in cfaces
          Mat obj_host;
          faces.colRange(0, detect_num).download(obj_host);
          Rect* cfaces = obj_host.ptr<Rect>();

          //loop to draw ROI boxes
          for( size_t i = 0; i < detect_num; i++ )
          {
	    pt1 = Point(cfaces[i].x, cfaces[i].y);
            sz = cfaces[i].size();
            pt2 = Point(pt1.x+sz.width, pt1.y+sz.height);

	    // define ROI box and draw on frame
            ROI_cropped = cv_ptr_copy(Rect(pt1.x, pt1.y, sz.width, sz.height));

            rectangle(cv_ptr_copy, pt1, pt2, Scalar(255));

	    // save cropped image, publish ROI frame for deep learning node, and publish
	    // center for face tracker
            if (ROI_cropped.empty() == false)
            {
		// call function to save cropped image
		//ct = saveCroppedImg(ROI_cropped, ct, "cropped_");

		//imshow(CROPPED_WINDOW, ROI_cropped);
                //cv::waitKey(3);

		// call function to publish the cropped ROI for deep learning
                publishCroppedImg(ROI_cropped);

		// handle multiple faces by placing center point at mean of all ROIs
		geometry_msgs::Point ROI_center = setCenterPoint(cfaces, detect_num);

                Point face_center(ROI_center.x, ROI_center.y);
                circle(cv_ptr_copy, face_center, 2, Scalar(255,0,255), 2, 8, 0 );
            	ROI_coordinate_pub_.publish(ROI_center);
	    }

            // define center zone for face tracking
            drawCenterZone(cv_ptr_copy);
	  }

	  // display all annotations
          imshow( OPENCV_WINDOW, cv_ptr_copy );
          cv::waitKey(3);
  }

  //subscriber callback function
  void callback(const sensor_msgs::ImageConstPtr& msg)
  {
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

	// Get image frame and apply classifier
	if (cv_ptr)
	{
	  //start timer
	  long frmCnt = 0;
	  double totalT = 0.0;
	  double t = (double)getTickCount();

	  //create a copy of the image for the cropped window
	  cv_ptr_copy = cv_ptr->image.clone();

	  //int a = saveCroppedImg(cv_ptr_copy, ct, "full_image");

	  // if waiting for cascade to detect face, turn off template matching and keep trying
	  // cascade
	  if (waitForCascade == 1)
	  {
	    tryCascade = 1;
	    tryTemplateMatching = 0;
	  }

	  // call cascade face classifier
	  if (tryCascade == 1) { cascadeClassifier(cv_ptr_copy); }

	  //check how long it took for cascade classifier
	  t=((double)getTickCount()-t)/getTickFrequency();
          totalT += t;
          frmCnt++;

	  // call template matching
	  if (tryTemplateMatching == 1) { templateMatching(ROI_cropped, cv_ptr_copy, pt1); }

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

  // Load cascade
  if( !face_cascade.load( face_cascade_name ) ){ std::cout << "Error loading face cascade" << endl; };

  // get frame dimensions from ROS
  ros::param::get("/usb_cam/image_width", frame_width);
  ros::param::get("/usb_cam/image_height", frame_height);

  // call face detector class
  faceDetector fd;
  ros::spin();
  return 0;
}
