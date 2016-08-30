#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <iostream>
#include <stdio.h>
#include <face_tracker/templMatch.h>

using namespace cv;
using namespace std;

int ct = 0;
int FRAME_W;
int FRAME_H;
int FRAME_AREA;

class takeFacePics
{
   ros::NodeHandle nh;
//   ros::Subscriber bboxes_sub;
   image_transport::ImageTransport it;
   image_transport::Subscriber image_sub;

public:
   takeFacePics() : it(nh)
   {
//      bboxes_sub = nh.subscribe("/face_bboxes", 1,
//                                  &takeFacePics::bboxesCallback, this);
      image_sub = it.subscribe("/usb_cam/image_raw", 1,
			        &takeFacePics::callback, this);

      namedWindow("Saved Pictures View", WINDOW_NORMAL);
   }

   ~takeFacePics()
   {
      destroyWindow("Saved Pictures View");
   }

private:
//   void bboxesCallback(const darknet_ros::bboxes msg)
// {
//   }

   void callback(const sensor_msgs::ImageConstPtr& msg)
   {
      cv_bridge::CvImagePtr cv_ptr;

      try {
         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      } catch (cv_bridge::Exception& e) {
         ROS_ERROR("cv_bridge exception: %s", e.what());
	 return;
      }

      if (ct >= 100 && ct < 4100)
      {
	 cout << "###### TAKING PICTURES ######" << endl;

         stringstream ss;

         string name = "/media/ubuntu/face_pictures/";
         string type = ".jpg";
         string title = "face_400cm_";

	 int idx = ct - 100;

         ss << name << title << idx << type;

         string filename = ss.str();
         ss.str("");

	 imwrite(filename, cv_ptr->image);

         imshow("Saved Pictures View", cv_ptr->image);
         waitKey(3);
      }
      if (ct >= 4100) { cout << "DONE" << endl; }

      ct++;
   }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "take_pics");

  ros::param::get("/usb_cam/image_width", FRAME_W);
  ros::param::get("/usb_cam/image_height", FRAME_H);
  FRAME_AREA = FRAME_W * FRAME_H;

  takeFacePics tfp;
  ros::spin();
  return 0;
}
