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

Mat cv_ptr_copy;

void runYOLO(Mat full_frame)
{
   Mat input_frame = full_frame.clone();

   // run yolo and get bounding boxes for objects
//   ROS_box *boxes = demo_yolo(cfg, weights, thresh, cam_index, filename);

   // get the number of bounding boxes found
//   int num = boxes[0].num;
   int num = 0;

      // if at least one bbox found, define center point and draw box
   if (num > 0  && num <= 50) {
      vector<int> face_boxes;
      vector<int> person_boxes;
      int num_faces = 0;
      int num_person = 0;
      cout << "# Objects: " << num << endl;

      // split bounding boxes by class
 //     for (int i = 0; i < num; i++) {
//          switch (boxes[i].Class) {
//               case 0:
//                  face_boxes.push_back(boxes[i]);
//                  num_faces++;
//                  break;
//               case 1:
//                  person_boxes.push_back(boxes[i]);
//                  num_person++;
//                  break;
//            }
//         }

//         publishCroppedFaces(face_boxes, num_faces);

         // send message that an object has been detected
//         std_msgs::Int8 msg;
//         msg.data = 1;
//         found_object_pub.publish(msg);

         // get templates and search ROI rectangles for template matching
//         getTemplates(boxes, input_frame, num);

//         if (num_faces > 0) input_frame = drawBBoxes(input_frame, face_boxes, num_faces);
 //        if (num_person > 0) input_frame = drawBBoxes(input_frame, person_boxes, num_person);
  //    } else {
  //        std_msgs::Int8 msg;
  //        msg.data = 0;
   //       found_object_pub.publish(msg);
      }

      imshow("test", input_frame);
      cv::waitKey(3);
   }

void callback(const sensor_msgs::ImageConstPtr& msg)
{
   cout << "usb image received" << endl;

   cv_bridge::CvImagePtr cv_ptr;

   try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
   } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
   }

   if (cv_ptr) {
      cv_ptr_copy = cv_ptr->image.clone();
      runYOLO(cv_ptr_copy);
   }
   return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub;
  image_sub = it.subscribe("/usb_cam/image_raw",1,callback);
  ros::spin();
  return 0;
}
