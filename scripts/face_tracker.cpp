#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <sstream>
#include <iostream>
#include <string>

using namespace std;

void callback(const geometry_msgs::Point::ConstPtr& msg)
{

  if (msg) {
  float center_x = msg->x;
  float center_y = msg->y;
  cout << "x: " << center_x << endl;
  cout << "y: " << center_y << endl;
  cout << " " << endl;

  float image_width;
  float image_height;
  ros::param::get("/usb_cam/image_width", image_width);
  ros::param::get("/usb_cam/image_height", image_height);

  float zone_x_min = (image_width * 0.5) - (image_width * 0.4 * 0.5);
  float zone_x_max = (image_width * 0.5) + (image_width * 0.4 * 0.5);
  float zone_y_min = (image_height * 0.5) - (image_height * 0.4 * 0.5);
  float zone_y_max = (image_height * 0.5) + (image_height * 0.4 * 0.5);

  float delay = 0.1;

  // Zone 1
  if (center_y < zone_y_min && center_x > zone_x_min && center_x < zone_x_max)
  {
	string command = "uvcdynctrl -s \"Tilt (relative)\" -- -320";
        system(command.c_str());
  }

  // Zone 2
  if (center_y > zone_y_max && center_x > zone_x_min && center_x < zone_x_max)
  {
	string command = "uvcdynctrl -s \"Tilt (relative)\" -- 320";
        system(command.c_str());
  }

  // Zone 3
  if (center_x < zone_x_min && center_y > zone_y_min && center_y < zone_y_max)
  {
	string command = "uvcdynctrl -s \"Pan (relative)\" -- 320";
        system(command.c_str());
  }

  // Zone 4
  if (center_x > zone_x_max && center_y > zone_y_min && center_y < zone_y_max)
  {
	string command = "uvcdynctrl -s \"Pan (relative)\" -- -320";
        system(command.c_str());
  }

  // Zone 5
  if (center_y < zone_y_min && center_x < zone_x_min)
  {
	string command1 = "uvcdynctrl -s \"Tilt (relative)\" -- -320";
	string command2 = "uvcdynctrl -s \"Pan (relative)\" -- 320";
	system(command1.c_str());
	ros::Duration(delay).sleep();
	system(command2.c_str());
	ros::Duration(delay).sleep();
  }

  // Zone 6
  if (center_y < zone_y_min && center_x > zone_x_max)
  {
	string command1 = "uvcdynctrl -s \"Tilt (relative)\" -- -320";
	string command2 = "uvcdynctrl -s \"Pan (relative)\" -- -320";
	system(command1.c_str());
        ros::Duration(delay).sleep();
        system(command2.c_str());
	ros::Duration(delay).sleep();
  }

  // Zone 7
  if (center_y > zone_y_max && center_x < zone_x_min)
  {
	string command1 = "uvcdynctrl -s \"Tilt (relative)\" -- 320";
	string command2 = "uvcdynctrl -s \"Pan (relative)\" -- 320";
	system(command1.c_str());
        ros::Duration(delay).sleep();
        system(command2.c_str());
	ros::Duration(delay).sleep();
  }

  //Zone 8
  if (center_y > zone_y_max && center_x > zone_x_max)
  {
	string command1 = "uvcdynctrl -s \"Tilt (relative)\" -- 320";
	string command2 = "uvcdynctrl -s \"Pan (relative)\" -- -320";
	system(command1.c_str());
        ros::Duration(delay).sleep();
        system(command2.c_str());
	ros::Duration(delay).sleep();
  }
  }
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "face_tracker");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("ROI_coordinate", 1, callback);
  ros::spin();
  return 0;
}
