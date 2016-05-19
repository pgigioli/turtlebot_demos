#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64MultiArray.h"
#include <sstream>
#include <iostream>
#include <string>

using namespace std;

float image_width;
float image_height;
float pan_angle = 0;
float tilt_angle = 0;

// pan/tilt ranges. Pan range = 189 degress, Tilt range = 102 degrees
// 1 degree horizontally = 47.41 steps, 1 degree vertically = 37.65 steps
float pan_max = 4480/47.41; float pan_min = -4480/47.41;
float tilt_max = 1920/37.65; float tilt_min = -1920/37.65;
float panIncr = 237/47.41; // 5 degrees
float tiltIncr = 188/37.65; // 5 degrees
string tiltIncrString = "188";
string panIncrString = "237";

class faceTracker
{
  ros::NodeHandle nh;
  ros::Subscriber coord_sub;
  ros::Publisher cam_angles;

public:
  faceTracker()
  {
	this->resetPanTilt();
	coord_sub = nh.subscribe("/ROI_coordinate", 1, &faceTracker::callback, this);
	cam_angles = nh.advertise<std_msgs::Float64MultiArray>("/cam_angles", 1);
  }

  ~faceTracker()
  {
	this->resetPanTilt();
  }

private:
  void resetPanTilt()
  {
	float delay = 1.0;
	string command1 = "uvcdynctrl -s \"Tilt Reset\" 1";
	string command2 = "uvcdynctrl -s \"Pan Reset\" 1";
	system(command1.c_str());
	ros::Duration(delay).sleep();
	system(command2.c_str());
	ros::Duration(delay).sleep();
  }

  void publishCamAngles(float pan_angle, float tilt_angle)
  {
	std_msgs::Float64MultiArray anglesArray;
	anglesArray.data.clear();
	anglesArray.data.push_back(pan_angle);
	anglesArray.data.push_back(tilt_angle);

	cam_angles.publish(anglesArray);
  }

  void callback(const geometry_msgs::Point::ConstPtr& msg)
  {
  	if (msg) {
	float center_x = msg->x;
  	float center_y = msg->y;
	//cout << "x: " << center_x << endl;
  	//cout << "y: " << center_y << endl;
  	//cout << " " << endl;

  	float zone_x_min = (image_width * 0.5) - (image_width * 0.4 * 0.5);
  	float zone_x_max = (image_width * 0.5) + (image_width * 0.4 * 0.5);
  	float zone_y_min = (image_height * 0.5) - (image_height * 0.4 * 0.5);
  	float zone_y_max = (image_height * 0.5) + (image_height * 0.4 * 0.5);

  	float delay = 0.1;

  	// Zone 1
  	if (center_y < zone_y_min && center_x > zone_x_min && center_x < zone_x_max)
  	{
	  tilt_angle -= tiltIncr;
	  string command = "uvcdynctrl -s \"Tilt (relative)\" -- -" + tiltIncrString;
          system(command.c_str());
  	}

  	// Zone 2
  	if (center_y > zone_y_max && center_x > zone_x_min && center_x < zone_x_max)
  	{
	  tilt_angle += tiltIncr;
	  string command = "uvcdynctrl -s \"Tilt (relative)\" -- " + tiltIncrString;
	  system(command.c_str());
  	}

  	// Zone 3
  	if (center_x < zone_x_min && center_y > zone_y_min && center_y < zone_y_max)
  	{
	  pan_angle += panIncr;
	  string command = "uvcdynctrl -s \"Pan (relative)\" -- " + panIncrString;
          system(command.c_str());
  	}

  	// Zone 4
  	if (center_x > zone_x_max && center_y > zone_y_min && center_y < zone_y_max)
  	{
	  pan_angle -= panIncr;
	  string command = "uvcdynctrl -s \"Pan (relative)\" -- -" + panIncrString;
          system(command.c_str());
  	}

  	// Zone 5
  	if (center_y < zone_y_min && center_x < zone_x_min)
  	{
	  tilt_angle -= tiltIncr;
	  pan_angle += panIncr;
	  string command1 = "uvcdynctrl -s \"Tilt (relative)\" -- -" + tiltIncrString;
	  string command2 = "uvcdynctrl -s \"Pan (relative)\" -- " + panIncrString;
	  system(command1.c_str());
	  ros::Duration(delay).sleep();
	  system(command2.c_str());
	  ros::Duration(delay).sleep();
  	}

  	// Zone 6
  	if (center_y < zone_y_min && center_x > zone_x_max)
  	{
	  tilt_angle -= tiltIncr;
	  pan_angle -= panIncr;
	  string command1 = "uvcdynctrl -s \"Tilt (relative)\" -- -" + tiltIncrString;
	  string command2 = "uvcdynctrl -s \"Pan (relative)\" -- -" + panIncrString;
	  system(command1.c_str());
          ros::Duration(delay).sleep();
          system(command2.c_str());
	  ros::Duration(delay).sleep();
  	}

  	// Zone 7
  	if (center_y > zone_y_max && center_x < zone_x_min)
  	{
	  tilt_angle += tiltIncr;
	  pan_angle += panIncr;
	  string command1 = "uvcdynctrl -s \"Tilt (relative)\" -- " + tiltIncrString;
	  string command2 = "uvcdynctrl -s \"Pan (relative)\" -- " + panIncrString;
	  system(command1.c_str());
          ros::Duration(delay).sleep();
          system(command2.c_str());
	  ros::Duration(delay).sleep();
  	}

  	//Zone 8
  	if (center_y > zone_y_max && center_x > zone_x_max)
  	{
	  tilt_angle += tiltIncr;
	  pan_angle -= panIncr;
	  string command1 = "uvcdynctrl -s \"Tilt (relative)\" -- " + tiltIncrString;
	  string command2 = "uvcdynctrl -s \"Pan (relative)\" -- -" + panIncrString;
	  system(command1.c_str());
          ros::Duration(delay).sleep();
          system(command2.c_str());
	  ros::Duration(delay).sleep();
  	}

	if (tilt_angle > tilt_max) { tilt_angle = tilt_max; }
	if (tilt_angle < tilt_min) { tilt_angle = tilt_min; }
	if (pan_angle > pan_max) { pan_angle = pan_max; }
	if (pan_angle < pan_min) { pan_angle = pan_min; }

	publishCamAngles(pan_angle, tilt_angle);

	cout << "pan: " << pan_angle << endl;
	cout << "tilt: " << tilt_angle << endl;
   	}
  }
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "face_tracker");

  ros::param::get("/usb_cam/image_width", image_width);
  ros::param::get("/usb_cam/image_height", image_height);

  faceTracker ft;
  ros::spin();
  return 0;
}
