#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <sstream>
#include <iostream>
#include <string>
#include <termios.h>

using namespace std;

float pan_angle = 0;
float tilt_angle = 0;
float pan_max = 4480/47.41; float pan_min = -4480/47.41;
float tilt_max = 1920/37.65; float tilt_min = -1920/37.65;
float panIncr = 237/47.41; // 5 degrees
float tiltIncr = 188/37.65; // 5 degrees
string tiltIncrString = "188";
string panIncrString = "237";

int main (int argc, char **argv)
{
  ros::init(argc, argv, "pan_tilt_controller");
  ros::NodeHandle nh;
  ros::Publisher cam_angles = nh.advertise<std_msgs::Float64MultiArray>("/new_cam_angles", 1);

  struct termios t;
  tcgetattr(STDIN_FILENO, &t);
  t.c_lflag &= ~ICANON;
  t.c_lflag &= ~ECHO;
  tcsetattr(STDIN_FILENO, TCSANOW, &t);

  cout << "j: left, l: right, i: up, k: down, o: pan reset, u: tilt reset" << endl;

  while (ros::ok())
  {
  	string key;
  	key = cin.get();

  	if (key == "o")
  	{
	  pan_angle = 0;
	  string command = "uvcdynctrl -s \"Pan Reset\" 1";
	  system(command.c_str());
  	}

  	if (key == "u")
  	{
	  tilt_angle = 0;
	  string command = "uvcdynctrl -s \"Tilt Reset\" 1";
	  system(command.c_str());
  	}

  	if (key == "j")
  	{
	  pan_angle += panIncr;
	  string command = "uvcdynctrl -s \"Pan (relative)\" -- " + panIncrString;
	  system(command.c_str());
  	}

  	if (key == "l")
  	{
	  pan_angle -= panIncr;
          string command = "uvcdynctrl -s \"Pan (relative)\" -- -" + panIncrString;
          system(command.c_str());
  	}

  	if (key == "i")
  	{
	  tilt_angle -= tiltIncr;
          string command = "uvcdynctrl -s \"Tilt (relative)\" -- -" + tiltIncrString;
          system(command.c_str());
  	}

  	if (key == "k")
  	{
	  tilt_angle += tiltIncr;
          string command = "uvcdynctrl -s \"Tilt (relative)\" -- " + tiltIncrString;
          system(command.c_str());
  	}

        if (tilt_angle > tilt_max) { tilt_angle = tilt_max; }
        if (tilt_angle < tilt_min) { tilt_angle = tilt_min; }
        if (pan_angle > pan_max) { pan_angle = pan_max; }
        if (pan_angle < pan_min) { pan_angle = pan_min; }

	std_msgs::Float64MultiArray anglesArray;
	anglesArray.data.clear();
	anglesArray.data.push_back(pan_angle);
	anglesArray.data.push_back(tilt_angle);

	cam_angles.publish(anglesArray);
	ros::spinOnce();
  }
  return 0;
}

