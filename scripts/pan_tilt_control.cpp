#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>
#include <termios.h>

using namespace std;

int main (int argc, char **argv)
{
  ros:: init(argc, argv, "pan_tilt_controller");
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
	  string command = "uvcdynctrl -s \"Pan Reset\" 1";
	  system(command.c_str());
  	}

  	if (key == "u")
  	{
	  string command = "uvcdynctrl -s \"Tilt Reset\" 1";
	  system(command.c_str());
  	}

  	if (key == "j")
  	{
	  string command = "uvcdynctrl -s \"Pan (relative)\" -- 320";
	  system(command.c_str());
  	}

  	if (key == "l")
  	{
          string command = "uvcdynctrl -s \"Pan (relative)\" -- -320";
          system(command.c_str());
  	}

  	if (key == "i")
  	{
          string command = "uvcdynctrl -s \"Tilt (relative)\" -- -320";
          system(command.c_str());
  	}

  	if (key == "k")
  	{
          string command = "uvcdynctrl -s \"Tilt (relative)\" -- 320";
          system(command.c_str());
  	}
  }
}

