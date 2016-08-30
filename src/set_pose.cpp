#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <map>
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <iterator>
#include <math.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

class Triple
{
  private:
	float a;
  	float b;
  	float c;
  	float d;
  public:
	Triple(float x, float y, float o, float w)
	{
	a = x;
	b = y;
	c = o;
	d = w;
	}
	float get_element(int i)
	{
		if (i == 0) {return a;}
		if (i == 1) {return b;}
		if (i == 2) {return c;}
		if (i == 3) {return d;}
	}
	friend ostream& operator << (ostream& os, const Triple& coord)
	{
	os << coord.a << "," << coord.b << "," << coord.c << "," << coord.d << endl;
	return os;
	}
};

void delay()
{
  cout<<"Press any key to show coordinate options" << "\n";
  cin.get();
}

/*float current_x;
float current_y;
float current_theta;

void checkCoordinatesCallback(const turtlesim::Pose pose_msg)
{
	cout << pose_msg << "\n";
	current_x = pose_msg.x;
	current_y = pose_msg.y;
	current_theta = pose_msg.theta;
}
*/
map<string, Triple> coordinates;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; //

int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_pose");
  //ros::NodeHandle n;
  MoveBaseClient action("move_base", true); //
  //ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  move_base_msgs::MoveBaseGoal goal;
  //gazebo_msgs::SetModelState setmodelstate;

  while(ros::ok())
  {
	delay();

	cout << " \n";
	cout << "Coordinates Map\n";
	cout << " \n";

	ifstream coord_file_new("/home/ubuntu/catkin_ws/src/turtlebot_demos/scripts/map_coordinates.txt");
	while(coord_file_new)
	{
		string s;
		string key;
		int i = 0;
		float elements[] = {0, 0, 0, 0};

		if(!getline(coord_file_new, s))
		{
			cout << "\n";
			break;
		}

		istringstream ss(s);

		while(ss)
		{
			string s;
			string add_format[] = {"Key: \"","\", x: ",", y: ",", quat_z: ",", quat_w: "};

			if(!getline(ss, s, ','))
			{
				cout<< " \n";
				i = 0;
				Triple add_coord = Triple(elements[0], elements[1], elements[2], elements[3]);
				coordinates.insert(pair<string,Triple>(key, add_coord));

				float elements[] = {0, 0, 0, 0};
				break;
			}
			cout << add_format[i] << s;
			if(i == 0)
			{
				key = s;
			}
			else
			{
				istringstream(s) >> elements[i-1];
			}
			i++;
		}
	}

	string destination;
	cout << "Select destination: ";
	getline(cin, destination);

	map<string, Triple>::iterator index;
	index = coordinates.find(destination);
	if(index == coordinates.end())
	{
		cout << "Coordinates not found\n";
	}
	else
	{
		cout << "x: " << index->second.get_element(0) << ", ";
		cout << "y: " << index->second.get_element(1) << ", ";
		cout << "quat_z: " << index->second.get_element(2) << endl;
		cout << "quat_w: " << index->second.get_element(3) << endl;

		goal.target_pose.header.frame_id = "map"; //
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = index->second.get_element(0);
		goal.target_pose.pose.position.y = index->second.get_element(1);
		goal.target_pose.pose.position.z = 0.0;
		goal.target_pose.pose.orientation.x = 0.0;
		goal.target_pose.pose.orientation.y = 0.0;
		goal.target_pose.pose.orientation.z = index->second.get_element(2);
		goal.target_pose.pose.orientation.w = index->second.get_element(3);

		action.sendGoal(goal);
	}
  }
}
