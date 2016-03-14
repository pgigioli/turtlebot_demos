#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

void get_pose_callback(const geometry_msgs::Pose msg)
{
	std::cout << "Input: " << std::flush;
	std::string >> input;
	std::cin >> input;

	if(input.compare("get pose"))
	{
		ROS_INFO("Pose received");
		std::cout << msg << std::endl;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "get_pose");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/robot_pose", 10, get_pose_callback);
	ros::spin();

	return 0;
}
