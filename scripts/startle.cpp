#include "ros/ros.h"
#include "audio_common_msgs/AudioData.h"
#include <unistd.h>

void startleCallback(const audio_common_msgs::AudioData msg)
{
	std::cout << msg << std::endl;
	usleep(2000);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "startle");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/audio", 1000, startleCallback);
	ros::spin();

	return 0;
}
