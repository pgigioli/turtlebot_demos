#include <ros/ros.h>
#include <map>
#include <iostream>
#include <fstream>
#include <tf/transform_listener.h>

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
	friend ostream& operator << (ostream& os, const Triple& coord)
		{
		os <<  coord.a << "," << coord.b << "," << coord.c << "," << coord.d << endl;
		return os;
		}
};

void delay()
{
	cout<<"Press any key to get coordinates"<<"\n";
	cin.get();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "get_pose");
	ros::NodeHandle n;

	tf::TransformListener listener;

	delay();

	while(ros::ok()) {

	geometry_msgs::PoseStamped pBase, pMap;
	pBase.header.frame_id = "base_link";
	pBase.pose.position.x = 0.0;
	pBase.pose.position.y = 0.0;
	pBase.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

	ros::Time current_transform = ros::Time::now();
	listener.getLatestCommonTime(pBase.header.frame_id, "map", current_transform, NULL);
	pBase.header.stamp = current_transform;
	listener.transformPose("map", pBase, pMap);

	geometry_msgs::Pose pose;
	pose.position.x = pMap.pose.position.x;
	pose.position.y = pMap.pose.position.y;
	pose.position.z = pMap.pose.position.z;
	pose.orientation.x = pMap.pose.orientation.x;
	pose.orientation.y = pMap.pose.orientation.y;
	pose.orientation.z = pMap.pose.orientation.z;
	pose.orientation.w = pMap.pose.orientation.w;

	cout << pose << endl;

	string yesno;
        cout<<"Save pose? (y/n): ";
        getline(cin, yesno);

        if (yesno == "y") {
                cout<<"Name: ";
                string name;
                getline(cin, name);
                Triple new_coordinates = Triple(pose.position.x, pose.position.y, pose.orientation.z, pose.orientation.w);

                ofstream coord_file;
                coord_file.open("/home/ubuntu/turtlebot_ws/src/turtlebot_demos/scripts/map_coordinates.txt", ios::app);

                coord_file << name << "," << new_coordinates;

                coord_file.close();

                cout << " \n";
                cout << "Coordinates Map\n";
		cout << " \n";

                ifstream coord_file_new("/home/ubuntu/turtlebot_ws/src/turtlebot_demos/scripts/map_coordinates.txt");
                while(coord_file_new)
                {
                        string s;
                        int i = 0;

                        if(!getline(coord_file_new, s)) break;
                                istringstream ss(s);
			while(ss)
                        {
                                string s;
                                string add_format[] = {"Key: \"","\", x: ",", y: ",", quat_z: ",", quat_w: "};

                                if(!getline(ss, s, ','))
                                {
                                        cout<< " \n";
                                        i = 0;
                                        break;
                                }
                                cout << add_format[i] << s;
				i++;
                        }
                }
		delay();
        }
	else { delay(); }
        }
}
