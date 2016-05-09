#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

const double pi = 3.1415926535897;

void callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  tf::Quaternion q;
  float pan_angleRAD = msg->data[0] * pi/180;
  float tilt_angleRAD = msg->data[1] * pi/180;
  q.setRPY(0.0, tilt_angleRAD, pan_angleRAD);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "table", "webcam"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "webcam_tf_broadcaster");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/cam_angles", 1, callback);
  ros::spin();
  return 0;
}
