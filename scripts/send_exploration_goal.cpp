#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <frontier_exploration/ExploreTaskAction.h>
#include <typeinfo>
#include <vector>

using namespace std;

//typedef actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> Client;

void delay()
{
        cout<<"Press any key to begin exploration" << "\n";
        cin.get();
}

double initial_localization()
{
  ros::NodeHandle n;
  ros::Publisher start_motion = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1000);
  ros::Rate loop_rate(10);
  int count = 0;

  double start_linear_vel;
  double start_angular_vel;
  double dim;
  int time_rot;
  int time_move;
  int time_accum = 0;

  ros::NodeHandle private_node_handle_("~");

  private_node_handle_.param("start_linear_vel", start_linear_vel, 0.25);
  private_node_handle_.param("start_angular_vel", start_angular_vel, 0.5);
  private_node_handle_.param("boundary_dimension", dim, 10.0);
  private_node_handle_.param("rotate_duration", time_rot, 147);
  private_node_handle_.param("move_forward_duration", time_move, 35);

  geometry_msgs::Twist cmd_vel;

/*cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.linear.z = 0.0;
  cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;
  cmd_vel.angular.z = start_angular_vel;

  while (ros::ok() && count < time_rot)
  {
    start_motion.publish(cmd_vel);
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }
*/
  time_accum += time_rot;
  count = time_rot; //
  cmd_vel.linear.x = start_linear_vel;
  cmd_vel.linear.y = 0.0;
  cmd_vel.linear.z = 0.0;
  cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;
  cmd_vel.angular.z = 0.0;

  while (ros::ok() && count >= time_accum && count < (time_accum + time_move))
  {
    start_motion.publish(cmd_vel);
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }

  time_accum += time_move;

  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.linear.z = 0.0;
  cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;
  cmd_vel.angular.z = start_angular_vel;

  while (ros::ok() && count >= time_accum && count < (time_accum + time_rot - 13))
  {
    start_motion.publish(cmd_vel);
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }

  time_accum += time_rot;

  return dim;
}

int main(int argc, char** argv)
{
  delay();

  ros::init(argc, argv, "send_exploration_goal");

  float dim = (float) initial_localization();
/*
  Client client("explore_server", true);
  client.waitForServer();
  ROS_INFO("Sending goal");
  frontier_exploration::ExploreTaskGoal goal;
*/
  actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> client("explore_server", true);//

  ROS_INFO("Waiting for action server");//
  client.waitForServer();//
  ROS_INFO("Sending goal");//
  frontier_exploration::ExploreTaskGoal goal;//

  geometry_msgs::PolygonStamped boundary;
  geometry_msgs::Point32 p1, p2, p3, p4, p5;

  p1.x = dim/2; p1.y = dim/2; p1.z = 0.0;
  p2.x = dim/2; p2.y = -dim/2; p2.z = 0.0;
  p3.x = -dim/2; p3.y = -dim/2; p3.z = 0.0;
  p4.x = -dim/2; p4.y = dim/2; p4.z = 0.0;
  p5.x = dim/2; p5.y = dim/2; p5.z = 0.0;

  boundary.header.seq = 0;
  boundary.header.stamp = ros::Time::now();
  boundary.header.frame_id = "map";
 /* boundary.polygon.points.reserve(5);
  boundary.polygon.points[0] = p1;
  boundary.polygon.points[1] = p2;
  boundary.polygon.points[2] = p3;
  boundary.polygon.points[3] = p4;
  boundary.polygon.points[4] = p5;
*/
  boundary.polygon.points.push_back(p1);
  boundary.polygon.points.push_back(p2);
  boundary.polygon.points.push_back(p3);
  boundary.polygon.points.push_back(p4);
  boundary.polygon.points.push_back(p5);

  goal.explore_center.header.frame_id = "map";
  goal.explore_center.point.x = -0.3;//
  goal.explore_center.point.y = 0.0; //
  goal.explore_center.point.z = 0.0; //
  goal.explore_boundary = boundary;
  cout << boundary.polygon << endl;
  cout << boundary << endl;
  client.sendGoal(goal);

  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Exploration started");
    }
}

