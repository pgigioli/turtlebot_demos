#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
import time #for sleep()
import roslib
from kobuki_msgs.msg import PowerSystemEvent, AutoDockingAction, AutoDockingGoal, SensorState
from kobuki_msgs.msg import ButtonEvent #for kobuki base's b0 button
import math #for comparing if Kobuki's power has changed using fabs
import yaml

move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

def WereCloseDock():
        _client = actionlib.SimpleActionClient('/dock_drive_action', AutoDockingAction)
        rospy.loginfo("waiting for auto_docking server")
        _client.wait_for_server()
        rospy.loginfo("auto_docking server found")
        goal = AutoDockingGoal()
        rospy.loginfo("Sending auto_docking goal and waiting for result")
        _client.send_goal(goal)
        success = _client.wait_for_result(rospy.Duration(180))
        if success:
                rospy.loginfo("Auto_docking succeeded")
                return True
        else:
                rospy.loginfo("Auto_docking failed")
                return False

def GoCloseToTheChargingStation():
        rospy.loginfo("Positioning close to docking station")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = '/map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(float(-0.545),float(-0.3115),float(0)), Quaternion(float(0),float(0),float(0.0085),float(1)))
        move_base.send_goal(goal)
        success = move_base.wait_for_result(rospy.Duration(60))
        if success:
                WereCloseDock()
        if not success:
                move_base.cancel_goal()
                rospy.loginfo("Failed to reached pose near charging station")
                return False

def ButtonEventCallback(data):
        if(data.button == ButtonEvent.Button0):
                GoCloseToTheChargingStation()

def Backup():
        cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        move_cmd = Twist()
        move_cmd.linear.x = -0.1
        move_cmd.angular.z = 0
        r = rospy.Rate(10)
        temp_count = 0
        while (not rospy.is_shutdown() and temp_count < 20):
                cmd_vel.publish(move_cmd)
                temp_count = temp_count+1
                r.sleep()
        cmd_vel.publish(Twist())
        return True
if __name__ == '__main__':
	rospy.init_node('autodocking', anonymous=True)
	rospy.loginfo("test3")
	Backup();
	while (not rospy.is_shutdown()):
		rospy.Subscriber("/mobile_base/events/button",ButtonEvent,ButtonEventCallback)
		rospy.spin()
