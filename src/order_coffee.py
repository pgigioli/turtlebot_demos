#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def send_order():
	pub = rospy.Publisher('coffee_request', String, queue_size=5)
	rospy.init_node('send_order', anonymous=True)
	while not rospy.is_shutdown():
		order = raw_input("Type 'coffee' to order: ")
		print order
		pub.publish(order)

if __name__ == '__main__':
	try:
		send_order()
	except rospy.ROSInterruptException:
		pass

	
