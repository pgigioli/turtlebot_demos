#!/usr/bin/env python
import roslib
import sys
import rospy
import numpy as np
import cv2
import time
#import cv2gpu
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class face_detector:

	def __init__(self):
		self.face_cascade = cv2.CascadeClassifier('/home/ubuntu/catkin_ws/src/turtlebot_demos/haarcascades/haarcascade_frontalface_default.xml')
		#self.profile_cascade = cv2.CascadeClassifier('/home/ubuntu/catkin_ws/src/turtlebot_demos/haarcascades/haarcascade_profileface.xml') 
		#self.eye_cascade = cv2.CascadeClassifier('/home/ubuntu/catkin_ws/src/turtlebot_demos/haarcascades/haarcascade_eye.xml') 
		
		self.image_pub = rospy.Publisher("faceDetection",Image)
		self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
		self.bridge = CvBridge()

	def callback(self,data):

		delay0 = time.clock()

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print e

		t0 = time.clock()
		frmcnt = 0
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

		#faces = self.cv2gpu.find_faces(cv_image)
	
		faces = self.face_cascade.detectMultiScale(gray, 1.3, 3)
		#profiles = self.profile_cascade.detectMultiScale(gray, 1.2, 6)
		
		t = time.clock() - t0
		frmcnt += 1

		for (x,y,w,h) in faces:
    			cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,255,0),2)
    			roi_gray = gray[y:y+h, x:x+w]
    			roi_color = cv_image[y:y+h, x:x+w]

		#for (x,y,w,h) in profiles:
    		#	cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
    		#	roi_gray = gray[y:y+h, x:x+w]
    		#	roi_color = cv_image[y:y+h, x:x+w]

		cv2.imshow("Image window", cv_image)
		cv2.waitKey(3)
		
		delayT = time.clock() - delay0
		print "Face Detection fps: ", frmcnt/t
#		print "OpenCV delay: ", delayT
#		print " "

		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print e

def main(args):
	fd = face_detector()
	rospy.init_node('face_detection', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
