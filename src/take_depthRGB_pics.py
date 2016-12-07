#!/usr/bin/env python
import roslib
import sys 
import rospy
import cv2
import numpy as np
import message_filters
import cv2.cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber

stream_on = 0
idx = 8020
frame_count = 0

def callback(depth_data, rgb_data):
  bridge = CvBridge()

  try:
      rgb_img = bridge.imgmsg_to_cv2(rgb_data)
      #print "rgb image received"
  except CvBridgeError, e:
      print e
  try:
      depth_img = bridge.imgmsg_to_cv2(depth_data)
      #print "depth image received" 
  except CvBridgeError, e:
      print e

  global stream_on
  global idx
  global frame_count
  if stream_on == 1:
    if frame_count == 20:
      frame_count = 0
      rgb_filename = "/media/ubuntu/depthRGB_pairs/rgb/%05d_rgb.jpg" % idx
      depth_filename = "/media/ubuntu/depthRGB_pairs/depth/%05d_depth.npy" % idx
      idx += 1
      #print depth_img.dtype
      #print rgb_img.dtype
      cv2.imwrite(rgb_filename, rgb_img)
      np.save(depth_filename, depth_img)
      print str(idx) + " RGB and depth images saved\n"
    else:
      frame_count += 1

def main(args):
  rospy.init_node('take_depthRGB_pics', anonymous=True)

  #tss = ApproximateTimeSynchronizer([Subscriber("/camera/depth/image", Image),
                        # Subscriber("/camera/rgb/image_rect_color", Image)], queue_size=1, slop=0.1)
  #tss.registerCallback(callback)
  global stream_on

  while (not rospy.is_shutdown()):
    get_input = 'x'   
    get_input = raw_input("Press 's' to start stream, 'p' to pause: \n")
    tss = ApproximateTimeSynchronizer([Subscriber("/camera/depth_registered/image_raw", Image),
                         Subscriber("/camera/rgb/image_raw", Image)], queue_size=1, slop=0.1)

    if str(get_input) == 's':
      stream_on = 1
      tss.registerCallback(callback)
    if str(get_input) == 'p':
      stream_on = 0
      tss.registerCallback(callback)

if __name__ == '__main__':
    main(sys.argv)
