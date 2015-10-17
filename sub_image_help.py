#!/usr/bin/env python

"""
-----------------------------------------
Description:
Class and methods to help subscribe to ROS image topic and get CV image from ROS image 

-----------------------------------------
Usage from commandline (As diagnostic tool):
$ rosrun baxter_examples sub_image_help.py  

-----------------------------------------
Usage in code:

from sub_image_help import SubImageHelp

sihObj = SubImageHelp("test_image_topic")
rate = rospy.Rate(10)	
while not rospy.is_shutdown():
	cv2.imshow("Test Image Display", sihObj.getCVImage())
	rate.sleep()
------------------------------------------
Author : Sthitapragyan Parida (Sid)
Date   : June 19, 2014 10:09 PM                           

-------------------------------------------
"""


import roslib
roslib.load_manifest('baxter_examples')
import sys
import rospy
import cv
import cv2
import numpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
 
class SubImageHelp:
	def _callback(self,data):
		self._imageSet = False
		try:	
			self._imageSet = True
			self._cvImage = self._bridge.imgmsg_to_cv2(data, self._subEncoding)
		except CvBridgeError, e:
			self._imageSet = False
			print e
	
	def __init__(self, sub_topic, sub_encoding="bgr8"):
		self.image_sub = rospy.Subscriber(sub_topic, Image, self._callback)
		self._subEncoding = sub_encoding		
		
		self._bridge = CvBridge()
		self._cvImage = -1
		self._imageSet = False	
		

	

	def getCVImage(self):
		if (self._imageSet == True):
			return self._cvImage
		else:
			return -1
 
def main():
	
	rospy.init_node('ImageSubscriber')
	sihObj = SubImageHelp("/cameras/right_hand_camera/image")
	
	rate = rospy.Rate(10)	
	while not rospy.is_shutdown():
		try:
			img = sihObj.getCVImage()
			print "hello"
			print img.shape
			
			rate.sleep()
		except:
			print "Error"
			
	
if __name__ == '__main__':
	sys.exit(main())

