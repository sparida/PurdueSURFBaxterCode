#!/usr/bin/env python

"""
-----------------------------------------
Description:
Class and methods to help publlish to ROS image topic after converting CV image to ROS image 

-----------------------------------------
Usage from commandline (As diagnostic tool):
$ rosrun baxter_examples pub_image_help.py  

-----------------------------------------
Usage in code:

from pub_image_help import PubImageHelp

image = cv2.imread('Ronaldo.jpg', 1)
pihObj = PubImageHelp("test_image_topic")
rate = rospy.Rate(10)	
while not rospy.is_shutdown():
	pihObj.publishROSImage(image)
	rate.sleep()

------------------------------------------
Author : Sthitapragyan Parida (Sid)
Date   : June 19, 2014 10:09 PM                           

-------------------------------------------
"""

#!/usr/bin/env python

import roslib
roslib.load_manifest('baxter_examples')
import sys
import rospy
import cv2
import numpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
 
class PubImageHelp:
	
	
	def __init__(self, pub_topic, pub_encoding="bgr8"):
		self.image_pub = rospy.Publisher(pub_topic, Image)
		self._pubEncoding = pub_encoding
		 	
		self._bridge = CvBridge()
			
	def publishROSImage(self, cv_image):
		try:
			self.image_pub.publish(self._bridge.cv2_to_imgmsg(cv_image, self._pubEncoding))
		except CvBridgeError, e:
			print 

 
def main():
	
	rospy.init_node('ImagePublisher', anonymous=True)
	image = cv2.imread("Ronaldo.jpg", 0)
	pihObj = PubImageHelp("/robot/xdisplay")
	rate = rospy.Rate(10)	
	while not rospy.is_shutdown():
		try:
			pihObj.publishROSImage(image)
			rate.sleep()
			print image
			print "OK"
		except:
			continue
	


if __name__ == '__main__':
	sys.exit(main())

