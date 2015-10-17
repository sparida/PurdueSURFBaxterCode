#!/usr/bin/env python
import roslib
roslib.load_manifest('baxter_examples')
import rospy
from tf import TransformListener
import os
import sys
import argparse
import cPickle
from numpy import *
from math import sqrt
from std_msgs.msg import (
    UInt16,
)

import baxter_interface
import baxter_interface.digital_io as DIO


import sys
import rospy
import cv2
import numpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ik_solve_baxter import IKSolveBaxter
from kinect_nite_help import KinectNiteHelp
from baxter_arm_endpoint import BaxterArmEndpoint
from svd_rt import *
from baxter_button_help import BaxterButtonHelp

from pub_image_help import PubImageHelp
from sub_image_help import SubImageHelp

def main():
	
	sihObj = SubImageHelp("/cameras/left_hand_camera/image")
	pihObj = PubImageHelp("/robot/xdisplay")
	rate = rospy.Rate(10)	
	while not rospy.is_shutdown():
		image = sihObj.getCVImage()
		print image
		try:
			pihObj.publishROSImage(image)
			rospy.sleep(1)
			print "OK"
		except:
			print "a"
		#pihObj.publishROSImage(image)
		


if __name__ == '__main__':
	rospy.init_node("ImageTest")
	sys.exit(main())
