#!/usr/bin/env python

from __future__ import division
import roslib
roslib.load_manifest('baxter_examples')
import sys
import rospy
import os
import cv
import cv2
import numpy
import cPickle
import baxter_interface
import baxter_interface.digital_io as DIO
import time
import sfml as sf
import math
from os.path import join
from glob import glob

from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError	
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from orientation_help import VectorOrientation
from ik_solve_baxter import IKSolveBaxter
from sub_image_help import SubImageHelp
from baxter_button_help import BaxterButtonHelp
from tool_image_process_help import ToolImageProcessHelp
from cam_arm_pix_trans_help import moveGetCamArmImage
from cam_arm_pix_trans_help import PixelToBaxterPosTransformer
from cam_arm_pix_trans_help import ImageClipper
from tool_identify import ToolIdentify
from leap_help import LeapHelp
from hist_back_project import *

def nothing(x):
	pass

	

def main():
		
	rospy.init_node("Surf_2014_Retrieve")
	
	tobj = moveGetCamArmImage(limb = 'left', file_name = "RetrieveBaxterCamArmJointPosFile.dat")
	tobj.moveCamArm()
	img = tobj.getCamArmImage()
	icObj = ImageClipper('RetrieveImageClipPosFile.dat')
	img = icObj.clipImage(img)

	back_img = cv2.imread('RetrieveImage.png')
	
	per_white, binary_img = getForeBack(back_img, img)
	
	#per_white = 100.0
	#while (per_white >= 12.5):
	#	img = tobj.getCamArmImage()
	#	img = icObj.clipImage(img)
	#	per_white, binary_img = getForeBack(back_img, img)

	cv2.imshow('Current_Image', img)
	cv2.imshow('Back_Image', back_img)
	cv2.imshow('Binary_Image', binary_img)
	print per_white
	print
	if per_white >= 2.0:
		print "TOOL PRESENT"
	else:
		print "NO TOOL"

	cv2.waitKey(0)
	
	
if __name__ == '__main__':
	sys.exit(main())
