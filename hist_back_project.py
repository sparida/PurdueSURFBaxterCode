#!/usr/bin/env python

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

from cam_arm_pix_trans_help import moveGetCamArmImage
from cam_arm_pix_trans_help import PixelToBaxterPosTransformer
from cam_arm_pix_trans_help import ImageClipper
from tool_identify import ToolIdentify
from leap_help import LeapHelp
from hist_back_project import *

def nothing(x):
	pass

def getForeBack(img, back):
	
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	hsvt = cv2.cvtColor(back, cv2.COLOR_BGR2HSV)
	# calculating object histogram
	imghist = cv2.calcHist([hsv],[0, 1], None, [180, 256], [0, 180, 0, 256] )
	# normalize histogram and apply backprojection
	cv2.normalize(imghist,imghist,0,255,cv2.NORM_MINMAX)
	dst = cv2.calcBackProject([hsvt],[0,1],imghist,[0,180,0,256],1)
	# Now convolute with circular disc
	disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
	cv2.filter2D(dst,-1,disc,dst)
	dst = 255 - dst
	# threshold and binary AND
	ret,binary_img = cv2.threshold(dst,254,255,0)
	
	height, width = binary_img.shape
	percent_white = ((cv2.countNonZero(binary_img)) / (width * height) * 100)
	return  percent_white, binary_img
	
def getForeBackMOG(img, back):
	fgbg = cv2.BackgroundSubtractorMOG()
	print "Training MOG..."
	for i in range(1000):
		binary_img = fgbg.apply(back)
	
	binary_img = fgbg.apply(img)
	
	height, width = binary_img.shape
	percent_white = ((cv2.countNonZero(binary_img)) / (width * height) * 100)
	return  percent_white, binary_img
def main():
		
	rospy.init_node("Surf_2014_Retrieve")
	
	tobj = moveGetCamArmImage(limb = 'left', file_name = "RetrieveBaxterCamArmJointPosFile.dat")
	tobj.moveCamArm()
	img = tobj.getCamArmImage()
	icObj = ImageClipper('RetrieveImageClipPosFile.dat')
	img = icObj.clipImage(img)

	back_img = cv2.imread('RetrieveImage.png')
	per_white = 100.0
	while (per_white >= 10.5):
		img = tobj.getCamArmImage()
		img = icObj.clipImage(img)
		per_white, binary_img = getForeBack(back_img, img)
	cv2.imshow('Current_Image', img)
	cv2.imshow('Back_Image', back_img)
	cv2.imshow('Binary_Image', binary_img)
	print per_white
	print
	
	if per_white >= 1.0:
		print "TOOL PRESENT"
	else:
		print "NO TOOL"

	cv2.waitKey(0)
	
	
if __name__ == '__main__':
	sys.exit(main())
