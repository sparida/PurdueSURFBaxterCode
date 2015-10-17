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
from tool_identify import ToolIdentify
from leap_help import LeapHelp


def nothing(x):
	pass


def main():
		
	rospy.init_node("Surf_2014_Final")
	
	fo = open('left_del_cam_pos.dat', "r")
	del_limb = cPickle.load(fo)
	#del_limb = 'left'
	left_del_pos = cPickle.load(fo)
	cam_right_pos = cPickle.load(fo)
	fo.close()
	
	posFile = open("BaxterCamArmJointPosFile.dat", "r")
	cam_arm_joint_pos = cPickle.load(posFile)
	posFile.close()
	
	mvObj = moveGetCamArmImage('right')
	mvObj.moveCamArm()
	img = mvObj.getCamArmImage()
			
	ppObj = PixelToBaxterPosTransformer()
	ppObj.setOffsets(0.0, 0.017, -0.035)
	
	timgObj = ToolImageProcessHelp()
	timgObj.setImage(img, 'Original')
	timgObj.processImage_BackProject()
	
	cv2.imshow('Contours', timgObj.getContourImage())
	cv2.waitKey(0)
	cv2.destroyAllWindows()

	num = timgObj.getNumTools()
	h = timgObj.getHuMomentsList()
	c = timgObj.getCentroidsList()
	
	ToolList = []
	
	for i in range(num):
		tool = ToolIdentify()
		tool.setHuMoments([ h[0,i], h[1,i], h[2,i], h[3,i], h[4,i], h[5,i], h[6,i], h[7,i] ])
		tool.setCentroid(c[0, i], c[1, i])
		tool.identifyTool()
		ToolList.append(tool)
		print "Tool #%s: %s" % (i+1, tool.getToolName())

	
	
	
	index = raw_input("Enter index of tool to deliver:")
	index = int(index) - 1 # Because tools are printed from 1 onwards but indexing starts from 0
		
	ppObj.calcBaxterPos(ToolList[index]._centroidX, ToolList[index]._centroidY)
	(joints_loc, inter_joints_loc1, inter_joints_loc2) = ppObj.getBaxterJointSpace(del_limb)
		
	print "Delivering %s..." % (ToolList[index].getToolName())
	#############Chaneg right to left in cam_arm_joint_pos################ 
	#dummy = []
	#for i in cam_arm_joint_pos.keys():
	#	new_key = i.replace('right', 'left')
	#	dummy.append(new_key)
	#cam_right_pos = dict(zip(dummy, cam_arm_joint_pos.values()))
	########################################################################
	
	# Move Delivery Arm
	control_arm = baxter_interface.limb.Limb(del_limb)
	control_arm.set_joint_position_speed(0.5) # Joint movement speed
	control_arm.move_to_neutral(5)
	control_arm.move_to_joint_positions(inter_joints_loc1, 7.0)
	time.sleep(1)
	control_arm.move_to_joint_positions(joints_loc, 7.0)
	time.sleep(1)
	control_arm.move_to_joint_positions(inter_joints_loc2, 7.0)
	time.sleep(1)
	control_arm.move_to_joint_positions(left_del_pos, 7.0)
	time.sleep(3)
	control_arm.move_to_neutral(5)
	
	return 0
	
	
if __name__ == '__main__':
	sys.exit(main())
