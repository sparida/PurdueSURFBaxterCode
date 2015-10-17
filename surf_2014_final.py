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
from hist_back_project import *

def nothing(x):
	pass

def send_image(img):
    msg = CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
    pub.publish(msg)
    # Sleep to allow for image to be published.
    rospy.sleep(1)

def getSMin(img_copy, num_tools, smin_range):
	HSVValFile = open('HSVValFile.dat', "r");
	hsvcutoffs = cPickle.load(HSVValFile);
	HSVValFile.close()
	initial_smin = hsvcutoffs[2]
	
	timgObj = ToolImageProcessHelp()
	timgObj.setImage(img_copy.copy(), 'Original')
	timgObj.setSMin(initial_smin)
	timgObj.processImage()
	
	cv2.waitKey(0)
	
	smin_list = []
	search_flag = True
	
	for smin_val in range(initial_smin - smin_range, initial_smin + smin_range + 1):
		timgObj.setImage(img_copy.copy(), 'Original')
		timgObj.setSMin(smin_val)
		timgObj.processImage()
		num = timgObj.getNumTools()
		h = timgObj.getHuMomentsList()
		c = timgObj.getCentroidsList()
		if num == num_tools:
			smin_list.append(smin_val)
		
	smin_list_errors = map(lambda x: (x - initial_smin) ** 2, smin_list)
	return smin_list[smin_list_errors.index(min(smin_list_errors))]
	
	
		
def main():
		
	rospy.init_node("Surf_2014_Final")
	lhObj = LeapHelp()
	initial_num_tools = int(raw_input("Enter number of tools on table:"))
	left_untuck = {"left_w0":-0.08, "left_w1":-1.0, "left_w2":-1.19, "left_e0":1.94,  "left_e1":0.67, "left_s0":1.03, "left_s1":-0.50}
	
	fo = open('left_del_cam_pos.dat', "r")
	del_limb = cPickle.load(fo)
	#del_limb = 'left'
	left_del_pos = cPickle.load(fo)
	cam_right_pos = cPickle.load(fo)
	fo.close()
	
	fo = open('left_high_pos.dat', "r")
	del_limb = cPickle.load(fo)
	left_high_pos = cPickle.load(fo)
	fo.close()
	
	fo = open('left_ret_pos.dat', "r")
	ret_limb = cPickle.load(fo)
	left_ret_del_pos = cPickle.load(fo)
	fo.close()
	
	posFile = open("BaxterCamArmJointPosFile.dat", "r")
	cam_arm_joint_pos = cPickle.load(posFile)
	posFile.close()
	
	posFile = open("BaxterRetArmJointPosFile.dat", "r")
	ret_arm_joint_pos = cPickle.load(posFile) # Ontable, moveCamArm above table, left_ret_del_pos:retrieval dleivery position
	posFile.close()
	
	
	ret_back_img = cv2.imread("RetrieveImage.png")
	ret_arm_cam = moveGetCamArmImage(limb = 'left', file_name = "RetrieveBaxterCamArmJointPosFile.dat")
	
	mvObj = moveGetCamArmImage('right')
	mvObj.moveCamArm()
	img = mvObj.getCamArmImage()
			
	ppObj = PixelToBaxterPosTransformer()
	ppObj.setOffsets(0.0, 0.019, -0.025)
	
	# Figuring out smin
	smin = getSMin(img.copy(), initial_num_tools, 10)
		
	timgObj = ToolImageProcessHelp()
	timgObj.setImage(img, 'Original')
	timgObj.setSMin(smin)
	#timgObj.processImage_BackProject()
	timgObj.processImage()
	
	num = timgObj.getNumTools()
	h = timgObj.getHuMomentsList()
	c = timgObj.getCentroidsList()
	
	baxter_img = timgObj.getContourImage()
	baxter_img = cv2.resize(baxter_img, (1024, 600)) 
	send_image(baxter_img)
	print "SMIN:%s" % (smin)
	
	
	ToolList = []
	
	for i in range(num):
		tool = ToolIdentify()
		tool.setHuMoments([ h[0,i], h[1,i], h[2,i], h[3,i], h[4,i], h[5,i], h[6,i], h[7,i] ])
		tool.setCentroid(c[0, i], c[1, i])
		tool.identifyTool()
		ToolList.append(tool)
		#print "Tool #%s: %s" % (i+1, tool.getToolName())

	last_ret_time = time.time()
	key_pos = 0
	just_delivered = False
	while(num > 0):
		just_delivered = False
		for i in range(num):
			print "Tool #%s: %s" % (i+1, ToolList[i].getToolName())

		##############LEAP STUFF########################################
		index = -1
		lhObj.processFrame()
		print "Fingers: %s" % (lhObj.getNumFingers())
		if(lhObj.getKeyTapDetected() == True):
			
			print "Extend Fingers:"
			
			key_pos = key_pos + 1
			key_pos = key_pos % 2
		
			time.sleep(2)
			lhObj.processFrame()
			finger_count = lhObj.getNumFingers()
				
			if (finger_count) in range(1, num+1):
				print "Tool Available"
				print finger_count
				index = int(finger_count) - 1
				
			else:
				print"Selected tool unavaialable"
	
			time.sleep(1)
			
		################################################################

		#index = raw_input("Enter index of tool to deliver:")
		#index = int(index) - 1 # Because tools are printed from 1 onwards but indexing starts from 0

		
		if(index != -1):
			ppObj.calcBaxterPos(ToolList[index]._centroidX, ToolList[index]._centroidY)
			(joints_loc, inter_joints_loc1, inter_joints_loc2) = ppObj.getBaxterJointSpace(del_limb)
			print "Delivering %s..." % (ToolList[index].getToolName())
			just_delivered = True		
			
			del(ToolList[index])
			num = num - 1
			
			
			#############Chaneg right to left in cam_arm_joint_pos################ 
			#dummy = []
			#for i in cam_arm_joint_pos.keys():
			#	new_key = i.replace('right', 'left')
			#	dummy.append(new_key)
			#cam_right_pos = dict(zip(dummy, cam_arm_joint_pos.values()))
			########################################################################
			
			#Move Delivery Arm
			control_arm = baxter_interface.limb.Limb(del_limb)
			control_arm.set_joint_position_speed(0.5) # Joint movement speed
			control_arm.move_to_joint_positions(left_high_pos, 7.0)
			control_arm.move_to_joint_positions(inter_joints_loc1, 7.0)
			time.sleep(1)
			control_arm.move_to_joint_positions(joints_loc, 7.0)
			time.sleep(1)
			control_arm.move_to_joint_positions(inter_joints_loc2, 7.0)
			time.sleep(1)
			control_arm.move_to_joint_positions(left_del_pos, 7.0)
			time.sleep(3)
			control_arm.move_to_neutral(timeout = 7)
			ret_arm_cam.moveCamArm()
	
	
		############################################################################################
		tool_detected = False
		if(time.time() > 15 + last_ret_time) and (just_delivered == False):
			print "Checking retrieval tray"
			ret_arm_cam.moveCamArm()
			icObj = ImageClipper('RetrieveImageClipPosFile.dat')
			per_white = 100.0
			count = 0
			while (per_white >= 18.0 or per_white <= 2.0) and count <= 3:
				img = ret_arm_cam.getCamArmImage()
				img = icObj.clipImage(img)
				per_white, binary_img = getForeBack(ret_back_img, img)
				print "Waiting for next iteration"
				print
				count += 1
			
			if per_white >= 2.0 and per_white <= 15.0:
				print "TOOL PRESENT"
				tool_detected = True
			else:
				print "NO TOOL"
				tool_detected = False
			last_ret_time = time.time()
			
		##########
		
		###########
		if tool_detected == True:
			# Move Retrieval Arm
			control_arm = baxter_interface.limb.Limb(ret_limb)
			control_arm.set_joint_position_speed(0.5) # Joint movement speed
			ret_arm_cam.moveCamArm()
			time.sleep(1)
			control_arm.move_to_joint_positions(ret_arm_joint_pos, 7.0)
			time.sleep(1)
			ret_arm_cam.moveCamArm()
			time.sleep(1)
			control_arm.move_to_joint_positions(left_ret_del_pos, 7.0)
			time.sleep(1)
			ret_arm_cam.moveCamArm()
			
		############################################################################################
		
		
	
	return 0
	
	
if __name__ == '__main__':
	sys.exit(main())
