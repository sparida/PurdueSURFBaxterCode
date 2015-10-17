#!/usr/bin/env python

"""
#HELP#
###############ALL THAT MATTERS TO USE MoveGetCamArmImage CLASS##########################
tobj = moveGetCamArmImage('right')
tobj.moveCamArm()
img = tobj.getCamArmImage()
###########################################################################################
######ALL THAT MATTERS TO USE PixelToBaxterPos CLASS######
obj = PixelToBaxterPosTransformer()
obj.setOffsets(0.0, 0.03, 0.11)
obj.calcBaxterPos(x, y)
joints_loc = obj.getBaxterJointSpace()	# Can be directly used with move to joint location in baxter interface
##############################################
"""


from __future__ import division
from orientation_help import VectorOrientation
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
import sys
from os.path import join
from glob import glob

	
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from orientation_help import VectorOrientation
from ik_solve_baxter import IKSolveBaxter
from sub_image_help import SubImageHelp
from baxter_button_help import BaxterButtonHelp
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class moveGetCamArmImage:

	def __init__(self, limb = 'right', file_name = "BaxterCamArmJointPosFile.dat"):
		# Load Baxter Cam Arm Joint Positions	
		posFile = open(file_name, "r")
		self._camArmJoints = cPickle.load(posFile)
		posFile.close()	
		topic_name = "/cameras/" + limb + "_hand_camera/image"
		self._sihObj = SubImageHelp(topic_name)
		self._limb = limb
		self._cam_arm = baxter_interface.limb.Limb(self._limb)
	
	def moveCamArm(self):
		
		# Move Baxter Arm to Joint Position
		print "Moving Camera Arm..."
		self._cam_arm.set_joint_position_speed(0.5) # Joint movement speed
		self._cam_arm.move_to_joint_positions(self._camArmJoints, 5.0)
		print "Camera Arm Moved to Position"


	def getCamArmImage(self):
		
		# Get image frame from baxter hand camera
		print
		print "Waiting for image..."
		error_flag = True
		rate = rospy.Rate(1)
		rate.sleep()	
		while error_flag == True:
			try:
				img = self._sihObj.getCVImage()
				error_flag = False
				print "BaxterImageObtained"
				rate.sleep()
			except:
				print
				print "Error in Obtaining Image"
		return img

##########################################################################################################
class PixelToBaxterPosTransformer:
	def __init__(self, tmat_filename="TransformationMatrixFile.dat", clip_pos_filename="ImageClipPosFile.dat"):
	
		# Load Trasformation Matrix
		tmatFile = open(tmat_filename, "r");
    		self._T = cPickle.load(tmatFile);
    		tmatFile.close()
	
		# Load Image Clipping Coordinates	
		fileObj = open(clip_pos_filename, "r")	
		self._image_clip = cPickle.load(fileObj)
		fileObj.close()
		
		# Intialize other variables
		self._del_arm_x = -1
		self._del_arm_y = -1
		self._del_arm_z = -1
		self._joints_loc = -1
		self._offsets = -1
		self._inter_joints1_loc = -1
		self._inter_joints2_loc = -1
	
	def calcBaxterPos(self, x, y):
				
		mod_x = float(x + self._image_clip[0])	
		mod_y = float(y + self._image_clip[1])	
	
		pixel_pos = numpy.matrix([[mod_x], [mod_y], [1.0]])
		baxter_pos = numpy.dot(self._T, pixel_pos)
	
		self._del_arm_x = baxter_pos[0, 0] + self._offsets[0]	#As camera not at center	
		self._del_arm_y = baxter_pos[1, 0] + self._offsets[1]	#As camera is not at center
		self._del_arm_z = baxter_pos[2, 0] + self._offsets[2]	# Height above table
		
		
	def getBaxterJointSpace(self, limb = 'right'):
		
		# All orientation calclulations
		voObj = VectorOrientation() 
		voObj.setVectorChange(0.00, 0.00, -1.0) # X, Y and Z Vector Axis Orientation
		#voObj.setAngleOrientation(-20, 0, 0,  'd')
		voObj.calcVectorOrientation()
		# Set gamma in degrees
		voObj.setGamma(180, 'd')
		# Trick for baxter orientation change for rigth arm:
		voObj.setAngleOrientation(-voObj._theta, voObj._phi, voObj._gamma, 'r')
		# Calculate quaternion
		quat = voObj.getQuaternionOrientation()
		
		# Inverse Kinematics
		arm = IKSolveBaxter(limb)
		arm.setArmEndPos(self._del_arm_x, self._del_arm_y, self._del_arm_z, quat[0], quat[1], quat[2], quat[3])
		arm.ik_solve();
		self._joints_loc = arm.getBaxterJointSpace()
		
		arm.setArmEndPos(self._del_arm_x, self._del_arm_y, self._del_arm_z + 0.10, quat[0], quat[1], quat[2], quat[3])
		arm.ik_solve();
		self._inter_joints_loc1 = arm.getBaxterJointSpace()
		
		arm.setArmEndPos(self._del_arm_x, self._del_arm_y, self._del_arm_z + 0.15, quat[0], quat[1], quat[2], quat[3])
		arm.ik_solve();
		self._inter_joints_loc2 = arm.getBaxterJointSpace()
		return self._joints_loc, self._inter_joints_loc1, self._inter_joints_loc2
	
	def getBaxter3DPos(self):
		return [self._del_arm_x, self._del_arm_y, self._del_arm_z]
	
	def setOffsets(self, x= 0.0, y = 0.0, z= 0.0):
		self._offsets = [x, y, z]

##########################################################################################################
class ImageClipper:
	def __init__(self, clip_pos_file = "ImageClipPosFile.dat"):
	
		fileObj = open(clip_pos_file, "r")
		self._image_clip = cPickle.load(fileObj)
		fileObj.close()	
	
	def clipImage(self, img):
		img = img[:, self._image_clip[0] : self._image_clip[2] ]
		img = img[self._image_clip[1] : self._image_clip[3], :]
		return img

##########################################################################################################

def main():
	
	# Start ROS Node	
	rospy.init_node('pixel_to_baxter_pos')

	###############ALL THAT MATTERS TO USE MoveGetCamArmImage CLASS##########################
	tobj = moveGetCamArmImage('right')
	tobj.moveCamArm()
	img = tobj.getCamArmImage()
	###########################################################################################
	
	print "Press C to continue and get image frame"
	while(not sf.Keyboard.is_key_pressed(sf.Keyboard.C)):
		pass
	time.sleep(1)


	# Load Image Clipping Coordinates	
	fileObj = open("ImageClipPosFile.dat", "r")
	image_clip = cPickle.load(fileObj)
	fileObj.close()
	
	# Clip Image
	img = img[:, image_clip[0] : image_clip[2] ]
	img = img[image_clip[1] : image_clip[3], :]
	
	
	
	x = 250
	y = 250
	limb = 'right'
	
	cv2.circle(img,(x,y),5,(255,0,0), -1)
	print "Press Key to Continue"	
	cv2.imshow('FinalImage', img)
	cv2.waitKey(0)
	
	######ALL THAT MATTERS TO USE PixelToBaxterPos CLASS######
	obj = PixelToBaxterPosTransformer()
	obj.setOffsets(0.0, 0.03, 0.11)
	obj.calcBaxterPos(x, y)
	joints_loc = obj.getBaxterJointSpace("right")	# Can be directly used with move to joint location in baxter interface
	##############################################
		
	# Move Arm
	control_arm = baxter_interface.limb.Limb(limb)
	control_arm.set_joint_position_speed(0.5) # Joint movement speed
	control_arm.move_to_joint_positions(joints_loc, 7.0)
	
	cv2.destroyAllWindows()
	cv2.waitKey(0)

if __name__ == '__main__':
	sys.exit(main())
