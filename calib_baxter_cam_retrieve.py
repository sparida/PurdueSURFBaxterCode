#!/usr/bin/env python

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
	


img = []
image_corners = []
click_count = 0	
baxter_endpoints = []

def handle_mouse(event, x, y, flags, param):
	
	global img
	global image_corners
	global click_count
	
	if event == cv2.EVENT_LBUTTONDBLCLK:
		click_count += 1

		cv2.circle(img,(x,y),5,(255,0,0), -1)
	
		font = cv2.FONT_HERSHEY_SIMPLEX
		cv2.putText(img,str(click_count), ( (x+10),(y+10) ), font, 1,(255,255,255), 3, 8)

		point = [x, y]
		image_corners.append(point)
		print image_corners
		
		cv2.imshow('RetrieveImage', img)
		
 
 

def main():

	global img
	global image_corners
	global click_count
	global baxter_endpoints
	
	rospy.init_node('RetrieveCamToolTableCalib', disable_signals=True)

	# Record Baxter Coordinates
	print
	print "Record arm endpoints using Cuff Button and Big Button when 4 points are done (Left Arm)"	
	cam_arm = baxter_interface.limb.Limb("left")
	bbhObj = BaxterButtonHelp("left")
	baxter_endpoints = []
	click_count = 0
	while_flag = True
	while(while_flag):
		cButtonPressed = bbhObj.getButtonStates()['cState']
		bButtonPressed = bbhObj.getButtonStates()['bState']		
		
		if(cButtonPressed):
			pose = cam_arm.endpoint_pose()
			xyz = pose["position"]
			baxter_endpoints.append(xyz)	
			click_count += 1
			time.sleep(1)
			print "Baxter Arm EndPoint %s added" % (click_count)

		if(bButtonPressed):	
			if (click_count == 4):
				while_flag = False
				print "Baxter Endpoints Recorded Recorded:"
				print baxter_endpoints
			elif (click_count < 4):	
				while_flag = True
				baxter_endpoints = []
				click_count = 0
				print "Less than 4 points recorded. Please re-record all corners"
			else:
				while_flag = True
				baxter_endpoints = []
				click_count = 0
				print "More than 4 points recorded. Please record only the corners"
			
			time.sleep(1)	

	##############################################################
	print "Storing Retrieval Position"
	# Store retrieval position
	print
	print "Moving baxter cam arm to desired location image frame" 
		
	cam_arm_x = ((baxter_endpoints[0].x + baxter_endpoints[2].x) / 2) + 0 #As camera not at center	
	cam_arm_y = (baxter_endpoints[0].y + baxter_endpoints[2].y) / 2	+ 0.003 #As camera is not at center
	cam_arm_z = ((baxter_endpoints[0].z + baxter_endpoints[2].z) / 2) + 0	 # Height above table
		
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
	arm = IKSolveBaxter("left")
	arm.setArmEndPos(cam_arm_x, cam_arm_y, cam_arm_z, quat[0], quat[1], quat[2], quat[3])
	arm.ik_solve();
	ret_joints = arm.getBaxterJointSpace()
	
	bcajpFile = open("BaxterRetArmJointPosFile.dat", "w");
    	cPickle.dump(ret_joints, bcajpFile);
    	bcajpFile.close()
	# Move Arm
	control_arm = baxter_interface.limb.Limb("left")
	control_arm.set_joint_position_speed(0.5) # Joint movement speed
	control_arm.move_to_joint_positions(ret_joints, 5.0)
	time.sleep(1)
	##############################################################
	# Move baxter arm
	print
	print "Moving baxter cam arm to desired location image frame" 
		
	cam_arm_x = ((baxter_endpoints[0].x + baxter_endpoints[2].x) / 2) + 0.075 #As camera not at center	
	cam_arm_y = (baxter_endpoints[0].y + baxter_endpoints[2].y) / 2	+ 0.005 #As camera is not at center
	cam_arm_z = ((baxter_endpoints[0].z + baxter_endpoints[2].z) / 2) + 0.15	 # Height above table
		
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
	arm = IKSolveBaxter("left")
	arm.setArmEndPos(cam_arm_x, cam_arm_y, cam_arm_z, quat[0], quat[1], quat[2], quat[3])
	arm.ik_solve();
	joints = arm.getBaxterJointSpace()
	
	# Move Arm
	control_arm = baxter_interface.limb.Limb("left")
	control_arm.set_joint_position_speed(0.5) # Joint movement speed
	control_arm.move_to_joint_positions(joints, 5.0)
	
	os.system("clear")
	
	print "Press C to continue and store baxter camera arm position"
	while(not sf.Keyboard.is_key_pressed(sf.Keyboard.C)):
		pass

	bcajpFile = open("RetrieveBaxterCamArmJointPosFile.dat", "w");
    	cPickle.dump(joints, bcajpFile);
    	bcajpFile.close()
	print
	print "Baxter Camera Arm Position Stored"
	
	# Get image frame from baxter hand camera
	print
	print "Waiting for image..."
	sihObj = SubImageHelp("/cameras/left_hand_camera/image")
	error_flag = True
	rate = rospy.Rate(1)
	rate.sleep()	
	while error_flag == True:
		try:
			img = sihObj.getCVImage()
			error_flag = False
			print "BaxterImageObtained"
			rate.sleep()
		except:
			print
			print "Error in Obtaining Image"
	
		
	# All recording points stuff
	print
	print "Record all 4 corner points using mouse double click and then press key to continue"
	img_copy = img.copy()
	while_flag = True
	while(while_flag):
		img = img_copy.copy()
		click_count = 0
		image_corners = [] 
		cv2.namedWindow('RetrieveImage')
		cv2.imshow('RetrieveImage', img)
		cv2.setMouseCallback('RetrieveImage',handle_mouse)
	
		cv2.waitKey(0)
		
		if (click_count == 4):
			while_flag = False
			print "Image Corners Recorded"
			print
			print "The image corners are:"
			print image_corners[0], image_corners[1], image_corners[2], image_corners[3]
		elif (click_count < 4):	
			while_flag = True
			print "Less than 4 points recorded. Please re-record all corners"
		else:
			while_flag = True
			print "More than 4 points recorded. Please record only the corners"
	
	#cv2.destroyAllWindows()
	
	time.sleep(1)
	
	X = numpy.matrix([
			[image_corners[0][0], image_corners[2][0], image_corners[3][0]],
			[image_corners[0][1], image_corners[2][1], image_corners[3][1]],
			[1.0, 1.0, 1.0] 
			])
	A = numpy.matrix([
			[baxter_endpoints[0].x, baxter_endpoints[2].x, baxter_endpoints[3].x], 
			[baxter_endpoints[0].y, baxter_endpoints[2].y, baxter_endpoints[3].y],
			[baxter_endpoints[0].z, baxter_endpoints[2].z, baxter_endpoints[3].z] 
			])
	# Add Image Clip positions and Display Clipped Image
	x1 = max( [image_corners[0][0], image_corners[3][0]] )
	y1 = max( [image_corners[0][1], image_corners[1][1]] )
	x2 = min( [image_corners[1][0], image_corners[2][0]] )
	y2 = min( [image_corners[2][1], image_corners[3][1]] )
	
	image_clip = [x1, y1, x2, y2]

	img = img_copy[:, image_clip[0] : image_clip[2] ]
	img = img[image_clip[1] : image_clip[3], :]
	
	
	cv2.imwrite('RetrieveImage.png', img)
	ImageClipPosFile = open("RetrieveImageClipPosFile.dat", "w");
    	cPickle.dump(image_clip, ImageClipPosFile);
	ImageClipPosFile.close()
	

	print "Image Clip Positions Added To File"
	time.sleep(1)
	
	print
	print "Press C to continue and calculate tranformation matrix"
	while(not sf.Keyboard.is_key_pressed(sf.Keyboard.C)):
		pass
	
	
	
	os.system("clear")

	print "Image Corner Pixel Locations:"
	print X
	print
	
	print "Baxter Endpoint Locations:"
	print A
	print
	
	XAPosFile = open("RetrieveXAPosFile.dat", "w");
    	cPickle.dump(X, XAPosFile);
	cPickle.dump(A, XAPosFile);
    	XAPosFile.close()
	time.sleep(1)
	print "X(ImageCorners) and A(BaxterEndpoints) Stored in XAPosFile.dat"
	print

	if(numpy.linalg.det(X) == 0):
		print "Determiant of image corner matrix = 0"
		print "Please retry calibration"
	else:
		T = numpy.matrix( numpy.dot( A, numpy.linalg.inv(X) ) )
		print "Tranformation Matrix:"
		print T
		print
		print "Writing to transformation matrix file..."
		tmatFile = open("RetrieveTransformationMatrixFile.dat", "w");
    		cPickle.dump(T, tmatFile);
    		tmatFile.close()
		print "Tranformation matrix added" 
		print

	time.sleep(1)
	print "Press C to continue and exit program"
	
	
	
	while(not sf.Keyboard.is_key_pressed(sf.Keyboard.C)):
		pass
	
	rospy.signal_shutdown("Shutting down ROS Node to calculate transformation")
	print
	print "Exit Program"
	print 
	return 0

if __name__ == '__main__':
	sys.exit(main())

