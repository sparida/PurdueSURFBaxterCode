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

def nothing(x):
	pass

def main():

	# Start ROS Node	
	rospy.init_node('GatherANNData')
	
	# Load Baxter Cam Arm Joint Positions	
	posFile = open("BaxterCamArmJointPosFile.dat", "r")
	joints = cPickle.load(posFile)
	posFile.close()
	
	# Move Baxter Arm to Joint Position
	print "Moving Camera Arm..."
	cam_arm = baxter_interface.limb.Limb("right")
	cam_arm.set_joint_position_speed(0.5) # Joint movement speed
	cam_arm.move_to_joint_positions(joints, 5.0)
	print "Camera Arm Moved to Position"

	print "Press C to continue and get image frame"
	while(not sf.Keyboard.is_key_pressed(sf.Keyboard.C)):
		pass
	time.sleep(1)

	# Get image frame from baxter hand camera
	print
	print "Waiting for image..."
	sihObj = SubImageHelp("/cameras/right_hand_camera/image")
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
	time.sleep(1)
	
	# Load Image Clipping Coordinates	
	fileObj = open("ImageClipPosFile.dat", "r")
	image_clip = cPickle.load(fileObj)
	fileObj.close()
	
	# Clip Image
	img = img[:, image_clip[0] : image_clip[2] ]
	img = img[image_clip[1] : image_clip[3], :]
	
	# Load HSV Filter Values
	fileObj = open("HSVValFile.dat", "r")
	hsvcutoffs = cPickle.load(fileObj)
	fileObj.close()
	
	# Continue to image analysis
		
	# Create Windows and TrackBars
	cv2.namedWindow('Binary', cv2.WINDOW_NORMAL)
	cv2.namedWindow('Contour', cv2.WINDOW_NORMAL)
	cv2.namedWindow('Trackbars', cv2.WINDOW_NORMAL)
	
	cv2.createTrackbar('HueMin','Trackbars', hsvcutoffs[0], 179, nothing)
	cv2.createTrackbar('HueMax','Trackbars', hsvcutoffs[1], 179, nothing)
	cv2.createTrackbar('SatMin','Trackbars', hsvcutoffs[2], 255, nothing)
	cv2.createTrackbar('SatMax','Trackbars', hsvcutoffs[3], 255, nothing)
	cv2.createTrackbar('ValMin','Trackbars', hsvcutoffs[4], 255, nothing)
	cv2.createTrackbar('ValMax','Trackbars', hsvcutoffs[5], 255, nothing)

	# Initialize kernel and font	
	kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
	font = cv2.FONT_HERSHEY_SIMPLEX
	
	# Preprocess with gaussian blur	
	blur = cv2.GaussianBlur(img,(5,5),0)
	blur_copy = img.copy()
	
	print
	print "Press C to Start Detection and C again to eneter values"
	while(not sf.Keyboard.is_key_pressed(sf.Keyboard.C)):	
		pass
	time.sleep(1)

	while(not sf.Keyboard.is_key_pressed(sf.Keyboard.C)):
		
		hmin = cv2.getTrackbarPos('HueMin', 'Trackbars')
		smin = cv2.getTrackbarPos('SatMin', 'Trackbars')
		vmin = cv2.getTrackbarPos('ValMin', 'Trackbars')
		hmax = cv2.getTrackbarPos('HueMax', 'Trackbars')
		smax = cv2.getTrackbarPos('SatMax', 'Trackbars')
		vmax = cv2.getTrackbarPos('ValMax', 'Trackbars')
			
		hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
			
		lower_thresh = numpy.array([hmin, smin, vmin])
		upper_thresh = numpy.array([hmax, smax, vmax])
		
		mask = cv2.inRange(hsv, lower_thresh, upper_thresh) 
		mask = cv2.bitwise_not(mask)
			
		median = cv2.medianBlur(mask,5)
	
		er = cv2.erode(median, kernel, iterations = 0)
		dl = cv2.dilate(er, kernel, iterations = 1)
		er = cv2.erode(dl, kernel, iterations = 1)
		dl = er
			
		cv2.imshow('Binary', dl)
		dl_copy = dl.copy()
		
			
		try:
			# Initialize necessary variables
			tc = 0
			HuMomentsList = numpy.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
			CentroidsList = numpy.array([[0.0], [0.0]])
			delta = 400
			blur_copy = img.copy()
			blur_copy2 = dl_copy.copy()
	
			# Find contours
			contours,hierarchy = cv2.findContours(dl, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			
			# Start loop to find relevant contours
			for i in range(len(contours)):
					
				if (cv2.arcLength(contours[i], True) > delta) and (hierarchy[0, i, 3] == -1):	
									
					cv2.drawContours(blur_copy, contours, i, (255,255,255), 1)
					
					m = cv2.moments(contours[i])
						
					cen_x = int(m["m10"] / m["m00"])
					cen_y = int(m["m01"] / m["m00"])
					centroid = numpy.array([[cen_x], [cen_y]])
					CentroidsList = numpy.hstack((CentroidsList, centroid))
					cv2.putText(blur_copy, str(tc + 1), (cen_x, cen_y), font, 1,(0,255,100), 3, 8)
				
					
					HuMoments = cv2.HuMoments(m)
					I8 = m["nu11"] * ((m["nu30"]+m["nu12"])**2 - (m["nu03"]+m["nu21"])**2) - (m["nu20"]-m["nu02"])*(m["nu30"]+m["nu12"])*(m["nu03"]+m["nu21"])
					HuMoments[6] = abs(HuMoments[6]) # Make reflection invariant
					HuMoments = numpy.vstack((HuMoments, I8)) # Modified Humoments

					HuMomentsList = numpy.hstack((HuMomentsList, HuMoments))
					tc += 1

									
			print "Number of instruments:", tc
			cv2.imshow('Contour', blur_copy)
			cv2.waitKey(1)
		except:	
			pass		
		
	cv2.destroyAllWindows()
	
	CentroidsList = CentroidsList[:, 1:(tc + 1)]
	HuMomentsList = HuMomentsList[:, 1:(tc + 1)]
	HuMomentsList = -numpy.sign(HuMomentsList) * numpy.log10(numpy.abs(HuMomentsList))
	
	print
	# For 5th HuMoment value of 2nd instrument (Total 8 HuMoment Values for each)
	# HuMomentsList[4,1]
	# For X value value of 5th instrument (X:0 Y:1)
	# CentroidsList[0,4]
	print
	print"Centroids:"
	print CentroidsList
	print 
	print "HuMoments:"
	print HuMomentsList 
	print
	
	# HSV Cutoffs
	hsvcutoffs = numpy.array([[hmin], [hmax], [smin], [smax], [vmin], [vmax]])
	hsvcutoffs_copy = hsvcutoffs.copy()
	for i in range(tc - 1):
		hsvcutoffs = numpy.hstack((hsvcutoffs, hsvcutoffs_copy.copy()))
	
	# Display Image
	print "Press Key to Continue"	
	cv2.imshow('FinalImage', blur_copy)
	cv2.waitKey(0)
	
	print "Tool Values:"
	print "Arbitary: 0 Scalpel:1 Retractor:2 Hemostat:3 Scissors:4 Hook:5 Needle:6"
	# Get the tool values and append to Hu Moments
	tool_values = numpy.array([0.0])
	for i in range(tc):
		value = raw_input("Enter value of tool %s:" % (i + 1))
		value = numpy.array([ float(value[(len(value) - 1)])  ])
		tool_values = numpy.hstack((tool_values, value))
		
	tool_values = tool_values[1 : (tc+1)]
	
	HuMomentsAndToolValues = numpy.vstack((HuMomentsList, tool_values))
	HuMomentsAndToolValues = numpy.vstack((HuMomentsAndToolValues, hsvcutoffs))
	
	# Add file to ANNData
	for i in range(tc):
		cc = len(glob(join("ANNData", "*.txt")))
		name_file = "Data" + str(cc + 1) + ".txt"
		full_name = join("ANNData", name_file)
			
		data = 	HuMomentsAndToolValues[:, i]
		print "Data added to %s" % full_name
		numpy.savetxt(full_name, data)	
	
	cv2.destroyAllWindows()
	cv2.waitKey(0)

if __name__ == '__main__':
	sys.exit(main())
