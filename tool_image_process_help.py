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
from hist_back_project import getForeBack
from cam_arm_pix_trans_help import moveGetCamArmImage
from cam_arm_pix_trans_help import PixelToBaxterPosTransformer
from cam_arm_pix_trans_help import ImageClipper

def nothing(x):
	pass

	
class ToolImageProcessHelp:

	def __init__(self, imgType = 'Original'):
		
		# imgType = Whtehr input image is clipe dor not
		
		# Load Image Clipping Coordinates	
		fileObj = open("ImageClipPosFile.dat", "r")	
		self._image_clip = cPickle.load(fileObj)
		fileObj.close()
		
		self._isImageClipped = (imgType == 'Clipped')
		
		# Initialize null variables
		
		self._backImg = cv2.imread('Image.png')
		self._img = -1
		self._contourImg = -1
		self._toolNum = -1
		self._HuMomentsList = -1
		self._CentroidsList = -1	
		self._smin = -1
	
	def setImage(self, img, imgType = 'Clipped'):
		self._img = img
		self._isImageClipped = (imgType == 'Clipped')
		
	def processImage_BackProject(self):
	
		# Clip Image
		if not self._isImageClipped:
			self._img = self._img[:, self._image_clip[0] : self._image_clip[2] ]
			self._img = self._img[self._image_clip[1] : self._image_clip[3], :]
		
		# Initialize kernel and font	
		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
		font = cv2.FONT_HERSHEY_SIMPLEX

		# Preprocess with gaussian blur	
		#blur = cv2.GaussianBlur(self._img,(5,5),0)
		blur = self._img.copy()
		blur_copy = self._img.copy()

		per_white, median = getForeBack(self._backImg, blur)
			
		er = cv2.erode(median, kernel, iterations = 0)
		dl = cv2.dilate(er, kernel, iterations = 1)
		er = cv2.erode(dl, kernel, iterations = 1)
		dl = er
			
		
		# Initialize necessary variables
		tc = 0 # Tool Count
		HuMomentsList = numpy.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
		CentroidsList = numpy.array([[0.0], [0.0]])
		delta = 400 # Min Arc Length of Selected Contours
		
		# Find contours, humoments and centroids
		try:
					
			# Make copies
			blur_copy = self._img.copy()
				
			# Find contours
			contours,hierarchy = cv2.findContours(dl, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			
			# Start loop to find relevant contours
			for i in range(len(contours)):
				
				# If arclength is greater than delta and contour is an outermost contour	
				if (cv2.arcLength(contours[i], True) > delta) and (hierarchy[0, i, 3] == -1):	
					
					# Draw Contours on copy				
					cv2.drawContours(blur_copy, contours, i, (255,255,255), 1)
					
					# Get all meoments upto 3rd order
					m = cv2.moments(contours[i])
					
					# calculate centroid and add to list	
					cen_x = int(m["m10"] / m["m00"])
					cen_y = int(m["m01"] / m["m00"])
					centroid = numpy.array([[cen_x], [cen_y]])
					CentroidsList = numpy.hstack((CentroidsList, centroid))
					
					# Put centroid on list
					cv2.putText(blur_copy, str(tc + 1), (cen_x, cen_y), font, 1,(0,255,100), 3, 8)
				
					# Calculate Modified HuMoments with 8th number and add to List
					HuMoments = cv2.HuMoments(m)
					I8 = m["nu11"] * ((m["nu30"]+m["nu12"])**2 - (m["nu03"]+m["nu21"])**2) - (m["nu20"]-m["nu02"])*(m["nu30"]+m["nu12"])*(m["nu03"]+m["nu21"])
					HuMoments[6] = abs(HuMoments[6]) # Make reflection invariant
					HuMoments = numpy.vstack((HuMoments, I8)) # Modified HuMoments

					HuMomentsList = numpy.hstack((HuMomentsList, HuMoments))
	
					# Increment Tool Count
					tc += 1
	
									
			
		except:	
			pass		
		
		self._contourImg = blur_copy
		self._CentroidsList = CentroidsList[:, 1:(tc + 1)]
		HuMomentsList = HuMomentsList[:, 1:(tc + 1)]
		self._HuMomentsList = -numpy.sign(HuMomentsList) * numpy.log10(numpy.abs(HuMomentsList))
		self._toolNum = tc
		# Post Process CentroidsList and HuMomentsList
		# For 5th HuMoment value of 2nd instrument (Total 8 HuMoment Values for each)
		# HuMomentsList[4,1]
		# For X value value of 5th instrument (X:0 Y:1)
		# CentroidsList[0,4]
		
	def _getHSVCutOffs(self):
		
		# This is the function that needs to be modified
		# Load HSV Filter Values
		fileObj = open("HSVValFile.dat", "r")
		hsvcutoffs = cPickle.load(fileObj)
		fileObj.close()
		return hsvcutoffs
	def setSMin(self, smin=-1):
		self._smin = smin

	def getHuMomentsList(self):
		return self._HuMomentsList

	def getCentroidsList(self):
		return self._CentroidsList
	
	def getNumTools(self):
		return self._toolNum
	
	def getOriginalImage(self):
		return self._img
	
	def getContourImage(self):
		return self._contourImg 
	
	# For 5th HuMoment value of 2nd instrument (Total 8 HuMoment Values for each)
	# HuMomentsList[4,1]
	# For X value value of 5th instrument (X:0 Y:1)
	# CentroidsList[0,4]
	
	
	def processImage(self):
	
		# Clip Image
		if not self._isImageClipped:
			self._img = self._img[:, self._image_clip[0] : self._image_clip[2] ]
			self._img = self._img[self._image_clip[1] : self._image_clip[3], :]
		
		# Initialize kernel and font	
		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
		font = cv2.FONT_HERSHEY_SIMPLEX

		# Preprocess with gaussian blur	
		blur = cv2.GaussianBlur(self._img,(5,5),0)
		blur_copy = self._img.copy()

		# Load HSV Cutfoffs
		self._hsvcutoffs = self._getHSVCutOffs() 

		hmin = self._hsvcutoffs[0]
		hmax = self._hsvcutoffs[1]
		smin = self._hsvcutoffs[2]
		smax = self._hsvcutoffs[3]
		vmin = self._hsvcutoffs[4]
		vmax = self._hsvcutoffs[5]
		
		if (self._smin != -1):
			smin = self._smin
		# Get HSV image and threshold
			
		hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
		lower_thresh = numpy.array([hmin, smin, vmin])
		upper_thresh = numpy.array([hmax, smax, vmax])
		mask = cv2.inRange(hsv, lower_thresh, upper_thresh) 

		# Invert Image		
		mask = cv2.bitwise_not(mask)
		
		# Post Processing including Erosion and Dilation	
		median = cv2.medianBlur(mask,5)
	
		er = cv2.erode(median, kernel, iterations = 0)
		dl = cv2.dilate(er, kernel, iterations = 1)
		er = cv2.erode(dl, kernel, iterations = 1)
		dl = er
			
		# Initialize necessary variables
		tc = 0 # Tool Count
		HuMomentsList = numpy.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
		CentroidsList = numpy.array([[0.0], [0.0]])
		delta = 400 # Min Arc Length of Selected Contours
		
		# Find contours, humoments and centroids
		try:
					
			# Make copies
			blur_copy = self._img.copy()
				
			# Find contours
			contours,hierarchy = cv2.findContours(dl, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			
			# Start loop to find relevant contours
			for i in range(len(contours)):
				
				# If arclength is greater than delta and contour is an outermost contour	
				if (cv2.arcLength(contours[i], True) > delta) and (hierarchy[0, i, 3] == -1):	
					
					# Draw Contours on copy				
					cv2.drawContours(blur_copy, contours, i, (255,255,255), 1)
					
					# Get all meoments upto 3rd order
					m = cv2.moments(contours[i])
					
					# calculate centroid and add to list	
					cen_x = int(m["m10"] / m["m00"])
					cen_y = int(m["m01"] / m["m00"])
					centroid = numpy.array([[cen_x], [cen_y]])
					CentroidsList = numpy.hstack((CentroidsList, centroid))
					
					# Put centroid on list
					cv2.putText(blur_copy, str(tc + 1), (cen_x, cen_y), font, 1,(0,255,100), 3, 8)
				
					# Calculate Modified HuMoments with 8th number and add to List
					HuMoments = cv2.HuMoments(m)
					I8 = m["nu11"] * ((m["nu30"]+m["nu12"])**2 - (m["nu03"]+m["nu21"])**2) - (m["nu20"]-m["nu02"])*(m["nu30"]+m["nu12"])*(m["nu03"]+m["nu21"])
					HuMoments[6] = abs(HuMoments[6]) # Make reflection invariant
					HuMoments = numpy.vstack((HuMoments, I8)) # Modified HuMoments

					HuMomentsList = numpy.hstack((HuMomentsList, HuMoments))
	
					# Increment Tool Count
					tc += 1
	
									
			
		except:	
			pass		
		
		self._contourImg = blur_copy
		self._CentroidsList = CentroidsList[:, 1:(tc + 1)]
		HuMomentsList = HuMomentsList[:, 1:(tc + 1)]
		self._HuMomentsList = -numpy.sign(HuMomentsList) * numpy.log10(numpy.abs(HuMomentsList))
		self._toolNum = tc
		# Post Process CentroidsList and HuMomentsList
		# For 5th HuMoment value of 2nd instrument (Total 8 HuMoment Values for each)
		# HuMomentsList[4,1]
		# For X value value of 5th instrument (X:0 Y:1)
		# CentroidsList[0,4]

def main():
		
	
	rospy.init_node("Tool_Image_Process_Help")
	
	tobj = moveGetCamArmImage(limb = 'right', file_name = "BaxterCamArmJointPosFile.dat")
	tobj.moveCamArm()
	img = tobj.getCamArmImage()
	icObj = ImageClipper('ImageClipPosFile.dat')
	img = icObj.clipImage(img)
	
	######ALL THAT MATTERS TO USE THIS CLASS######
	obj = ToolImageProcessHelp()
	obj.setImage(img)
	obj.processImage_BackProject()
	
	print obj.getNumTools()
	print obj.getHuMomentsList()
	print obj.getCentroidsList()
	
	# For 5th HuMoment value of 2nd instrument (Total 8 HuMoment Values for each)
	# HuMomentsList[4,1]
	# For X value value of 5th instrument (X:0 Y:1)
	# CentroidsList[0,4]
	##############################################
	cv2.imshow('HeyOriginal', obj.getOriginalImage())
	cv2.imshow('HeyContour', obj.getContourImage())
	
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	
	return 0
	
	
	
	
	
	
	
if __name__ == '__main__':
	sys.exit(main())
