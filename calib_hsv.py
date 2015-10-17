#!/usr/bin/env python

import cv2
import numpy
import sfml as sf
import cPickle
import time
import sys

from os.path import join

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
def main():
	rospy.init_node('Calib_HSV')
	print 1
	cv2.namedWindow('Binary', cv2.WINDOW_NORMAL)
	cv2.namedWindow('Contour', cv2.WINDOW_NORMAL)
	cv2.namedWindow('Trackbars', cv2.WINDOW_NORMAL)
	print 1
	kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
	font = cv2.FONT_HERSHEY_SIMPLEX
	print 1	
	cv2.createTrackbar('HueMin','Trackbars', 42, 179, nothing)
	cv2.createTrackbar('HueMax','Trackbars', 179, 179, nothing)
	cv2.createTrackbar('SatMin','Trackbars', 94, 255, nothing)
	cv2.createTrackbar('SatMax','Trackbars', 255, 255, nothing)
	cv2.createTrackbar('ValMin','Trackbars', 0, 255, nothing)
	cv2.createTrackbar('ValMax','Trackbars', 255, 255, nothing)
	
	
	print 1
	tobj = moveGetCamArmImage(limb = 'right', file_name = "BaxterCamArmJointPosFile.dat")
	tobj.moveCamArm()
	img = tobj.getCamArmImage()
	icObj = ImageClipper('ImageClipPosFile.dat')
	img = icObj.clipImage(img)
	blur = cv2.GaussianBlur(img,(5,5),0)
	blur_copy = img.copy()
	
	print "Set hsv values and press c to store in HSV file."
	print "Press C to Start Calibration"
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
			tc = 0
			contours,hierarchy = cv2.findContours(dl, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			HuMomentsList = numpy.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
			delta = 400
			blur_copy = img.copy()
			blur_copy2 = dl_copy.copy()
			CentroidsList = numpy.array([[0.0], [0.0]])
			
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
					HuMoments[6] = abs(HuMoments[6]) # Make erflection invariant
					HuMoments = numpy.vstack((HuMoments, I8)) # Modified Humoments

					HuMomentsList = numpy.hstack((HuMomentsList, HuMoments))
					tc += 1

					
		
					
			print "Number of instruments:", tc
			cv2.imshow('Contour', blur_copy)
			cv2.waitKey(1)
		except:	
			pass		
		
		
	hsvcutoffs = [hmin, hmax, smin, smax, vmin, vmax]
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
	print "HSV Cutoffs:"
	print hsvcutoffs
	print
	
	HSVValFile = open('HSVValFile.dat', "w");
	cPickle.dump(hsvcutoffs, HSVValFile);
	HSVValFile.close()	
	print "HSV Cutoffs Added to HSVValFile"
	print
		
	cv2.destroyAllWindows()
if __name__ == '__main__':
	sys.exit(main())
