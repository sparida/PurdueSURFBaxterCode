#!/usr/bin/env python
from __future__ import division
import csv
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
from std_msgs.msg import (
    String,
)
import Leap, sys
import baxter_interface
import baxter_interface.digital_io as DIO
from leap_help import LeapHelp
from demo26_help import Demo26Help
from kinect_nite_help import KinectNiteHelp
from baxter_arm_endpoint import BaxterArmEndpoint
from svd_rt import *
from baxter_button_help import BaxterButtonHelp
import sfml as sf
import json

import rospy

from std_msgs.msg import (
   String
)
import cv
import cv_bridge

from sensor_msgs.msg import (
    Image,
)

import math
import numpy
import math
from symbol_eval import *
from hmm import HMM
ARM = "left" # Arm to use from baxter and human, also change getleftHandPos function in knObj call when changing to right


    

class Demo26Master:

    def _callback(self, data):
		self._startGesture = data.data
	
    def __init__(self, limb, name='Demo_26_Final'):
        rospy.init_node(name, anonymous=True)
	self._startGesture = '0'
	self._limb = limb
        self._knhObj = KinectNiteHelp()
	self._baeObj = BaxterArmEndpoint(self._limb)
	
	self._dmObj = Demo26Help()
	
	self._lhObj = LeapHelp()
	
	self._handCoordinates = []
	self._baxterCoordinates = []
	self._posCount = 0
	rtMatFile = open("RTMatFile.dat", "r")
	self._rotMat = cPickle.load(rtMatFile)
	self._transMat = cPickle.load(rtMatFile)
	self._gCount = 0
	self._gOn = 0
	self._gPointCount = 0
	self._gPoints = []
	self._hmmObjG1 = HMM('G1.hmm')
	self._hmmObjG2 = HMM('G2.hmm')
	self._hmmObjG3 = HMM('G3.hmm')
	self._hmmObjG4 = HMM('G4.hmm')
	self._flag = '0'
	self._pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
	#self._flagPub = rospy.Publisher('flag_topic', String)
	self._sub = rospy.Subscriber('/key_tap_topic', String, self._callback) 
	rtMatFile.close()
	img = cv.LoadImage('Welcome.png')
    	msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
    	self._pub.publish(msg)
    	# Sleep to allow for image to be published.
    	rospy.sleep(3)
	
	
    
    def runMaster(self):
	
	# Get and store kinect coordinates
	try:
		hand_endpoint = self._knhObj.getLeftHandPos()
		#print hand_endpoint
		self._handCoordinates = array([[hand_endpoint[0]], [hand_endpoint[1]], [hand_endpoint[2]]])
		(rows, cols) = self._rotMat.shape
		print	
			
		#print self._rotMat
		#print self._handCoordinates
		#print self._transMat
		self._baxterCoordinates = self._rotMat * mat(self._handCoordinates) + self._transMat
		print "Hand Position: %s" %(self._baxterCoordinates)
		#print "Gold Man"

		
	except:
		print "Error in kinect_nite_help part."	
	
	## Cd eto set left wo and w1 value
	try:

		self._dmObj.setHandPoint(self._baxterCoordinates[0], self._baxterCoordinates[1], self._baxterCoordinates[2])	
		self._dmObj.calcAngleOrientation()
		angles = self._dmObj.getAngleOrientation('r')
		
		self._lhObj.processFrame()
		
		handEmpty = self._lhObj.getIsHandEmpty()
		print "Hand Empty: %s" %(handEmpty)
		if handEmpty == True:
			fCount = 0
		else:
			fCount = self._lhObj.getNumFingers()
			
		print "Finger Count: %s" %(fCount)
		print "GestureState: %s" %(self._startGesture)
	except:
		print "Error in joint angle set part"
	try:
		
		if (self._startGesture == '1'):
			if(self._gOn == 0):
				self._gCount += 1
				self._gOn = 1
				img = cv.LoadImage('GestureRecord.png')
    				msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
    				self._pub.publish(msg)
    				# Sleep to allow for image to be published.
    				rospy.sleep(2)
			if (fCount == 0) or (fCount == 2) or (fCount == 4):
				fCount = fCount + 1
			handPoint = [self._baxterCoordinates[0, 0], self._baxterCoordinates[1, 0], self._baxterCoordinates[2, 0], fCount]
			print "Point Appended"
			self._gPoints.append(handPoint)
			print self._gPoints
			print self._gOn
			self._gPointCount += 1
			
			
		elif(self._startGesture == '0'):
			print "Else"
			if(self._gOn == 1):
				
				self._gPointCount = 0
				self._gOn = 0
				img = cv.LoadImage('GestureAnalyze.png')
    				msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
    				self._pub.publish(msg)
    				# Sleep to allow for image to be published.
    				rospy.sleep(3)

				print "recorded"
								
				# Process points
				print "GPoINTS"
				print self._gPoints
				
				symbols = getsym(numpy.matrix(self._gPoints))
				
				
				log_prob = [self._hmmObjG1.viterbi(symbols), self._hmmObjG2.viterbi(symbols), self._hmmObjG3.viterbi(symbols), self._hmmObjG4.viterbi(symbols)] 
				
				print log_prob
				max_value = max(log_prob)
				max_index = log_prob.index(max_value)
				self._gPoints = [] 
				# Do gesture recognised processing
				self._flag = str(max_index + 1)
				gflag = max_index + 1
				
				if gflag == 1:
					#while(not sf.Keyboard.is_key_pressed(sf.Keyboard.R_SHIFT)):
					#	print max_index + 1
					#	print log_prob
					img = cv.LoadImage('Gesture1.png')
    					msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
    					self._pub.publish(msg)
    					# Sleep to allow for image to be published.
    					rospy.sleep(3)   
					
				elif gflag == 2:
					#while(not sf.Keyboard.is_key_pressed(sf.Keyboard.R_SHIFT)):
					#	print max_index + 1
					#	print log_prob
					
					img = cv.LoadImage('Gesture2.png')
    					msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
    					self._pub.publish(msg)
    					# Sleep to allow for image to be published.
    					rospy.sleep(3)
				elif gflag == 3:
					#while(not sf.Keyboard.is_key_pressed(sf.Keyboard.R_SHIFT)):
					#	print max_index + 1
					#	print log_prob
					img = cv.LoadImage('Gesture3.png')
    					msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
    					self._pub.publish(msg)
    					# Sleep to allow for image to be published.
    					rospy.sleep(3)

				elif gflag == 4:
					#while(not sf.Keyboard.is_key_pressed(sf.Keyboard.R_SHIFT)):
					#	print max_index + 1
					#	print log_prob
					img = cv.LoadImage('Gesture4.png')
    					msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
    					self._pub.publish(msg)
    					# Sleep to allow for image to be published.
    					rospy.sleep(3)

				else:
					#while(not sf.Keyboard.is_key_pressed(sf.Keyboard.R_SHIFT)):
					#	print "You are welcome. No gesture detected"
					#	print log_prob
					img = cv.LoadImage('Welcome.png')
    					msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
    					self._pub.publish(msg)
    					# Sleep to allow for image to be published.
    					rospy.sleep(3)

				
								
		

	

	except Exception,e: 
		print str(e)
		print "Error in baxter move part"    
 		
       

def main():
	
    
   masterObj = Demo26Master(ARM)
   rate = rospy.Rate(10)
   while not rospy.is_shutdown():	
		masterObj.runMaster()
		#masterObj._flagPub.publish(masterObj._flag)
		rate.sleep()

if __name__ == '__main__':
    sys.exit(main())


