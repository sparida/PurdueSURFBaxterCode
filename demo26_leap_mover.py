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

ARM = "left" # Arm to use from baxter and human, also change getleftHandPos function in knObj call when changing to right

class Demo26Master:

    def __init__(self, limb, name='Demo_26_Master_Leap'):
        rospy.init_node(name, anonymous=True)
	self._limb = limb
        self._knhObj = KinectNiteHelp()
	self._baeObj = BaxterArmEndpoint(self._limb)
	self._bbhObj = BaxterButtonHelp(self._limb)
	self._dmObj = Demo26Help()
	self._baeObj.startSubscriber()
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
	rtMatFile.close()
	
	self._control_arm = baxter_interface.limb.Limb(ARM)
	self._control_arm.set_joint_position_speed(0.9)
    
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
		joint_angles = self._control_arm.joint_angles()	
		joint_angles["left_w0"] = angles[0]
		joint_angles["left_w1"] = angles[1]
		self._lhObj.processFrame()
		
		handEmpty = self._lhObj.getIsHandEmpty()
		print "Hand Empty: %s" %(handEmpty)
		if handEmpty == True:
			fCount = 0
		else:
			fCount = self._lhObj.getNumFingers()
		print "Finger Count: %s" %(fCount)
	except:
		print "Error in joint angle set part"
	try:
		
								
		# Move arm to space bro	
		#print "Gonna Move" 
		#print joint_angles		
		self._control_arm.move_to_joint_positions(joint_angles, 1.0)

	except:
		print "Error in baxter move part"    
 		
       

def main():
	
    
   masterObj = Demo26Master(ARM)
   rate = rospy.Rate(10)
   while not rospy.is_shutdown():	
		masterObj.runMaster()
		rate.sleep()

if __name__ == '__main__':
    sys.exit(main())


