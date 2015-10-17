#!/usr/bin/env python
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

import baxter_interface
import baxter_interface.digital_io as DIO



from kinect_nite_help import KinectNiteHelp
from baxter_arm_endpoint import BaxterArmEndpoint
from svd_rt import *
from baxter_button_help import BaxterButtonHelp

ARM = "left" # Arm to use from baxter and human, also change getleftHandPos function in knObj call when changing to right

"""
-----------------------------------------
Description:
Class and methods to caliberate baxter oordinate sysytem with kinect coordinate system and return rot and trans matrix

-----------------------------------------
Usage from commandline (As diagnostic tool for left arm):
$ rosrun baxter_examples calib_baxter_kinect.py  

-----------------------------------------
Usage in code:
calibObj = CalibBaxterKinect(ARM)
rate = rospy.Rate(10)
calibObj.openingStatement()
while not rospy.is_shutdown():	
	calibObj. calibrate()
	rate.sleep()

------------------------------------------
Author : Sthitapragyan Parida (Sid)
Date   : June 9, 2014 5:00 PM                           

-------------------------------------------
"""


class CalibBaxterKinect:

    def __init__(self, limb):
        
	# 
	self._limb = limb
        self._knObj = KinectNiteHelp()
	self._baeObj = BaxterArmEndpoint(self._limb)
	self._bbhObj = BaxterButtonHelp(self._limb)
	self._baeObj.startSubscriber()

	self._posCount = 0

	self._rotMat = []
	self._transMat = []
	self._handCoordinates = [0, 0, 0]
	self._baxterCoordinates = [0, 0, 0]
    
    def openingStatement(self):
	
	# Display opening statement with instructions for caliberation
	print "Baxter Kinect Calibration Program"
	print "Press circle button to start calibration when Baxter arm is in desired position."
	print "Press long button hen user hand is in desired position"
	print "Press big button on Baxter arm to calculate rotation and translation matrix"	
	print

    def calibrate(self):
	buttonStates = self._bbhObj.getButtonStates()
	if(buttonStates["cState"]):
		print "Registered Baxter Hand Pos."
		print "Move arm to desired pos and press long button on cuff."	
		buttonStates = self._bbhObj.getButtonStates()
		while(buttonStates["lState"] == False):
			buttonStates = self._bbhObj.getButtonStates()
			continue
		
		# Get and store baxter coordinates
		baxter_endpoint = self._baeObj.getArmEndPos()
		baxterTempPos = [baxter_endpoint["x"], baxter_endpoint["y"], baxter_endpoint["z"]]
		self._baxterCoordinates = vstack((self._baxterCoordinates, baxterTempPos))
		
		# Get and store kinect coordinates
		try:

			hand_endpoint = self._knObj.getLeftHandPos()
	
			handTempPos = [hand_endpoint[0], hand_endpoint[1], hand_endpoint[2]]
			self._handCoordinates = vstack((self._handCoordinates, handTempPos))
		except:
			print "Error in kinect_nite_help part."	

		print "Baxter and Kinect Pos %d added." % (self._posCount + 1)
		print "Current Baxter Matrix:"
		print self._baxterCoordinates
		print "Current Hand Matrix"
		print self._handCoordinates
		self._posCount = self._posCount + 1
	
	elif(buttonStates["bState"]):
		if(self._posCount >= 4):
			tempBaxterMat = mat(self._baxterCoordinates[1:])
			tempHandMat = mat(self._handCoordinates[1:])
			print tempBaxterMat
			print tempHandMat
			print "Calculate rotation and transalation matrix"
			self._rotMat, self._transMat = svd_rt(tempHandMat, tempBaxterMat)
			print "Rotation matrix:"
			print self._rotMat

			print "Translation matrix:"
			print self._transMat
			
			rtMatFile = open("RTMatFile.dat", "w");
			cPickle.dump(self._rotMat, rtMatFile)
                	cPickle.dump(self._transMat, rtMatFile);
                	rtMatFile.close()
				
			print "Rotation and Translation Matrices Stored To File"
		else:
			print "Not Enough Points to perform SVD. You need %d more points" % (3 - self._posCount)
		
			
    
    
                 
 		
       

def main():
	
    calibObj = CalibBaxterKinect(ARM)
    rate = rospy.Rate(10)
    calibObj.openingStatement()
    while not rospy.is_shutdown():	
		calibObj. calibrate()
		rate.sleep()

if __name__ == '__main__':
    rospy.init_node('calib_baxter_kinect', anonymous=True)
    sys.exit(main())


