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


from ik_solve_baxter import IKSolveBaxter
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
masterObj = MasterBaxterProgram(ARM)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():	
		masterObj.runMaster()
		rate.sleep()

------------------------------------------
Author : Sthitapragyan Parida (Sid)
Date   : June 9, 2014 6:40 PM                           

-------------------------------------------
"""


class MasterBaxterProgram:

    def __init__(self, limb, name='master_baxter_program'):
        rospy.init_node(name, anonymous=True)
	self._limb = limb
        self._knhObj = KinectNiteHelp()
	self._baeObj = BaxterArmEndpoint(self._limb)
	self._bbhObj = BaxterButtonHelp(self._limb)
	self._iksbObj = IKSolveBaxter(self._limb)
	self._baeObj.startSubscriber()
	
	self._handCoordinates = []
	self._baxterCoordinates = []
	self._posCount = 0
	rtMatFile = open("RTMatFile.dat", "r")
	self._rotMat = cPickle.load(rtMatFile)
	self._transMat = cPickle.load(rtMatFile)
	rtMatFile.close()
	
	self._control_arm = baxter_interface.limb.Limb(ARM)
	self._control_arm.set_joint_position_speed(0.9)
    
    def runMaster(self):
	# Get and store kinect coordinates
	try:
		hand_endpoint = self._knhObj.getLeftHandPos()
		print hand_endpoint
		self._handCoordinates = array([[hand_endpoint[0]], [hand_endpoint[1]], [hand_endpoint[2]]])
		(rows, cols) = self._rotMat.shape
		print	
			
		print self._rotMat
		print self._handCoordinates
		print self._transMat
		self._baxterCoordinates = self._rotMat * mat(self._handCoordinates) + self._transMat
		print self._baxterCoordinates
		print "Gold Man"

		
	except:
		print "Error in kinect_nite_help part."	
	
	
	
	try:
			
		self._iksbObj.setArmEndPos(self._baxterCoordinates[0, 0], self._baxterCoordinates[1, 0], self._baxterCoordinates[2, 0], -0.140368694896, 0.9899737577306, -0.0098400631999, 0.0248863560409)
		#self._iksbObj.setArmEndPos(0.5, 0, 0, -0.140368694896, 0.9899737577306, -0.0098400631999, 0.0248863560409)
		print self._iksbObj._armEndPos
		print "ArmPosSet"
		
	except:
		print "Error in ik Set part"
    
    
                 
	try:
		print 	
		joints = {}
		self._iksbObj._validJointSolutionFound = False			
		self._iksbObj.ik_solve();
		joints = self._iksbObj.getBaxterJointSpace()
		print joints
	except:
		print "Error in ik Solve part"

	try:
	
		# Move arm to space bro	
				
		self._control_arm.move_to_joint_positions(joints, 1.0)

	except:
		print "Error in Baxter Movement Part"    
 		
       

def main():
	
    
   masterObj = MasterBaxterProgram(ARM)
   rate = rospy.Rate(10)
   while not rospy.is_shutdown():	
		masterObj.runMaster()
		rate.sleep()

if __name__ == '__main__':
    sys.exit(main())


