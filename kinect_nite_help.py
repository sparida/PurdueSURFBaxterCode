#!/usr/bin/env python
import roslib
roslib.load_manifest('baxter_examples')
import rospy
import tf
from tf import TransformListener
import os
import sys



"""
-----------------------------------------
Description:
Class and methods to get human arm coordinates from kinect
Output: Tuple with human user arm endpoint states in cartesian coordinates from the Kinect

-----------------------------------------
Usage from commandline (As diagnostic tool for left arm):
$ rosrun baxter_examples kinect_nite_help.py  

-----------------------------------------
Usage in code:

from kinect_nite_help import KinectNiteHelp

obj = KinectNiteHelp()
rate = rospy.Rate(10.0);
while not rospy.is_shutdown():	
		left = obj.getLeftHandPos()
		right = obj.getRightHandPos()
		try:
    			print "Left Pos  : ", left
			print "Right Pos : ", right
		
		except:
			print
		rate.sleep()
x, y and z can be accessed as left[0], left[1] and left[2] respectively.

------------------------------------------
Author : Sthitapragyan Parida (Sid)
Date   : June 9, 2014 3:57 PM                           

-------------------------------------------
"""


BASE_FRAME = '/openni_depth_frame'
FRAMES = [
        'head',
        'neck',
        'torso',
        'left_shoulder',
        'left_elbow',
        'left_hand',
        'left_hip',
        'left_knee',
        'left_foot',
        'right_shoulder',
        'right_elbow',
        'right_hand',
        'right_hip',
        'right_knee',
        'right_foot',
        ]



class KinectNiteHelp:

    def __init__(self, name='kinect_listener', user=1):
        #rospy.init_node(name, anonymous=True)
        self.tf = TransformListener()
        self.user = user

    
    def getLeftHandPos(self):
		 if self.tf.frameExists(BASE_FRAME) and self.tf.frameExists("left_hand_1"):
			print" Inside TF IF Get LEft HAnd POS "
			t = self.tf.getLatestCommonTime(BASE_FRAME, "left_hand_1")
            		(leftHandPos, quaternion) = self.tf.lookupTransform(BASE_FRAME, "left_hand_1", t)
            		return leftHandPos

    def getRightHandPos(self):
		 if self.tf.frameExists(BASE_FRAME) and self.tf.frameExists("right_hand_1"):
			t = self.tf.getLatestCommonTime(BASE_FRAME, "right_hand_1")
            		(rightHandPos, quaternion) = self.tf.lookupTransform(BASE_FRAME, "right_hand_1", t)
            		return rightHandPos
                 
 		
       

def main():
    obj = KinectNiteHelp()
    rate = rospy.Rate(10.0);
    while not rospy.is_shutdown():	
		left = obj.getLeftHandPos()
		right = obj.getRightHandPos()
		try:
    			print "Left Pos  : ", left
			print "Right Pos : ", right
		
		except:
			print
		rate.sleep()

if __name__ == '__main__':
    sys.exit(main())


