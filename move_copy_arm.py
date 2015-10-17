#!/usr/bin/env python
import argparse
import sys
import os
import cPickle
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

import baxter_interface

class Dummy:
	def __init__(self):
		self._flag = '0'
		
	def _callback(self, data):
		self._flag = data
	
	def startSub(self):
		self._sub = rospy.Subscriber('/flag_topic', String, self._callback)

	def getFlag(self):
		return str(self._flag)

def moveArm(self, objPosition, speed=0.5, wait=2.0):
	control_arm.set_joint_position_speed(speed)
	control_arm.move_to_joint_positions(objPosition, wait)
       

def send_image(path):
    
    img = cv.LoadImage(path)
    msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
    pub.publish(msg)
    # Sleep to allow for image to be published.
    rospy.sleep(1)       

def main():

     
	rospy.init_node("move_copy_arm")
	obj = Dummy()
	obj.startSub()
	images = ['Welcome.png', 'Gesture1.png', 'Gesture2.png', 'Gesture3.png', 'Gesture4.png']
	posFile = open("posFile.dat", "r")
	posArmName = cPickle.load(posFile)
	
	points = []
	for i in range(0, 9):
		points.append(cPickle.load(posFile))
	posFile.close()
	
	control_arm = baxter_interface.limb.Limb(posArmName)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		flag = obj.getFlag()
		
			
		if flag == '1':
			send_image('Gesture1.png')
		elif flag == '2':
			send_image('Gesture1.png')
		elif flag == '3':
			send_image('Gesture1.png')
		elif flag == '4':
			send_image('Gesture1.png')
		else:
			send_image('Welcome.png')
		print flag
		#rate.sleep()

	return 0
if __name__ == '__main__':
    sys.exit(main())
