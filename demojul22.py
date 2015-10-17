#!/usr/bin/env python
import argparse
import sys
import os
import cPickle
import json
import os
import rospy
import time
import baxter_interface
import Leap

from leap_help import LeapHelp

import cv
import cv_bridge

from sensor_msgs.msg import (
    Image,
)


def send_image(path):
    
	img = cv.LoadImage(path)
	msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
	pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
	pub.publish(msg)
	# Sleep to allow for image to be published.
	rospy.sleep(1)


def get_positions(color = 'blue', limb = 'left'):
	name = limb + "PosFile1.dat"
	print name
	posFile = open(name, "r")
	#dummy_load = cPickle.load(posFile)
	
	if(color == 'blue'):
		dummy_num = 0
	elif(color == 'green'):
		dummy_num = 2
	elif(color == 'yellow'):
		dummy_num = 4
	elif(color == 'red'):
		dummy_num = 6
	
	posFile = open(name, "r")
	
	
	count = -1
	while(count < dummy_num):
		pos = cPickle.load(posFile) 
		count += 1
	
	
	first_point = cPickle.load(posFile)
	second_point = cPickle.load(posFile)
	
	posFile.close()
	
	return (first_point, second_point)
def main():
	rospy.init_node("Demo_Jul_22")
	color_num = {'1':'blue', '2':'green', '3':'yellow', '4':'red'}
	color = 'black'
	place_num = 0
	used_nums = [0, 5] # Fingers not to be processed
	
	posFile = open('RestPosFile.dat', "r")
	dummy_load = cPickle.load(posFile)
	rest_pos = cPickle.load(posFile)
	posFile.close()	
	
	print "Initial Calibration"
	ll = baxter_interface.limb.Limb("left")
	rl = baxter_interface.limb.Limb("right")
	
	lg = baxter_interface.gripper.Gripper("left")
	rg = baxter_interface.gripper.Gripper("right")
	
	ll.set_joint_position_speed(0.9)
	rl.set_joint_position_speed(0.9)	
	
	lg.calibrate()
	rg.calibrate()
	
	lg.open()
	rg.open()

	ll.move_to_neutral(5)
	rl.move_to_neutral(5)
	
	key_pos = 0
	lhObj = LeapHelp()
	
	color = 'black'
	place_num = 0

	send_image('DemoJuly22Images/DemoJul22Registering.png')

	while(place_num <= 3):
		lhObj.processFrame()
		print "Fingers: %s" % (lhObj.getNumFingers())
		if(lhObj.getKeyTapDetected() == True):
			
			send_image('DemoJuly22Images/DemoJul22ColorSelect.png')		
			print "Extend Fingers:"
			
			key_pos = key_pos + 1
			key_pos = key_pos % 2
		
			time.sleep(2)
			lhObj.processFrame()
			finger_count = lhObj.getNumFingers()
				
			if finger_count not in used_nums:
				used_nums.append(finger_count)
				color = color_num[str(finger_count)]
				
				
				image_name = 'DemoJuly22Images/DemoJul22' + color + '.png'
				send_image(image_name)

				print "FingerCount:"
				print finger_count
				print "Color:"
				print color
				
				place_num += 1
				print "Place Num:"
				print place_num
	
			else:
				send_image('DemoJuly22Images/DemoJul22AnotherColor.png')
				print"Selected color has alrady been assembled"
	
			time.sleep(1)
		
		if color == 'black':
			continue
	
		ll = baxter_interface.limb.Limb("left")
		rl = baxter_interface.limb.Limb("right")
		
		lg = baxter_interface.gripper.Gripper("left")
		rg = baxter_interface.gripper.Gripper("right")
		
		ll.set_joint_position_speed(0.9)
		rl.set_joint_position_speed(0.9)	
		
		lg.calibrate()
		rg.calibrate()
		
		lg.open()
		rg.open()
		
		ll.move_to_neutral(5)
		rl.move_to_neutral(5)
		
		
		send_image('DemoJuly22Images/DemoJul22PickingParts.png')	
		print "Moving Left Arm"
		(j1, j2) = get_positions(color, 'left')
		
		ll.set_joint_position_speed(0.7)
		ll.move_to_neutral(timeout = 5.0)
		ll.move_to_joint_positions(j1, 5.0)
		ll.set_joint_position_speed(0.5)
		ll.move_to_joint_positions(j2, 3.0)
		lg.close()		
		ll.move_to_joint_positions(j1, 3.0)
		ll.set_joint_position_speed(0.7)
		ll.move_to_neutral(timeout = 5.0)
		

		print "Moving Right Arm"
		(j1, j2) = get_positions(color, 'right')
		
		rl.set_joint_position_speed(0.5)
		rl.move_to_neutral(timeout = 5.0)
		rl.move_to_joint_positions(j1, 5.0)
		rl.set_joint_position_speed(0.3)
		rl.move_to_joint_positions(j2, 3.0)
		rg.close()		
		rl.move_to_joint_positions(j1, 3.0)
		rl.set_joint_position_speed(0.5)
		rl.move_to_neutral(timeout = 5.0)
		
		
		send_image('DemoJuly22Images/DemoJul22Assembly.png')
		#time.sleep(1)
		os.system("rosrun baxter_examples joint_position_file_playback.py -f Assemble ")
		time.sleep(1)
		rg.open()
		time.sleep(1)
		rl.move_to_neutral(timeout = 5.0)
		
		send_image('DemoJuly22Images/DemoJul22PlacingProduct.png')
		#time.sleep(1)
		command = "rosrun baxter_examples joint_position_file_playback.py -f Place" + str(place_num)
		os.system(command)
		lg.open()	
		time.sleep(1)
		ll.set_joint_position_speed(0.5)
		ll.move_to_joint_positions(rest_pos, 5.0)
		ll.set_joint_position_speed(0.7)
		ll.move_to_neutral(timeout = 5.0)
		color = 'black'
		if key_pos == 1:
			color = 'black'
			send_image('DemoJuly22Images/DemoJul22Registering.png')
			key_pos = 0
		rate = rospy.Rate(10)
	
	print "Demo Complete" 
	send_image('DemoJuly22Images/DemoJul22Welcome.png')	
	return 0	
if __name__ == '__main__':
    sys.exit(main())
