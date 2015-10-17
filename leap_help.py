#!/usr/bin/env python

"""
-----------------------------------------
Description:
Class and methods to help with Leap Motion Controller

-----------------------------------------
Usage from commandline (As diagnostic tool):
$ rosrun baxter_examples leap_help.py  

-----------------------------------------
Usage in code:

from leap_help import LeapHelp

lhObj = LeapHelp()
while(True):
	lhObj.processFrame()
	print lhObj.getIsHandEmpty()
	print lhObj.getNumFingers()
------------------------------------------
Author : Sthitapragyan Parida (Sid)
Date   : June 19, 2014 9:03 PM                           

-------------------------------------------
"""

import Leap, sys
import time
import sfml as sf
import rospy

from std_msgs.msg import (
    String,
)
class LeapHelp():
    # Class to help with leap stuff
	
    def __init__(self):
	# Controller definition	
	self._controller = Leap.Controller()
	self._controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP);
	# Add other proprites here
	self._numFingers = 0
	self._isHandEmpty = True
	self._keyTapDetected = False
	
    
    def processFrame(self):
        # Get the most recent frame and evaluate proprties decalred in __init__

        frame = self._controller.frame()
	
	self._keyTapDetected = False

        if not frame.hands.is_empty:
            # Get the first hand
	    self._isHandEmpty = False
	
            hand = frame.hands[0]

            # Check if the hand has any fingers
            fingers = hand.fingers
		
	    # Set the number of fingers appropriately
            if not fingers.is_empty:
                self._numFingers = len(fingers)
            else:
		self._numFingers = 0
		
	    for gesture in frame.gestures():
		if gesture.type == Leap.Gesture.TYPE_KEY_TAP:
			self._keyTapDetected = True
			time.sleep(0.1)
			break
		
	else:
	    self._isHandEmpty = True
	    self._numFingers = 0

    # Add functions to return properties
    def getNumFingers(self):
	    return self._numFingers

    def getIsHandEmpty(self):
	    return self._isHandEmpty

    def getKeyTapDetected(self):
	    return self._keyTapDetected

def main():
    rospy.init_node('KeyTapPublisher')
    key_pos = 0
    lhObj = LeapHelp()
    keyTapPub = rospy.Publisher('key_tap_topic', String)
    while(True):
	lhObj.processFrame()
		
	if(key_pos == 1):
		print "KeyTapON"
	else:
		print lhObj.getIsHandEmpty()
		print lhObj.getNumFingers()
	
	if(lhObj.getKeyTapDetected() == True):
		
		key_pos = key_pos + 1
		key_pos = key_pos % 2
	keyTapPub.publish(str(key_pos))
   
if __name__ == "__main__":
    main()
