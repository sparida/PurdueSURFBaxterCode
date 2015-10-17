#!/usr/bin/env python

"""
-----------------------------------------
Description:
Class and methods to help with Endpint state for Baxter's arms
Output: Dictionary with Baxter arm endpoint states in cartesian coordinates and quaternion orientation

-----------------------------------------
Usage from commandline (As diagnostic tool for left arm):
$ rosrun baxter_examples baxter_arm_endpoint.py -l left  

-----------------------------------------
Usage in code:

from baxter_arm_endpoint import BaxterArmEndpoint

obj = BaxterArmEndpoint(args.limb)
obj.startSubscriber()
rate = rospy.Rate(10.0);
while not rospy.is_shutdown():	
	endpoint = obj.getArmEndPos()  
	print endpoint
	rate.sleep()

endpoint is  a dictionary with entries x, y, z, qx, qy, qz and qw.

------------------------------------------
Author : Sthitapragyan Parida (Sid)
Date   : June 9, 2014 1:00 PM                           

-------------------------------------------
"""

import argparse
import sys
import baxter_interface
import rospy

from baxter_core_msgs.msg import EndpointState
from std_msgs.msg import Header

class BaxterArmEndpoint(object):

	def __init__(self, limb):
		
		# Initializes arm endpoint object and appropriate variables
		self._limb = limb		
		self._tn = "/robot/limb/" + self._limb + "/endpoint_state"
    		self._armEndPosFound = False
		self._armEndPos = {} # Empty dictinary initialized later
			

	def setArmEndPos(self, x, y, z, qx, qy, qz, qw):
		
		# Sets arm end position into dictionary. An internal function used by other functions
		self._armEndPos["x"] = x
		self._armEndPos["y"] = y
		self._armEndPos["z"] = z
		self._armEndPos["qx"] = qx
		self._armEndPos["qy"] = qy
		self._armEndPos["qz"] = qz
		self._armEndPos["qw"] = qw
		self._armEndPosFound = True
		

	def getArmEndPos(self):
		
		# Returns arm end position if valid arm end position is found else returns -1
		if (self._armEndPosFound == True):
			return self._armEndPos
		else:
			print "Valid arm end position was not found. Hence returning {error:-1}"
			return {"error" : -1}
	
	def _Callback(self, data):
		
		# Calls setArmPos to change data recieved from ROS message to a dictionary
		self.setArmEndPos(
				data.pose.position.x, 
				data.pose.position.y, 
				data.pose.position.z, 
				data.pose.orientation.x, 
				data.pose.orientation.y, 
				data.pose.orientation.z, 
				data.pose.orientation.w,  
				)
		

	def startSubscriber(self):
		# Starts subscriber to endpoint state topic
		rospy.Subscriber(self._tn, EndpointState, self._Callback)
		
		
  	  		
    			
    			  			
        

def main():

    # Parse command line arguments and make limb a requisite argument
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        "-l", "--limb", required=True, choices=['left', 'right'],
        help="Specify Control Limb"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    # Initialize object and call appropriate functions to get arm endpoint position
    obj = BaxterArmEndpoint(args.limb)
    obj.startSubscriber()
    rate = rospy.Rate(10.0);
    while not rospy.is_shutdown():	
		endpoint = obj.getArmEndPos()
    		print endpoint
		rate.sleep()
    

    return 0

if __name__ == '__main__':
    # Start a ROS node and run main fucntion
    rospy.init_node("baxter_arm_endpoint")
    sys.exit(main())
