#!/usr/bin/env python

"""
-----------------------------------------
Description:
Class and methods to help with Inverse Kinematics for Baxter's arms
Input: Cartesian Coordinates
Output: Baxter Python API compatible Joint Space Values

-----------------------------------------
Usage from commandline (As diagnostic tool for left arm):
$ rosrun baxter_examples ik_solve_baxter  

-----------------------------------------
Usage in code:

from ik_solve_baxter import IKSolveBaxter

arm = IKSolveBaxter("left")
arm.setArmEndPos(x, y, z, qx, qy, qz, qw)
arm.ik_solve();
joints = arm.getBaxterJointSpace()

print joints

Now joints can be directly used in baxter_interface command move_to_position() [Baxter Python API]
control_arm = baxter_interface.limb.Limb("left")
control_arm.set_joint_position_speed(0.8) # Joint movement speed
control_arm.move_to_joint_positions(joints, 3.0)

------------------------------------------
Author : Sthitapragyan Parida (Sid)
Date   : June 6, 2014 5:00 PM                           

-------------------------------------------
"""
import argparse
import struct
import sys
import baxter_interface

import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

class IKSolveBaxter(object):

	def __init__(self, limb):
		self._limb = limb
		self._ns = "ExternalTools/" + self._limb + "/PositionKinematicsNode/IKService"
    		self._iksvc = rospy.ServiceProxy(self._ns, SolvePositionIK)
    		self._ikreq = SolvePositionIKRequest()
		self._armEndPosSet = False
		self._validJointSolutionFound = False
  		self._baxterJoints = {}
		self._armEndPos = {}
		self._iksvc = rospy.ServiceProxy(self._ns, SolvePositionIK)

	def setArmEndPos(self, x, y, z, qx, qy, qz, qw):
		self._armEndPos["x"] = x
		self._armEndPos["y"] = y
		self._armEndPos["z"] = z
		self._armEndPos["qx"] = qx
		self._armEndPos["qy"] = qy
		self._armEndPos["qz"] = qz
		self._armEndPos["qw"] = qw
		self._armEndPosSet = True

	def getBaxterJointSpace(self):
		if (self._validJointSolutionFound == True):
			return self._baxterJoints
		else:
			print "Valid solution was not found. Hence returning -1"
			return -1

	def ik_solve(self):
		if(self._armEndPosSet == True):
	
  	  					
    			
    			ikreq = SolvePositionIKRequest()
    			hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    			arm_pose = PoseStamped(
        			    header=hdr,
        			    pose=Pose(
        			        position=Point(
        			           x=self._armEndPos["x"],
        			           y=self._armEndPos["y"],
        			           z=self._armEndPos["z"]
        			        ),
        			        orientation=Quaternion(
        			           x=self._armEndPos["qx"],
        			           y=self._armEndPos["qy"],
        			           z=self._armEndPos["qz"],
        			           w=self._armEndPos["qw"],
        			        ),
        			    ),
       				 )
 
			ikreq.pose_stamp.append(arm_pose)
    			try:
        			rospy.wait_for_service(self._ns, 5.0)
        			resp = self._iksvc(ikreq)
    			except (rospy.ServiceException, rospy.ROSException), e:
        			rospy.logerr("Service call failed: %s" % (e,))
        			return 1

    			# Check if result valid, and type of seed ultimately used to get solution
   			# convert rospy's string representation of uint8[]'s to int's
    			resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
    			if (resp_seeds[0] != resp.RESULT_INVALID):
        				seed_str = {
  				                  ikreq.SEED_USER: 'User Provided Seed',
  				                  ikreq.SEED_CURRENT: 'Current Joint Angles',
  				                  ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
  				                 }.get(resp_seeds[0], 'None')

					self._validJointSolutionFound = True
        				print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" % (seed_str,))
					
					# Format solution into Limb API-compatible dictionary
					self._baxterJoints = dict(zip(resp.joints[0].name, resp.joints[0].position))
			else:
        				print("INVALID POSE - No Valid Joint Solution Found.")

		else:
			print "Arm End Position is not set. Please set Arm End Position by using setArmEndPos()"

        

def main():
    
    # Initializes left arm variable and calls class fucntions to solve IK
    arm = IKSolveBaxter("left")
    arm.setArmEndPos(0.5, 0.5, 0, 0.140368694896, 0.9899737577306, 0.0098400631999, 0.0248863560409);
    arm.ik_solve();
    joints = arm.getBaxterJointSpace()
	
    # Prints joints and uses baxter interface to move arm to appropriate joints
    print joints
    control_arm = baxter_interface.limb.Limb("left")
    control_arm.set_joint_position_speed(0.8)
    control_arm.move_to_joint_positions(joints, 3.0)

    return 0

if __name__ == '__main__':
    rospy.init_node("ik_solve_baxter")
    sys.exit(main())
