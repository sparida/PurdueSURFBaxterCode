#!/usr/bin/env python

"""
Baxter RSDK Inverse Kinematics Example
"""

from __future__ import division
from orientation_help import VectorOrientation

import warnings
import math

import numpy
import argparse
import struct
import sys
import baxter_interface
import math
import rospy
from tf.transformations import *
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
# All orientation calclulations
voObj = VectorOrientation() 
voObj.setVectorChange(0.0, -1.0, 0.0) # X, Y and Z Vector Axis Orientation
#voObj.setAngleOrientation(-20, 0, 0,  'd')
voObj.calcVectorOrientation()

# Set gamma in degrees
voObj.setGamma(90, 'd')

# Trick for baxter orientation change for rigth arm:
voObj.setAngleOrientation(-voObj._theta, voObj._phi, voObj._gamma, 'r')

# Calculate quaternion and print angle orienttaion in degrees

deg = voObj.getAngleOrientation('d')
print deg

quat = voObj.getQuaternionOrientation()
print quat


def ik_test(limb):
    rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
               	    x=0.407452355331,
                    y=0.309120991482,
                    z=0.099060338914
                ),
                orientation=Quaternion(
                    x = quat[0],
                    y = quat[1],
                    z = quat[2],
                    w = quat[3],
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.807099581086,
                    y=0.20744601124,
                    z=0.301341861796
,
                ),
                orientation=Quaternion(
			
		    x = quat[0],
                    y = quat[1],
                    z = quat[2],
                    w = quat[3],
                ),
            ),
        ),
    }

    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print "\nIK Joint Solution:\n", limb_joints
        print "------------------"
        print "Response Message:\n", resp
	control_arm = baxter_interface.limb.Limb(limb)
	control_arm.set_joint_position_speed(0.8)
	control_arm.move_to_joint_positions(limb_joints, 7.0)
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    return 0


def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-l', '--limb', choices=['left', 'right'], required=True,
        help="the limb to test"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    return ik_test(args.limb)

if __name__ == '__main__':
    sys.exit(main())

