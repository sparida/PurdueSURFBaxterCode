#!/usr/bin/env python
import argparse
import sys
import cPickle

import rospy

from std_msgs.msg import (
    UInt16,
)

import baxter_interface
import baxter_interface.digital_io as DIO

class Puppeteer(object):

    def __init__(self, limb, amplification=1.0):
        
        puppet_arm = {"left": "right", "right": "left"}
        self._control_limb = limb
        self._puppet_limb = puppet_arm[limb]
        self._control_arm = baxter_interface.limb.Limb(self._control_limb)
        self._puppet_arm = baxter_interface.limb.Limb(self._puppet_limb)
        self._amp = amplification

        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable()
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def getButtonStates(self, limb):
        objectButton = limb + "_lower_button"
        deliveryButton = limb + "_upper_button"
	neutralButton = limb + "_itb_button0"
        
        oButton = DIO.DigitalIO(objectButton)
        dButton = DIO.DigitalIO(deliveryButton)
	nButton = DIO.DigitalIO(neutralButton)

        return {'oState':oButton.state, 'dState':dButton.state, 'nState':nButton.state}

    def _reset_control_modes(self):
        rate = rospy.Rate(100)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self._control_arm.exit_control_mode()
            self._puppet_arm.exit_control_mode()
            rate.sleep()
        return True

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose.
        """
        
    def clean_shutdown(self):
        print("\nExiting example...")
        #return to normal
        self._reset_control_modes()
        self.set_neutral()
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True

    def puppet(self):
        """

        """
        self._objCount = 0
        rate = rospy.Rate(100)
        self.set_neutral()

        print ("Grab %s cuff and move arm.\n"
               "Press circle button button to set point or delivery positions respectively...") % (self._control_limb,)
	

        r = rospy.Rate(100000) # 10hz
        print

        while not rospy.is_shutdown():
            
            buttonState = self.getButtonStates(self._control_limb)
        
            if(buttonState["oState"]):
                cmdc = self._control_arm.joint_angles()
		name =  "posFile.dat"
                objPosFile = open(name, "a");
                cPickle.dump(cmdc, objPosFile);
                objPosFile.close
                rospy.sleep(1)
                self._objCount = self._objCount + 1
                print ("Added point %d to pos position file") % (self._objCount)
            
         
                
            r.sleep()


def main():

    max_gain = 3.0
    min_gain = 0.1

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        "-l", "--limb", required=True, choices=['left', 'right'],
        help="Specify Control Limb"
    )
    parser.add_argument(
        "-a", "--amplification", type=float, default=1.0,
        help=("amplification to apply to the puppeted arm [%g, %g]"
              % (min_gain, max_gain))
    )
    args = parser.parse_args(rospy.myargv()[1:])
    if (args.amplification < min_gain or max_gain < args.amplification):
        print("Exiting: Amplification must be between: [%g, %g]" %
              (min_gain, max_gain))
        return 1

    rospy.init_node("pos_file_record")
    name = "posFile.dat"
    dummyFile = open(name, "w");
    cPickle.dump(args.limb, dummyFile);
    dummyFile.close()
    puppeteer = Puppeteer(args.limb, args.amplification)
    rospy.on_shutdown(puppeteer.clean_shutdown)
    puppeteer.puppet()

    print("Done.")
    return 0

if __name__ == '__main__':
    sys.exit(main())

