#!/usr/bin/env python

"""
-----------------------------------------
Description:
Class and methods to help with Baxter's arm buttons

-----------------------------------------
Usage from commandline (As diagnostic tool):
$ rosrun baxter_button_help -l left   

-----------------------------------------
Usage in code:

from baxter_button_help import BaxterButtonHelp

bbhObj = BaxterButtonHelp("left")
buttonStates = bbhObj.getButtonStates()
cButtonPressed = buttonStates['cState']        
if(cButtonPressed):
	...
	Other Commands
	...

------------------------------------------
Buttons available:
cState : Circular button on Baxter cuff
lState : Long button on Baxter cuff
bState : Big rotating button on arm link next to Baxter cuff 

------------------------------------------
Author : Sthitapragyan Parida (Sid)
Date   : June 6, 2014 1:21 PM                           

-------------------------------------------
"""


import argparse
import sys
import rospy
from std_msgs.msg import (
    UInt16,
)
import baxter_interface
import baxter_interface.digital_io as DIO

class BaxterButtonHelp(object):

    def __init__(self, limb):
        
        self._circleButtonName = limb + "_lower_button"
        self._longButtonName = limb + "_upper_button"
	self._bigButtonName = limb + "_itb_button0"
		
	import baxter_interface.digital_io as DIO

    def importStatements(self):
	import argparse
	import sys
	import rospy
	from std_msgs.msg import (UInt16)
	import baxter_interface
	import baxter_interface.digital_io as DIO
        
    def getButtonStates(self):

        self._cButton = DIO.DigitalIO(self._circleButtonName)
        self._lButton = DIO.DigitalIO(self._longButtonName)
	self._bButton = DIO.DigitalIO(self._bigButtonName)

        return {'cState':self._cButton.state, 'lState':self._lButton.state, 'bState':self._bButton.state}

         
            
def main():

    # Parse commandline arguments
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        "-l", "--limb", required=True, choices=['left', 'right'],
        help="Specify Control Limb"
    )
   
    args = parser.parse_args(rospy.myargv()[1:])

    # Initialize node to print values

    rospy.init_node("Baxter_Button_Help_Node")
    
    # Initialize variables to get button states

    bbhObj = BaxterButtonHelp(args.limb)

    buttonStates = bbhObj.getButtonStates()

    cButtonPressed = buttonStates['cState']
    lButtonPressed = buttonStates['lState']        
    bButtonPressed = buttonStates['bState']

    print

    # Print Baxter Arm 

    print "Baxter Arm: %s" % args.limb

    # Print button states

    if(cButtonPressed):
	print "Circular Button IS pressed."
    else:
	print "Circular Button is NOT pressed."

    if(lButtonPressed):
	print "Long Button IS pressed."
    else:
	print "Long Button is NOT pressed."

    if(bButtonPressed):
	print "Big Button IS pressed."
    else:
	print "Big Button is NOT pressed."
   
    print
   
    return 0

if __name__ == '__main__':
    sys.exit(main())

