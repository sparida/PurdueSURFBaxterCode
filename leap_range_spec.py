#!/usr/bin/env python

"""
-----------------------------------------
Description:
Display leap motion range, and vertical horizontal angle of boundary

-----------------------------------------
Usage from commandline (As diagnostic tool):
$ rosrun baxter_examples leap_range_spec.py  

-----------------------------------------
Author : Sthitapragyan Parida (Sid)
Date   : June 20, 2014 00:46 AM                           

-------------------------------------------
"""

import Leap, sys


def main():
    
	controller = Leap.Controller()
	devices = controller.devices[0]
	print devices
	"""	
	print device
	ranges = device.range
	angleOnLongDimension = device.horizontal_view_angle
	angleOnShortDimension = device.vertical_view_angle
	
	print "Range:"
	print ranges
	print "Long Dimension Angle:"
	print angleOnLongDimension
	print "Short Dimension Angle:"
	print angleOnShortDimension
	"""
	
if __name__ == "__main__":
    main()
