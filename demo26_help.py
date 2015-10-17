#!/usr/bin/env python



from __future__ import division

import warnings
import math
import numpy
import struct
import sys
import math
import rospy
import baxter_interface
import sfml as sf
class Demo26Help:
	# Note for all class values: Make sure all values are floating points and not integers.
	
	def __init__(self, theta = -1.432693424, phi = 0.12163683364, lP = 40.0/100.0, lE = 26.0/100.0):
		# Initialzes x, y and z axes and declares various internal varaiables	
		self._theta = theta
		self._phi = phi
		self._lP = lP
		self._lE = lE
		self._handPoint = [0.0, 0.0, 0.0]
		self._fixedPoint = [0.407452355331, 0.309120991482, 0.099060338914]
		self._dtheta = -1.432693424
		self._dphi = 0.12163683364
		self._k1 = 6 * 2.54 / 100.0
		self._phirange = 120
	
	def setHandPoint(self, x=1.0, y=1.0, z=1.0):
		# Set point 1 coordiantes
		self._handPoint[0] = x
		self._handPoint[1] = y
		self._handPoint[2] = z
		
	
	def setAngleOrientation(self, theta = 0.0, phi = 0.0,  units = 'r'):
		
		units = units.lower()
		if units == 'd':	
			self._theta = (theta/180.0 * math.pi) 
			self._phi = (phi/180.0 * math.pi)
		else:
			self._theta = theta
			self._phi = phi
		
		self._theta = self._theta + self._dtheta
		self._phi = -self._phi + self._dphi
			
	
	def calcAngleOrientation(self):
		print "Hello"		
		if (self._handPoint[1] - self._fixedPoint[1]) > 0:
			#print "In +ve"
			self._phi = math.atan( (self._handPoint[1] - self._fixedPoint[1]) / (self._handPoint[0] - self._fixedPoint[0]))	
			pd = (self._phi * 180 /math.pi)
			if (pd >= 0) and (pd < (self._phirange/4)):
				self._phi = (self._phirange/8)/180 * math.pi
			elif (pd >= (self._phirange/4)) and (pd < (self._phirange/2)):
				self._phi = (self._phirange * 3 / 8)/180 * math.pi
			else:
				self._phi = 0
				
			#print self._phi / math.pi * 180			
			self._phi = -self._phi + self._dphi
			
		elif (self._handPoint[1] - self._fixedPoint[1]) < 0:
			#print "In -ve"
			self._phi = math.atan((self._fixedPoint[1] - self._handPoint[1]) / (self._handPoint[0] - self._fixedPoint[0]))
			pd = (self._phi * 180 /math.pi)
			if (pd >= 0) and (pd < (self._phirange/4)):
				self._phi = (self._phirange/8)/180 * math.pi
			elif (pd >= (self._phirange/4)) and (pd < (self._phirange/2)):
				self._phi = (self._phirange * 3 / 8)/180 * math.pi
			else:
				self._phi = 0
			#print self._phi / math.pi * 180
			self._phi = self._phi + self._dphi	
		else:
			#print "In 0"
			self._phi = self._dphi
		
		self._theta = math.asin((self._fixedPoint[2] - self._handPoint[2] + self._k1) / self._lP)
		
		if (self._theta) < 0:
			self._theta = 5 * math.pi /180
		
		if (self._theta / math.pi * 180) > 15:
			self._theta = 20 * math.pi /180
		
		self._theta = self._theta + self._dtheta
		
		
		
	def getAngleOrientation(self, units = 'r'):
		# In degrees if units is 'd' else radians if units is 'r' or unspecified
		# Access by indexing
		# vect_orient[0] = theta
		units = units.lower() 
		angles = numpy.array([ self._phi, self._theta])
		if units == 'd':
			return (angles / math.pi * 180.0)
		else:
			return (angles)
		
	
def main():
	control_arm = baxter_interface.limb.Limb("left")
	control_arm.set_joint_position_speed(1.0)
	obj = Demo26Help()
	obj.setHandPoint(0.80, -0.309, 0.00)
	obj.calcAngleOrientation()
	#obj.setAngleOrientation(10.0, 00.0, 'd')
	angles = obj.getAngleOrientation('r')
	print angles
	joint_angles = control_arm.joint_angles()	
	joint_angles["left_w0"] = angles[0]
	joint_angles["left_w1"] = angles[1]
	print joint_angles		
	control_arm.move_to_joint_positions(joint_angles, 1.0)
if __name__ == '__main__':
	rospy.init_node("Demo26_Help")
	sys.exit(main())
