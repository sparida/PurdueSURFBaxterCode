#!/usr/bin/env python

"""
-----------------------------------------
Description:
Class and methods to help with Baxter's arm orietntation
-----------------------------------------
Usage from commandline (As diagnostic tool):
$ rosrun baxter_examples orientation_help   

-----------------------------------------
Usage in code:

from orientation_help import VectorOrientation

# All orientation calclulations
voObj = VectorOrientation() 
voObj.setVectorChange(2.0, 1.0, 1.0) # X, Y and Z Vector Axis Orientation
voObj.calcVectorOrientation()

# Set gamma in degrees
voObj.setGamma(10.0, 'd')

# Trick for baxter orientation change for arms:
voObj.setAngleOrientation(-voObj._theta, voObj._phi, voObj._gamma, 'r')

# Calculate quaternion and print angle orienttaion in degrees
deg = voObj.getAngleOrientation('d')
print deg
quat = voObj.getQuaternionOrientation()

# Quaternion Representation:
qx = quat[0], qy = quat[1], qz = quat[2], qw = quat[3]

# Note for all class values: Make sure all values are floating points and not integers.

FIXES TO BE DONE: 
ISSUE 1: Change in sign of Y axis has no effect
------------------------------------------
Author : Sthitapragyan Parida (Sid)
Date   : June 16, 2014 11:21 PM                           

-------------------------------------------
"""

from __future__ import division

import warnings
import math
import numpy
import struct
import sys
import math
import rospy

from tf.transformations import *

class VectorOrientation:
	# Note for all class values: Make sure all values are floating points and not integers.
	
	def __init__(self):
		# Initialzes x, y and z axes and declares various internal varaiables	
		self._xaxis = (1, 0, 0)
		self._yaxis = (0, 1, 0)
		self._zaxis = (0, 0, 1)
		self._point1 = numpy.array([0.0, 0.0, 0.0])
		self._point2 = numpy.array([1.0, 1.0, 1.0])
		self._vector_change = self._point1 - self._point2
		self._theta = 0.0 # Rotation around x axis
		self._phi = 0.0   # Rotation around y axis
		self._gamma = 0.0 # Rotation around z axis
		
	
	def setPoint1(self, x=0.0, y=0.0, z=0.0):
		# Set point 1 coordiantes
		self._point1[0] = x
		self._point1[1] = y
		self._point1[2] = z
		self._vector_change = self._point2 - self._point1
	
	def setPoint2(self, x=1.0, y=1.0, z=1.0):
		# Set point 2 coordiantes
		self._point2[0] = x
		self._point2[1] = y
		self._point2[2] = z
		self._vector_change = self._point2 - self._point1
	
	def setAngleOrientation(self, theta = 0.0, phi = 0.0, gamma = 0.0, units = 'r'):
		# Set the angles of rotation around x, y and z axis respectively
		# If units is 'd', angles are converted to radian before assigning
		units = units.lower()
		if units == 'd':	
			self._theta = (theta/180.0 * math.pi) 
			self._phi = (phi/180.0 * math.pi)
			self._gamma = (gamma/180.0 * math.pi)
		else:
			self._theta = theta
			self._phi = phi
			self._gamma = gamma
		
	def setGamma(self, gamma = 0.0, units = 'r'):
		# Set the angle of rotation around z axis specifically
		# If units is 'd', angles are converted to radian before assigning
		units = units.lower()
		if units == 'd':	
			self._gamma = (gamma/180.0 * math.pi)
		else:
			self._gamma = gamma
	
	def setVectorChange(self, x, y, z):
		# Sets vector change and changes point2 to point1 + vector_change
		self._vector_change[0] = x
		self._vector_change[1] = y
		self._vector_change[2] = z	
		self._point2 = self._point1 + self._vector_change
		
	def getVectorChange(self):
		# Returns vector change form point1 to point2 as a numpy array
		return(numpy.array([self._point2[0] - self._point1[0], 			        
			self._point2[1] - self._point1[1],
 			self._point2[2] - self._point1[2]]))
	def calcVectorOrientation(self):
		
		# Fix for Gimbal Lock of Left Hand:
		if(self._vector_change[0] == 0) and (self._vector_change[1] == 0) and (self._vector_change[2] >= 0):
			self.setAngleOrientation(0.0, 0.0, self._gamma, 'd')
			return
	
		if(self._vector_change[0] == 0) and (self._vector_change[1] == 0) and (self._vector_change[2] < 0):
			self.setAngleOrientation(180.0, 0.0, self._gamma, 'd')
			return

		# Calculates appropriate angle of orientation of vector change from point1 and point2 
		self._vector_change = self.getVectorChange()	
		magnitude = numpy.linalg.norm(self._vector_change)
		self._theta = math.acos(self._vector_change[2] / magnitude)
		self._phi = math.asin(self._vector_change[0] / (magnitude * math.sin(self._theta)))
		
		# ISSUE 1 Temporary Fix
		if(self._vector_change[1] < 0):
			self._phi = math.pi - self._phi
		
		# print self._vector_change
		# print magnitude
		# self._gamma = self._gamma
	
	def getAngleOrientation(self, units = 'r'):
		# Returns theta(rotation about x axis) and phi(rotation about y axis) and gamma(rotation about y axis) as a numpy array
		# In degrees if units is 'd' else radians if units is 'r' or unspecified
		# Access by indexing
		# vect_orient[0] = theta
		units = units.lower() 
		vect_orient = numpy.array([self._theta, self._phi , self._gamma])
		if units == 'd':
			return (vect_orient / math.pi * 180.0)
		else:
			return (vect_orient)
		
	def getQuaternionOrientation(self):
		# Returns quaternion represtion of orienation from theta, phi and gamma as a list[]
		# x = quat[0], y = quat[1], z = quat[2], w = quat[3] 
		
		Rx = rotation_matrix(self._theta, self._xaxis)
		Ry = rotation_matrix(self._phi, self._yaxis)
		Rz = rotation_matrix(self._gamma, self._zaxis)
		R = concatenate_matrices(Rx, Ry, Rz)
		quat = quaternion_from_matrix(R)
		return quat

def main():
	voObj = VectorOrientation() 
	voObj.setVectorChange(1.0, 0.0, 1.0)
	voObj.calcVectorOrientation()
	voObj.setAngleOrientation(-voObj._theta, voObj._phi, voObj._gamma, 'r')
	deg = voObj.getAngleOrientation('d')
	quat = voObj.getQuaternionOrientation()
	print deg
	print quat	

if __name__ == '__main__':
	
	sys.exit(main())
