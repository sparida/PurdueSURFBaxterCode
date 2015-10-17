#!/usr/bin/env python

from pybrain.tools.shortcuts import buildNetwork
import numpy
import sfml as sf
from os.path import join
from glob import glob
import cPickle
import time
import sys
import copy_reg

class ToolIdentify:
	def __init__(self):
		self._toolIDNum = -1	
		self._names = {'1':'Scalpel', '2':'Retractor', '3':'Hemostat','4':'Scisscors', '5':'Hook', '6':'Needle'}
		self._toolName = "Nothing"
		self._HuMoments = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self._centroidX = -1
		self._centroidY = -1
	
		ANNFile = open('ANNP025File10.dat', "rb")
		self._net = cPickle.load(ANNFile)
		ANNFile.close()
		

	def setHuMoments(self, HuMoments = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
		self._HuMoments = HuMoments
	
	def setNet(self, net):
		self._net = net
		
	def setNetFromFile(self, nameOfNetFile):
		ANNFile = open(nameOfNetFile, "r")
		self._net = cPickle.load(ANNFile)
		ANNFile.close()
	
	def setCentroid(self, x, y):
		self._centroidX = x
		self._centroidY = y

	def identifyTool(self):
		
		output = self._net.activate([self._HuMoments[0], self._HuMoments[1], self._HuMoments[2], 
						self._HuMoments[3],  self._HuMoments[4], self._HuMoments[5], 
						self._HuMoments[6], self._HuMoments[7]])

		numpy_array = numpy.array(output)
		self._toolIDNum = numpy.argmax(numpy_array) + 1
		self._toolName = self._names[str(self._toolIDNum)]
	
	def getToolIDNum(self):
		return self._toolIDNum
	
	def getToolName(self):
		return self._toolName

def main():
	
	obj = ToolIdentify()
	obj.setNetFromFile('ANNP025File10.dat')
	obj.setHuMoments([0, 0, 0, 0, 1, 1, 0, 0])
	obj.identifyTool()
	print obj.getToolName()
	
if __name__ == '__main__':
	sys.exit(main())
