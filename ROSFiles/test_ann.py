from pybrain.tools.shortcuts import buildNetwork
from pybrain.datasets import SupervisedDataSet
from pybrain.supervised.trainers import BackpropTrainer
import numpy
import sfml as sf
from os.path import join
from glob import glob
import cPickle
import time

names = {'1':'Scalpel', '2':'Retractor', '3':'Hemostat','4':'Scisscors', '5':'Hook', '6':'Needle'}

ANNFile = open("ANNP01File50.dat", "r")
net = cPickle.load(ANNFile)
ANNFile.close()
start = 301
stop = 400
print

ac = 0
tc = 0

for i in range(start, stop + 1):

	name = 'Data' + str(i) + ".txt"
	name = join("TestANNData", name)
	d = numpy.loadtxt(name)
	output = net.activate([d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]])
	numpy_array = numpy.array(output)
	ind = numpy.argmax(numpy_array)
	
	if (int(d[8])) == (ind + 1):
		ac += 1
	tc += 1	
	print "%s" % (i)
	print names[str(ind+1)]
	print output
	print
	

print "Accuracy: %s" % (float(ac) / tc * 100.0) 

