import os
import sys
import numpy
import cPickle
from os.path import join
from pybrain.tools.shortcuts import buildNetwork

tool_names = {'1':'Scalpel', '2':'Retractor', '3':'Hemostat','4':'Scisscors', '5':'Hook', '6':'Needle'}

data_start = 301
data_stop = 400

NNList = map(lambda x: str(x), range(5, 51, 5))
LRList = map(lambda x: str(x).zfill(3), [1] + range(5, 101, 5))
a_table = numpy.zeros((22, 11), dtype = "int")

for index, value in enumerate(['0'] + NNList, start = 0):
	a_table[0, index] = int(value)

for index, value in enumerate(['0'] + LRList, start = 0):
	a_table[index, 0] = int(value)


for lr_index, lr_value in enumerate(LRList, start = 1):
	for nn_index, nn_value in enumerate(NNList, start = 1):
		
		outerfolder = "TrainedANNNets"
		foldername = "LR0P" + lr_value + "Iter10000"
		filename = "ANNP" + lr_value + "File" + nn_value + ".dat"
		nnt_name = join(outerfolder, foldername, filename)
		
		ANNFile = open(nnt_name, "r")
		net = cPickle.load(ANNFile)
		ANNFile.close()
		
		ac = 0
		tc = 0
		
		for i in range(data_start, data_stop + 1):
			print "Testing: LR - %s NN - %s Data - %s" % (lr_value, nn_value, i)
			data_name = 'Data' + str(i) + ".txt"
			data_name = join("TestANNData", data_name)
			d = numpy.loadtxt(data_name)
			output = net.activate([d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]])
			numpy_array = numpy.array(output)
			ind = numpy.argmax(numpy_array)
			tc += 1
		
			if (int(d[8])) == (ind + 1):
				ac += 1
		
		a_table[lr_index, nn_index] = int(float(ac) / tc * 100.0) 
		
os.system("clear")
print "Accuracy Table:"
print a_table
print len(numpy.where( a_table >= 89 )[0])
numpy.savetxt("ANNAccuracyTable.txt", a_table)
