import os
import sys
import numpy
import itertools
import cPickle
import cv2
import time
from os.path import join
from collections import Counter
from pybrain.tools.shortcuts import buildNetwork

tool_names = {'1':'Scalpel', '2':'Retractor', '3':'Hemostat','4':'Scisscors', '5':'Hook', '6':'Needle'}
a_table = numpy.loadtxt("ANNAccuracyTable.txt")

data_start = 301
data_stop = 400

num_ann = int(raw_input("Enter number of neurons in each batch: "))
min_accuracy = int(raw_input("Enter minimum accuracy threshold: "))

NNList = map(lambda x: str(x), range(5, 51, 5))
LRList = map(lambda x: str(x).zfill(3), [1] + range(5, 101, 5))
nnt_table = [([None] * (len(NNList))) for i in range(len(LRList)) ]
data_list = [None] * (data_stop - data_start + 1)
accuracy_matched_batches = []

for lr_index, lr_value in enumerate(LRList, start = 0):
	for nn_index, nn_value in enumerate(NNList, start = 0):
		outerfolder = "TrainedANNNets"
		foldername = "LR0P" + lr_value + "Iter10000"
		filename = "ANNP" + lr_value + "File" + nn_value + ".dat"
		nnt_name = join(outerfolder, foldername, filename)
		
		ANNFile = open(nnt_name, "r")
		net = cPickle.load(ANNFile)
		ANNFile.close()

		nnt_table[lr_index][nn_index] = net
print "Nets Initialized"

for i in range(len(data_list)):
	data_name = 'Data' + str(i + data_start) + ".txt"
	data_name = join("TestANNData", data_name)
	d = numpy.loadtxt(data_name)
	data_list[i] = d
print "Data Loaded"
time.sleep(2)

LRCombinations = list(itertools.combinations(LRList, num_ann))
NNPermutations = list(itertools.permutations(NNList, num_ann))
print "Expected Batch Iterations: %s" % (len(LRCombinations) * len(NNPermutations))
time.sleep(2)
print

batch_count = 0
accuracy_match_count = 0
for i_idx, i_val in enumerate(LRCombinations, start = 0):
	for j_idx, j_val in enumerate(NNPermutations, start = 0):
		ac = 0
		tc = 0	
		batch_count += 1

		for i in range(data_stop - data_start + 1):
			d = data_list[i]
					
			print "Batch#: %s   Test Data: #%s " % (batch_count, i+1)
			print "LearningRate NeuronNumber Weight(Accuracy)"
			detections = numpy.zeros(num_ann)
		
			for k_idx, k_val in enumerate(range(num_ann), start = 0):
				
				lr_index = LRList.index(LRCombinations[i_idx][k_idx])
				nn_index = NNList.index(NNPermutations[j_idx][k_idx])
				net = nnt_table[lr_index][nn_index]

				row = list(a_table[:, 0]).index(float(LRCombinations[i_idx][k_idx]))
				col = list(a_table[0, :]).index(float(NNPermutations[j_idx][k_idx]))
				weight = a_table.item((row, col))	
				print LRCombinations[i_idx][k_idx], NNPermutations[j_idx][k_idx], weight
				output = net.activate([d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]])
				numpy_array = numpy.array(output)
				ind = numpy.argmax(numpy_array) + 1
				detections[k_idx] = int(ind)
			
			tc += 1
			tool_id = Counter(detections).most_common(1)[0][0]
			
			print
			if (int(d[8])) == tool_id:
				ac += 1			


		if ac >= min_accuracy:
			
			print ac
			accuracy_match_count += 1
			neurons_in_batch = []	
			for k_idx, k_val in enumerate(range(num_ann), start = 0):
				specific_net = [LRCombinations[i_idx][k_idx], NNPermutations[j_idx][k_idx], ac]
				neurons_in_batch.append(specific_net)

			accuracy_matched_batches.append(neurons_in_batch)


print "Accuracy Matched Network Batches:"
for i in range(accuracy_match_count):
	print "Batch#%s:" % (i+1)
	print "LR  NN  WT"
	print accuracy_matched_batches[i]

print
print "Batches Found Count:"
print accuracy_match_count
