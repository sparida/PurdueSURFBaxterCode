from pybrain.tools.shortcuts import buildNetwork
from pybrain.datasets import SupervisedDataSet
from pybrain.supervised.trainers import BackpropTrainer
import numpy
import sfml as sf
from os.path import join
from glob import glob
import cPickle
import time

net = buildNetwork(8, 30, 6, bias=True)
ds_training = SupervisedDataSet(8, 6)
ds_validation = SupervisedDataSet(8, 6)

cc = len(glob(join("TestANNData", "*.txt")))
start = 1
end = 240
for i in range(start, end + 1):

	name_file = "Data" + str(i + 1) + ".txt"
	full_name = join("TestANNData", name_file)
	d = numpy.loadtxt(full_name)
	output = numpy.zeros(6)
	output[(int(d[8]) - 1)] = 1
	ds_training.addSample([d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]], [output[0], output[1], output[2], output[3], output[4], output[5]])
	print ds_training

start = 241 
end = 300
for i in range(start, end + 1):

	name_file = "Data" + str(i + 1) + ".txt"
	full_name = join("TestANNData", name_file)
	d = numpy.loadtxt(full_name)
	output = numpy.zeros(6)
	output[(int(d[8]) - 1)] = 1
	ds_validation.addSample([d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]], [output[0], output[1], output[2], output[3], output[4], output[5]])
	print ds_validation


print
print "Press C to train ANN"
while(not sf.Keyboard.is_key_pressed(sf.Keyboard.C)):
	pass
print



trainer = BackpropTrainer(net, ds_validation, verbose=True, learningrate = 0.1)
trainer.trainUntilConvergence(verbose=True, trainingData=ds_training, validationData=ds_validation, convergence_threshold=10, maxEpochs = 20000)
print "Training Complete"
print

print "Press C to store ANN"
while(not sf.Keyboard.is_key_pressed(sf.Keyboard.C)):
	pass
print
time.sleep(1)

ANNFile = open("ANNFile.dat", "w");
cPickle.dump(net, ANNFile);
ANNFile.close()
time.sleep(1)

print
print "ANN Stored"

