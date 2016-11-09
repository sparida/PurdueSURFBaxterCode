from pybrain.tools.shortcuts import buildNetwork
from pybrain.datasets import SupervisedDataSet
from pybrain.supervised.trainers import BackpropTrainer
import numpy
from os.path import join
from glob import glob
import cPickle
import time

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


net = buildNetwork(8, 5, 6, bias=True)
trainer = BackpropTrainer(net, ds_validation, verbose=True, learningrate = 0.01)
trainer.trainUntilConvergence(verbose=True, trainingData=ds_training, validationData=ds_validation, convergence_threshold=10, maxEpochs = 10000)
print "Net 5 Training Complete"
print

time.sleep(1)

ANNFile = open("ANNP01File5.dat", "w");
cPickle.dump(net, ANNFile);
ANNFile.close()
time.sleep(1)

print
print "ANN 5 Stored"

net = buildNetwork(8, 10, 6, bias=True)
trainer = BackpropTrainer(net, ds_validation, verbose=True, learningrate = 0.01)
trainer.trainUntilConvergence(verbose=True, trainingData=ds_training, validationData=ds_validation, convergence_threshold=10, maxEpochs = 10000)
print "Net 10 Training Complete"
print

time.sleep(1)

ANNFile = open("ANNP01File10.dat", "w");
cPickle.dump(net, ANNFile);
ANNFile.close()
time.sleep(1)

print
print "ANN 10 Stored"

net = buildNetwork(8, 15, 6, bias=True)
trainer = BackpropTrainer(net, ds_validation, verbose=True, learningrate = 0.01)
trainer.trainUntilConvergence(verbose=True, trainingData=ds_training, validationData=ds_validation, convergence_threshold=10, maxEpochs = 10000)
print "Net 15 Training Complete"
print

time.sleep(1)

ANNFile = open("ANNP01File15.dat", "w");
cPickle.dump(net, ANNFile);
ANNFile.close()
time.sleep(1)

print
print "ANN 15 Stored"

net = buildNetwork(8, 20, 6, bias=True)
trainer = BackpropTrainer(net, ds_validation, verbose=True, learningrate = 0.01)
trainer.trainUntilConvergence(verbose=True, trainingData=ds_training, validationData=ds_validation, convergence_threshold=10, maxEpochs = 10000)
print "Net 20 Training Complete"
print

time.sleep(1)

ANNFile = open("ANNP01File20.dat", "w");
cPickle.dump(net, ANNFile);
ANNFile.close()
time.sleep(1)

print
print "ANN 20 Stored"

net = buildNetwork(8, 25, 6, bias=True)
trainer = BackpropTrainer(net, ds_validation, verbose=True, learningrate = 0.01)
trainer.trainUntilConvergence(verbose=True, trainingData=ds_training, validationData=ds_validation, convergence_threshold=10, maxEpochs = 10000)
print "Net 25 Training Complete"
print

time.sleep(1)

ANNFile = open("ANNP01File25.dat", "w");
cPickle.dump(net, ANNFile);
ANNFile.close()
time.sleep(1)

print
print "ANN 25 Stored"

net = buildNetwork(8, 30, 6, bias=True)
trainer = BackpropTrainer(net, ds_validation, verbose=True, learningrate = 0.01)
trainer.trainUntilConvergence(verbose=True, trainingData=ds_training, validationData=ds_validation, convergence_threshold=10, maxEpochs = 10000)
print "Net 30 Training Complete"
print

time.sleep(1)

ANNFile = open("ANNP01File30.dat", "w");
cPickle.dump(net, ANNFile);
ANNFile.close()
time.sleep(1)

print
print "ANN 30 Stored"

net = buildNetwork(8, 35, 6, bias=True)
trainer = BackpropTrainer(net, ds_validation, verbose=True, learningrate = 0.01)
trainer.trainUntilConvergence(verbose=True, trainingData=ds_training, validationData=ds_validation, convergence_threshold=10, maxEpochs = 10000)
print "Net 35 Training Complete"
print

time.sleep(1)

ANNFile = open("ANNP01File35.dat", "w");
cPickle.dump(net, ANNFile);
ANNFile.close()
time.sleep(1)

print
print "ANN 35 Stored"

net = buildNetwork(8, 40, 6, bias=True)
trainer = BackpropTrainer(net, ds_validation, verbose=True, learningrate = 0.01)
trainer.trainUntilConvergence(verbose=True, trainingData=ds_training, validationData=ds_validation, convergence_threshold=10, maxEpochs = 10000)
print "Net 40 Training Complete"
print

time.sleep(1)

ANNFile = open("ANNP01File40.dat", "w");
cPickle.dump(net, ANNFile);
ANNFile.close()
time.sleep(1)

print
print "ANN 40 Stored"

net = buildNetwork(8, 45, 6, bias=True)
trainer = BackpropTrainer(net, ds_validation, verbose=True, learningrate = 0.01)
trainer.trainUntilConvergence(verbose=True, trainingData=ds_training, validationData=ds_validation, convergence_threshold=10, maxEpochs = 10000)
print "Net 45 Training Complete"
print

time.sleep(1)

ANNFile = open("ANNP01File45.dat", "w");
cPickle.dump(net, ANNFile);
ANNFile.close()
time.sleep(1)

print
print "ANN 45 Stored"

net = buildNetwork(8, 50, 6, bias=True)
trainer = BackpropTrainer(net, ds_validation, verbose=True, learningrate = 0.01)
trainer.trainUntilConvergence(verbose=True, trainingData=ds_training, validationData=ds_validation, convergence_threshold=10, maxEpochs = 10000)
print "Net 50 Training Complete"
print

time.sleep(1)

ANNFile = open("ANNP01File50.dat", "w");
cPickle.dump(net, ANNFile);
ANNFile.close()
time.sleep(1)

print
print "ANN 50 Stored"
