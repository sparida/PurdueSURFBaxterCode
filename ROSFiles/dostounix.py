import os
from os.path import join
import sys

NNList = map(lambda x: str(x), range(5, 51, 5))
LRList = map(lambda x: str(x).zfill(3), [1] + range(5, 101, 5))
	
for lr in LRList:
	for nn in NNList:
		outerfolder = "TrainedANNNets"
		foldername = "LR0P" + lr + "Iter10000"
		filename = "ANNP" + lr + "File" + nn + ".dat"
		name = join(outerfolder, foldername, filename)
		command = "dos2unix " + name
	
		print NNList.index(nn)
		#print command
		#os.system(command)
