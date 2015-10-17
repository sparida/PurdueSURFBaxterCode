import numpy
import math
from symbol_eval import *

class HMM:
	def __init__(self, filename):
		
		self._fileName = filename
		fileData = numpy.loadtxt(self._fileName)
		print fileData[6:30]
		self._numStates = int(fileData[0])
		PVector = numpy.matrix(fileData[1:self._numStates + 1])
		PVector = PVector.reshape((self._numStates, 1))	
		nArray = fileData[(self._numStates + 1): (self._numStates + 1 + self._numStates*self._numStates)]
		print self._numStates
		print (self._numStates + 1)
		print (self._numStates + self._numStates*self._numStates)
		print nArray
		AMatrix = numpy.matrix(nArray.reshape((self._numStates, self._numStates)))
		lastIndex = (self._numStates + self._numStates**2)
		self._numSym = fileData[lastIndex + 1]
		lastIndex = lastIndex + 1 + self._numSym
		mArray = fileData[(lastIndex + 1) : lastIndex + (self._numStates * self._numSym + 1)]
		lastIndex = lastIndex + (self._numStates * self._numSym)
		BMatrix = numpy.matrix(mArray.reshape((self._numStates, self._numSym)))
		
		self._logP = numpy.matrix(numpy.log(PVector))
		self._logA = numpy.matrix(numpy.log(AMatrix))
		self._logB = numpy.matrix(numpy.log(BMatrix))
	
	
	def viterbi(self, O): # O is symbol colum matrix
		T = numpy.size(O, axis = 0)
		print T
		log_del = numpy.zeros((T, self._numStates))
		print log_del.size
		for i in range(0, self._numStates):
			print self._logP[i, 0]
			print self._logB[i, O[0, 0]] 
			log_del[0,i] = self._logP[i, 0] + self._logB[i, O[0, 0]]
			
			
		for t in range(1, T):
			for j in range(0, self._numStates):
				best_option = []
				for i in range(0, self._numStates):
					this_log_prob = log_del[t-1, i] + self._logA[i, j]
					if i == 0:
						best_option = this_log_prob
					elif(this_log_prob > best_option):
						best_option = this_log_prob
				
				log_del[t, j] = best_option + self._logB[j, O[t, 0]]	
		
		temp_matrix = log_del[T-1, :]
		print "Ans:"
		return  numpy.amax(temp_matrix)	

def main():
	obj = HMM('G2.hmm')
	O = getsym('PointDataG3DA52S3.txt')
	print O
	
	print obj.viterbi(O)

if __name__ == '__main__':
	main()
