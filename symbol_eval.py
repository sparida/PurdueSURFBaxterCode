
import numpy
import math

def getsym(data):
	
	rows = numpy.size(data, axis = 0)
	cols = numpy.size(data, axis = 1)

	x = numpy.zeros((rows, cols))
	mag = numpy.zeros((rows,1))
	theta = numpy.zeros((rows,1))
	vel = numpy.zeros((rows,1))
	angle = numpy.zeros((rows,1))
	print rows
	for i in range(rows - 1):
		x[i+1,0:2] = data [i+1,0:2] - data[i,0:2]
		mag[i+1, 0] = numpy.linalg.norm(numpy.array(x[i+1, 0:2]))
		if(mag[i+1, 0] == mag[i, 0]):
			vel[i+1, 0] = 0
		elif(mag[i+1, 0] > mag[i, 0]):
		        vel[i+1, 0] = 1
		else:
		        vel[i+1, 0] = 2
	    	print i
		try:
		
			theta[i+1, 0] = numpy.real(math.acos(numpy.dot(x[i, 0:2], x[i+1, 0:2]) / mag[i, 0] * mag[i+1, 0])) * 180.0 / math.pi
			if (numpy.isnan(theta[i+1, 0])):
				theta[i+1, 0] = 0
				
		except:
	
			print "Error"
		# Nana stuff
		angle[i+1, 0] = math.floor(theta[i+1, 0]/10.0);
		
	
	
	c = numpy.mat(numpy.hstack((vel, angle)))
	feat_vec = numpy.mat(numpy.hstack((c, data[:, 3])))  
	
	weights = numpy.mat([[1], [3], [54]])
	symbols = feat_vec * weights
	return symbols	
	#np.savetxt('SymbolG1SQ135S1.txt', symbols, delimiter='\n') 	
