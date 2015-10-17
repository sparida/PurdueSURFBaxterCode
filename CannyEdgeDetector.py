#!/usr/bin/env python

import cv2
import numpy
import sys

def nothing(x):
	pass


def main():
	
	
	cv2.namedWindow('Normal', cv2.WINDOW_AUTOSIZE)
	cv2.namedWindow('Trackbars', cv2.WINDOW_AUTOSIZE)
	cv2.namedWindow('BinaryLines', cv2.WINDOW_AUTOSIZE)
	cv2.namedWindow('ColLines', cv2.WINDOW_AUTOSIZE)
	cv2.createTrackbar('Threshold1','Trackbars', 0, 1000, nothing)
	cv2.createTrackbar('Threshold2','Trackbars', 0, 1000, nothing)
	cv2.createTrackbar('GaussianBlur','Trackbars', 0, 1, nothing)
			
	cap = cv2.VideoCapture(0)
	#frame = cv2.imread('Ronaldo.jpg', -1)
	
	while(True):
		ret, frame = cap.read()
		
		
		thresh1 = cv2.getTrackbarPos('Threshold1', 'Trackbars')
		thresh2 = cv2.getTrackbarPos('Threshold2', 'Trackbars')
		blur = cv2.getTrackbarPos('GaussianBlur', 'Trackbars')
		
		if blur == 1:
			img = cv2.GaussianBlur(frame, (5, 5), 0)
		else:
			img = frame
		
		edge = cv2.Canny(img, thresh1, thresh2)
		
		t_img = cv2.bitwise_and(frame, frame, mask = edge)
		
		cv2.imshow('Normal', frame)
		cv2.imshow('ColLines', t_img)
		cv2.imshow('BinaryLines', edge)	
		
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	cap.release()
	cv2.destroyAllWindows()
	
if __name__ == '__main__':
    sys.exit(main())

