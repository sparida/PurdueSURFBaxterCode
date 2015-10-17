#!/usr/bin/env python

import cv2
import numpy
import sys

def nothing(x):
	pass


def main():
	
	
	cap = cv2.VideoCapture(0)
	cv2.namedWindow('Normal', cv2.WINDOW_AUTOSIZE)
	cv2.namedWindow('Trackbars', cv2.WINDOW_AUTOSIZE)
	cv2.namedWindow('Binary', cv2.WINDOW_AUTOSIZE)
	cv2.namedWindow('Object', cv2.WINDOW_AUTOSIZE)
	cv2.createTrackbar('HueMin','Trackbars', 0, 179, nothing)
	cv2.createTrackbar('HueMax','Trackbars', 0, 179, nothing)
	cv2.createTrackbar('SatMin','Trackbars', 0, 255, nothing)
	cv2.createTrackbar('SatMax','Trackbars', 0, 255, nothing)
	cv2.createTrackbar('ValMin','Trackbars', 0, 255, nothing)
	cv2.createTrackbar('ValMax','Trackbars', 0, 255, nothing)
	kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
	
	while(True):
		ret, frame = cap.read()
		cv2.imshow('Normal', frame)
		
		hmin = cv2.getTrackbarPos('HueMin', 'Trackbars')
		smin = cv2.getTrackbarPos('SatMin', 'Trackbars')
		vmin = cv2.getTrackbarPos('ValMin', 'Trackbars')
		hmax = cv2.getTrackbarPos('HueMax', 'Trackbars')
		smax = cv2.getTrackbarPos('SatMax', 'Trackbars')
		vmax = cv2.getTrackbarPos('ValMax', 'Trackbars')
		
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		
		lower_thresh = numpy.array([hmin, smin, vmin])
		upper_thresh = numpy.array([hmax, smax, vmax])
	
		mask = cv2.inRange(hsv, lower_thresh, upper_thresh)
		
		er = cv2.erode(mask, kernel, iterations = 2)
		dl = cv2.dilate(er, kernel, iterations = 2)
		
		t_img = cv2.bitwise_and(frame, frame, mask = dl)
		
		cv2.imshow('Normal', frame)
		cv2.imshow('Object', t_img)
		cv2.imshow('Binary', dl)	
		"""
		"""	
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	cap.release()
	cv2.destroyAllWindows()

if __name__ == '__main__':
    sys.exit(main())

