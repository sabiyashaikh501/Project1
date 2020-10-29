from transform import four_point_transform
import numpy as np
import argparse
import cv2
import imutils
import statistics as s
from tkinter import messagebox


def proc(img):	
	image = cv2.imread(img)
	ratio = image.shape[0] / 500.0
	orig = image.copy()
	image = imutils.resize(orig, height = 500)
	xr=[]
	yr=[]
	xg=[]
	yg=[]
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(gray, (5, 5), 0)
	edged = cv2.Canny(gray, 75, 200)
	 
	#find edges
	cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:5]
	 

	for c in cnts:
		# approximate the contour
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.02 * peri, True)
		if len(approx) == 4:
			screenCnt = approx
			break
	 
	# show the contour (outline) of the piece of paper
	cv2.drawContours(image, [screenCnt], -1, (0, 255, 0), 2)


	warped = four_point_transform(orig, screenCnt.reshape(4, 2) * ratio)
	scale_percent = 18 # percent of original size
	width = int(warped.shape[1] * scale_percent / 100)
	height = int(warped.shape[0] * scale_percent / 100)
	dim = (width, height)
	w = cv2.resize(warped, dim, interpolation = cv2.INTER_AREA)

	imgdn = cv2.fastNlMeansDenoisingColored(w,None,10,10,7,21)
	imgdn = cv2.fastNlMeansDenoisingColored(imgdn,None,10,10,7,21)

	img = cv2.cvtColor(imgdn, cv2.COLOR_BGR2HSV)

	#search the start and end points in the maze
	roi2 = cv2.inRange(img, (0,70,50), (10,255,255))
	roi1=cv2.inRange(img,(36, 25, 50), (70, 255,255))
	kernel=np.ones((3,3),np.uint8)
	roi1=cv2.erode(roi1,kernel,iterations=1)

	b=roi2.tolist()
	c=roi1.tolist()

	for i in range(height):
		for j in range(width):
			if b[i][j]==255:
				xr.append(i)
				yr.append(j)
			if c[i][j]==255:
				xg.append(i)
				yg.append(j)


	if len(xr)==0 or len(yr)==0:
		print("Colors not detected")
		messagebox.showerror("ERROR","Start not detected")
	if len(xg)==0 or len(yg)==0:
		print("Colors not detected")
		messagebox.showerror("ERROR","End point not detected")

	xr=int(s.mean(xr))
	yr=int(s.mean(yr))
	xg=int(s.mean(xg))
	yg=int(s.mean(yg))

	cv2.circle(w,(yr,xr),2,(0,0,255),-1)
	cv2.circle(w,(yg,xg),2,(0,255,0),-1)

	# convert the warped image to grayscale, then threshold it
	# to give it that 'black and white' paper effect
	war = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
	k=7
	blurred = cv2.GaussianBlur(src = war, ksize = (k, k), sigmaX = 0)
	ret,th3 = cv2.threshold(blurred,90,255,cv2.THRESH_BINARY)


	kernel = np.ones((40,40),np.uint8)
	opening = cv2.morphologyEx(th3, cv2.MORPH_OPEN, kernel)

	scale_percent = 18 # percent of original size
	width = int(opening.shape[1] * scale_percent / 100)
	height = int(opening.shape[0] * scale_percent / 100)
	dim = (width, height)
	opening = cv2.resize(opening, dim, interpolation = cv2.INTER_AREA)

	for d in range(-20,20,1):
		for e in range(-20,20,1):
			opening[xr+e,yr-d] = 255
			opening[xg+e,yg-d] = 255


	_,maze = cv2.threshold(opening,100,1,cv2.THRESH_BINARY_INV)

	kernel = np.ones((12,12),np.uint8)
	maze = cv2.dilate(maze,kernel,iterations = 5)

	grid = maze.tolist()
	return(grid,xr,xg,yr,yg,w)