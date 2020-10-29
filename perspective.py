from transform import four_point_transform
#from skimage.filters import threshold_local
import numpy as np
import argparse
import cv2
import imutils

def perspective(image):
	ratio = image.shape[0] / 500.0
	orig = image.copy()
	image = imutils.resize(orig, height = 500)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(gray, (5, 5), 0)
	edged = cv2.Canny(gray, 75, 200)
 
# show the original image and the edge detected image
#print("STEP 1: Edge Detection")
#cv2.imshow("Image", image)
#cv2.imshow("Edged", edged)
#cv2.waitKey(0)
	cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:5]
 
# loop over the contours
	for c in cnts:
	# approximate the contour
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.02 * peri, True)
	#print(approx)
		if len(approx) == 4:
			screenCnt = approx
			break
 
# show the contour (outline) of the piece of paper
#print("STEP 2: Find contours of paper")
	cv2.drawContours(image, [screenCnt], -1, (0, 255, 0), 2)
#cv2.imshow("Outline", image)
#cv2.waitKey(0)
#cv2.destroyAllWindows()

	warped = four_point_transform(orig, screenCnt.reshape(4, 2) * ratio)
	scale_percent = 20 # percent of original size
	width = int(warped.shape[1] * scale_percent / 100)
	height = int(warped.shape[0] * scale_percent / 100)
	dim = (width, height)
	w = cv2.resize(warped, dim, interpolation = cv2.INTER_AREA)
	cv2.imshow("perspective",w)
	return w