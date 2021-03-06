from transform import four_point_transform
#from skimage.filters import threshold_local
import numpy as np
import argparse
import cv2
import imutils

image = cv2.imread("3.jpg")
ratio = image.shape[0] / 500.0
orig = image.copy()
image = imutils.resize(image, height = 500)
 
# convert the image to grayscale, blur it, and find edges
# in the image
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

gray = cv2.GaussianBlur(gray, (5, 5), 0)
edged = cv2.Canny(gray, 75, 200)
 
# show the original image and the edge detected image
print("STEP 1: Edge Detection")
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
print("STEP 2: Find contours of paper")
cv2.drawContours(image, [screenCnt], -1, (0, 255, 0), 2)
#cv2.imshow("Outline", image)
#cv2.waitKey(0)
#cv2.destroyAllWindows()

warped = four_point_transform(orig, screenCnt.reshape(4, 2) * ratio)
 
# convert the warped image to grayscale, then threshold it
# to give it that 'black and white' paper effect
warped = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
#equ=cv2.equalizeHist(warped)
k=7
blurred = cv2.GaussianBlur(src = warped, ksize = (k, k), sigmaX = 0)
ret,th3 = cv2.threshold(blurred,100,255,cv2.THRESH_BINARY)
#cv2.imshow("Scanned", imutils.resize(th3, height = 650))
#cv2.waitKey(0)
kernel = np.ones((40,40),np.uint8)
opening = cv2.morphologyEx(th3, cv2.MORPH_OPEN, kernel)
#closing = cv2.morphologyEx(th3, cv2.MORPH_CLOSE, kernel)
#warped = (warped > th3).astype("uint8") * 255
 
# show the original and scanned images
print("STEP 3: Apply perspective transform")
#cv2.imshow("Original", imutils.resize(orig, height = 650))
cv2.imshow("Sd", imutils.resize(opening, height = 650))
cv2.waitKey(0)
'''

#ig = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)

equ=cv2.equalizeHist(warped)
k=7
blurred = cv2.GaussianBlur(src = equ, ksize = (k, k), sigmaX = 0)

ret,th3 = cv2.threshold(blurred,100,255,cv2.THRESH_BINARY)
print(th3)
#im = cv2.cvtColor(th3, cv2.COLOR_GRAY2RGB)
#opening=cv2.resize(opening,(318,232))
cv2.imshow('output',opening)

cv2.waitKey(0)'''