import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('IMG_20190715_133550.jpg')
ims = cv2.resize(img, (600, 600))
hsv = cv2.cvtColor(ims, cv2.COLOR_BGR2HSV)
low = np.array([0,0,140], dtype=np.uint8)
high = np.array([180,252,255], dtype=np.uint8)

ir = cv2.inRange(hsv, low, high)

cv2.imshow('ir',ir)
cv2.imshow('img',ims)
cv2.waitKey(0)
cv2.destroyAllWindows()