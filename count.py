'''from PIL import Image,ImageColor
import cv2
im = Image.open("3.jpg")
img = ImageColor.getcolor("red", "L") 
print(img)
pix_val = list(im.getdata())
#print(pix_val)
#pix_val_flat = [x for sets in pix_val for x in sets]
if (255,0,0) in pix_val:
    print("yes")
else:
    print("no")
'''
import cv2
import numpy as np
import statistics as s
img = cv2.imread("3.jpg")
scale_percent = 50 # percent of original size
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)
x=[]
y=[]
xg=[]
yg=[]
# resize image
img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
cv2.imshow("image",img)
cv2.waitKey(0)
img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
lower_skin = np.array([0,70,50], dtype=np.uint8)
upper_skin = np.array([10,255,255], dtype=np.uint8)
roi2 = cv2.inRange(img, lower_skin, upper_skin)
roi1=cv2.inRange(img,(36, 25, 25), (70, 255,255))
cv2.imshow("ROI", roi2)
cv2.waitKey(0)
cv2.imshow("ROI green", roi1)
cv2.waitKey(0)
b=roi2.tolist()
c=roi1.tolist()

#rows=a[0]
#columns=a[1]
#print(b)
for i in range(height):
  for j in range(width):
    if b[i][j]==255:
        x.append(i)
        y.append(j)
    if c[i][j]==255:
        xg.append(i)
        yg.append(j)
x=int(s.mean(x))
y=int(s.mean(y))
xg=int(s.mean(xg))
yg=int(s.mean(yg))
print(x,y)
print(xg,yg)