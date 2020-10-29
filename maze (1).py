from transform import four_point_transform
import numpy as np
import cv2
import imutils
import statistics as s
import os
import serial
import time

def get_latest_image(dirpath, valid_extensions=('jpg','jpeg','png')):
    """
    Get the latest image file in the given directory
    """

    # get filepaths of all files and dirs in the given dir
    valid_files = [os.path.join(dirpath, filename) for filename in os.listdir(dirpath)]
    # filter out directories, no-extension, and wrong extension files
    valid_files = [f for f in valid_files if '.' in f and \
        f.rsplit('.',1)[-1] in valid_extensions and os.path.isfile(f)]

    if not valid_files:
        raise ValueError("No valid images in %s" % dirpath)

    return max(valid_files, key=os.path.getmtime)
print("Start")
dir="C:/Users/Edelquinn/Desktop/Fp/mazes"
img=get_latest_image(dir)
#print(img)

#img = input("Enter filename : ")
#img = 'C:/Users/Edelquinn/Desktop/Fp/mazes/m2.jpg'
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
 
# show the original image and the edge detected image
#print("Perspective Transformation")


cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(cnts)
cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:5]
 

for c in cnts:
    # approximate the contour
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.02 * peri, True)
    #print(approx)
    if len(approx) == 4:
        screenCnt = approx
        break
 
# show the contour (outline) of the piece of paper
cv2.drawContours(image, [screenCnt], -1, (0, 255, 0), 2)


warped = four_point_transform(orig, screenCnt.reshape(4, 2) * ratio)
scale_percent = 20 # percent of original size
width = int(warped.shape[1] * scale_percent / 100)
height = int(warped.shape[0] * scale_percent / 100)
dim = (width, height)
w = cv2.resize(warped, dim, interpolation = cv2.INTER_AREA)

imgdn = cv2.fastNlMeansDenoisingColored(w,None,10,10,7,21)
imgdn = cv2.fastNlMeansDenoisingColored(imgdn,None,10,10,7,21)

img = cv2.cvtColor(imgdn, cv2.COLOR_BGR2HSV)


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
#print(xr,yr)
xr=int(s.mean(xr))
yr=int(s.mean(yr))
xg=int(s.mean(xg))
yg=int(s.mean(yg))

cv2.circle(w,(yr,xr),2,(0,0,255),-1)
cv2.circle(w,(yg,xg),2,(0,255,0),-1)

# convert the warped image to grayscale, then threshold it
# to give it that 'black and white' paper effect
war = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
#equ=cv2.equalizeHist(warped)
k=7
blurred = cv2.GaussianBlur(src = war, ksize = (k, k), sigmaX = 0)
ret,th3 = cv2.threshold(blurred,90,255,cv2.THRESH_BINARY)


kernel = np.ones((40,40),np.uint8)
opening = cv2.morphologyEx(th3, cv2.MORPH_OPEN, kernel)

scale_percent = 20 # percent of original size
width = int(opening.shape[1] * scale_percent / 100)
height = int(opening.shape[0] * scale_percent / 100)
dim = (width, height)
opening = cv2.resize(opening, dim, interpolation = cv2.INTER_AREA)

for d in range(-10,10,1):
    for e in range(-10,10,1):
        opening[xr+e,yr-d] = 255
        opening[xg+e,yg-d] = 255


_,maze = cv2.threshold(opening,100,1,cv2.THRESH_BINARY_INV)

kernel = np.ones((12,12),np.uint8)
maze = cv2.dilate(maze,kernel,iterations = 3)

grid = maze.tolist()

init = [xr, yr]
goal = [xg, yg]#[len(grid) - 1, len(grid[0]) - 1]  # all coordinates are given in format [y,x]
cost = 1
cl = []

# the cost map which pushes the path closer to the goal
heuristic = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
for i in range(len(grid)):
    for j in range(len(grid[0])):
        heuristic[i][j] = abs(i - goal[0]) + abs(j - goal[1])
        if grid[i][j] == 1:
            heuristic[i][j] = 99  # added extra penalty in the heuristic map


# the actions we can take
delta = [[-1, 0], [0, -1], [1, 0], [0, 1]]  # go up  # go left  # go down  # go right


# function to search the path
def search(grid, init, goal, cost, heuristic):
    closed = [
        [0 for col in range(len(grid[0]))] for row in range(len(grid))
    ]  # the referrence grid
    closed[init[0]][init[1]] = 1
    action = [
        [0 for col in range(len(grid[0]))] for row in range(len(grid))
    ]  # the action grid

    x = init[0]
    y = init[1]
    g = 0
    f = g + heuristic[init[0]][init[1]]
    cell = [[f, g, x, y]]

    found = False  # flag that is set when search is complete
    resign = False  # flag set if we can't find expand

    while not found and not resign:
        if len(cell) == 0:
            resign = True
            return "FAIL"
        else:
            cell.sort()  # to choose the least costliest action so as to move closer to the goal
            cell.reverse()
            next = cell.pop()
            x = next[2]
            y = next[3]
            g = next[1]
            f = next[0]

            if x == goal[0] and y == goal[1]:
                found = True
            else:
                for i in range(len(delta)):  # to try out different valid actions
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            f2 = g2 + heuristic[x2][y2]
                            cell.append([f2, g2, x2, y2])
                            closed[x2][y2] = 1
                            action[x2][y2] = i
                            cl.append([x2,y2])
    invpath = []
    x = goal[0]
    y = goal[1]
    invpath.append([x, y])  # we get the reverse path from here
    while x != init[0] or y != init[1]:
        x2 = x - delta[action[x][y]][0]
        y2 = y - delta[action[x][y]][1]
        x = x2
        y = y2
        invpath.append([x, y])
        
        

    path = []
    #print("Grid")
    '''for i in range(len(grid)):
        print(grid[i])'''
    for i in range(len(invpath)):
        path.append(invpath[len(invpath) - 1 - i])
    #print("ACTION MAP")
    '''for i in range(len(action)):
        print(action[i])'''
    return path


a = search(grid, init, goal, cost, heuristic)
    
for i in range(len(a)):
    b=a[i]
    #print(a[i])
    w[b[0],b[1]] = (255,0,0)
cv2.circle(w, (yg, xg), 2, (0,255,0), -1)
cv2.circle(w, (yr, xr), 2, (0,0,255), -1)
cv2.imshow('Solution',w)
cv2.waitKey(0)

chain=[]
prev=a[0]
next=a[1]
#print('\n\n\n\n',prev,next)
for i in range(1,len(a)-1):
    if prev[0]==next[0] and prev[1]>next[1]:
        chain.append('L')
    elif prev[0]==next[0] and prev[1]<next[1]:
        chain.append('R')
    elif prev[0]<next[0] and prev[1]==next[1]:
        chain.append('B')
    elif prev[0]>next[0] and prev[1]==next[1]:
        chain.append('F')
    prev=a[i]
    next=a[i+1]

fchain=[]
comp=5
l,r,f,b=0,0,0,0
for i in chain:
	if i=='F':
		f+=1
		if f==comp:
			f=0
			fchain.append('F')
	elif i=='L':
		l+=1
		if l==comp:
			l=0
			fchain.append('L')
	elif i=='R':
		r+=1
		if r==comp:
			r=0
			fchain.append('R')
	elif i=='B':
		b+=1
		if b==comp:
			b=0
			fchain.append('B')
	
print("Start")
port="COM9" #This will be different for various devices and on windows it will probably be a COM port.
bluetooth=serial.Serial(port, 9600)#Start communications with the bluetooth unit
print("Connected")
bluetooth.flushInput() #This gives the bluetooth a little kick
current=fchain[0]
bluetooth.write(b""+str.encode(fchain[0]))
for i in fchain:
    print("Ping")
    if(i!=current):
        current=i
        bluetooth.write(b""+str.encode(i))#These need to be bytes not unicode, plus a number
    time.sleep(0.1) #A pause between bursts
bluetooth.close() #Otherwise the connection will remain open until a timeout which ties up the /dev/thingamabob
print("Done")




