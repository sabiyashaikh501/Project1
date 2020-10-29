import cv2
from tkinter import messagebox

def solution(w,xg,yg,xr,yr,a):   	
	for i in range(len(a)):
		b=a[i]
		try:
			w[b[0],b[1]] = (255,0,0)
		except:
			messagebox.showerror("ERROR","Path not found")
			break
	cv2.circle(w, (yg, xg), 2, (0,255,0), -1)
	cv2.circle(w, (yr, xr), 2, (0,0,255), -1)
	cv2.imshow('Solution',w)
	cv2.waitKey(0)
	cv2.destroyAllWindows()