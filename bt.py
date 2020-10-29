import serial
import time
from tkinter import messagebox

def bt(fchain):	
	print("Start")
	port="COM4" #specify the outgoing port that the bluetooth module is connected to
	try:
		bluetooth=serial.Serial(port, 9600)#Start communications with the bluetooth unit
		print("Connected")
		bluetooth.flushInput() #This gives the bluetooth a little kick
		current=fchain[0]
		bluetooth.write(b""+str.encode(fchain[0]))
		time.sleep(1)
		for i in fchain:
			print("Ping")
			if(i!=current):
				current=i
				bluetooth.write(b""+str.encode(i))
				time.sleep(1)#Converting to bytes 
			time.sleep(0.1) #A pause between bursts
		bluetooth.write(b"Z")
		bluetooth.close() #Otherwise the connection will remain open until a timeout which ties up the COM port
		print("Done")
	except:
		messagebox.showerror("ERROR","Bluetooth Not Found")
	
