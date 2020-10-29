from tkinter import*
from maze import project

root =Tk()
root.title("Maze Solving Bot")
root.geometry('1920x1080')
win=Frame(root,relief=RIDGE,borderwidth=2)
win.pack(fill=BOTH,expand=1)
win.config(background='#A6F1F5')
def run():
    project()

photo=PhotoImage(file="maze1.png",height=350,width=350)
maze_label=Label(image=photo).place(x=610,y=140)
label1=Label(root, text='Robot Maze Solver',fg ="#FFFFFF",bg='#01053b',font=("Times 20 italic",50)).place(x=500,y=10)
b=Button(root,text='Compile',fg='white',bg='#051DFA',command=run,relief=RIDGE,font=("arial",20,"bold")).place(x=700,y=520)
root.mainloop()
