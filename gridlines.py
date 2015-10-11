import _tkinter
from Tkinter import *


class Application(Frame):
    def say_hi(self):
        print "hi there, everyone!"

    def createWidgets(self):
        self.QUIT = Button(self)
        self.QUIT["text"] = "QUIT"
        self.QUIT["fg"]   = "red"
        self.QUIT["command"] =  self.quit

        self.QUIT.pack({"side": "left"})

        self.hi_there = Button(self)
        self.hi_there["text"] = "Hello",
        self.hi_there["command"] = self.say_hi

        self.hi_there.pack({"side": "left"})

    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.pack()
        self.createWidgets()

    def makeCanvas(self):
		pass    	

root = Tk()
#app = Application(master=root)
#app.mainloop()
#root.destroy()
gridwidth = 1000
gridheight = 1000
widthofline = 2.5

canvas = Canvas(root, width=gridwidth, height=gridheight)

def drawLine(canvas, x0, y0, x1, y1):
	canvas.create_rectangle(x0,y0,x1, y1, fill="black", width=0)
	
def drawGrid(canvas):
	x0 = 0
	x1 = gridwidth
	y0 = 0
	y1 = widthofline

	while (y0 < gridwidth):
		y0 += gridheight/20
		y1 += gridheight/20
		drawLine(canvas, x0,y0,x1,y1)

	x0 = 0
	x1 = widthofline
	y0 = 0
	y1 = gridheight

	while (x0 < gridheight):
		x0 += gridheight/20
		x1 += gridheight/20
		drawLine(canvas, x0,y0,x1,y1)

def onCanvasClick(event):
    print 'Got canvas click', event.x, event.y

canvas.bind('<Button-1>', onCanvasClick) 
canvas.pack()
canvas.create_rectangle(0,0,gridheight,gridwidth,fill="gray")
drawGrid(canvas)
root.mainloop()
