import _tkinter
from Tkinter import *

gridwidth = 1000
gridheight = 1000
widthofline = 2.5

class UAV:
	
	def __init__(self, grid):
		self.x = 0
		self.y = 0

	def goTo(self,x,y):
		self.x = x*gridwidth/20
		self.y = y*gridheight/20
		self.draw()

	def draw(self):
		grid.draw()

class Grid:

	def __init__(self):
		root = Tk()
		#app = Application(master=root)
		self.canvas = Canvas(root, width=gridwidth, height=gridheight)
		self.uav = UAV(self.canvas)
		self.canvas.bind('<Button-1>', self.onCanvasClick) 
		self.canvas.pack()
		self.canvas.create_rectangle(0,0,gridheight,gridwidth,fill="gray")
		self.drawGrid(self.canvas)
		root.mainloop()

	def drawLine(self, canvas, x0, y0, x1, y1):
		self.canvas.create_rectangle(x0,y0,x1, y1, fill="black", width=0)

	def drawGrid(self, canvas):
		x0 = 0
		x1 = gridwidth
		y0 = 0
		y1 = widthofline

		while (y0 < gridwidth):
			y0 += gridheight/20
			y1 += gridheight/20
			self.drawLine(self.canvas, x0,y0,x1,y1)

		x0 = 0
		x1 = widthofline
		y0 = 0
		y1 = gridheight

		while (x0 < gridheight):
			x0 += gridheight/20
			x1 += gridheight/20
			self.drawLine(self.canvas, x0,y0,x1,y1)

	def onCanvasClick(self, event):
		print 'Got canvas click', event.x, event.y


Grid()







