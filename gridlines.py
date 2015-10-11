import _tkinter
from Tkinter import *

gridwidth = 1000
gridheight = 1000
widthofline = 2.5
circleRadius = 4

class UAV:
	
	def __init__(self, grid):
		self.x = 0
		self.y = 0

	def goTo(self,x,y):
		self.setPosition(x*gridwidth/20, y*gridheight/20)
		self.draw()

	def draw(self):
		self.grid.drawUAV(self.x,self.y)

	def updatePosition(self, x, y):
		self.x = x
		self.y = y


class Grid:

	def __init__(self):
		root = Tk()
		#app = Application(master=root)
		self.canvas = Canvas(root, width=gridwidth, height=gridheight)
		self.canvas.bind('<Button-1>', self.onCanvasClick) 
		self.canvas.pack()
		self.canvas.create_rectangle(0,0,gridheight,gridwidth,fill="gray")
		self.drawGrid(self.canvas)
		root.mainloop()

	def drawLine(self, canvas, x0, y0, x1, y1):
		self.canvas.create_rectangle(x0,y0,x1, y1, fill="black", width=0)

	def drawGrid(self, canvas, uav):
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

		self.drawUAV(uav)

	def onCanvasClick(self, event):
		print 'Got canvas click', event.x, event.y

	def drawUAV(self, uav):
		#update uav
		self.canvas.create_circle(uav.x,uav.y,circleRadius, fill="red", outline="black", width=1)

	def clearCanvas(self):
		canvas.delete("all")


uav=UAV()
g=Grid()
while True:
	g.clearCanvas()
	g.drawGrid(uav)
	#g.updateUAV()







