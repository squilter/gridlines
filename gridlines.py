import time
import _tkinter
from Tkinter import *
from dronekit import connect, mavutil

gridwidth = 1000
gridheight = 1000
widthofline = 2.5
circleRadius = 4

class UAV:
	def __init__(self):
		self.vehicle = connect('udpin:0.0.0.0:14550', await_params=False)
		self.vehicle.add_attribute_observer('local_position', self.location_callback)
		self.x = 0
		self.y = 0

	def set_position(self, x, y):
		self.x = x*gridwidth/20
		self.y = y*gridheight/20
		
	def location_callback(self, location):
		print self.vehicle.local_position
		self.set_position(self.vehicle.local_position.north, self.vehicle.local_position.east)
	
	def start_vehicle(altitude):
		if v.mode.name == "INITIALISING":
			time.sleep(1)
		while v.gps_0.fix_type < 2:
			time.sleep(1)

		v.armed = True
		v.flush()
	
		while not v.armed:
			time.sleep(1)
	
		vehicle.mode = VehicleMode("GUIDED")
		Vehicle.commands.takeoff(altitude)
		
	def goto_local_position(self, north, east, down):
		if(self.vehicle.armed is False):
			self.start_vehicle(5)
	
		msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
			0,       # time_boot_ms (not used)
			0, 0,    # target system, target component
			mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
			0b0000111111111000, # type_mask (only positions enabled)
			north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
			0, 0, 0, # x, y, z velocity in m/s  (not used)
			0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
			0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
		self.vehicle.send_mavlink(msg)
		self.vehicle.flush()	
	

class Grid:

	def __init__(self, uav):
		self.uav = uav
		
		root = Tk()
		#app = Application(master=root)
		self.canvas = Canvas(root, width=gridwidth, height=gridheight)
		self.canvas.bind('<Button-1>', self.onCanvasClick) 
		self.canvas.pack()
		self.canvas.create_rectangle(0,0,gridheight,gridwidth,fill="gray")
		self.drawGrid(self.canvas, uav)
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
		self.uav.goto_local_position(event.x/50, event.y/50, -5)

	def drawUAV(self, uav):
		#update uav
		self.canvas.create_circle(uav.x,uav.y,circleRadius, fill="red", outline="black", width=1)

	def clearCanvas(self):
		canvas.delete("all")


uav=UAV()
g=Grid(uav)
while True:
	g.clearCanvas()
	g.drawGrid(uav)
	time.sleep(0.1)
	#g.updateUAV()
