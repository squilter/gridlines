#!/usr/bin/env python

import mavproxy
from subprocess import Popen
import os
import time
import rospy
import sys
#from uav_msgs.msg import WayPointPost.msg
import droneapi.lib
from pymavlink import mavutil


velocityFreq=20

class fakeLoc():
	def __init__(self,x,y,z):
		self.x = x
		self.y = y
		self.z = z

class UAV():
	def __init__(self):
		self.api = local_connect()
		self.v = None

	def initialize(self):
	 	#api = local_connect()
		self.v = self.api.get_vehicles()[0]
		if self.v.mode.name == "INITIALISING":
		    print "Vehicle still booting, try again later"
		    return
		#rospy.Subscriber('way_point_post',WayPointPost,next_cmd)

	def next_cmd(self, a): 
		dest = droneapi.lib.Location(a.x,a.y,a.z, is_relative=True)

		if self.v.mode.name != "GUIDED":
			print "Vehicle is not in guided mode."
			return
	 	self.v.commands.goto(dest)
	 	is_guided = True
	 	self.v.flush()

	def testAttitude(self):
		if (self.v != None):
			while(True):
				time.sleep(.1)
				print self.v.attitude.yaw;

	def testNextCmd(self):
		if (self.v != None):
			uav.next_cmd(fakeLoc(100,50,10));

	def testLocation(self):
		if (self.v != None):
			while(True):
				time.sleep(.1)
				print self.v.location

uav=UAV()
uav.initialize()
# uav.testAttitude()
uav.testNextCmd()
uav.testLocation()
# uav.next_cmd(fakeLoc(100,50,10))
